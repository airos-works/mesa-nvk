#include "nvkmd_nvrm.h"

#include "vk_log.h"
#include "util/os_time.h"
#include "util/u_atomic.h"
#include "util/u_debug.h"
#include "util/macros.h"
#include "nvk_device.h"

/* Syncs live on the processor. A submission ends by making its channel
 * write a sequence number to the channel's semaphore; each signal it carries
 * becomes a point that is pending until the semaphore reaches that number,
 * and is then applied by whoever looks at the sync next. Submissions do not
 * wait for the GPU.
 *
 * A timeline keeps every pending point, so waiting for an early value waits
 * only for the submission that signals it, not for the latest one.
 */

void
nvkmd_nvrm_poll_step(int64_t start_ns)
{
   static int64_t spin_ns = -1, max_sleep_us = -1;
   if (unlikely(p_atomic_read(&max_sleep_us) < 0)) {
      p_atomic_set(&spin_ns,
                   debug_get_num_option("NVK_NVRM_WAIT_SPIN_US", 20) * 1000);
      p_atomic_set(&max_sleep_us,
                   MAX2(debug_get_num_option("NVK_NVRM_WAIT_SLEEP_US", 100), 10));
   }

   const int64_t waited_ns = os_time_get_nano() - start_ns;
   if (waited_ns < p_atomic_read(&spin_ns)) {
#if defined(__i386__) || defined(__x86_64__)
      for (int i = 0; i < 32; i++)
         __builtin_ia32_pause();
#endif
      return;
   }

   /* Sleep for a thirty-second of what was waited so far, between 10 us
    * and the maximum: a short wait ends promptly, a long one costs little
    * processor time and ends at most a few percent late. */
   int64_t sleep_us = waited_ns / 1000 / 32;
   sleep_us = CLAMP(sleep_us, 10, p_atomic_read(&max_sleep_us));
   os_time_sleep(sleep_us);
}

static struct nvkmd_nvrm_sync *
to_nvkmd_nvrm_sync(struct vk_sync *sync)
{
   assert(vk_sync_type_is_nvkmd_nvrm_sync(sync->type));
   return container_of(sync, struct nvkmd_nvrm_sync, base);
}

static bool
is_timeline(const struct nvkmd_nvrm_sync *sync)
{
   return (sync->base.flags & VK_SYNC_IS_TIMELINE) != 0;
}

static bool
point_reached(const struct nvkmd_nvrm_sync_point *point)
{
   return nvkmd_nvrm_seq_reached(nvkmd_nvrm_sem_read(point->sem, point->wide),
                                 point->seq, point->wide);
}

static void
set_value_locked(struct nvkmd_nvrm_sync *sync, uint64_t value)
{
   if (!is_timeline(sync)) {
      p_atomic_set(&sync->value, 1);
      return;
   }
   /* A timeline only moves forward. */
   if (value > p_atomic_read(&sync->value))
      p_atomic_set(&sync->value, value);
}

static void
clear_points_locked(struct nvkmd_nvrm_sync *sync)
{
   util_dynarray_foreach(&sync->points, struct nvkmd_nvrm_sync_point, point)
      nvkmd_mem_unref(point->sem);
   util_dynarray_clear(&sync->points);
}

/* Applies the points whose submissions have run, and drops those a value
 * already reached makes pointless. */
static void
retire_locked(struct nvkmd_nvrm_sync *sync)
{
   const unsigned count =
      util_dynarray_num_elements(&sync->points, struct nvkmd_nvrm_sync_point);
   if (count == 0)
      return;

   struct nvkmd_nvrm_sync_point *points = sync->points.data;
   for (unsigned i = 0; i < count; i++) {
      if (point_reached(&points[i]))
         set_value_locked(sync, points[i].value);
   }

   const bool timeline = is_timeline(sync);
   const uint64_t value = p_atomic_read(&sync->value);
   unsigned kept = 0;
   for (unsigned i = 0; i < count; i++) {
      const bool done = timeline ? points[i].value <= value : value != 0;
      if (done) {
         nvkmd_mem_unref(points[i].sem);
      } else {
         points[kept++] = points[i];
      }
   }
   sync->points.size = kept * sizeof(struct nvkmd_nvrm_sync_point);
}

static bool
reached_locked(struct nvkmd_nvrm_sync *sync, uint64_t wait_value)
{
   retire_locked(sync);
   const uint64_t value = p_atomic_read(&sync->value);
   if (is_timeline(sync))
      return value >= wait_value;
   return value != 0;
}

/* The first point that brings the sync to wait_value, if any. */
static struct nvkmd_nvrm_sync_point *
find_point_locked(struct nvkmd_nvrm_sync *sync, uint64_t wait_value)
{
   const bool timeline = is_timeline(sync);
   util_dynarray_foreach(&sync->points, struct nvkmd_nvrm_sync_point, point) {
      if (!timeline || point->value >= wait_value)
         return point;
   }
   return NULL;
}

void
nvkmd_nvrm_sync_set_signaled(struct vk_sync *_sync, uint64_t value)
{
   struct nvkmd_nvrm_sync *sync = to_nvkmd_nvrm_sync(_sync);

   simple_mtx_lock(&sync->lock);
   set_value_locked(sync, value);
   if (!is_timeline(sync))
      clear_points_locked(sync);
   else
      retire_locked(sync);
   simple_mtx_unlock(&sync->lock);
}

void
nvkmd_nvrm_sync_set_pending(struct vk_sync *_sync, struct nvkmd_mem *sem,
                            bool wide, uint64_t seq, uint64_t value)
{
   struct nvkmd_nvrm_sync *sync = to_nvkmd_nvrm_sync(_sync);

   simple_mtx_lock(&sync->lock);
   if (!is_timeline(sync)) {
      /* A new signal of a binary sync replaces what it had: it is
       * unsignalled until this one has run. */
      clear_points_locked(sync);
      p_atomic_set(&sync->value, 0);
   } else {
      retire_locked(sync);
      if (value <= p_atomic_read(&sync->value)) {
         simple_mtx_unlock(&sync->lock);
         return;
      }
   }
   nvkmd_mem_ref(sem);
   struct nvkmd_nvrm_sync_point point = {
      .sem = sem,
      .seq = seq,
      .value = value,
      .wide = wide,
   };
   util_dynarray_append(&sync->points, struct nvkmd_nvrm_sync_point, point);
   simple_mtx_unlock(&sync->lock);
}

bool
nvkmd_nvrm_sync_ordered_before(struct vk_sync *_sync, uint64_t value,
                               struct nvkmd_mem *sem,
                               struct nvkmd_nvrm_sync_point *point_out)
{
   struct nvkmd_nvrm_sync *sync = to_nvkmd_nvrm_sync(_sync);

   point_out->sem = NULL;
   simple_mtx_lock(&sync->lock);
   bool ordered = reached_locked(sync, value);
   if (!ordered) {
      struct nvkmd_nvrm_sync_point *point = find_point_locked(sync, value);
      if (point != NULL && point->sem == sem) {
         ordered = true;
      } else if (point != NULL) {
         *point_out = *point;
         nvkmd_mem_ref(point->sem);
      }
   }
   simple_mtx_unlock(&sync->lock);
   return ordered;
}

static VkResult
nvkmd_nvrm_sync_init(struct vk_device *device,
                     struct vk_sync *_sync,
                     uint64_t initial_value)
{
   struct nvkmd_nvrm_sync *sync = to_nvkmd_nvrm_sync(_sync);

   simple_mtx_init(&sync->lock, mtx_plain);
   p_atomic_set(&sync->value, initial_value);
   util_dynarray_init(&sync->points, NULL);
   return VK_SUCCESS;
}

void
nvkmd_nvrm_sync_finish(struct vk_device *device,
                       struct vk_sync *_sync)
{
   struct nvkmd_nvrm_sync *sync = to_nvkmd_nvrm_sync(_sync);

   clear_points_locked(sync);
   util_dynarray_fini(&sync->points);
   simple_mtx_destroy(&sync->lock);
}

static VkResult
nvkmd_nvrm_sync_signal(struct vk_device *device,
                       struct vk_sync *_sync,
                       uint64_t value)
{
   nvkmd_nvrm_sync_set_signaled(_sync, value);
   return VK_SUCCESS;
}

static VkResult
nvkmd_nvrm_sync_get_value(struct vk_device *device,
                          struct vk_sync *_sync,
                          uint64_t *value)
{
   struct nvkmd_nvrm_sync *sync = to_nvkmd_nvrm_sync(_sync);

   simple_mtx_lock(&sync->lock);
   retire_locked(sync);
   *value = p_atomic_read(&sync->value);
   simple_mtx_unlock(&sync->lock);
   return VK_SUCCESS;
}

static VkResult
nvkmd_nvrm_sync_reset(struct vk_device *device,
                      struct vk_sync *_sync)
{
   struct nvkmd_nvrm_sync *sync = to_nvkmd_nvrm_sync(_sync);

   simple_mtx_lock(&sync->lock);
   clear_points_locked(sync);
   p_atomic_set(&sync->value, 0);
   simple_mtx_unlock(&sync->lock);
   return VK_SUCCESS;
}

static bool
wait_satisfied(const struct vk_sync_wait *wait, enum vk_sync_wait_flags flags)
{
   struct nvkmd_nvrm_sync *sync = to_nvkmd_nvrm_sync(wait->sync);

   simple_mtx_lock(&sync->lock);
   bool done = reached_locked(sync, wait->wait_value);
   /* Waiting for a signal to be pending: submitted is enough. */
   if (!done && (flags & VK_SYNC_WAIT_PENDING))
      done = find_point_locked(sync, wait->wait_value) != NULL;
   simple_mtx_unlock(&sync->lock);
   return done;
}

static VkResult
nvkmd_nvrm_sync_wait_many(struct vk_device *device,
                          uint32_t wait_count,
                          const struct vk_sync_wait *waits,
                          enum vk_sync_wait_flags wait_flags,
                          uint64_t abs_timeout_ns)
{
   const int64_t start = os_time_get_nano();
   for (;;) {
      uint32_t done = 0;
      for (uint32_t i = 0; i < wait_count; i++) {
         if (wait_satisfied(&waits[i], wait_flags))
            done++;
      }
      if (done == wait_count || (done > 0 && (wait_flags & VK_SYNC_WAIT_ANY)))
         return VK_SUCCESS;
      if ((uint64_t)os_time_get_nano() >= abs_timeout_ns)
         return VK_TIMEOUT;
      nvkmd_nvrm_poll_step(start);
   }
}

VkResult
nvkmd_nvrm_sync_cpu_wait(struct vk_sync *sync, uint64_t value)
{
   const struct vk_sync_wait wait = {
      .sync = sync,
      .stage_mask = ~(VkPipelineStageFlags2)0,
      .wait_value = value,
   };
   return nvkmd_nvrm_sync_wait_many(NULL, 1, &wait, VK_SYNC_WAIT_COMPLETE,
                                    UINT64_MAX);
}

static VkResult
nvkmd_nvrm_sync_import_opaque_fd(struct vk_device *device,
                                 struct vk_sync *sync,
                                 int fd)
{
   return vk_errorf(device, VK_ERROR_UNKNOWN,
                    "sync_import_opaque_fd: not implemented");
}

static VkResult
nvkmd_nvrm_sync_export_opaque_fd(struct vk_device *device,
                                 struct vk_sync *sync,
                                 int *fd)
{
   return vk_errorf(device, VK_ERROR_UNKNOWN,
                    "sync_export_opaque_fd: not implemented");
}

static VkResult
nvkmd_nvrm_sync_move(struct vk_device *device,
                     struct vk_sync *_dst,
                     struct vk_sync *_src)
{
   struct nvkmd_nvrm_sync *dst = to_nvkmd_nvrm_sync(_dst);
   struct nvkmd_nvrm_sync *src = to_nvkmd_nvrm_sync(_src);

   simple_mtx_lock(&src->lock);
   simple_mtx_lock(&dst->lock);
   clear_points_locked(dst);
   p_atomic_set(&dst->value, p_atomic_read(&src->value));
   util_dynarray_foreach(&src->points, struct nvkmd_nvrm_sync_point, point) {
      /* the reference moves with the point */
      util_dynarray_append(&dst->points, struct nvkmd_nvrm_sync_point, *point);
   }
   util_dynarray_clear(&src->points);
   p_atomic_set(&src->value, 0);
   simple_mtx_unlock(&dst->lock);
   simple_mtx_unlock(&src->lock);
   return VK_SUCCESS;
}

struct vk_sync_type
nvkmd_nvrm_sync_get_type(struct nvkmd_nvrm_pdev *pdev)
{
   struct vk_sync_type type = {
      .size = sizeof(struct nvkmd_nvrm_sync),
      .features = VK_SYNC_FEATURE_BINARY |
                  VK_SYNC_FEATURE_GPU_WAIT |
                  VK_SYNC_FEATURE_CPU_RESET |
                  VK_SYNC_FEATURE_CPU_SIGNAL |
                  VK_SYNC_FEATURE_WAIT_PENDING |
                  VK_SYNC_FEATURE_CPU_WAIT |
                  VK_SYNC_FEATURE_TIMELINE,
      .init = nvkmd_nvrm_sync_init,
      .finish = nvkmd_nvrm_sync_finish,
      .signal = nvkmd_nvrm_sync_signal,
      .get_value = nvkmd_nvrm_sync_get_value,
      .reset = nvkmd_nvrm_sync_reset,
      .move = nvkmd_nvrm_sync_move,
      .wait_many = nvkmd_nvrm_sync_wait_many,
      .import_opaque_fd = nvkmd_nvrm_sync_import_opaque_fd,
      .export_opaque_fd = nvkmd_nvrm_sync_export_opaque_fd,
   };
   return type;
}
