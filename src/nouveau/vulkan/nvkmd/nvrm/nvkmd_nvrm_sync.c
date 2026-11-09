#include "nvkmd_nvrm.h"

#include "vk_log.h"
#include "util/os_time.h"
#include "util/u_atomic.h"
#include "util/u_debug.h"
#include "nvk_device.h"

/* Syncs live on the processor. A submission ends by making its channel
 * write a sequence number to the channel's semaphore; a signal it carries
 * is pending until the semaphore reaches that number, and is then applied
 * by whoever looks at the sync next. Submissions do not wait for the GPU. */

/* One step of a polled wait that started at start_ns: spin for the first
 * NVK_NVRM_WAIT_SPIN_US (most waits end within microseconds), then sleep. */
static void
poll_step(int64_t start_ns)
{
   static int64_t spin_ns = -1;
   if (spin_ns < 0)
      spin_ns = debug_get_num_option("NVK_NVRM_WAIT_SPIN_US", 200) * 1000;
   if (os_time_get_nano() - start_ns < spin_ns) {
#if defined(__i386__) || defined(__x86_64__)
      for (int i = 0; i < 64; i++)
         __builtin_ia32_pause();
#endif
      return;
   }
   os_time_sleep(20);
}

static struct nvkmd_nvrm_sync *
to_nvkmd_nvrm_sync(struct vk_sync *sync)
{
   assert(vk_sync_type_is_nvkmd_nvrm_sync(sync->type));
   return container_of(sync, struct nvkmd_nvrm_sync, base);
}

static void
set_value_locked(struct nvkmd_nvrm_sync *sync, uint64_t value)
{
   if (!(sync->base.flags & VK_SYNC_IS_TIMELINE)) {
      p_atomic_set(&sync->value, 1);
      return;
   }
   /* A timeline only moves forward. */
   if (value > p_atomic_read(&sync->value))
      p_atomic_set(&sync->value, value);
}

static void
clear_pending_locked(struct nvkmd_nvrm_sync *sync)
{
   if (sync->pendingSem != NULL) {
      nvkmd_mem_unref(sync->pendingSem);
      sync->pendingSem = NULL;
   }
}

/* Applies the pending signal if its submission has run. */
static void
retire_locked(struct nvkmd_nvrm_sync *sync)
{
   if (sync->pendingSem == NULL)
      return;
   uint64_t current = nvkmd_nvrm_sem_read(sync->pendingSem, sync->pendingWide);
   if (!nvkmd_nvrm_seq_reached(current, sync->pendingSeq, sync->pendingWide))
      return;
   set_value_locked(sync, sync->pendingValue);
   clear_pending_locked(sync);
}

static bool
reached_locked(struct nvkmd_nvrm_sync *sync, uint64_t wait_value)
{
   retire_locked(sync);
   uint64_t value = p_atomic_read(&sync->value);
   if (sync->base.flags & VK_SYNC_IS_TIMELINE)
      return value >= wait_value;
   return value != 0;
}

void
nvkmd_nvrm_sync_set_signaled(struct vk_sync *_sync, uint64_t value)
{
   struct nvkmd_nvrm_sync *sync = to_nvkmd_nvrm_sync(_sync);

   simple_mtx_lock(&sync->lock);
   set_value_locked(sync, value);
   /* A pending signal of no more than this has nothing left to do. */
   if (sync->pendingSem != NULL &&
       (!(sync->base.flags & VK_SYNC_IS_TIMELINE) || sync->pendingValue <= value))
      clear_pending_locked(sync);
   simple_mtx_unlock(&sync->lock);
}

/* Waits until the semaphore of the pending signal reaches its sequence
 * number; called with the lock held, returns with it held. */
static VkResult
wait_pending_locked(struct nvkmd_nvrm_sync *sync, uint64_t abs_timeout_ns)
{
   const int64_t start = os_time_get_nano();
   while (sync->pendingSem != NULL) {
      retire_locked(sync);
      if (sync->pendingSem == NULL)
         break;
      if (os_time_get_nano() >= abs_timeout_ns)
         return VK_TIMEOUT;
      simple_mtx_unlock(&sync->lock);
      poll_step(start);
      simple_mtx_lock(&sync->lock);
   }
   return VK_SUCCESS;
}

void
nvkmd_nvrm_sync_set_pending(struct vk_sync *_sync, struct nvkmd_mem *sem,
                            bool wide, uint64_t seq, uint64_t value)
{
   struct nvkmd_nvrm_sync *sync = to_nvkmd_nvrm_sync(_sync);

   simple_mtx_lock(&sync->lock);
   retire_locked(sync);
   /* One pending signal is kept. A later one on the same channel runs after
    * the earlier one and replaces it; one on another channel could run
    * first, so the earlier one is waited for (rare). */
   if (sync->pendingSem != NULL && sync->pendingSem != sem)
      wait_pending_locked(sync, UINT64_MAX);
   if (sync->pendingSem == NULL) {
      nvkmd_mem_ref(sem);
      sync->pendingSem = sem;
      sync->pendingWide = wide;
      sync->pendingValue = value;
   } else if (value > sync->pendingValue) {
      sync->pendingValue = value;
   }
   sync->pendingSeq = seq;
   simple_mtx_unlock(&sync->lock);
}

bool
nvkmd_nvrm_sync_ordered_before(struct vk_sync *_sync, uint64_t value,
                               struct nvkmd_mem *sem)
{
   struct nvkmd_nvrm_sync *sync = to_nvkmd_nvrm_sync(_sync);

   simple_mtx_lock(&sync->lock);
   bool ordered = reached_locked(sync, value);
   if (!ordered && sync->pendingSem == sem) {
      ordered = !(sync->base.flags & VK_SYNC_IS_TIMELINE) ||
                sync->pendingValue >= value;
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
   sync->pendingSem = NULL;
   return VK_SUCCESS;
}

void
nvkmd_nvrm_sync_finish(struct vk_device *device,
                       struct vk_sync *_sync)
{
   struct nvkmd_nvrm_sync *sync = to_nvkmd_nvrm_sync(_sync);

   clear_pending_locked(sync);
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
   clear_pending_locked(sync);
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
   if (!done && (flags & VK_SYNC_WAIT_PENDING) && sync->pendingSem != NULL)
      done = !(sync->base.flags & VK_SYNC_IS_TIMELINE) ||
             sync->pendingValue >= wait->wait_value;
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
   /* Polled: resman does not deliver channel events on these GPUs. Spin
    * briefly, as most waits end within microseconds, then sleep in short
    * steps. */
   const int64_t start = os_time_get_nano();
   for (;;) {
      uint32_t done = 0;
      for (uint32_t i = 0; i < wait_count; i++) {
         if (wait_satisfied(&waits[i], wait_flags))
            done++;
      }
      if (done == wait_count || (done > 0 && (wait_flags & VK_SYNC_WAIT_ANY)))
         return VK_SUCCESS;
      if (os_time_get_nano() >= abs_timeout_ns)
         return VK_TIMEOUT;
      poll_step(start);
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
   clear_pending_locked(dst);
   p_atomic_set(&dst->value, p_atomic_read(&src->value));
   dst->pendingSem = src->pendingSem;
   dst->pendingWide = src->pendingWide;
   dst->pendingSeq = src->pendingSeq;
   dst->pendingValue = src->pendingValue;
   src->pendingSem = NULL;
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
