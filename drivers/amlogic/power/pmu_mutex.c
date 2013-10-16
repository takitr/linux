
#include <linux/mutex.h>
#include <linux/lockdep.h>
#include <linux/slab.h>
#include <linux/printk.h>
#include <linux/module.h>
#include <linux/gfp.h>

/*
 * some data-struct is depend on kernel's menuconfig
 * so make these calls in a single file with open source.
 * This can help compatibility of PMU driver liberary.
 */
static int mutex_cnt = 0;
const  char mutex_name[20] = {};

void *pmu_alloc_mutex(void)
{
    struct mutex *pmutex = NULL;
    struct lock_class_key *key = NULL;

    pmutex = kzalloc(sizeof(struct mutex), GFP_KERNEL);
    if (!pmutex) {
        printk("%s, alloc mutex failed\n", __func__);
        return NULL;
    }
    key = kzalloc(sizeof(struct lock_class_key), GFP_KERNEL);
    if (!key) {
        printk("%s, alloc key failed\n", __func__);
        return NULL;
    }
    sprintf((char *)mutex_name, "pmu_mutex%d", mutex_cnt++); 
    __mutex_init(pmutex, mutex_name, key);
    return (void *)pmutex;
}
EXPORT_SYMBOL_GPL(pmu_alloc_mutex);

void pmu_mutex_lock(void *mutex)
{
    mutex_lock((struct mutex *)mutex);    
}
EXPORT_SYMBOL_GPL(pmu_mutex_lock);

void pmu_mutex_unlock(void *mutex)
{
    mutex_unlock((struct mutex *)mutex);    
}
EXPORT_SYMBOL_GPL(pmu_mutex_unlock);

