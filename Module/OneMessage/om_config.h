#include "tx_api.h"
/* Debug */
#define OM_DEBUG (1)

/* 使用用户自定义的内存分配 */
#define OM_USE_USER_MALLOC (1)

/* 用户内存分配函数 */
void* user_malloc(size_t size);

#if OM_USE_USER_MALLOC
#define om_malloc user_malloc
#define om_free tx_byte_release
#endif

/* 非阻塞延时函数 */
#define om_delay_ms(arg) tx_thread_sleep(arg);

/* OS层互斥锁api */
#define om_mutex_t TX_MUTEX
#define om_mutex_init(arg,name) tx_mutex_create(arg,name,TX_NO_INHERIT);
#define om_mutex_lock(arg) tx_mutex_get(arg,TX_WAIT_FOREVER)
#define om_mutex_trylock(arg) tx_mutex_get(arg,TX_NO_WAIT) == TX_SUCCESS ? OM_OK:OM_ERROR
#define om_mutex_unlock(arg) tx_mutex_put(arg)
#define om_mutex_lock_isr(arg) tx_mutex_get(arg,TX_NO_WAIT) == TX_SUCCESS ? OM_OK:OM_ERROR
#define om_mutex_unlock_isr(arg) tx_mutex_put(arg)
#define om_mutex_delete(arg) tx_mutex_delete(arg)

/* 将运行时间作为消息发出的时间 */
#define OM_TIME (1)

#if OM_TIME
#define om_time_t ULONG
#define om_time_get(_time) *_time=tx_time_get()
#endif

/* 开启"om_log"话题作为OneMessage的日志输出 */
#define OM_LOG_OUTPUT (1)

#if OM_LOG_OUTPUT
/* 按照日志等级以不同颜色输出 */
#define OM_LOG_COLORFUL (1)
/* 日志最大长度 */
#define OM_LOG_MAX_LEN (60)
/* 日志等级 1:default 2:notice 3:pass 4:warning 5:error  */
#define OM_LOG_LEVEL (1)
#endif

/* 话题名称最大长度 */
#define OM_TOPIC_MAX_NAME_LEN (15)

/* for OneMessageCPP:https://github.com/Jiu-xiao/OneMessageCPP */

/*
#include <stdbool.h>
static inline bool om_in_isr() { return false; }
*/
