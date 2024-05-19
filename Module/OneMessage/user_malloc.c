#include "om_config.h"
/* 用户内存分配函数 */
#include "tx_api.h"
extern TX_BYTE_POOL MsgPool;
void* user_malloc(size_t size){
    void* address = 0;
    tx_byte_allocate(&MsgPool,&address,size,TX_NO_WAIT);
    return address;
}