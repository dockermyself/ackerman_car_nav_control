//
// Created by server on 2022/4/28.
//

#include "msg_cors.h"
#include "utility/log.h"
static pthread_mutex_t lock = PTHREAD_MUTEX_INITIALIZER;
struct CORS_MSG cors_msg = {0};

void msg_cors_init(){
    memset(&cors_msg,0,sizeof(struct CORS_MSG));
    cors_msg.buffer = malloc(256);
    memset(cors_msg.buffer,0,256);
    cors_msg.buffer_size = 256;
    cors_msg.data_size = 0;
    cors_msg.status = CORS_EMPTY;
    pthread_mutex_init(&lock, NULL);
}

void msg_cors_destroy(){
    free(cors_msg.buffer);
    pthread_mutex_destroy(&lock);
}

void msg_cors_update(const char *msg,int size){

    if(cors_msg.buffer_size < size){
        return;
    }
    pthread_mutex_lock(&lock);
    memset(cors_msg.buffer,0,cors_msg.data_size);
    memcpy(cors_msg.buffer,msg,size);
    cors_msg.data_size = size;
    cors_msg.status = CORS_FULL;
    pthread_mutex_unlock(&lock);
    //logger_info("update cors msg:%s",cors_msg.buffer);
    logger_write_message(MESSAGE_LOGGER_FILE,"update cors msg");
    logger_write_message(MESSAGE_LOGGER_FILE,msg);

}
/*
 * pthread_cond_wait会先解除之前的pthread_mutex_lock锁定的mtx，
 * 然后阻塞在等待队列里休眠，直到再次被唤醒（大多数情况下是等待的条件成立而被唤醒，
 * 唤醒后，该线程会先锁定先pthread_mutex_lock(&mtx); 再读取资源
 */
int msg_cors_get(char *msg){
    if(NULL == msg || NULL == cors_msg.buffer || CORS_EMPTY == cors_msg.status){
        return 0;
    }
    pthread_mutex_lock(&lock);
    memcpy(msg,cors_msg.buffer,cors_msg.data_size);
    pthread_mutex_unlock(&lock);
    return 1;
}
