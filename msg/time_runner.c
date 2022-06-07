//
// Created by server on 2022-05-20.
//
#include "utility/log.h"
#include "time_runner.h"
#include "include/header.h"


#define MAX_TIME_RUNNERS 10

pthread_mutex_t runner_pool_lock = PTHREAD_MUTEX_INITIALIZER;
static struct Time_Runner *runner_pool[MAX_TIME_RUNNERS] = {0};

//find an empty time runner
int time_runner_find_empty() {
    for (int i = 0; i < MAX_TIME_RUNNERS; i++) {
        if (runner_pool[i] == NULL || TIME_RUNNER_EMPTY == runner_pool[i]->run_status) {
            return i;
        }
    }
    return -1;
}

//arg_mem > 0 : malloc ,malloc == 0 : no malloc,use arg pointer
int time_runner_register(int usec, time_runner_callback_t callback,void *arg, size_t arg_mem) {
    pthread_mutex_lock(&runner_pool_lock);//对runner_pool_lock加锁
    int runner_fd = time_runner_find_empty();
    if (runner_fd < 0) {
        logger_warn("no empty time runner\n");
        pthread_mutex_unlock(&runner_pool_lock);
        return -1;

    }
    if (runner_pool[runner_fd] == NULL) {
        runner_pool[runner_fd] = malloc(sizeof(struct Time_Runner));
        memset(runner_pool[runner_fd], 0, sizeof(struct Time_Runner));
    }
    runner_pool[runner_fd]->run_status = TIME_RUNNER_INIT;
    pthread_mutex_unlock(&runner_pool_lock);//先设置TIME_RUNNER_INIT再解锁
    runner_pool[runner_fd]->fd = runner_fd;
    runner_pool[runner_fd]->usec = usec;

    if (0 == arg_mem) {
        runner_pool[runner_fd]->arg = arg;
    } else {
        runner_pool[runner_fd]->arg = malloc(arg_mem);
        memcpy(runner_pool[runner_fd]->arg, arg, arg_mem);
    }
    runner_pool[runner_fd]->arg_mem = arg_mem;
    runner_pool[runner_fd]->callback = callback;
    pthread_mutex_init(&runner_pool[runner_fd]->lock, NULL);
    pthread_cond_init(&runner_pool[runner_fd]->cond, NULL);
    return runner_fd;
}

void set_time_runner_status(int runner_fd, enum TIME_RUNNER_STATUS status) {
    pthread_mutex_lock(&runner_pool_lock);
    runner_pool[runner_fd]->run_status = status;
    pthread_mutex_unlock(&runner_pool_lock);
}


void time_runner_destroy_all() {
    for (int runner_fd = 0; runner_fd < MAX_TIME_RUNNERS; runner_fd++) {
        if (runner_pool[runner_fd] != NULL) {
            pthread_mutex_destroy(&runner_pool[runner_fd]->lock);
            pthread_cond_destroy(&runner_pool[runner_fd]->cond);
            if(runner_pool[runner_fd]->arg_mem > 0) {
                free(runner_pool[runner_fd]->arg);
            }
            free(runner_pool[runner_fd]);
            runner_pool[runner_fd] = NULL;
        }
    }
}

int time_runner_destroy(int runner_fd) {
    if (runner_fd < 0 || runner_fd >= MAX_TIME_RUNNERS) {
        logger_warn("runner_fd is invalid\n");
        return -1;
    }
    else if (runner_pool[runner_fd] == NULL || TIME_RUNNER_BUSY == runner_pool[runner_fd]->run_status) {
        logger_warn("time runner %d is null or busy\n", runner_fd);
        return -1;
    }
    pthread_mutex_destroy(&runner_pool[runner_fd]->lock);
    pthread_cond_destroy(&runner_pool[runner_fd]->cond);
    if(runner_pool[runner_fd]->arg_mem > 0) {
        free(runner_pool[runner_fd]->arg);
    }
    free(runner_pool[runner_fd]);
    runner_pool[runner_fd] = NULL;
    return 0;
}

int time_runner_reset(int runner_fd){
    if(runner_fd < 0 || runner_fd >= MAX_TIME_RUNNERS){
        logger_warn("runner_fd is invalid\n");
        return -1;
    }
    else if(runner_pool[runner_fd] == NULL || TIME_RUNNER_BUSY != runner_pool[runner_fd]->run_status){
        logger_warn("time runner %d is null or not running\n", runner_fd);
        return -1;
    }
    pthread_mutex_lock(&runner_pool[runner_fd]->lock);
    pthread_cond_signal(&runner_pool[runner_fd]->cond);
    pthread_mutex_unlock(&runner_pool[runner_fd]->lock);
    return 0;
}

void *runner_thread(void *arg) {
    struct Time_Runner *runner = (struct Time_Runner *) arg;
    while (runner->run_status == TIME_RUNNER_BUSY) {
        struct timeval tm;
        gettimeofday(&tm, NULL);
        struct timespec ts;
        ts.tv_sec = tm.tv_sec + runner->usec / 1000000;
        ts.tv_nsec = tm.tv_usec * 1000 + (runner->usec % 1000000) * 1000;
        pthread_mutex_lock(&runner->lock);
        if (pthread_cond_timedwait(&runner->cond,
                                   &runner->lock, &ts) == ETIMEDOUT) {
//            logger_warn("wait time runner fd:%d timeout\n", runner->fd);
            runner->callback(runner->arg);
        }
        pthread_mutex_unlock(&runner->lock);
    }
}

int time_runner_start(int runner_fd) {
    if(runner_fd < 0 || runner_fd >= MAX_TIME_RUNNERS){
        logger_warn("runner_fd is invalid\n");
        return -1;
    }
    else if (runner_pool[runner_fd] == NULL || runner_pool[runner_fd]->run_status != TIME_RUNNER_INIT) {
        logger_warn("time runner %d is null or not init\n", runner_fd);
        return -1;
    }
    struct Time_Runner *runner = runner_pool[runner_fd];
    set_time_runner_status(runner_fd, TIME_RUNNER_BUSY);
    pthread_t  thread_id;
    if(pthread_create(&thread_id, NULL, runner_thread, runner) != 0){
        logger_warn("create time runner thread failed\n");
        return -1;
    }
    pthread_detach(thread_id);
    return 0;
}

int time_runner_stop(int runner_fd) {
    if (runner_fd < 0 || runner_fd >= MAX_TIME_RUNNERS ||
        runner_pool[runner_fd] == NULL ||
        runner_pool[runner_fd]->run_status == TIME_RUNNER_EMPTY) {
        logger_warn("time runner %d is null or empty\n", runner_fd);
        return -1;
    }
    set_time_runner_status(runner_fd, TIME_RUNNER_EMPTY);
    return 0;
}






