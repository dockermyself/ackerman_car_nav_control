//
// Created by server on 2022-05-20.
//

#ifndef RTK_NAV_TIME_RUNNER_H
#define RTK_NAV_TIME_RUNNER_H

#include "include/header.h"

enum TIME_RUNNER_STATUS {
    TIME_RUNNER_EMPTY = 0,
    TIME_RUNNER_INIT = 1,
    TIME_RUNNER_BUSY = 2
};

typedef void (*time_runner_callback_t)(void *arg);

struct Time_Runner {
    int fd;
    pthread_mutex_t lock;
    pthread_cond_t cond;
    int usec;
    void *arg;
    size_t arg_mem;
    time_runner_callback_t callback;
    enum TIME_RUNNER_STATUS run_status;
};

int time_runner_register(int usec, time_runner_callback_t callback,void *arg, size_t arg_mem);
int time_runner_start(int runner_fd);
int time_runner_reset(int runner_fd);
void time_runner_destroy_all();
int time_runner_destroy(int runner_fd);
int time_runner_stop(int runner_fd);
#endif //RTK_NAV_TIME_RUNNER_H
