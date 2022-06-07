//
// Created by server on 2022/4/19.
//

#ifndef RTK_NAV_TASK_H
#define RTK_NAV_TASK_H

#include "include/header.h"
#include "msg/msg_rtk.h"
#include "msg/msg_cors.h"
#include "transfer/command.h"
#include "transfer/corsclient.h"
#include "transfer/serial.h"
#include "control/control.h"
#include "msg/time_runner.h"
#include "utility/log.h"
#include "msg/time_runner.h"
struct TASK_EVENT_BUFFER {
    char *buffer;
    int buffer_size;
    int data_len;
};
enum TASK_STATUS{
    TASK_STATUS_STOP = 0,          //停止处理非启动命令   -> TASK_STATUS_INIT     <- 任何状态可以切换到TASK_STATUS_STOP
    TASK_STATUS_INIT = 1,          //初始化状态          -> TASK_STATUS_IDLE     <- TASK_STATUS_STOP
    TASK_STATUS_TRACE = 3,         //寻迹               -> TASK_STATUS_IDLE
    TASK_STATUS_IDLE = 4,          //已经登录且空闲       -> TASK_STATUS_STOP , TASK_STATUS_TRACE
    TASK_STATUS_RTK_ERROR = 7,     //  RTK异常 <- 任何状态可以切换到TASK_STATUS_RTK_ERROR
    TASK_STATUS_EXIT = 8           //   退出程序,任何状态都可以切换到TASK_STATUS_EXIT
};

struct TASK_PARAM{
    struct sockaddr_in to_addr;
    struct CMD_MSG msg;
};

struct TASK_USER_LOGIN{
    char ip[20];
    short port;
    char user[20];
    char password[20];
    char mount_point[20];
};


void task_set_status(enum TASK_STATUS status);
enum TASK_STATUS task_get_status();
void task_init_buffer(struct TASK_EVENT_BUFFER *event, int buffer_size);
void task_set_buffer(struct TASK_EVENT_BUFFER *event,const char* buffer,int data_len);
void task_release_buffer(struct TASK_EVENT_BUFFER *event);


struct TASK_PARAM *task_init_param(struct CMD_MSG *msg, struct sockaddr_in* to_addr);//check len
void task_destroy_param(struct TASK_PARAM** param);
void task_rtk_to_location(struct RTK_MSG* rtk_msg,struct LLA* target_lla,struct LOCATION* location);
void task_rtk_to_point3d(struct RTK_MSG *rtk_msg, struct LLA *target_lla, struct POINT3D *point3d);
void task_rtk_to_point2d(struct RTK_MSG *rtk_msg, struct LLA *target_lla, struct POINT2D *point2d);
void TASK_EXIT();
void TASK_MAIN();
void TASK_CORS_COMMUNICATE();
void TASK_RECV_FROM_UDP();
void *TASK_LOGIN_CORS(void *args);
void *TASK_RECV_FROM_SERIAL(void *args);

void *TASK_TRACKING(void *args);
#endif //RTK_NAV_TASK_H
