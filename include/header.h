//
// Created by server on 2022/5/4.
//

#ifndef RTK_NAV_HEADER_H
#define RTK_NAV_HEADER_H
#include <memory.h>
#include <stdint.h>
#include <stdio.h>
#include <pthread.h>
#include <netdb.h>
#include <math.h>
#include <stdlib.h>
#include <errno.h>
#include <time.h>
#include <unistd.h>
#include <string.h>
#include <termios.h>
#include <sys/time.h>
#include <sys/socket.h>
#include <sys/select.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/fcntl.h>
#include <sys/signal.h>
#include <assert.h>
#include <limits.h>
#include "utility/mem.h"

//GPS接收机串口
#define SERIAL_DEV0 "/dev/ttyUSB0"
#define BAUDRATE 460800
//与上位机通信
#define UDP_LOCAL_IP "192.168.10.12"
#define UDP_LOCAL_PORT 6020
#define UDP_RESPOND_PORT 6030
//与底盘通信
#define CTL_REMOTE_IP "192.168.10.10"
#define CTL_REMOTE_PORT 8025
//与显示通信
#define VIEW_REMOTE_IP "192.168.10.12"
#define VIEW_REMOTE_PORT 1010
#define MESSAGE_LOGGER_FILE "log_message.txt"
#define RTCM_LOGGER_FILE "log_rtcm.txt"
#define NEMA_LOGGER_FILE "log_nema.txt"
#define MEMORY_LOGGER_FILE "log_memory.txt"
#define SWAP_BYTE 1


#endif //RTK_NAV_HEADER_H
