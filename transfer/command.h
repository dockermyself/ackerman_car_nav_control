//
// Created by server on 2022/4/22.
//

#ifndef RTK_NAV_CMD_H
#define RTK_NAV_CMD_H

#include "include/header.h"
#include "msg/nema.h"
//上位机下发的消息[FF AA ID type data_len CRC]
#define CMD_MSG_MIN_LEN 7
enum WALKER_ID{

    CMD_INIT = 0x21,//启动0x01, 初始化, 进入工作状态;  停止工作0x00
    CMD_QUERY_RTKSTATUS = 0x022,//工作状态0x00没有准备，0x01准备就绪
    CMD_LOGIN_DATA = 0x23,//IP4字节, 端口(2字节uint16), user用户名(20字节字符),密码(20字节字符);
    CMD_QUERY_WAYPOINT = 0x24,//坐标经度(8)  坐标纬度(8)  坐标高度(8)  方向(4)  速度(4) 坐标x,y,z为double 方向角正东为0, 正北为π/2构成的坐标系。
    CMD_SPEED = 0x25, //Walker(动力电机)保持以某速度移动
    CMD_WALK_AND_TURN = 0x26, //通知Walker车轮以某弧度拐弯
    CMD_WALKER_STOP = 0x27, //通知Walker刹车电机刹车停车
    CMD_RELEASE_BRAKE = 0x28, //通知Walker刹车电机释放刹车
    CMD_KEEP_SPEED_WALK_RAD = 0x29,//通知Walker车轮以某速度移动一段弧度
    CMD_TRACE_TO_TARGET = 0x2a,//Walker前往某目的位置
    CMD_INTERRUPT_TRACE = 0x2b,//中断寻迹
};
#pragma pack(1)

struct CMD_MSG{
    uint8_t head[2];
    uint16_t ID;//消息ID
    uint8_t type;
    uint16_t data_len;
    char data[];
};

struct MESSAGE_NODE{
    char* msg;
    int size;
    struct sockaddr_in remote_addr;
    struct MESSAGE_NODE* next;

};

#pragma pack()
int message_queue_init();
struct MESSAGE_NODE *message_new(char *msg, int size, struct sockaddr_in *addr);
void del_message(struct MESSAGE_NODE *node);
void push_message(struct MESSAGE_NODE *node);
struct MESSAGE_NODE *pop_message();


int cmd_check_crc(const unsigned char *buf, int len);
int cmd_check_len(const uint8_t *buf, int len);
int cmd_gen_crc(const unsigned char *buf, int len);
int cmd_check_head(const unsigned char *buf, int len);

int cmd_create_socket(const char *ip, int port);
int cmd_send_data(const unsigned char *buf, int len,struct sockaddr_in* to_addr);
int cmd_recv_data(unsigned char *buf, int len, struct sockaddr_in *from_addr);
void cmd_init_remote(struct sockaddr_in *to_addr,uint16_t remote_port, const char* remote_ip);
int cmd_send_ctl_status(struct CMD_MSG *walk_msg,struct sockaddr_in *to_addr,char status);
int cmd_send_status_respond(struct CMD_MSG *msg, struct sockaddr_in *to_addr, char status);
int cmd_send_rtkdata(struct CMD_MSG *msg,struct sockaddr_in *to_addr, struct LLA *lla);
int cmd_send_rtkdata_with_status(struct CMD_MSG *msg, struct sockaddr_in *to_addr, struct LLA *lla,char status);



uint16_t swap_uint16(uint16_t x);
uint32_t swap_uint32(uint32_t x);
#endif //RTK_NAV_CMD_H
