//
// Created by server on 2022/4/26.
//

#ifndef RTK_NAV_CONTROL_H
#define RTK_NAV_CONTROL_H

#include "include/header.h"
//车轮半径单位cm
#define CAR_WHEEL_RADIUS    2.7f
#define CAR_BODY_X  16f
#define CAR_BODY_Y  23.2f
//车轮间距
#define CAR_WHEEL_Y 14.8f
#define CAR_WHEEL_X 15.0f

//数据周期200ms
#define CAR_MSG_PERIOD  200
//命令下发周期
#define CAR_CMD_PERIOD  50
//直线行走
#define CAR_TURN_ANGLE_MAX 22.5f
#define CAR_MANUAL_TURN_MAX 18

#define CAR_WALK_REF_SPEED 0.2f
#define CAR_SLOW_WALK_REF_SPEED 0.1f
#define CAR_TURN_WALK_REF_SPEED 0.1f
#define CAR_STOP_SPEED 0.0f
#define CAR_TURN_MOVE_MAX_LINE 0.5f
#define CAR_MOVE_ERROR_ALLOW 0.01f

enum CTL_STATUS {
    CTL_STOP = 0x00,//停车
    CTL_PARK = 0x01,//驻车
    CTL_SPEED_RUN = 0x02, //速度运行
    CTL_DISPLACE_RUN = 0x03, //位移运行
    CTL_UNPARK = 0x04,//解除驻车
    CTL_SLOW = 0x05,  //减速运行


};


#pragma pack(1)

//按照byte对齐，每次都从最低开始，如果表示的字节大于1个，根据大小端进行转换，如果是小端，高位在前，低位在后
typedef union {
    struct {
        uint16_t degree: 10;//bit 1-10 ,    byte & 0x3FF
        uint16_t minute: 6; //bit 11-16 ,   byte & 0xFC00
    } bit;
    uint16_t byte;
} ctl_angle_t;

struct CONTROL_CMD {
    char status;//CTL_STATUS
    ctl_angle_t wheel;
    ctl_angle_t turn;
};


struct POINT3D {
    double x;
    double y;
    double z;
};

struct POINT2D {
    double x;
    double y;
};
struct POINT2I {
    int x;
    int y;
};

//diff_head > 0,当前车体需要顺时针旋转diff_head角度与目标航向角一致
//diff_head < 0,当前车体需要逆时针旋转diff_head角度与目标航向角一致
struct LOCATION {
    struct POINT3D point3d;     //ENU坐标系
    float relative_head;        //目标航向角与当前车体航向角的差值,范围-180~180
    float absolute_head;        //0-360
    float speed;                //速度m/s
};
struct PACKAGE_HEAD {
    uint8_t code[2];
    uint16_t ID;
    uint16_t data_len;
};


typedef uint16_t package_ex_t;
struct CONTROL_PACKAGE {
    struct PACKAGE_HEAD head;
    struct CONTROL_CMD cmd;
    uint8_t speed;
    uint8_t times;
    uint8_t crc_code;
    package_ex_t package_size;
};

#pragma pack()
#define CTL_PACKAGE_EX_LEN (sizeof(package_ex_t))
#define CTL_PACKAGE_DATA_LEN  (sizeof(struct CONTROL_PACKAGE) - sizeof(package_ex_t) - sizeof(struct PACKAGE_HEAD))

float ctl_point_distance(const struct POINT3D *P);

uint16_t ctl_interrupt_status(void);

float ctl_point_direction(const struct POINT3D *point3d);

void ctl_coordinate_convert(struct POINT3D* point, double alf,double distance) ;

int ctl_cmd_check(const uint8_t *buf, int len);

struct CONTROL_PACKAGE *ctl_auto_cmd_init(const struct LOCATION *location, float current_head, char status);

struct CONTROL_PACKAGE *ctl_manual_status_cmd_init(char status);

struct CONTROL_PACKAGE *ctl_manual_speed_cmd_init(float f_speed, float radian);

struct CONTROL_PACKAGE *ctl_displace_cmd_init(float f_speed, float radian);

void ctl_change_cmd_status(struct CONTROL_PACKAGE *package, char status);

void ctl_package_destroy(struct CONTROL_PACKAGE **package);

int ctl_target_forward(const struct POINT3D *point3d, float target_head);

#endif //RTK_NAV_CONTROL_H
