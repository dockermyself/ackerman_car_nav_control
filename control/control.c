//
// Created by server on 2022/4/26.
//

#include "control.h"
#include "transfer/command.h"
#include "utility/log.h"
#include "include/header.h"
#include "utility/cmath.h"

#define CAR_TURN_RIGHT 0
#define CAR_TURN_LEFT 1
#define CAR_WALK_FORWARD 1
#define CAR_WALK_BACKWARD 0

static uint16_t CTL_PACKAGE_ID = 0;

float ctl_point_distance(const struct POINT3D *P) {
    assert(NULL != P);
    return (float) sqrt(P->x * P->x + P->y * P->y);
}


float ctl_point_norm(const struct POINT3D *point) {
    assert(NULL != point);
    return (float) sqrt(point->x * point->x + point->y * point->y);
}

//HDT航向角：北向为0，顺时针递增,定义（0,360）
//LOCATION 是相对于ENU坐标系，计算AB向量与北向夹角
float ctl_point_direction(const struct POINT3D *point3d) {//[0,pi]
    assert(NULL != point3d);

    if (float_compare((float) point3d->x, 0) == 0 &&
        float_compare((float) point3d->y, 0) >= 0) {
        return 0;
    } else if (float_compare((float) point3d->x, 0) == 0) {
        return M_PI;
    }

    //计算AB与北向夹角(0,1) ENU坐标系
    float alf = (float) acos(point3d->y / ctl_point_norm(point3d));
    //顺时针旋转为正(0,360)
    if (float_compare((float) point3d->x, 0) < 0) {
        alf = (float) (2.0f * M_PI - alf);
    }

    return alf;
}

//目标航向角与计算航向角要求的角度差小于PI/2
int ctl_target_forward(const struct POINT3D *point3d, float target_direction) {
    assert(NULL != point3d);
    const
    float distance = ctl_point_distance(point3d);
    float move_direction = (float) (ctl_point_direction(point3d) * 180 / M_PI);//计算得到的航向角:弧度0-2pi
    /*
     * 如果当前点到目标点向量角度与建图是途径目标点的航向角相差大于90度，则判定已经该目标点
     * 允许当前航向角与目标航向角大于90
     */
    if (target_direction - move_direction > 90 || target_direction - move_direction < -90 ||
        float_compare(distance, CAR_MOVE_ERROR_ALLOW) <= 0) {
        return 0;
    }
    return 1;
}


void ctl_set_direction(ctl_angle_t *angle, char direction) {//direction:1 最高位设置为1
    assert(NULL != angle);
    if (direction) {
        angle->bit.degree |= 0x200;
    } else {
        angle->bit.degree &= 0x1FF;
    }
}

void ctl_package_cmd_fill(struct CONTROL_CMD *cmd, float move_angle, float turn_angle,
                          char status, char turn_direction, char forward_direction) {
    assert(NULL != cmd);

    cmd->turn.bit.degree = floor((double) turn_angle);
    cmd->turn.bit.minute = floor(((double) turn_angle - cmd->turn.bit.degree) * 60);
    //ctl_set_direction
    ctl_set_direction(&cmd->turn, turn_direction);

    cmd->wheel.bit.degree = floor((double) move_angle);
    cmd->wheel.bit.minute = floor(((double) move_angle - cmd->wheel.bit.degree) * 60);
    ctl_set_direction(&cmd->wheel, forward_direction);
    cmd->status = status;
}


//通过 2PI周期调整角度，输出为[-PI,PI]
void ctl_angle_pipi(float *radian) {
    assert(radian != NULL);
    if (float_compare(*radian, M_PI) > 0) {
        *radian = (float) (*radian - 2 * M_PI);
    } else if (float_compare(*radian, -(float) M_PI) < 0) {
        *radian = (float) (*radian + 2 * M_PI);
    } else if (float_compare(*radian, M_PI) == 0) {
        *radian = (float) (-M_PI);
    } else if (float_compare(*radian, -(float) M_PI) == 0) {
        *radian = (float) (M_PI);
    }
}

void ctl_angle_threshold(float *radian, float min_threshold, float max_threshold) {
    assert(NULL != radian);
    if (*radian > max_threshold) {
        *radian = max_threshold;
    } else if (*radian < min_threshold) {
        *radian = min_threshold;
    }
}

void ctl_package_head_init(struct PACKAGE_HEAD *package, uint16_t ID, uint16_t len) {
    package->code[0] = 0xEE;
    package->code[1] = 0xAA;
    package->ID = swap_uint16(ID);
    package->data_len = swap_uint16(len);
}

struct CONTROL_PACKAGE *ctl_package_init() {
    struct CONTROL_PACKAGE *package = malloc(sizeof(struct CONTROL_PACKAGE));
    assert(NULL != package);
    memset(package, 0, sizeof(struct CONTROL_PACKAGE));
    package->package_size = sizeof(struct CONTROL_PACKAGE) - CTL_PACKAGE_EX_LEN;
    return package;
}

void ctl_package_destroy(struct CONTROL_PACKAGE **package) {
    if (NULL == package || NULL == *package) {
        return;
    }
    free(*package);
    *package = NULL;
}

int ctl_check_len(const uint8_t *buf, int len) {
    struct PACKAGE_HEAD *msg = (struct PACKAGE_HEAD *) buf;
    if (swap_uint16(msg->data_len) + sizeof(struct PACKAGE_HEAD) + 1 != len) {
        return 0;
    }
    return 1;
}


int ctl_check_head(const uint8_t *buf, int len) {//OXFF 0XAA
    if (NULL == buf || len < sizeof(struct PACKAGE_HEAD)) {
        return 0;
    }
    if (buf[0] != 0xEE || buf[1] != 0xAB) {
        return 0;
    }

    return 1;
}

int ctl_check_crc(const uint8_t *buf, int len) {
    if (NULL == buf || len < CMD_MSG_MIN_LEN) {
        return 0;
    }
    uint8_t code = 0;
    for (int i = 0; i < len - 1; ++i) {
        code += buf[i];
    }
    if (code != buf[len - 1]) {
        return 0;
    }
    return 1;
}


int ctl_cmd_check(const uint8_t *buf, int len) {
    if (ctl_check_len(buf, len) && ctl_check_head(buf, len) && ctl_check_crc(buf, len)) {
        return 1;
    }
    return 0;
}

//和校验,len包含crc位置
void ctl_gen_crc(uint8_t *buf, int len) {
    assert(NULL != buf);
    uint8_t code = 0;
    for (int i = 0; i < len - 1; ++i) {
        code += buf[i];
    }
    buf[len - 1] = code;


}

void ctl_change_cmd_status(struct CONTROL_PACKAGE *package, char status) {
    assert(NULL != package);
    package->cmd.status = status;
    ctl_gen_crc((uint8_t *) package, package->package_size);
}

int ctl_stop_status(char status) {
    if (status == CTL_PARK || status == CTL_STOP) {
        return 1;
    }
    return 0;
}

int ctl_run_status(char status) {
    if (status == CTL_SPEED_RUN || status == CTL_DISPLACE_RUN) {
        return 1;
    }
    return 0;
}

void ctl_forward_point_insert(struct LOCATION *forward_location, const struct LOCATION *target_location,
                              float forward_distance) {
    assert(NULL != forward_location && NULL != target_location);


    float alf = (float) ((90 - target_location->absolute_head) / 180 * M_PI);
    forward_location->point3d.x = target_location->point3d.x + forward_distance * cosf(alf);
    forward_location->point3d.y = target_location->point3d.y + forward_distance * sinf(alf);
    forward_location->point3d.z = target_location->point3d.z;
    forward_location->absolute_head = target_location->absolute_head;
    forward_location->relative_head = target_location->relative_head;
    forward_location->speed = target_location->speed;
    logger_info("TARGET POINT:(x:%f cm,y:%f cm)\n", target_location->point3d.x * 100,
                target_location->point3d.y * 100);
    logger_info("FORWARD POINT:(x:%f cm,y:%f cm)\n", forward_location->point3d.x * 100,
                forward_location->point3d.y * 100);
}


//根据status构造控制命令，并把命令封装在package中
struct CONTROL_PACKAGE *ctl_manual_status_cmd_init(char status) {
    struct CONTROL_PACKAGE *package = ctl_package_init();
    ctl_package_head_init(&package->head, CTL_PACKAGE_ID++, CTL_PACKAGE_DATA_LEN);
    if (CTL_STOP == status) {
        status = CTL_SPEED_RUN;
    }
    ctl_package_cmd_fill(&package->cmd, 0, 0, status, 0, 0);
    package->speed = 0;
    package->times = 1;
    ctl_gen_crc((uint8_t *) package, package->package_size);
    return package;

}


/*
 * f_speed:目标速度,单位:m/s
 * radian:目标转弯弧度,单位:rad
 * status:控制状态
 */
//根据目标speed，转角radian构造控制命令，并把命令封装在package中
struct CONTROL_PACKAGE *ctl_manual_speed_cmd_init(float f_speed, float radian) {
    char direction = CAR_WALK_FORWARD, turnRorL = CAR_TURN_RIGHT;
    if (radian < 0) {
        radian = -radian;
        turnRorL = CAR_TURN_LEFT;
    }
    float manual_turn_threshold = (float) (CAR_MANUAL_TURN_MAX / 180.0f * M_PI);
    if (radian > manual_turn_threshold) {
        radian = manual_turn_threshold;
    }

    if (f_speed < 0) {
        f_speed = -f_speed;
        direction = CAR_WALK_BACKWARD;
    }
    //设置速度上限CAR_STRAIGHT_WALK_REF_SPEED
    if (f_speed > CAR_WALK_REF_SPEED) {
        f_speed = CAR_WALK_REF_SPEED;
    }

    const int KEEP_TIMES = CAR_MSG_PERIOD / CAR_CMD_PERIOD;

    struct CONTROL_PACKAGE *package = ctl_package_init();
    ctl_package_head_init(&package->head, CTL_PACKAGE_ID++, CTL_PACKAGE_DATA_LEN);
    //一个小周期内的位移,单位:m
    package->times = (char) KEEP_TIMES;
    package->speed = (uint8_t) (f_speed * 100);
    ctl_package_cmd_fill(&package->cmd, 0, (float) (radian * 180 / M_PI),
                         CTL_SPEED_RUN, turnRorL, direction);
    ctl_gen_crc((uint8_t *) package, package->package_size);
    return package;
}

//根据目标speed，目标位移radian构造控制命令，并把命令封装在package中
struct CONTROL_PACKAGE *ctl_displace_cmd_init(float f_speed, float radian) {
    char direction = CAR_WALK_FORWARD;
    if (f_speed < 0) {
        f_speed = -f_speed;
        direction = CAR_WALK_BACKWARD;
    }
    if (f_speed > CAR_WALK_REF_SPEED) {
        f_speed = CAR_WALK_REF_SPEED;
    }

    struct CONTROL_PACKAGE *package = ctl_package_init();
    ctl_package_head_init(&package->head, CTL_PACKAGE_ID++, CTL_PACKAGE_DATA_LEN);
    package->speed = (uint8_t) (f_speed * 100);
    package->times = 1;
    ctl_package_cmd_fill(&package->cmd, (float) (radian * 180 / M_PI), 0,
                         CTL_DISPLACE_RUN, 0, direction);
    ctl_gen_crc((uint8_t *) package, package->package_size);
    return package;

}

/*阿克曼转向控制,双转向，前后轮角度相同
 * 车轮间距L
 * 轮子转角alf
 * 轮子位移S
 * 航向角变化 = 目标航向角 - 当前航向角
 * alf = asin(0.5L x d_head / S)
 * 例子：S > 0向前位移
 * 1.d_head > 表示 逆时针旋转达到目标航向角，
 * 2.轮子逆时针旋转
 * 假设s = 0.5m 车轮最大转角为30度，那么经过s后航向角最大变化1/3近似20度
 */
float ctl_ackerman_turn(float course_diff, float walk) {    //顺时针转为正，逆时针转为负,asin 输出[-pi/2,+pi/2]
    float x = 0.5f * CAR_WHEEL_X * course_diff / walk;//限制最大转角30度
    if (float_compare(fabsf(x), sinf(CAR_TURN_ANGLE_MAX / 180 * M_PI)) > 0) {
        return (float) float_sign(course_diff) * CAR_TURN_ANGLE_MAX;
    }
    return (float) (asinf(x) * 180 / M_PI);
}

//RTK body坐标系转化到ENU坐标系
void ctl_coordinate_convert(struct POINT3D* point, double alf,double distance) {
    assert(point != NULL);

    point->x =  point->x +  distance* sin(alf);
    point->y = point->y +  distance* cos(alf);
}


float ctl_move_strategy(const struct LOCATION *target_location, float current_head) {
    assert(target_location != NULL);
    const float stanley_forward_distance = 1.0f;//stanley前瞻距离
    const float pure_pursuit_angle_min = M_PI / 18;
    const float pure_forward_distance_max = 1.0f;
    const struct POINT3D *point3d = &target_location->point3d;
    const float radian = ctl_point_distance(point3d);

    float move_direction_diff = ctl_point_direction(point3d) - (float) (current_head / 180 * M_PI);
    float target_direction_diff = (float) (target_location->relative_head / 180 * M_PI);//[-M_PI/2,M_PI/2]

    //head_diff 调节到[-PI,PI]
    ctl_angle_pipi(&move_direction_diff);

    logger_warn("radian:%.2f,move_direction_diff:%.2f,target_direction_diff:%.2f\n",
                radian, move_direction_diff * 180 / M_PI, target_direction_diff * 180 / M_PI);

    //横向误差引起的转角，horizon > 0表示右转,horizon < 0表示左转
    float horizon = radian * sinf(move_direction_diff - target_direction_diff);//横向偏差
    //当偏离航向大于一定值时，减小横向影响
    float horizon_angle = atanf(horizon / stanley_forward_distance);

    logger_warn("run horizon offset:%.2f,horizontal angle:%.2f,course angle:%.2f\n",
                horizon, horizon_angle * 180 / M_PI, target_direction_diff * 180 / M_PI);

    //航向角控制
    if (float_compare(fabsf(target_direction_diff), M_PI_2) >= 0 ||
        float_compare(fabsf(horizon), CAR_MOVE_ERROR_ALLOW) <= 0) {
        return (float) (target_direction_diff * 180 / M_PI);
    }

    /*
     * 小车几何中心作为等效运动质心,那么质心切线方向与航向角重合
     */
    float turn_angle;
    float pure_pursuit_turn = target_direction_diff; //[-PI/2,PI/2]
    //计算等效质心转弯半径
    float pure_radius = float_compare(pure_pursuit_turn, 0.0f) == 0 ?
                        10000.0f : fabsf(horizon) / (1 - cosf(pure_pursuit_turn));
    //look ahead distance
    float pure_forward_distance = pure_radius * sinf(fabsf(pure_pursuit_turn)) -
                                  sqrtf(radian * radian - horizon * horizon);

    logger_warn("pure_radius:%.2f,forward_distance:%.2f\n", pure_radius, pure_forward_distance);

    if (float_sign(pure_pursuit_turn) * float_sign(horizon_angle) < 0 &&
        float_compare(fabsf(pure_pursuit_turn), pure_pursuit_angle_min) > 0 &&
        float_compare(pure_forward_distance, pure_forward_distance_max) <= 0) {
        //计算内轮转弯角
        float angle = (float) (atanf(0.5f * CAR_WHEEL_Y / (pure_radius - 0.5f * CAR_WHEEL_X)) * 180 / M_PI);
        //pure pursuit控制
        logger_important("CONTROL WITH PURE PURSUIT\n");
        if (float_sign(pure_pursuit_turn) > 0) {
            turn_angle = angle;
        } else if (float_sign(pure_pursuit_turn) < 0) {
            turn_angle = -angle;
        } else {
            turn_angle = 0;
        }

    } else {
        //Stanley控制
        //horizon_angle 最大60 degree
        logger_important("CONTROL WITH STANLEY\n");
        ctl_angle_threshold(&horizon_angle, (float) -M_PI / 2,
                            (float) M_PI / 2);
        float coefficient = 1.0f;
        //调节系数coefficient
        /*
        const float stanley_offset_max = 2.0f;
        if (float_compare(fabsf(target_direction_diff), M_PI_4) > 0) {
            coefficient = 2.0f - (float) (fabsf(target_direction_diff) / M_PI_4);
        }

        if (float_compare(fabsf(horizon), stanley_offset_max) < 0) {
            float k = sqrtf(fabsf(horizon) / stanley_offset_max);
            if (float_compare(k, 0.5f) < 0) {
                coefficient = coefficient * 0.5f;
            } else {
                coefficient = coefficient * k;
            }
        }
         */

        //需要调节的航向角
        float head_diff = coefficient * horizon_angle + target_direction_diff;
        turn_angle = radian > CAR_TURN_MOVE_MAX_LINE ?
                     ctl_ackerman_turn(head_diff, CAR_TURN_MOVE_MAX_LINE) :
                     ctl_ackerman_turn(head_diff, radian);
    }
    return turn_angle;
}

//自动寻迹计算转弯角度和前进距离
/*
 * current_speed:单位:m/s
 * heading:当前车身航向角,单位rad
 * point3d:目标点
 * 控制思想:当前点与目标点方向距离较远时，以靠近目标点的方向运动为主，当距离较近时，目标点速度切线方向为主
 */

struct CONTROL_PACKAGE *ctl_auto_cmd_init(const struct LOCATION *location, float current_head, char status) {
    assert(location != NULL);
    float current_speed = location->speed;
    float speed;//单位m/s
    char direction = CAR_WALK_FORWARD, turnRorL = CAR_TURN_RIGHT;
    if (current_speed < 0) {
        current_speed = -current_speed;
        direction = CAR_WALK_BACKWARD;
    }

    if (current_speed > CAR_WALK_REF_SPEED) {
        current_speed = CAR_WALK_REF_SPEED;
    }

    if (status == CTL_SPEED_RUN) {
        speed = (CAR_WALK_REF_SPEED + current_speed) / 2.0f;
    } else if (status == CTL_SLOW) {
        speed = (CAR_SLOW_WALK_REF_SPEED + current_speed) / 2.0f;
    } else if (ctl_stop_status(status)) {
        speed = CAR_SLOW_WALK_REF_SPEED;
    } else {
        speed = current_speed;
    }

    float radian = ctl_point_distance(&location->point3d);//m
    //以速度speed运行radian米，需要T个周期
    int period = floor((double) radian / speed * 1000 / CAR_MSG_PERIOD);
    //当前目标点与当前位置距离太近，跟踪下一个目标点
    if (period == 0 && !ctl_stop_status(status)) {
        return NULL;
    }
    //计算转弯轮角度和前进轮角度
    const char KEEP_TIMES = CAR_MSG_PERIOD / CAR_CMD_PERIOD;

    struct CONTROL_PACKAGE *package = ctl_package_init();
    ctl_package_head_init(&package->head, CTL_PACKAGE_ID++, CTL_PACKAGE_DATA_LEN);
    //如果是停车状态，位移模式运行
    if (ctl_stop_status(status)) {
        package->speed = (uint8_t) (speed * 100);
        package->times = 1;
        //根据radian = R * alf
        float move_angle = (float) (radian / CAR_WHEEL_RADIUS * 180 / M_PI);
        float turn_angle = ctl_ackerman_turn((float) (location->relative_head / 180 * M_PI), radian);
        if (turn_angle < 0) {//逆时针
            turn_angle = -turn_angle;
            turnRorL = CAR_TURN_LEFT;
        }
        logger_warn("APTER MOVE ANGLE:%f and TURN ANGLE %f,CAR WILL STOP\n", move_angle, turn_angle);
        ctl_package_cmd_fill(&package->cmd, move_angle, turn_angle,
                             CTL_DISPLACE_RUN, turnRorL, direction);

    } else {//速度模式，200ms周期
        //根据FORWARD POINT，计算运动趋势的偏差和目标的

        package->speed = (uint8_t) (speed * 100);
        package->times = KEEP_TIMES;
        /*
         * 可以通过CAR_TURN_MOVE_MAX_LINE调节轨迹的平滑，以及与目标的偏差，PID比例
         */
        float turn_angle = ctl_move_strategy(location, current_head);
        if (turn_angle < 0) {//逆时针
            turn_angle = -turn_angle;
            turnRorL = CAR_TURN_LEFT;
        }
        logger_warn("RUN WITH SPEED:%.2f m/s,TURN ANGLE:%.2f degree, DIRECTION %s\n",
                    speed, turn_angle, CAR_TURN_LEFT == turnRorL ? "LEFT" : "RIGHT");

        ctl_package_cmd_fill(&package->cmd, 0, turn_angle, CTL_SPEED_RUN,
                             turnRorL, direction);
    }
    //计算校验
    ctl_gen_crc((uint8_t *) package, package->package_size);
    return package;
}












