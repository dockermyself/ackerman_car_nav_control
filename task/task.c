//
// Created by server on 2022/4/19.
//

#include "task.h"



static int task_status = TASK_STATUS_STOP;
static int task_keepalive = 1;

void task_set_status(enum TASK_STATUS status) {
    task_status = status;
}

enum TASK_STATUS task_get_status() {
    return task_status;
}

//心跳
void task_control_disconnected(void* arg){
    task_keepalive = 0;
    logger_warn("control is disconnected,maybe in PARK STATUS or control unit not start\n");
}

void task_control_connected(){
    task_keepalive = 1;
}

int task_control_keepalive(){
    return task_keepalive;
}
void task_view_send_message(struct TASK_EVENT_BUFFER *event, const char *name,
                            double x, double y, struct sockaddr_in *remote_addr) {
    assert(event != NULL && name != NULL && remote_addr != NULL);
    struct POINT2I point2i = {
            .x = (int) (1000 * x),
            .y = (int) (1000 * y)
    };
    sprintf(event->buffer, "%s,%d,%d", name, point2i.x, point2i.y);
    cmd_send_data((uint8_t *) event->buffer, (int) strlen(event->buffer), remote_addr);
}

struct TASK_PARAM *task_init_param(struct CMD_MSG *msg, struct sockaddr_in *to_addr) {
    struct TASK_PARAM *param = malloc(sizeof(struct TASK_PARAM) + msg->data_len);
    memcpy(&param->to_addr, to_addr, sizeof(struct sockaddr_in));
    memcpy(&param->msg, msg, sizeof(struct CMD_MSG) + msg->data_len);
    return param;
}

void task_destroy_param(struct TASK_PARAM **param) {
    if (param == NULL || *param == NULL) {
        return;
    }
    free(*param);
    *param = NULL;
}

//在非TASK_STATUS_STOP和TASK_STATUS_TRACE可以手动控制
int task_manual_control() {
    if (task_status == TASK_STATUS_STOP || task_status == TASK_STATUS_TRACE) {
        return 0;
    }
    return 1;
}

void task_init_buffer(struct TASK_EVENT_BUFFER *event, int buffer_size) {
    event->buffer = malloc(buffer_size);
    memset(event->buffer, 0, buffer_size);
    event->buffer_size = buffer_size;
    event->data_len = 0;
}


void task_set_buffer(struct TASK_EVENT_BUFFER *event, const char *buffer, int data_len) {
    if (data_len > event->buffer_size) {
        return;
    }
    memcpy(event->buffer, buffer, data_len);
    event->data_len = data_len;
}


void task_release_buffer(struct TASK_EVENT_BUFFER *event) {
    free(event->buffer);
}


void task_rtk_to_location(struct RTK_MSG *rtk_msg, struct LLA *target_lla, struct LOCATION *location) {
    if (rtk_msg == NULL || location == NULL) return;
    lla_to_enu(&rtk_msg->lla, target_lla,
               &location->point3d.x, &location->point3d.y, &location->point3d.z);
    //location的速度和向根据当前RTK速度和航向确定
    location->speed = rtk_msg->lla.speed;
    location->relative_head = target_lla->head - rtk_msg->lla.head;//目标航向与当前航向的差值

    if (location->relative_head > 180) {
        location->relative_head -= 360;
    } else if (location->relative_head < -180) {
        location->relative_head += 360;
    }
    location->absolute_head = target_lla->head;

    //logger_important("convert before location:x=%.2f,y=%.2f\n", location->point3d.x, location->point3d.y);
    ctl_coordinate_convert(&location->point3d, target_lla->head / 180 * M_PI, CAR_BODY_Y / 2);
    ctl_coordinate_convert(&location->point3d, rtk_msg->lla.head / 180 * M_PI, -CAR_BODY_Y / 2);
    //logger_important("convert after location:x=%.2f,y=%.2f\n", location->point3d.x, location->point3d.y);
}

void task_rtk_to_point3d(struct RTK_MSG *rtk_msg, struct LLA *target_lla, struct POINT3D *point3d) {
    assert(rtk_msg != NULL && target_lla != NULL && point3d != NULL);
    lla_to_enu(&rtk_msg->lla, target_lla,
               &point3d->x, &point3d->y, &point3d->z);
}

void task_rtk_to_point2d(struct RTK_MSG *rtk_msg, struct LLA *target_lla, struct POINT2D *point2d) {
    assert(rtk_msg != NULL && target_lla != NULL && point2d != NULL);
    double z;
    lla_to_enu(&rtk_msg->lla, target_lla,
               &point2d->x, &point2d->y, &z);
}

struct LLA *task_trace_location(int pos, struct LLA *location_array, int array_size) {
    assert(location_array != NULL);
    if (pos >= array_size) {
        return NULL;
    }
    return location_array + pos;
}


int task_trace_point(struct POINT3D *point3d, float current_direction,
                     float distance_threshold, float angle_threshold) {
    assert(point3d != NULL);
    if (ctl_point_distance(point3d) < distance_threshold) {
        return 0;
    }
    float angle = (float) (ctl_point_direction(point3d) * 180 / M_PI) - current_direction;
    if (angle > angle_threshold || angle < -angle_threshold) {
        return 0;
    }
    return 1;
}

void task_send_control_cmd(const uint8_t *buf, int len, struct sockaddr_in *to_addr) {
    assert(buf != NULL && to_addr != NULL);
    int n = cmd_send_data(buf, len, to_addr);
    if (n <= 0) {
        logger_error("send control cmd failed\n");
    }
}


//寻迹任务
void *TASK_TRACKING(void *args) {
    struct TASK_PARAM *param = (struct TASK_PARAM *) args;
    struct CMD_MSG *param_msg = &param->msg;
    struct RTK_MSG rtk_msg = {0}, zero_pos = {0};
    struct sockaddr_in ctl_remote_addr = {0};
    struct sockaddr_in view_remote_addr = {0};
    cmd_init_remote(&ctl_remote_addr, CTL_REMOTE_PORT, CTL_REMOTE_IP);
    cmd_init_remote(&view_remote_addr, VIEW_REMOTE_PORT, VIEW_REMOTE_IP);
    struct TASK_EVENT_BUFFER view_event = {0};
    task_init_buffer(&view_event, 128);

    //解析数据
    const int PACKAGE_SIZE = sizeof(struct LLA);
    const int SLOWDOWN_BEFORE = 2;
    const int FORWARD_LOOKING = 1;
    int cur_location = 0;
    enum CTL_STATUS trace_status = CTL_SPEED_RUN;
    int point_num = param_msg->data_len / PACKAGE_SIZE;//获取点的数量
    struct POINT3D pos_diff = {0};
    //设置view的初始位置作为原点
    msg_rtk_get(&zero_pos);
    logger_write_message(MESSAGE_LOGGER_FILE, "Tracking TASK START");
    logger_info("recv point_num:%d\n", point_num);
    const int CONTROL_CMD_INTERVAL = CAR_MSG_PERIOD * 1000;//us
    while (cur_location < point_num && trace_status != CTL_STOP && TASK_STATUS_TRACE == task_get_status()) {

        logger_info("Tracking TASK: point_num = %d,current point:%d\n", point_num, cur_location);
        //获取目标位置
        struct LLA *target_lla = task_trace_location(cur_location, (struct LLA *) param_msg->data, point_num);
        if (NULL == target_lla) {
            break;
        }
        //获取前瞻位置
        struct LLA *forward_lla = task_trace_location(cur_location + FORWARD_LOOKING, (struct LLA *) param_msg->data,
                                                      point_num);

        task_rtk_to_point3d(&zero_pos, target_lla, &pos_diff);
        task_view_send_message(&view_event, "TARGET", pos_diff.x, pos_diff.y, &view_remote_addr);

        cur_location = cur_location + 1;
        if (cur_location == point_num || NULL == forward_lla) {//到达目标点
            trace_status = CTL_STOP;
        } else if (SLOWDOWN_BEFORE + cur_location == point_num) {//减速
            trace_status = CTL_SLOW;
        }
        while (TASK_STATUS_TRACE == task_get_status()) {//完成target_lla的寻迹，直到越过该路径点
            memset(&rtk_msg, 0, sizeof(struct RTK_MSG));
            msg_rtk_get(&rtk_msg);
            struct LOCATION target_enu = {0}, forward_enu = {0};
            //根据target_lla和rtk_msg计算运动增量:保存到enu
            task_rtk_to_location(&rtk_msg, target_lla, &target_enu);

            /*
             * 判断是否已经到达目标点,如果到达目标点且有前瞻点，则重新计算当前点和前瞻点
             * 如果到达目标点且没有前瞻点，也就是status == CTL_PARK，则停止任务
             */
            if (trace_status != CTL_STOP &&
                !ctl_target_forward(&target_enu.point3d, target_lla->head)) {
                logger_warn("car have moved this target point,go to next target point\n");
                break;
            }

            //计算相对于零点的位置变化保存到pos_diff，发送数据到view显示
            task_rtk_to_point3d(&zero_pos, &rtk_msg.lla, &pos_diff);
            task_view_send_message(&view_event, "CURRENT", pos_diff.x, pos_diff.y, &view_remote_addr);


            //计算控制命令RTK_CONTROL_MSG
            struct CONTROL_PACKAGE *package = NULL;
            //根据target_enu和forward_enu，当前航向角，计算控制命令
            if (CTL_STOP == trace_status) {
                package = ctl_auto_cmd_init(&target_enu, rtk_msg.lla.head, CTL_STOP);
            } else {
                //计算前瞻位置
                task_rtk_to_location(&rtk_msg, forward_lla, &forward_enu);
                package = ctl_auto_cmd_init(&forward_enu, rtk_msg.lla.head, CTL_SPEED_RUN);
            }

            if (package == NULL)//该轨迹连运行结束
            {
                logger_warn("TRACK TO NEXT POINT\n");
                logger_write_message(MESSAGE_LOGGER_FILE, "TRACK TO NEXT POINT");
                break;
            } else if (cmd_send_data((uint8_t *) package, package->package_size, &ctl_remote_addr) < 0) {
                logger_error("send control cmd failed\n");
                ctl_package_destroy(&package);
                continue;
            } else if (CTL_STOP == trace_status) {
                logger_warn("TRACE TO THE END POINT BY DISPLACEMENT...WAIT CAR STOP\n");
                ctl_package_destroy(&package);
                break;
            }
            ctl_package_destroy(&package);

            usleep(CONTROL_CMD_INTERVAL);

        }
    }

    logger_write_message(MESSAGE_LOGGER_FILE, "Tracking TASK END");
    while (task_control_keepalive()){
        sleep(1);
    }
    sleep(10);
    logger_important("CAR STOPPED\n");
    msg_rtk_get(&rtk_msg);//获取最后一个RTK数据
    task_rtk_to_point3d(&zero_pos, &rtk_msg.lla, &pos_diff);
    task_view_send_message(&view_event, "CURRENT", pos_diff.x, pos_diff.y, &view_remote_addr);
    task_view_send_message(&view_event, "END", 0, 0, &view_remote_addr);

    if (task_get_status() == TASK_STATUS_TRACE) {//TASK_STATUS_TRACE -> TASK_PICK,完成一段寻迹任务
        cmd_send_rtkdata_with_status(param_msg, &param->to_addr, &rtk_msg.lla, 0x02);
        task_set_status(TASK_STATUS_IDLE);
    } else if (task_get_status() == TASK_STATUS_RTK_ERROR) {
        struct CONTROL_PACKAGE *package = ctl_manual_status_cmd_init(CTL_STOP);
        cmd_send_data((uint8_t *) package, package->package_size, &ctl_remote_addr);
        usleep(CONTROL_CMD_INTERVAL);
        ctl_change_cmd_status(package, CTL_PARK);
        cmd_send_data((uint8_t *) package, package->package_size, &ctl_remote_addr);
        ctl_package_destroy(&package);
        cmd_send_rtkdata_with_status(param_msg, &param->to_addr, &rtk_msg.lla, 0x03);
    } else {//中断任务
        struct CONTROL_PACKAGE *package = ctl_manual_status_cmd_init(CTL_STOP);
        cmd_send_data((uint8_t *) package, package->package_size, &ctl_remote_addr);
        ctl_package_destroy(&package);
        cmd_send_rtkdata_with_status(param_msg, &param->to_addr, &rtk_msg.lla, 0x02);
    }
    task_destroy_param(&param);//释放任务参数资源
    task_release_buffer(&view_event);
    logger_write_message(MESSAGE_LOGGER_FILE, "Tracking task end");
    logger_warn("Tracking task end\n");
    return NULL;
}


//接收UDP消息
void TASK_RECV_FROM_UDP() {

    struct TASK_EVENT_BUFFER udp_data;
    task_init_buffer(&udp_data, 1024 * 8);

    struct sockaddr_in from_addr = {0};
    struct sockaddr_in to_addr = {0};
    struct sockaddr_in ctl_remote_addr = {0};
    cmd_init_remote(&ctl_remote_addr, CTL_REMOTE_PORT, CTL_REMOTE_IP);

    //register timer runner
    int runner_fd = time_runner_register(1000000,task_control_disconnected,NULL,0);
    if(runner_fd < 0 || time_runner_start(runner_fd) < 0){
        logger_error("register or start time runner failed\n");
        return;
    }

    logger_write_message(MESSAGE_LOGGER_FILE, "recv data from udp task start");
    while (task_get_status() != TASK_STATUS_EXIT) {
        memset(udp_data.buffer, 0, udp_data.data_len);
        int len = cmd_recv_data((uint8_t *) udp_data.buffer, udp_data.buffer_size, &from_addr);
        memcpy(&to_addr, &from_addr, sizeof(struct sockaddr_in));
        to_addr.sin_port = htons(UDP_RESPOND_PORT);
        if (len < 0) {
            continue;
        }

        udp_data.data_len = len;
        //logger_print_bytes((uint8_t*)udp_data.buffer, udp_data.data_len);
        //接收到的数据是否是底盘response的命令
        if (ctl_cmd_check((uint8_t *) udp_data.buffer, udp_data.data_len)) {
            //接收到的是控制命令

//            struct CONTROL_PACKAGE *package = (struct CONTROL_PACKAGE *) udp_data.buffer;
//            logger_info("HEAD : %02X %02X , ID : %d , LEN:%d , STATUS:%02X , WHEEL:%.2f , TURN:%.2f\n",
//                        package->head.code[0], package->head.code[1],
//                        swap_uint16(package->head.ID), swap_uint16(package->head.data_len), package->cmd.status,
//                        (float) (package->cmd.wheel.bit.degree & 0x1FF) + (package->cmd.wheel.bit.minute & 0x40) / 60.0,
//                        (float) (package->cmd.turn.bit.degree & 0x1FF) + (package->cmd.turn.bit.minute & 0x40) / 60.0);
            time_runner_reset(runner_fd);
            if(!task_control_keepalive()){
                task_control_connected();
            }
            continue;

        }
        //判断是否是上位机发送的指令
        if (!cmd_check_head((const uint8_t *) udp_data.buffer, udp_data.data_len) ||
            !cmd_check_crc((const uint8_t *) udp_data.buffer, udp_data.data_len) ||
            !cmd_check_len((const uint8_t *) udp_data.buffer, udp_data.data_len)) {
            continue;
        }


        struct CMD_MSG *msg = (struct CMD_MSG *) udp_data.buffer;

        //只有在TASK_STATUS_LOGIN或者TASK_STATUS_RESUME状态才转移到TASK_STATUS_TRACE
        if (msg->type == CMD_TRACE_TO_TARGET) {
            logger_write_message(MESSAGE_LOGGER_FILE, "RECV CMD_TRACE_TO_TARGET");
            logger_warn("RECV CMD_TRACE_TO_TARGET\n");
            struct RTK_MSG rtk_msg = {0};
            if (task_get_status() == TASK_STATUS_IDLE) {
                pthread_t thread_id;
                msg_rtk_get(&rtk_msg);
                cmd_send_rtkdata_with_status(msg, &to_addr, &rtk_msg.lla, 0x01);
                struct TASK_PARAM *param = task_init_param(msg, &to_addr);

                if (pthread_create(&thread_id, NULL, TASK_TRACKING, param) != 0) {
                    task_destroy_param(&param);
                    logger_error("create TASK_TRACKING thread failed\n");
                } else {
                    logger_info("create TASK_TRACKING thread success\n");
                    pthread_detach(thread_id);
                }
                task_set_status(TASK_STATUS_TRACE);
                logger_write_message(MESSAGE_LOGGER_FILE, "SET STATUS TASK_STATUS_TRACE");
            } else {//无法完成寻迹
                cmd_send_rtkdata_with_status(msg, &to_addr, &rtk_msg.lla, 0x03);
            }
        }
            //中断寻迹
        else if (msg->type == CMD_INTERRUPT_TRACE) {
            logger_write_message(MESSAGE_LOGGER_FILE, "RECV CMD_INTERRUPT_TRACE");
            if (task_get_status() == TASK_STATUS_TRACE) {
                task_set_status(TASK_STATUS_IDLE);
                cmd_send_ctl_status(msg, &to_addr, 0x01);
            } else {
                cmd_send_ctl_status(msg, &to_addr, 0x00);
            }

        }
            //只有在初始化状态才可以转移到登录状态
        else if (msg->type == CMD_LOGIN_DATA) {
            logger_write_message(MESSAGE_LOGGER_FILE, "RECV CMD_LOGIN_DATA");
            logger_info("RECV CMD_LOGIN_DATA\n");
            if (task_get_status() == TASK_STATUS_INIT) {
                task_set_status(TASK_STATUS_IDLE);//防止连续多包CMD_LOGIN_DATA数据
                struct TASK_PARAM *param = task_init_param(msg, &to_addr);//创建任务参数
                pthread_t thread_id;

                if (pthread_create(&thread_id, NULL, TASK_LOGIN_CORS, param) != 0) {
                    task_destroy_param(&param);
                    logger_info("create thread failed\n");
                } else {
                    pthread_detach(thread_id);
                }
            } else {
                cmd_send_ctl_status(msg, &to_addr, 0x00);
            }
        }
            //初始化命令:TASK_STATUS_STOP与TASK_STATUS_INIT相互转移
        else if (msg->type == CMD_INIT) {
            logger_write_message(MESSAGE_LOGGER_FILE, "RECV CMD_INIT");
            logger_info("RECV CMD_INIT\n");
            char status = msg->data[0];//启动或者停止
            if (status == 0x01 && task_get_status() == TASK_STATUS_STOP) {
                task_set_status(TASK_STATUS_INIT);
            } else if (status == 0x00 &&
                       task_get_status() != TASK_STATUS_TRACE) {//TODO:是否限定只有在TASK_STATUS_INIT 或者 TASK_STATUS_IDLE状态才能停止
                task_set_status(TASK_STATUS_STOP);
            }
            cmd_send_ctl_status(msg, &to_addr, status);
        }
            //查询RTK状态0x01准备就绪,0x00没有连接，不受状态影响，只与cors连接状态相关
        else if (msg->type == CMD_QUERY_RTKSTATUS) {
            logger_write_message(MESSAGE_LOGGER_FILE, "RECV CMD_QUERY_RTK STATUS");
            logger_info("RECV CMD_QUERY_RTK STATUS\n");
            if (cors_keep_connected()) {
                cmd_send_ctl_status(msg, &to_addr, 0x01);
            } else {
                cmd_send_ctl_status(msg, &to_addr, 0x00);
            }
        }
            //查询RTK当前的位置，方向与速度，不受状态影响，只与cors连接状态相关
        else if (msg->type == CMD_QUERY_WAYPOINT) {
            logger_write_message(MESSAGE_LOGGER_FILE, "RECV CMD_QUERY_WAYPOINT");
            logger_info("RECV CMD_QUERY_WAYPOINT\n");
            if (cors_keep_connected()) {
                struct RTK_MSG rtk_msg;
                msg_rtk_get(&rtk_msg);
                logger_info("lon:%f,lat:%f,alt:%f,head:%f,speed:%f\n",
                            rtk_msg.lla.lon, rtk_msg.lla.lat, rtk_msg.lla.alt,
                            rtk_msg.lla.head, rtk_msg.lla.speed);
                //upload rtk data
                cmd_send_rtkdata(msg, &to_addr, &rtk_msg.lla);
            } else {
                cmd_send_rtkdata(msg, &to_addr, NULL);
            }
        }
            //以某速度移动，命令在TASK_STATUS_IDLE状态下可以执行
        else if (msg->type == CMD_SPEED) {
            logger_write_message(MESSAGE_LOGGER_FILE, "RECV CMD_SPEED");
            struct CONTROL_PACKAGE *package = NULL;
            float speed = 0;
            memcpy(&speed, msg->data, msg->data_len);
//            logger_info("move speed:%f\n", speed);
            if (0 == (int) (speed * 100)) {
                package = ctl_manual_status_cmd_init(CTL_STOP);
            } else {
                package = ctl_manual_speed_cmd_init(speed, 0);
            }
            if (NULL == package || !task_manual_control()) {//在非TASK_STATUS_STOP 和 TASK_STATUS_TRACE状态下才可以执行
                cmd_send_status_respond(msg, &to_addr, 0x00);
            } else {
                cmd_send_data((uint8_t *) package, package->package_size, &ctl_remote_addr);
                cmd_send_status_respond(msg, &to_addr, 0x01);
            }
            ctl_package_destroy(&package);
        }
            // 按弧度转弯
        else if (msg->type == CMD_WALK_AND_TURN) {
            logger_write_message(MESSAGE_LOGGER_FILE, "RECV CMD_TURN");
            float radian = 0;
            float speed = 0;
            memcpy(&speed, msg->data, sizeof(float));
            memcpy(&radian, msg->data + sizeof(float), sizeof(float));
//            logger_info("move speed:%f,turn radian:%f\n", speed, radian);

            struct CONTROL_PACKAGE *package = ctl_manual_speed_cmd_init(speed, radian);
            if (NULL == package || !task_manual_control()) {//在非TASK_STATUS_STOP 和 TASK_STATUS_TRACE状态下才可以执行
                cmd_send_status_respond(msg, &to_addr, 0x00);
            } else {
                cmd_send_data((uint8_t *) package, package->package_size, &ctl_remote_addr);
                cmd_send_status_respond(msg, &to_addr, 0x01);
            }
            ctl_package_destroy(&package);

        }
            //刹车停车
        else if (msg->type == CMD_WALKER_STOP) {
            logger_write_message(MESSAGE_LOGGER_FILE, "RECV CMD_WALKER_STOP");
            //先发送STOP,然后再发送PARK
            struct CONTROL_PACKAGE *package = ctl_manual_status_cmd_init(CTL_STOP);
            cmd_send_data((uint8_t *) package, package->package_size, &ctl_remote_addr);
            usleep(1000 * 100);
            ctl_change_cmd_status(package, CTL_PARK);
            cmd_send_data((uint8_t *) package, package->package_size, &ctl_remote_addr);
            cmd_send_status_respond(msg, &to_addr, 0x01);
            ctl_package_destroy(&package);
            logger_info("CTL_PARK\n");
        }
            //释放刹车
        else if (msg->type == CMD_RELEASE_BRAKE) {
            logger_write_message(MESSAGE_LOGGER_FILE, "RECV CMD_RELEASE_BRAKE");
            logger_info("RECV CMD_RELEASE_BRAKE\n");
            struct CONTROL_PACKAGE *package = ctl_manual_status_cmd_init(CTL_UNPARK);
            cmd_send_data((uint8_t *) package, package->package_size, &ctl_remote_addr);
            cmd_send_status_respond(msg, &to_addr, 0x01);
            ctl_package_destroy(&package);
        }
            //向前微移动:一定speed移动一段弧度，命令在TASK_STATUS_IDLE状态下可以执行
        else if (msg->type == CMD_KEEP_SPEED_WALK_RAD) {
            logger_write_message(MESSAGE_LOGGER_FILE, "RECV CMD_WALK_AND_TURN");

            float speed;
            float radian;
            memcpy(&speed, msg->data, sizeof(float));
            memcpy(&radian, msg->data + sizeof(float), sizeof(float));
//            logger_info("move speed:%f,move radian:%f\n", speed, radian);
            struct CONTROL_PACKAGE *package = ctl_displace_cmd_init(speed, radian);
            if (NULL == package || !task_manual_control()) {//在非TASK_STATUS_STOP 和 TASK_STATUS_TRACE状态下才可以执行
                cmd_send_status_respond(msg, &to_addr, 0x00);
            } else {
                cmd_send_data((uint8_t *) package, package->package_size, &ctl_remote_addr);
                cmd_send_status_respond(msg, &to_addr, 0x01);
            }
            ctl_package_destroy(&package);
        }

    }
    task_release_buffer(&udp_data);
    time_runner_destroy(runner_fd);
    logger_write_message(MESSAGE_LOGGER_FILE, "task udp recv exit");
    logger_warn("task udp recv exit\n");
}

void *TASK_RECV_FROM_SERIAL(void *args) {
    struct TASK_EVENT_BUFFER nema;
    task_init_buffer(&nema, 128);
    struct TASK_EVENT_BUFFER param;
    task_init_buffer(&param, 1024);

    char head_code[10] = {0};
    const int CHECK_CODE_LEN = 2;
    int buffer_start = 0, buffer_end = 0;//设置start，end指针读取串口

    //如果第一包数据为RMC,那么等待HDT报文

    while (task_get_status() != TASK_STATUS_EXIT) {
        int n_serial = serial_read(SERIAL_DEV0, param.buffer + buffer_end, param.buffer_size / 2);
        //int n_serial = GGARead(param.buffer + buffer_end, para
        // m->buffer_size / 2);
        if (n_serial <= 0) {
            continue;
        }

        buffer_end += n_serial;//有效数据[buffer_start,buffer_end)
        buffer_start = nema_find_head(param.buffer, buffer_start, buffer_end, head_code, 10);


        int package_end;
        //有效数据[buffer_start,package_end]
        while ((package_end = nema_find_tail(param.buffer, buffer_start, buffer_end)) < buffer_end)//一次处理多包数据
        {

            //param.buffer[start] == '$',param.buffer[package_end] == ‘*’,查找[start,package_end)满足校验的'$'
            if (!nema_bbc_check_match(param.buffer, &buffer_start, package_end, head_code, 10)) {//没有查找到
                buffer_start = nema_find_head(param.buffer, package_end + 1, buffer_end, head_code, 10);
                continue;
            }

            memset(nema.buffer, 0, nema.buffer_size);
            nema_format_data(param.buffer, buffer_start, package_end + CHECK_CODE_LEN + 1,
                             nema.buffer, &nema.data_len);
            //%GNGGA发送给基准站
            if (strcmp(head_code, "$GPGGA") == 0 || strcmp(head_code, "$GNGGA") == 0) {
                if (nema_parse_gga(nema.buffer, NULL)) {
                    logger_write_message(NEMA_LOGGER_FILE, nema.buffer);
                    msg_cors_update(nema.buffer, nema.data_len);
                }


            } else if (strcmp(head_code, "$GNRMC") == 0 || strcmp(head_code, "$GPRMC") == 0) {
                struct Longitude longitude;
                struct Latitude latitude;
                double speed;
                if (nema_parse_rmc(nema.buffer, &latitude, &longitude, &speed)) {//解析经度，纬度信息
                    logger_write_message(NEMA_LOGGER_FILE, nema.buffer);
                    msg_rtk_update(&longitude, &latitude, &speed, NULL);
                }
            } else if (strcmp(head_code, "$GNHDT") == 0 || strcmp(head_code, "$GPHDT") == 0)//先获得$GNRMC，再获得$GNHDT
            {
                double heading = 0.0;
                if (nema_parse_hdt(nema.buffer, &heading)) {
                    logger_write_message(NEMA_LOGGER_FILE, nema.buffer);
                    msg_rtk_update(NULL, NULL, NULL, &heading);
                }

            }
            //更新start,准备解析下一包数据
            buffer_start = nema_find_head(param.buffer, package_end + CHECK_CODE_LEN + 1,
                                          buffer_end, head_code, 10);
        }
        if (buffer_start >= buffer_end) {
            buffer_start = 0, buffer_end = 0;
        } else if (buffer_start >= param.buffer_size / 2) {
            memcpy(param.buffer, param.buffer + buffer_start, buffer_end - buffer_start);
            buffer_end = buffer_end - buffer_start;
            buffer_start = 0;
        } else {
            //printf("data not complete,continue to read\n");
        }
    }

    task_release_buffer(&nema);
    task_release_buffer(&param);
    logger_write_message(MESSAGE_LOGGER_FILE, "task recv from serial exit");
    logger_warn("task recv from serial exit\n");
    return NULL;
}


void TASK_EXIT() {
    logger_write_message(MESSAGE_LOGGER_FILE, "task exit");
    logger_warn("wait for task exit...about 1 seconds\n");
    sleep(1);
    exit(0);
}

void TASK_MESSAGE_SEND(void *args) {
    struct MESSAGE_NODE *node = NULL;
    while (task_get_status() != TASK_STATUS_EXIT) {

        node = pop_message();
        if (NULL == node) {
            continue;
        }
        int n = cmd_send_data((uint8_t *) node->msg, node->size, &node->remote_addr);
        if (n < 0) {
            logger_write_message(MESSAGE_LOGGER_FILE, "send message error");
        }
        del_message(node);
        usleep(1000 * 50);
    }
}

void TASK_MAIN() {
    if (!open_logger(MESSAGE_LOGGER_FILE) || !open_logger(RTCM_LOGGER_FILE) ||
        !open_logger(NEMA_LOGGER_FILE) || !open_logger(MEMORY_LOGGER_FILE)) {
        logger_error("open logger failed\n");
        return;
    }
    //串口初始化
    if (serial_init(SERIAL_DEV0, BAUDRATE) < 0)//申请串口资源
    {
        logger_error("serial init failed\n");
        logger_write_message(MESSAGE_LOGGER_FILE, "serial init failed");
        return;
    }

    //创建RTK消息和cors消息
    msg_rtk_init();
    msg_cors_init();

    pthread_t tid_serial;
    //create thread for serial recv
    if (pthread_create(&tid_serial, NULL, TASK_RECV_FROM_SERIAL, NULL) != 0) {
        logger_error("create thread TASK_RECV_FROM_SERIAL error\n");
        return;
    }
    pthread_detach(tid_serial);
    //create udp socket
    if (cmd_create_socket(UDP_LOCAL_IP, UDP_LOCAL_PORT) < 0) {
        return;
    }


    //UDP接收上位机指令
    TASK_RECV_FROM_UDP();

    //等待资源释放
    logger_warn("wait for process exit...\n");
    logger_write_message(MESSAGE_LOGGER_FILE, "wait for process exit...");
    sleep(1);
    msg_rtk_destroy();
    msg_cors_destroy();
    serial_close(SERIAL_DEV0);//关闭串口并释放资源

    close_logger(MESSAGE_LOGGER_FILE);
    close_logger(RTCM_LOGGER_FILE);
    close_logger(NEMA_LOGGER_FILE);
    close_logger(MEMORY_LOGGER_FILE);
    logger_warn("all task now exit\n");
}

void TASK_CORS_COMMUNICATE() {
    struct TASK_EVENT_BUFFER param;
    task_init_buffer(&param, 1024);
    uint8_t gga_data[256] = {0};
    int time_out = 0;
    const int TIME_OUT_MAX = 50;//超时时间
    const int SLEEP_TIME = 200 * 1000;//休眠时间
    while (task_get_status() != TASK_STATUS_EXIT) {
        int recv_n = cors_recv(param.buffer, param.buffer_size);
        //print_bytes((const uint8_t *) param.buffer, param.buffer_size);
        if (recv_n < 0) {
            if (errno == EINTR) {
                continue;
            }
            if (++time_out > TIME_OUT_MAX) {
                logger_write_message(MESSAGE_LOGGER_FILE, "cors recv timeout");
                logger_warn("cors recv timeout\n");
                break;
            }

        } else if (recv_n == 0) {//接收到FIN位，服务器断开连接
            logger_warn("cors disconnected\n");
            logger_write_message(MESSAGE_LOGGER_FILE, "cors disconnected");
            break;
        } else if (recv_n > 0) {
            //发送数据
            logger_write_bytes(RTCM_LOGGER_FILE, (uint8_t *) param.buffer, recv_n);
            if (serial_write(SERIAL_DEV0, param.buffer, recv_n) < 0) {
                break;
            }
            time_out = 0;
        }
        memset(gga_data, 0, sizeof(gga_data));
        if (msg_cors_get((char *) gga_data) <= 0) {
            usleep(SLEEP_TIME);
            continue;
        }
        int send_n = cors_send((char *) gga_data, (int) strlen((const char *) gga_data) + 1);
        if (send_n == 0) {
            logger_warn("cors disconnected\n");
            logger_write_message(MESSAGE_LOGGER_FILE, "cors disconnected");
            break;
        }
        usleep(SLEEP_TIME);
    }
    set_connect_status(cors_disconnected);
    cors_close();
    task_release_buffer(&param);
    logger_write_message(MESSAGE_LOGGER_FILE, "cors communicate end");
    logger_warn("cors communicate end\n");

}


//登录CORS（Continuously Operating Reference Stations）就是网络基准站
void *TASK_LOGIN_CORS(void *args) {

    struct TASK_PARAM *param = (struct TASK_PARAM *) args;
    struct TASK_USER_LOGIN *login = (struct TASK_USER_LOGIN *) param->msg.data;
    cors_client_t *client = cors_client_init(login->ip, login->port, login->mount_point,
                                             login->user, login->password);
    logger_write_message(MESSAGE_LOGGER_FILE, "task login cors start");
    while (task_get_status() != TASK_STATUS_EXIT) {//loop for connect server

        if (cors_connect(client) < 0) {
            task_set_status(TASK_STATUS_INIT);
            cmd_send_ctl_status(&param->msg, &param->to_addr, 0x00);
            break;
        }
        task_set_status(TASK_STATUS_IDLE);
        cmd_send_ctl_status(&param->msg, &param->to_addr, 0x01);

        TASK_CORS_COMMUNICATE();
        task_set_status(TASK_STATUS_RTK_ERROR);
        logger_write_message(MESSAGE_LOGGER_FILE, "TASK_STATUS_RTK_ERROR");
    }
    cors_client_destroy(client);
    //release param
    task_destroy_param(&param);
    logger_write_message(MESSAGE_LOGGER_FILE, "task login cors end");
    logger_warn("task login cors end\n");
    return NULL;

}