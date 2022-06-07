//
// Created by server on 2022/4/9.
//
#include "task/task.h"
#include "include/header.h"


static void TEST_CMD_PACKAGE(struct CONTROL_PACKAGE *package, struct sockaddr_in *addr, int mem_size) {

    cmd_send_data((uint8_t *) package, mem_size, addr);
    logger_print_bytes((uint8_t *) package, mem_size);
    ctl_package_destroy(&package);

    usleep(1000 * 100);
    printf("-----------------------------------------\n");
}

static void TEST_CMD_PACKAGE_REPLY(struct CONTROL_PACKAGE *package, struct sockaddr_in *addr, int mem_size) {
    struct sockaddr_in from_addr = {0};


    cmd_send_data((uint8_t *) package, mem_size, addr);
    logger_print_bytes((uint8_t *) package, mem_size);
    ctl_package_destroy(&package);


    struct TASK_EVENT_BUFFER udp_data;
    task_init_buffer(&udp_data, 1500);
    while (1){
        udp_data.data_len = cmd_recv_data((uint8_t *) udp_data.buffer, udp_data.buffer_size, &from_addr);
        logger_print_bytes((uint8_t *) udp_data.buffer, udp_data.data_len);

        if (ctl_cmd_check((uint8_t *) udp_data.buffer, udp_data.data_len)) {
            struct CONTROL_PACKAGE *recv_package = (struct CONTROL_PACKAGE *) udp_data.buffer;
            logger_info("HEAD : %02X %02X , ID : %d , LEN:%d , STATUS:%02X , WHEEL:%.02f , TURN:%.02f\n",
                        recv_package->head.code[0], recv_package->head.code[1],
                        swap_uint16(recv_package->head.ID), swap_uint16(recv_package->head.data_len),
                        recv_package->cmd.status,
                        (float) (recv_package->cmd.wheel.bit.degree & 0x1FF) +
                        (recv_package->cmd.wheel.bit.minute & 0x40) / 60.0,
                        (float) (recv_package->cmd.turn.bit.degree & 0x1FF) +
                        (recv_package->cmd.turn.bit.minute & 0x40) / 60.0);

        } else {
            printf("response package check failed\n");
        }
    }
    task_release_buffer(&udp_data);
    usleep(1000 * 200);

}

static void TEST_CMD() {
    struct sockaddr_in ctl_remote_addr = {0};
    cmd_init_remote(&ctl_remote_addr, CTL_REMOTE_PORT, CTL_REMOTE_IP);

    if (cmd_create_socket(UDP_LOCAL_IP, UDP_LOCAL_PORT) < 0) {
        printf("create socket error\n");
    }
    struct CONTROL_PACKAGE *package = ctl_manual_status_cmd_init(CTL_UNPARK);
    TEST_CMD_PACKAGE(package, &ctl_remote_addr, package->package_size);


    //以某速度移动
    float speed = 0.2f;
    float arc = (float) -(30.0f / 180 * M_PI);
//    struct LOCATION location;
//
//    location.point3d.x = -1.0;
//    location.point3d.y = 1.0;
//    location.point3d.z = 0;
//    location.diff_head = (float)(30.0f / 180 * M_PI);
//    location.speed = 0.2f;

//    mem_size = ctl_manual_status_cmd_init(&package,  CTL_SPEED_RUN);
//    TEST_CMD_PACKAGE_REPLY(package,&ctl_remote_addr,mem_size);
//    sleep(1);
//    mem_size = ctl_manual_status_cmd_init(&package,  CTL_PARK);
//    TEST_CMD_PACKAGE_REPLY(package,&ctl_remote_addr,mem_size);

//    package = ctl_displace_cmd_init(speed, arc);
//    TEST_CMD_PACKAGE(package,&ctl_remote_addr,package->package_size);
    while (1){
        package = ctl_manual_speed_cmd_init(speed, arc);
        printf("**********************************************:%d\n", swap_uint16(package->head.ID));
        TEST_CMD_PACKAGE_REPLY(package,&ctl_remote_addr,package->package_size);

    }
    arc = (float) (100.0f / 180 * M_PI);
    package = ctl_displace_cmd_init(speed, arc);
    TEST_CMD_PACKAGE_REPLY(package, &ctl_remote_addr, package->package_size);
    arc = (float) (30.0f / 180 * M_PI);
    for(int i = 0; i < 200; i++){
        package = ctl_manual_speed_cmd_init(speed, arc);
        printf("**********************************************:%d\n", swap_uint16(package->head.ID));
        TEST_CMD_PACKAGE(package,&ctl_remote_addr,package->package_size);

    }
}


static int TEST_LOGIN() {
    cors_client_t *client = cors_client_init("120.253.226.97", 8002, "RTCM33_GRCE", "abcv21002", "ec55ad68");
    //cors_client_t *client = cors_client_init("rtk2go.com", 2101, "A_CMS_01", "zxfheu@126.com", "zhu.123");
    while (1) {//loop for connect server

        if (cors_connect(client) < 0) {
            task_set_status(TASK_STATUS_INIT);
            break;
        }
        TASK_CORS_COMMUNICATE();
    }
    cors_client_destroy(client);
    return 0;
}


static int TEST() {

    if (!open_logger(MEMORY_LOGGER_FILE)) {
        printf("open logger failed\n");
    }

    TEST_CMD();
    close_logger(MEMORY_LOGGER_FILE);
    return 0;
}

int main() {
    signal(SIGINT, TASK_EXIT);
    TASK_MAIN();
    return 0;
}
