//
// Created by server on 2022/4/22.
//
#include "command.h"
#include "msg/nema.h"
#include "utility/log.h"

static int CMD_SOCKET_FD = -1;
struct MESSAGE_NODE *message_head = NULL;
struct MESSAGE_NODE *message_tail = NULL;
pthread_cond_t mq_cond = PTHREAD_COND_INITIALIZER;
pthread_mutex_t mq_mutex = PTHREAD_MUTEX_INITIALIZER;

//message queue
int message_queue_init() {
    message_head = NULL;
    message_tail = NULL;
    pthread_mutex_init(&mq_mutex, NULL);
    pthread_cond_init(&mq_cond, NULL);
    return 0;
}

//new message node
struct MESSAGE_NODE *message_new(char *msg, int size, struct sockaddr_in *addr) {
    if (NULL == msg || NULL == addr || size <= 0) {
        return NULL;
    }
    struct MESSAGE_NODE *node = (struct MESSAGE_NODE *) malloc(sizeof(struct MESSAGE_NODE));
    if (NULL == node) {
        return NULL;
    }
    node->msg = (char *) malloc(size);
    if (NULL == node->msg) {
        free(node);
        return NULL;
    }
    memcpy(node->msg, msg, size);
    node->size = size;
    node->next = NULL;
    return node;
}

void del_message(struct MESSAGE_NODE *node) {
    if (NULL == node) {
        return;
    }
    if (NULL != node->msg) {
        free(node->msg);
    }
    free(node);
}

//push message  to queue tail
void push_message(struct MESSAGE_NODE *node) {
    pthread_mutex_lock(&mq_mutex);
    if (message_head == NULL) {
        message_head = node;
        message_tail = node;
    } else {
        message_tail->next = node;
        message_tail = node;
    }
    pthread_mutex_unlock(&mq_mutex);
    pthread_cond_signal(&mq_cond);
}


//pop message from  queue head
struct MESSAGE_NODE *pop_message() {
    struct MESSAGE_NODE *node = NULL;
    pthread_mutex_lock(&mq_mutex);
    pthread_cond_wait(&mq_cond, &mq_mutex);
    if (message_head != NULL) {
        node = message_head;
        message_head = node->next;
        if (message_head == NULL) {
            message_tail = NULL;
        }
    }
    pthread_mutex_unlock(&mq_mutex);
    return node;
}




int cmd_check_len(const uint8_t *buf, int len) {
    struct CMD_MSG *msg = (struct CMD_MSG *) buf;
    if (msg->data_len + sizeof(struct CMD_MSG) + 1 != len) {
        return 0;
    }
    return 1;
}

int cmd_check_crc(const uint8_t *buf, int len) {
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

//为了与check_crc一致传入比原来多一个
int cmd_gen_crc(const uint8_t *buf, int len) {
    if (NULL == buf) {
        return 0;
    }
    uint8_t code = 0;
    for (int i = 0; i < len - 1; ++i) {
        code += buf[i];
    }
    return code;
}


int cmd_check_head(const uint8_t *buf, int len) {//OXFF 0XAA + 5
    if (NULL == buf || len < CMD_MSG_MIN_LEN) {
        return 0;
    }
    if (buf[0] != 0xFF || buf[1] != 0xAA) {
        return 0;
    }

    return 1;
}

//create cmd server socket
int cmd_create_socket(const char *ip, int port) {
    assert(NULL != ip);
    struct sockaddr_in local_addr;
    if ((CMD_SOCKET_FD = socket(AF_INET, SOCK_DGRAM, 0)) < 0) {
        printf("create socket error: %s(errno: %d)\n", strerror(errno), errno);
        return -1;
    }
    memset(&local_addr, 0, sizeof(local_addr));
    local_addr.sin_family = AF_INET;
    local_addr.sin_port = htons(port);
    if (inet_aton(ip, &local_addr.sin_addr) == 0) {
        return -1;
    }
    if (bind(CMD_SOCKET_FD, (struct sockaddr *) &local_addr, sizeof(local_addr)) != 0) {
        printf("bind socket error: %s(errno: %d)\n", strerror(errno), errno);
        return -1;
    }
    return CMD_SOCKET_FD;
}

//UDP数据发送具备原子性:多线程同时对同一个fd收发线程安全
int cmd_send_data(const uint8_t *buf, int len, struct sockaddr_in *to_addr) {
    assert(NULL != buf && len > 0 && NULL != to_addr);
    int n;
    if ((n = (int) sendto(CMD_SOCKET_FD, buf, len, 0, (struct sockaddr *) to_addr, sizeof(struct sockaddr))) < 0) {
        printf("sendto error: %s(errno: %d)\n", strerror(errno), errno);
        return -1;
    }
    return n;
}

//UDP数据接收具备原子性:多线程同时对同一个fd收发线程安全
int cmd_recv_data(uint8_t *buf, int len, struct sockaddr_in *from_addr) {
    assert(buf != NULL && len > 0 && from_addr != NULL);
    int n;
    socklen_t from_len = sizeof(struct sockaddr_in);
    if ((n = (int) recvfrom(CMD_SOCKET_FD, buf, len, 0, (struct sockaddr *) from_addr, &from_len)) < 0) {
        printf("recv from error: %s(errno: %d)\n", strerror(errno), errno);
        return -1;
    }
    return n;
}

void cmd_init_remote(struct sockaddr_in *to_addr, uint16_t remote_port, const char *remote_ip) {
    assert(NULL != to_addr);
    to_addr->sin_family = AF_INET;
    to_addr->sin_port = htons(remote_port);
    to_addr->sin_addr.s_addr = inet_addr(remote_ip);
}


//发送消息

int cmd_send_ctl_status(struct CMD_MSG *msg, struct sockaddr_in *to_addr, char status) {
    assert(msg != NULL && to_addr != NULL);
    //发送接收
    int head_len = sizeof(struct CMD_MSG);
    uint8_t *buffer = malloc(head_len + 2);//包含crc和status
    struct CMD_MSG *send_msg = (struct CMD_MSG *) buffer;
    memcpy(buffer, msg, head_len);
    send_msg->head[0] = 0xFF;
    send_msg->head[1] = 0xAB;
    send_msg->data_len = 1;
    //type 和 ID 由接收命令决定
    buffer[head_len] = status;
    buffer[head_len + 1] = cmd_gen_crc(buffer, head_len + 2);
    int n = cmd_send_data(buffer, head_len + 2, to_addr);
    if (n <= 0) {
        printf("send rtk data error\n");
        char log_msg[50] = {0};
        snprintf(log_msg, 50, "send cmd status error,ID:%d", msg->ID);
        logger_write_message(MESSAGE_LOGGER_FILE, log_msg);

    }
    free(buffer);
    return n;
}


int cmd_send_status_respond(struct CMD_MSG *msg, struct sockaddr_in *to_addr, char status) {
    assert(msg != NULL && to_addr != NULL);
    int head_len = sizeof(struct CMD_MSG);
    uint8_t *buffer = malloc(head_len + msg->data_len + 2);//包含crc和status
    struct CMD_MSG *send_msg = (struct CMD_MSG *) buffer;
    memcpy(buffer, msg, head_len + msg->data_len);
    send_msg->head[0] = 0xFF;
    send_msg->head[1] = 0xAB;
    send_msg->data_len = msg->data_len + 1;
    //type 和 ID 由接收命令决定
    buffer[head_len + msg->data_len] = status;
    buffer[head_len + msg->data_len + 1] = cmd_gen_crc(buffer, head_len + msg->data_len + 2);
    int n = cmd_send_data(buffer, head_len + msg->data_len + 2, to_addr);
    if (n <= 0) {
        printf("send rtk data error\n");
        char log_msg[50] = {0};
        snprintf(log_msg, 50, "send cmd status error,ID:%d", msg->ID);
        logger_write_message(MESSAGE_LOGGER_FILE, log_msg);

    }
    free(buffer);
    return n;
}


//经度8BYTE 纬度8BYTE 高度8BYTE 方向4BYTE 速度4BYTE
int cmd_send_rtkdata(struct CMD_MSG *msg, struct sockaddr_in *to_addr, struct LLA *lla) {
    assert(msg != NULL && to_addr != NULL);
    uint16_t data_len = NULL == lla ? 0 : sizeof(struct LLA);
    int head_len = sizeof(struct CMD_MSG);
    uint8_t *buffer = malloc(head_len + data_len + 1);
    struct CMD_MSG *send_msg = (struct CMD_MSG *) buffer;
    memcpy(buffer, msg, head_len);
    send_msg->head[0] = 0xFF;
    send_msg->head[1] = 0xAB;
    send_msg->data_len = data_len;
    if (NULL != lla) {
        memcpy(buffer + head_len, lla, sizeof(struct LLA));
    }
    buffer[head_len + data_len] = cmd_gen_crc(buffer, head_len + data_len + 1);
    int n = cmd_send_data(buffer, head_len + data_len + 1, to_addr);
    free(buffer);
    if (n <= 0) {
        printf("send rtk data error\n");
        char log_msg[50] = {0};
        snprintf(log_msg, 50, "send rtk data error,ID:%d", msg->ID);
        logger_write_message(MESSAGE_LOGGER_FILE, log_msg);

    }
    return n;
}

int cmd_send_rtkdata_with_status(struct CMD_MSG *msg, struct sockaddr_in *to_addr, struct LLA *lla, char status) {
    assert(msg != NULL && to_addr != NULL);
    int data_len = NULL == lla ? 1 : sizeof(struct LLA) + 1;
    int head_len = sizeof(struct CMD_MSG);
    uint8_t *buffer = malloc(head_len + data_len + 1);
    struct CMD_MSG *send_msg = (struct CMD_MSG *) buffer;
    memcpy(buffer, msg, head_len);
    send_msg->head[0] = 0xFF;
    send_msg->head[1] = 0xAB;
    send_msg->data_len = data_len;
    if (lla) {
        memcpy(buffer + head_len, lla, sizeof(struct LLA));
    }
    buffer[head_len + data_len - 1] = status;
    buffer[head_len + data_len] = cmd_gen_crc(buffer, head_len + data_len + 1);
    int n = cmd_send_data(buffer, head_len + data_len + 1, to_addr);
    free(buffer);
    if (n <= 0) {
        printf("send rtk data error\n");
        char log_msg[50] = {0};
        snprintf(log_msg, 50, "send rtk data error,ID:%d", msg->ID);
        logger_write_message(MESSAGE_LOGGER_FILE, log_msg);
    }
    return n;
}


//大小端转换
uint16_t swap_uint16(uint16_t x) {
#if SWAP_BYTE
    return (x >> 8) | (x << 8);
#else
    return x;
#endif
}


uint32_t swap_uint32(uint32_t x) {
#if SWAP_BYTE
    x = ((x << 8) & 0xFF00FF00) | ((x >> 8) & 0xFF00FF);
    return (x << 16) | (x >> 16);
#else
    return x;
#endif
}






