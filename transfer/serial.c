//
// Created by server on 2022/4/18.
//

#include "serial.h"
#include "utility/log.h"
#include "include/header.h"
/*
 * 用户维护COM_PORT对象，支持多个串口调用,初始化传入COM_PORT **,ComportInit构造内存
 */

#define MAX_PORT_NUM 4
static int COMPORT_USE = 0;
serial_map_t COMPORT_MAP[MAX_PORT_NUM] = {0};


void comport_set(comport_t *comport, const char *settings);
comport_t *comport_init(const char *dev_name, unsigned long baudrate, const char *settings);
void comport_release(comport_t** comport);
void comport_close(comport_t *comport);

comport_t *comport_init(const char *dev_name, unsigned long baudrate, const char *settings)
{
    comport_t *comport = NULL;
    if (NULL == (comport = (comport_t *)malloc(sizeof(comport_t)))) //分配空间
    {
        return NULL;
    }
    memset(comport, 0, sizeof(comport_t)); //初始化0
    comport->frag_size = 128;

    strncpy(comport->dev_name, dev_name, 64); //将dev_name中的前64个字符复制到comport->dev_name
    comport->baudrate = baudrate;

    comport_set(comport, settings);
    return comport;
}
void comport_release(comport_t** comport){
    if(comport==NULL){
        return;
    }
    if(*comport == NULL){
        return;
    }

    free(*comport);
    *comport = NULL;
}

void comport_set(comport_t *comport, const char *settings) //settings 8N1
{
    if (NULL == comport || NULL == settings)
        return;

    switch (settings[0]) /* databit */
    {
        case '7':
            comport->databit = 7;
            break;
        case '8':
            comport->databit = 8;
            break;
        default:
            comport->databit = 8;
            break;
    }

    switch (settings[1]) /*parity check*/
    {
        case 'O': /*Odd check*/
        case 'o':
            comport->parity = 1;
            break;
        case 'E': /* Even check */
        case 'e':
            comport->parity = 2;
            break;
        case 'N': /* None parity check */
        case 'n':
        default:
            comport->parity = 0;
            break;
    }

    switch (settings[2]) /*stop bit*/
    {
        case '1':
            comport->stopbit = 1;
            break;
        case '2':
        default:
            comport->stopbit = 2;
            break;
    }
}

void comport_close(comport_t *comport)
{
    if (0 != comport->fd)
        close(comport->fd);
    comport->fd = -1;
}

int comport_open(comport_t *comport) {
    int retval = -1;
    struct termios old_cfg, new_cfg;
    long temp;

    if (NULL == comport)
        return -1;

    comport_close(comport);

    /* not a TTY device */
    if (!strstr(comport->dev_name, "tty")) //查找dev_name中是否有tty字符
    {
        printf("open not tty device \"%s\".\n", comport->dev_name);
        comport->fd = open(comport->dev_name, O_RDWR);
        retval = comport->fd < 0 ? -2 : comport->fd;
        goto CleanUp;
    }
    //comport->fd = open(comport->dev_name, O_RDWR | O_NOCTTY | O_NONBLOCK);
    comport->fd = open(comport->dev_name, O_RDWR);//阻塞IO
    if (comport->fd < 0)
    {
        retval = -3;
        goto CleanUp;
    }

    if (0 != tcgetattr(comport->fd, &old_cfg))
    {
        retval = -4; /* failed to get com settings*/
        goto CleanUp;
    }

    memset(&new_cfg, 0, sizeof(new_cfg));

    /*  Configure comport */

    new_cfg.c_cflag &= ~CSIZE; /*character length is 0 */
    new_cfg.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    new_cfg.c_iflag &= ~(BRKINT | ICRNL | INPCK | ISTRIP | IXON);
    new_cfg.c_oflag &= ~(OPOST);

    new_cfg.c_cflag |= CLOCAL;
    new_cfg.c_cflag |= CREAD;

    /* set the databit */
    switch (comport->databit)
    {
        case 0x07:
            new_cfg.c_cflag |= CS7;
            break;
        case 0x08:
        default:
            new_cfg.c_cflag |= CS8;
            break;
    }

    /* set the parity */
    switch (comport->parity)
    {
        case 0x01: //Odd check
            new_cfg.c_cflag |= PARENB;
            new_cfg.c_iflag |= (INPCK | ISTRIP);
            new_cfg.c_cflag |= PARODD;
            break;
        case 0x02: //Even check
            new_cfg.c_cflag |= PARENB;
            new_cfg.c_iflag |= (INPCK | ISTRIP);
            new_cfg.c_cflag &= ~PARODD;
            break;
        case 0x00:
        default:
            new_cfg.c_cflag &= ~PARENB;
            break;
    }

    /*  set stop bit */
    if (0x01 != comport->stopbit)
        new_cfg.c_cflag != CSTOPB;
    else
        new_cfg.c_cflag &= ~CSTOPB;

    switch (comport->baudrate)
    {
        case 921600:
            temp = B921600;
            break;
        case 460800:
            temp = B460800;
            break;
        case 230400:
            temp = B230400;
            break;
        case 115200:
            temp = B115200;
            break;
        case 9600:
            temp = B9600;
            break;
        case 4800:
            temp = B4800;
            break;
        case 2400:
            temp = B2400;
            break;
        case 1800:
            temp = B1800;
            break;
        case 1200:
            temp = B1200;
            break;
        default:
            temp = B115200;
            break;
    }
    cfsetispeed(&new_cfg, temp);
    cfsetospeed(&new_cfg, temp);

    /* Set the com port timeout settings  */
    new_cfg.c_cc[VMIN] = 0;
    new_cfg.c_cc[VTIME] = 0;
    tcflush(comport->fd, TCIFLUSH);

    if (0 != tcsetattr(comport->fd, TCSANOW, &new_cfg))
    {
        retval = -5; //failed to set device comport settings
        goto CleanUp;
    }

    retval = comport->fd;

    CleanUp:
    printf("open device \"%s\" %s.\n", comport->dev_name, retval > 0 ? "successfully" : "failure");
    return retval;
}

void serial_close(const char *dev_name){
    if(dev_name == NULL){
        return;
    }
    comport_t *comport = NULL;
    for(int i = 0;i < MAX_PORT_NUM;++i){
        if(strcmp(dev_name,COMPORT_MAP[i].dev_name) == 0){
            comport = COMPORT_MAP[i].port;
            COMPORT_MAP[i].port = NULL;
            break;
        }
    }
    if(comport == NULL){
        return;
    }
    COMPORT_USE--;
    comport_close(comport);
    comport_release(&comport);
}


int serial_init(const char *dev_name, unsigned long baudrate)
{
    if(dev_name == NULL || baudrate == 0 || COMPORT_USE >= MAX_PORT_NUM){
        return -1;
    }
    //check if the device is already open
    for(int i = 0;i < MAX_PORT_NUM;++i){
        if(strcmp(dev_name,COMPORT_MAP[i].dev_name) == 0){
            return COMPORT_MAP[i].port->fd;
        }
    }
    comport_t *comport = comport_init(dev_name, baudrate, "8N1");
    if (NULL == comport)
    {
        printf("init serial port failure\n");
        return -1;
    }

    comport_open(comport);
    if (comport->fd < 0)
    {
        printf("open error :%s\n", strerror(errno));
        return -1;
    }

    //add to map
    for(int i = 0;i < MAX_PORT_NUM;++i){
        if(COMPORT_MAP[i].port == NULL){
            COMPORT_MAP[i].port = comport;
            strcpy(COMPORT_MAP[i].dev_name,dev_name);
            break;
        }
    }
    COMPORT_USE++;
    return comport->fd;
}

int serial_read(const char *dev_name, char *buffer, int size)
{
    if(dev_name == NULL || buffer == NULL || size == 0){
        return -1;
    }
    comport_t *comport = NULL;
    for(int i = 0; i < MAX_PORT_NUM; i++){
        if(strcmp(COMPORT_MAP[i].dev_name, dev_name) == 0){
            comport = COMPORT_MAP[i].port;
            break;
        }
    }
    if(comport == NULL){
        return -1;
    }
    int n = (int)read((comport)->fd, buffer, size);
    if(n < 0)
    {
        if(errno == EAGAIN)
            return 0;
        else
            return n;
    }
    return n;
}

int serial_write(const char *dev_name, const char *buffer, int size)
{
    if(dev_name == NULL || buffer == NULL || size == 0){
        return -1;
    }
    comport_t *comport = NULL;
    for(int i = 0; i < MAX_PORT_NUM; i++){
        if(strcmp(COMPORT_MAP[i].dev_name, dev_name) == 0){
            comport = COMPORT_MAP[i].port;
            break;
        }
    }
    if(comport == NULL){
        logger_write_message(MESSAGE_LOGGER_FILE,"comport is null");
    }

    int n = (int)write(comport->fd, buffer, size);
    if(n < 0)
    {
        if(errno == EAGAIN)
            return 0;
        else
            return n;
    }
    return n;
}
