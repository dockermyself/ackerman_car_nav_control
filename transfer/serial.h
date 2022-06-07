//
// Created by server on 2022/4/18.
//

#ifndef RTK_NAV_SERIAL_H
#define RTK_NAV_SERIAL_H

typedef struct
{
    unsigned char databit, parity, stopbit, flowctrl;
    char dev_name[64];
    int fd;
    int frag_size;
    unsigned long baudrate;
} comport_t;


typedef struct {
    char dev_name[10];
    comport_t* port;
}serial_map_t;



void serial_close(const char *dev_name);
int serial_init(const char *dev_name, unsigned long baudrate);
int serial_read(const char *dev_name, char *buffer, int size);
int serial_write(const char *dev_name, const char *buffer, int size);
#endif //RTK_NAV_SERIAL_H
