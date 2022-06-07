//
// Created by server on 2022-05-18.
//

#ifndef RTK_NAV_LOG_H
#define RTK_NAV_LOG_H
#include <stdio.h>
#include <stdint.h>

struct LogNode {
    char path[20];
    FILE * logger;
    struct LogNode * next;
};

void logger_warn(const char *__restrict _format, ...);
void logger_error(const char *__restrict _format, ...);
void logger_info(const char *__restrict _format, ...);
void logger_write_bytes(const char *file_name, const uint8_t *buffer, int n);
void logger_print_bytes( const uint8_t *buffer, int n);
void logger_write_message(const char *path,const char *message);
FILE* open_logger(const char *path);
void close_logger(const char *path);
void logger_important(const char *__restrict _format, ...);

#endif //RTK_NAV_LOG_H
