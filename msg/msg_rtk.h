//
// Created by server on 2022/4/27.
//

#ifndef RTK_NAV_MSG_RTK_H
#define RTK_NAV_MSG_RTK_H

#include "include/header.h"
#include "nema.h"

enum RTK_DATA_STATUS {
    RTK_EMPTY = 0x00,
    RTK_GNRMC = 0x01,
    RTK_GNHDT = 0x02,
};

#pragma pack(1)
struct RTK_MSG {
    enum RTK_DATA_STATUS status;
    struct LLA lla;
};
#pragma pack()

void msg_rtk_init();
void msg_rtk_update(const struct Longitude* lon,const struct Latitude* lat,const double * speed,const double * heading);
void msg_rtk_get(struct RTK_MSG* msg);
void msg_rtk_destroy();
#endif //RTK_NAV_MSG_RTK_H
