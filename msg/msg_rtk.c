//
// Created by server on 2022/4/27.
//

#include "msg_rtk.h"
#include "utility/log.h"

static pthread_mutex_t lock = PTHREAD_MUTEX_INITIALIZER;
static struct RTK_MSG RTK_MUTEX_MSG = {0};



void msg_rtk_init() {
    memset(&RTK_MUTEX_MSG, 0, sizeof(struct RTK_MSG));
    RTK_MUTEX_MSG.status = RTK_EMPTY;
    pthread_mutex_init(&lock, NULL);
}

void msg_rtk_update(const struct Longitude *lon, const struct Latitude *lat,
                    const double *speed, const double *heading) {
    pthread_mutex_lock(&lock);
    //更新数据
    if (lon && lat && speed) {//必须先获取RMC报文
        RTK_MUTEX_MSG.status |= RTK_GNRMC;
        RTK_MUTEX_MSG.lla.lon = get_longitude(lon);
        RTK_MUTEX_MSG.lla.lat = get_latitude(lat);
        RTK_MUTEX_MSG.lla.alt = 0.0;
        RTK_MUTEX_MSG.lla.speed = (float) ((*speed) * 0.514444);//单位：米/秒
    }


    if (heading) {
        RTK_MUTEX_MSG.lla.head = (float) *heading;
        RTK_MUTEX_MSG.status |= RTK_GNHDT;
    }
    pthread_mutex_unlock(&lock);
}
//直接获取RTK_MUTEX_MSG
void msg_rtk_get(struct RTK_MSG *msg) {
    assert(msg != NULL);
    pthread_mutex_lock(&lock);
    memcpy(msg, &RTK_MUTEX_MSG, sizeof(struct RTK_MSG));
    pthread_mutex_unlock(&lock);
}


void msg_rtk_destroy() {
    pthread_mutex_destroy(&lock);
}

