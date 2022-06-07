//
// Created by server on 2022/4/19.
//

#ifndef RTK_NAV_NEMA_H
#define RTK_NAV_NEMA_H

#include "include/header.h"
/*$GNRMC,030407.10,A,3039.12994399,N,12047.40797616,E,0.017,154.5,210422,5.9,W,D*34\r\n
 <1> UTC（Coordinated Universal Time）时间，hhmmss（时分秒）格式
 <2> 定位状态，A=有效定位，V=无效定位
 <3> Latitude，纬度 ddmm.mmmm（度分）格式（前导位数不足则补 0）
 <4> 纬度半球 N（北半球）或 S（南半球）
 <5> Longitude，经度 dddmm.mmmm（度分）格式（前导位数不足则补 0）
 <6> 经度半球 E（东经）或 W（西经）
 <7> 地面速率（000.0~999.9 节，Knot，前导位数不足则补 0）
 <8> 地面航向（000.0~359.9 度，以真北为参考基准，前导位数不足则补 0）
 <9> UTC 日期，ddmmyy（日月年）格式
 <10> Magnetic Variation，磁偏角（000.0~180.0 度，前导位数不足则补 0）
 <11> Declination，磁偏角方向，E（东）或 W（西）
 <12> Mode Indicator，模式指示（仅 NMEA0183 3.00 版本输出，A=自主定位，D=差分，E=估算，N=数据无效）
 <13> 校验和
 */

enum GNRMC{
    GNRMC_TIME = 1,   //卫星时间。格式为hhmmss.毫秒
    GNRMC_STATUS = 2,    //定位状态：A=定位，V=未定位
    GNRMC_LATITUDE = 3,//纬度。格式为ddmm.mmmm，前导位数不足则补0。
    GNRMC_N_S = 4,//北纬或南纬。N为北纬，S为南纬
    GNRMC_LONGITUDE = 5,//经度。格式为dddmm.mmmm，前导位数不足则补0。
    GNRMC_E_W = 6,//东经或西经。E为东经，W为西经
    GNRMC_SPEED = 7,//速度。格式为000.0，前导位数不足则补0。
    GNRMC_COURSE = 8,//航向。格式为000.0，前导位数不足则补0。
    GNRMC_DATE = 9,//定位日期。格式为ddmmyy，前导位数不足则补0。
    GNRMC_MAGNETIC_VARIATION = 10,//磁偏角。格式为000.0，前导位数不足则补0。
    GNRMC_MAGNETIC_VARIATION_E_W = 11,//磁偏角方向。E为东，W为西
    GNRMC_MODE_INDICATOR = 12,//模式指示。仅 NMEA0183 3.00 版本输出，A=自主定位，D=差分，E=估算，N=数据无效
    GNRMC_CHECKSUM = 13,//校验和
};
/*$GNGGA,073146.00,3039.13926086,N,12047.40511997,E,1,28,0.6,6.1538,M,9.8227,M,,*48\r\n"
<1> UTC 时间，hhmmss（时分秒）格式 和GNZDA的UTC一样，属于协调世界时
<2> 纬度 ddmm.mmmm（度分）格式（前面的 0 也将被传输）
<3> 纬度半球 N（北半球）或 S（南半球）
<4> 经度 dddmm.mmmm（度分）格式（前面的 0 也将被传输）
<5> 经度半球 E（东经）或 W（西经）
<6> GPS 状态：0=未定位，1=非差分定位，2=差分定位，6=正在估算
<7> 正在使用解算位置的卫星数量（00~12）（前面的 0 也将被传输）
<8> HDOP 水平精度因子（0.5~99.9）
<9> 海拔高度（‐9999.9~99999.9）
<10> 地球椭球面相对大地水准面的高度
<11> 差分时间（从最近一次接收到差分信号开始的秒数，如果不是差分定位将为空）
<12> 差分站 ID 号 0000~1023（前面的 0 也将被传输，如果不是差分定位将为空）
 */
enum GNGGA{
    GNGGA_TIME = 1,   //卫星时间。格式为hhmmss.毫秒
    GNGGA_LATITUDE = 2,//纬度。格式为ddmm.mmmm，前导位数不足则补0。
    GNGGA_N_S = 3,//北纬或南纬。N为北纬，S为南纬
    GNGGA_LONGITUDE = 4,//经度。格式为dddmm.mmmm，前导位数不足则补0。
    GNGGA_E_W = 5,//东经或西经。E为东经，W为西经
    GNGGA_FIX_QUALITY = 6,//定位状态：0=未定位，1=非差分定位，2=差分定位，6=正在估算
    GNGGA_SATELLITES_NUM = 7,//正在使用解算位置的卫星数量（00~12）（前面的 0 也将被传输）
    GNGGA_HDOP = 8,//水平精度因子（0.5~99.9）
    GNGGA_ALTITUDE = 9,//海拔高度（‐9999.9~99999.9）
    GNGGA_GEOID_HEIGHT = 11,//地球椭球面相对大地水准面的高度
    GNGGA_DIFF_TIME = 13,//差分时间（从最近一次接收到差分信号开始的秒数，如果不是差分定位将为空）
    GNGGA_DIFF_STATION_ID = 14,//差分站 ID 号 0000~1023（前面的 0 也将被传输，如果不是差分定位将为空）
    GNGGA_CHECKSUM = 15,//校验和
};


enum GNHDT{
    GNHDT_HEADING = 1,//航向。格式为000.0，前导位数不足则补0。
    GNHDT_TURN = 2,   //ndicates heading relative to True North
    GNHDT_CHECKSUM = 3,//校验和
};

#pragma pack(1)
struct Latitude {//纬度
    int degree;
    double minute;
    uint8_t ns;
};
struct Longitude {//经度
    int degree;
    double minute;
    uint8_t ew;
};
//WGS-84坐标系
struct LLA {
    double lon; //经度，纬度值在-180°到+180°之间。北半球为正，南半球为负。
    double lat; //纬度，经度值在-90°到+90°之间。东经为正，西经为负。
    double alt; //基准椭球面以内为负，以外为正,单位米
    float head; //航向角,0-360°，顺时针为正，北向0度
    float speed;//m/s
};
#pragma pack()

double get_latitude(const struct Latitude *lat);
double get_longitude(const struct Longitude *lon);
void lla_to_ecef(double lat, double lon, double alt, double *x, double *y, double *z);
void lla_to_enu(struct LLA* lla1 ,struct LLA* lla2,double *de, double *dn, double *du);
void lla_to_utm(struct LLA* lla, double *x, double *y);
int nema_bbc_check_compare(const char *nema);
int nema_bbc_check(const char *nema,int len);
int nema_parse_rmc(const char *nema, struct Latitude *lat, struct Longitude *lon,double *speed) ;
int nema_parse_hdt(const char *nema, double *heading);
int nema_parse_gga(const char *nema, struct LLA *outputs);
int nema_find_head(const char *buffer, int start, int end, char* head,int head_len);
int nema_find_tail(const char *buffer, int start, int end);
void nema_format_data(char *buffer, int start, int end, char *nema_data, int *len);
int nema_bbc_check_match(const char *buffer, int *pbuffer_start, int package_end, char *head_code, int code_len);
#endif //RTK_NAV_NEMA_H
