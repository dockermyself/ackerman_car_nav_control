//
// Created by server on 2022/4/19.
//

#include "nema.h"
#include "utility/log.h"


// $GNGGA,045928.00,3039.13517200,N,12047.40621204,E,1,28,0.7,1.4130,M,9.8228,M,,*4E\r\n
// $GNRMC,030407.10,A,3039.12994399,N,12047.40797616,E,0.017,154.5,210422,5.9,W,D*34\r\n
// 返回0校验成功，否则失败
int nema_bbc_check_compare(const char *nema) {
    assert(nema != NULL);
    int sum = 0;
    int num = 0;
//    sscanf(nema, "%*[^*]*%x", &num);//提取到*，以*分割，提取数字
    int i;
    for (i = 1; nema[i] != '*'; ++i) {
        sum ^= nema[i];
    }
    sscanf(nema + i + 1, "%x", &num);//提取数字
    return sum - num;
}

int nema_bbc_check(const char *nema, int len) {
    assert(nema != NULL);
    int sum = 0;
    for (int i = 1; i < len; ++i) {
        sum ^= nema[i];
    }
    return sum;
}

int nema_bbc_check_match(const char *buffer, int *pbuffer_start, int package_end, char *head_code, int code_len) {
    assert(buffer != NULL && pbuffer_start != NULL && head_code != NULL && code_len > 0);
    int check_code = 0;
    while (*pbuffer_start < package_end) {
        if (sscanf(buffer + package_end + 1, "%x", &check_code) > 0 &&
            nema_bbc_check(buffer + *pbuffer_start, package_end - *pbuffer_start) == check_code) {
            return 1;
        }
        *pbuffer_start = nema_find_head(buffer, *pbuffer_start + 1, package_end, head_code, code_len);
    }
    printf("bbc check not match\n");
    return 0;
}

/*创建一个数组块存放分割的字符串
 * [str1]
 * [str2]
 * ......
 */
int nema_split(const char *nema, char *data, int STR_LENGTH, int data_size) {
    assert(nema != NULL && data != NULL && STR_LENGTH > 0 && data_size > 0);
    int begin = 0;
    int i;
    for (i = 0; nema[begin] != '*' && i < data_size; ++i) {
        begin++;
        sscanf(nema + begin, "%[^,*]", data + STR_LENGTH * i);
//        printf("%s\n", data + STR_LENGTH * i);
        int len = (int) strlen(data + STR_LENGTH * i);
        begin += len;

    }
    return i;
}


//$GNRMC,030407.10,A,3039.12994399,N,12047.40797616,E,0.017,154.5,210422,5.9,W,D*34\r\n
int nema_parse_rmc(const char *nema, struct Latitude *lat, struct Longitude *lon, double *speed) {
    assert(nema != NULL);
    const int array_size = GNRMC_CHECKSUM;
    const int STR_LENGTH = 20;
    char *data = malloc(STR_LENGTH * (array_size + 1));
    memset(data, 0, STR_LENGTH * (array_size + 1));
    //验证GNRMC有效性
    if (array_size != nema_split(nema, data, STR_LENGTH, array_size + 1)) {
        logger_warn("split rmc failed\n");
        free(data);
        return 0;
    }
    const char *str_lat = data + STR_LENGTH * GNRMC_LATITUDE;
    const char *str_lon = data + STR_LENGTH * GNRMC_LONGITUDE;
    if(strlen(str_lat) == 0 || strlen(str_lon) == 0) {
        logger_warn("lat or lon is null\n");
        free(data);
        return 0;
    }
    double lan_degree = atof(data + STR_LENGTH * GNRMC_LATITUDE);
    double lon_degree = atof(data + STR_LENGTH * GNRMC_LONGITUDE);

    lat->degree = (int) (lan_degree / 100);
    lat->minute = lan_degree - lat->degree * 100;
    lat->ns = data[STR_LENGTH * GNRMC_N_S];

    lon->degree = (int) (lon_degree / 100);
    lon->minute = lon_degree - lon->degree * 100;
    lon->ew = data[STR_LENGTH * GNRMC_E_W];

    *speed = strtod(data + STR_LENGTH * GNRMC_SPEED, NULL);
    free(data);
    return 1;

}

//$GNHDT,143.7366,T*19
int nema_parse_hdt(const char *nema, double *heading) {
    assert(nema != NULL);
    const int array_size = GNHDT_CHECKSUM;
    const int STR_LENGTH = 20;
    char *data = malloc(STR_LENGTH * (array_size + 1));
    memset(data, 0, STR_LENGTH * (array_size + 1));
    //验证GNHDT有效性
    if (array_size != nema_split(nema, data, STR_LENGTH, array_size + 1)) {
        logger_warn("split hdt failed\n");
        free(data);
        return 0;
    }
    const char *str_heading = data + STR_LENGTH * GNHDT_HEADING;
    if(strlen(str_heading) == 0) {
        logger_warn("heading is null\n");
        free(data);
        return 0;
    }
    *heading = atof(str_heading);
    free(data);
    return 1;
}

//$GNGGA,092320.000,2519.0490,N,11024.8391,E,1,23,0.7,175.7,M,0.0,M,*7D
int nema_parse_gga(const char *nema, struct LLA *outputs) {
    assert(nema != NULL);
    const int array_size = GNGGA_CHECKSUM;
    const int STR_LENGTH = 20;
    char *data = malloc(STR_LENGTH * (array_size + 1));
    memset(data, 0, STR_LENGTH * (array_size + 1));
    //验证GNRGGA有效性
    if (array_size != nema_split(nema, data, STR_LENGTH, array_size + 1)) {
        logger_warn("split gga failed\n");
        free(data);
        return 0;
    }
    const char *lat_str = data + STR_LENGTH * GNGGA_LATITUDE;
    const char *lon_str = data + STR_LENGTH * GNGGA_LONGITUDE;
    if (strlen(lat_str) == 0 || strlen(lon_str) == 0) {
        logger_warn("lat or lon is null\n");
        free(data);
        return 0;
    }
    if(outputs != NULL) {
        double lon = atof(data + STR_LENGTH * GNGGA_LONGITUDE);
        double lat = atof(data + STR_LENGTH * GNGGA_LATITUDE);

        outputs->lat = floor(lat / 100) + (lat - (int) (lat / 100) * 100) / 60.0;
        if (data[STR_LENGTH * GNRMC_N_S] == 'S') {
            outputs->lat = -outputs->lat;
        }
        outputs->lon = floor(lon / 100) + (lon - (int) (lon / 100) * 100) / 60.0;
        if (data[STR_LENGTH * GNRMC_E_W] == 'W') {
            outputs->lon = -outputs->lon;
        }
        outputs->alt = atof(data + STR_LENGTH * GNGGA_ALTITUDE);
    }

    free(data);
    return 1;
}

int nema_find_head(const char *buffer, int start, int end, char *head_code, int code_len) {
    assert(buffer != NULL);
    memset(head_code, 0, code_len);
    int dollar_index = -1;
    for (int i = start; i < end; ++i) {
        if (buffer[i] == '$') {
            dollar_index = i;
            for (int j = i + 1; j < end; ++j) {
                if (j - i > code_len || buffer[j] == '$') {//防止越界
                    logger_warn("nema head code is too long, serial message maybe mistake\n");
                    dollar_index = -1;//重新开始,不存在head大于code_len的情形
                    break;
                }
                else if (buffer[j] == ',') {
                    memcpy(head_code, buffer + i, j - i);
                    return i;
                }
            }
        }
    }
    if (dollar_index >= 0) {
        return dollar_index;
    }
    return end;
}

int nema_find_tail(const char *buffer, int start, int end) {
    assert(buffer != NULL);
    //*后面XX0x0D,0x0A
    const int extra_len = 4;
    for (int i = start; i < end - extra_len; ++i) {
        if (buffer[i] == '*') {
            return i;
        }
    }
    return end;
}

void nema_format_data(char *buffer, int start, int end, char *nema_data, int *len) {
    assert(buffer != NULL && nema_data != NULL && len != NULL);
    *len = end - start;
    memcpy(nema_data, buffer + start, *len);
    //添加/r/n
    nema_data[*len] = '\r';
    nema_data[*len + 1] = '\n';
    *len = *len + 2;
}

double get_latitude(const struct Latitude *lat) {//获取纬度
    assert(lat != NULL);
    double degree = lat->degree + lat->minute / 60.0;
    if (lat->ns == 'S') {
        degree = -degree;
    }
    return degree;
}


double get_longitude(const struct Longitude *lon) {//获取经度
    assert(lon != NULL);
    double degree = lon->degree + lon->minute / 60.0;
    if (lon->ew == 'W') {
        degree = -degree;
    }
    return degree;
}

//LLA TO ECEF
void lla_to_ecef(double lat, double lon, double alt, double *x, double *y, double *z) {
    assert(x != NULL && y != NULL && z != NULL);
    const double a = 6378137.0;     //WGS-84椭球体长半轴
    const double b = 6356752.314245;//WGS-84椭球体短半轴
    double e2 = 1 - (b * b) / (a * a);//WGS-84椭球体第一偏心率的平方
    double N = a / sqrt(1 - e2 * pow(sin(lat), 2));//WGS-84椭球体卯酉圈半径
    *x = (N + alt) * cos(lat) * cos(lon);//ECEF坐标系X轴坐标
    *y = (N + alt) * cos(lat) * sin(lon);//ECEF坐标系Y轴坐标
    *z = (N * (1 - e2) + alt) * sin(lat);//ECEF坐标系Z轴坐标
}

//LLA TO ENU
void lla_to_enu(struct LLA *lla1, struct LLA *lla2, double *de, double *dn, double *du) {
    assert(lla1 != NULL && lla2 != NULL && de != NULL && dn != NULL && du != NULL);
    const double a = 6378137.0;     //WGS-84椭球体长半轴
    const double b = 6356752.314245;//WGS-84椭球体短半轴
    const double R = (a + b) / 2;
    *de = (lla2->lon - lla1->lon) * M_PI / 180 * R * cos(lla1->lat);
    *dn = (lla2->lat - lla1->lat) * M_PI / 180 * R;
    *du = (lla2->alt - lla1->alt);
}

//LLA TO UTM
void lla_to_utm(struct LLA *lla, double *x, double *y) {
    assert(lla != NULL && x != NULL && y != NULL);
    const double a = 6378137.0;     //WGS-84椭球体长半轴
    const double b = 6356752.314245;//WGS-84椭球体短半轴
    const double R = (a + b) / 2;
    *x = lla->lon * M_PI / 180 * R;
    *y = lla->lat * M_PI / 180 * R;
}







