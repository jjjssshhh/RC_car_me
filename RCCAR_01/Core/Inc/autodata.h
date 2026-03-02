#ifndef AUTODATA_H
#define AUTODATA_H

#include <stdint.h>

typedef struct {
    int32_t flag;
    int32_t sonic_L, sonic_M, sonic_R;
    int32_t v_l, v_r;
    int32_t interval;
} DriveStep;



//첫번쨰
//#define TRACK_DATA_SIZE 284
//두번째
#define TRACK_DATA_SIZE 271

#endif
