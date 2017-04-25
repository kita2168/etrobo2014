/* 2013/08/17 created by k.nakagawa bluetoothでデータログ出力する */

#include "ecrobot_interface.h"
#include "CourseDef.h"

#ifndef USE_1MS_BT_LOG
#define DATA_LOG_BUFFER_LEN 0 //使わない時用
#else
#if 1
#define DATA_LOG_BUFFER_LEN 10000 //6byte×1000ms
#else
#define DATA_LOG_BUFFER_LEN 13312 //(64-51)Kbytes×1024bytes=13312
#endif
#endif

extern void bt_data_log_init(void);
extern S8 bt_data_log_append(U8 dataID, S8 Data1, S8 Data2, U16 lightIn);
extern S8 bt_data_log_send(void);
