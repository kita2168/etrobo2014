// 2012/08/14 created by s.sahara

// OUT側ベーシック走行状態遷移マシン

#include "ecrobot_interface.h"

// OUTコースベーシックSeq
#define LT_OUT1_SEQ_00	0
#define LT_OUT1_SEQ_01	1
#define LT_OUT1_SEQ_02	2
#define LT_OUT1_SEQ_03	3
#define LT_OUT1_SEQ_04	4
#define LT_OUT1_SEQ_05	5
#define LT_OUT1_SEQ_06	6
#define LT_OUT1_SEQ_07	7
#define LT_OUT1_SEQ_08	8
#define LT_OUT1_SEQ_09	9
#define LT_OUT1_SEQ_10	10
#define LT_OUT1_SEQ_11	11
#define LT_OUT1_SEQ_MAX	12


extern void LineTraceOutsideBasicSmc_Init(signed char forward);
extern signed char LineTraceOutsideBasicSmc_GetForward();

extern int LineTraceOutsideBasicSmc_Smc(UINT seqNo, float distanceAbs, int* courseJointDetected, float* distanceCorrection);

//extern void LineTraceOutsideBasicSmc_BTSendLapTime();
