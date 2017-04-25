// 2012/08/13 created by s.sahara

// IN側ボーナス走行状態遷移マシン

#include "ecrobot_interface.h"

// INコース後半Seq
#if 0
#define LT_IN2_SEQ_00	0
#define LT_IN2_SEQ_01	1
#define LT_IN2_SEQ_02	2
#define LT_IN2_SEQ_03	3
#define LT_IN2_SEQ_04	4
#define LT_IN2_SEQ_04_1	5
#define LT_IN2_SEQ_04_2	6
#define LT_IN2_SEQ_04_3	7
#define LT_IN2_SEQ_05	8
#define LT_IN2_SEQ_MAX	18
#else
#define LT_IN2_SEQ_00		0
#define LT_IN2_SEQ_00_1		1
#define LT_IN2_SEQ_01		2
#define LT_IN2_SEQ_02		3
#define LT_IN2_SEQ_02_1		4
#define LT_IN2_SEQ_03		5
#define LT_IN2_SEQ_04		6
#define LT_IN2_SEQ_05		7
#define LT_IN2_SEQ_06		8
#define LT_IN2_SEQ_MAX		9
#endif


extern void LineTraceInsideBonusSmc_Init(signed char forward);
extern int LineTraceInsideBonusSmc_Smc(UINT seqNo, float distanceAbs, int* courseJointDetected, float* distanceCorrection);

//extern void LineTraceInsideBonusSmc_BTSendLapTime();
