// 2012/08/14 created by s.sahara

// OUT側ボーナス走行状態遷移マシン

#include "ecrobot_interface.h"

// OUTコース後半Seq
#if 0
#define LT_OUT2_SEQ_00		0
#define LT_OUT2_SEQ_01		1
#define LT_OUT2_SEQ_02		2
#define LT_OUT2_SEQ_03		3
#define LT_OUT2_SEQ_04		4
#define LT_OUT2_SEQ_04_1	5
#define LT_OUT2_SEQ_04_2	6
#define LT_OUT2_SEQ_05		7
#define LT_OUT2_SEQ_06		8
#define LT_OUT2_SEQ_07		9
#define LT_OUT2_SEQ_08		10
#define LT_OUT2_SEQ_09		11
#define LT_OUT2_SEQ_MAX		12
#else
#define LT_OUT2_SEQ_00		0
#define LT_OUT2_SEQ_00_1	1
#define LT_OUT2_SEQ_01		2
#define LT_OUT2_SEQ_01_1	3
#define LT_OUT2_SEQ_02		4
#define LT_OUT2_SEQ_02_1	5
#define LT_OUT2_SEQ_03		6
#define LT_OUT2_SEQ_03_1	7
#define LT_OUT2_SEQ_03_2	8
#define LT_OUT2_SEQ_04		9
#define LT_OUT2_SEQ_05		10
#define LT_OUT2_SEQ_06		11
#define LT_OUT2_SEQ_MAX		12
#endif


extern void LineTraceOutsideBonusSmc_Init(signed char arg_forward);
extern int LineTraceOutsideBonusSmc_Smc(UINT seqNo, float distanceAbs, int* courseJointDetected, float* distanceCorrection);

// extern void LineTraceOutsideBonusSmc_BTSendLapTime();
