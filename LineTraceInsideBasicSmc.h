// 2012/08/13 created by s.sahara

// IN���x�[�V�b�N���s��ԑJ�ڃ}�V��

#include "ecrobot_interface.h"

// IN�R�[�X�x�[�V�b�NSeq
#define LT_IN1_SEQ_00	0
#define LT_IN1_SEQ_01	1
#define LT_IN1_SEQ_02	2
#define LT_IN1_SEQ_03	3
#define LT_IN1_SEQ_04	4
#define LT_IN1_SEQ_05	5
#define LT_IN1_SEQ_06	6
#define LT_IN1_SEQ_07	7
#define LT_IN1_SEQ_08	8
#define LT_IN1_SEQ_09	9
#define LT_IN1_SEQ_10	10
#define LT_IN1_SEQ_11	11
#define LT_IN1_SEQ_12	12
#define LT_IN1_SEQ_13	13
#define LT_IN1_SEQ_MAX	14


extern void LineTraceInsideBasicSmc_Init(signed char forward);
extern signed char LineTraceInsideBasicSmc_GetForward();

extern int LineTraceInsideBasicSmc_Smc(UINT seqNo, float distanceAbs, int* courseJointDetected, float* distanceCorrection);

//extern void LineTraceInsideBasicSmc_BTSendLapTime();
