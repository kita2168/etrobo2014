// 2012/08/13 created by s.sahara

//#ifdef USE_SAMPLE_COURCE

#include "ecrobot_interface.h"

#define LT_TEST_SEQ_MAX	10

extern void LineTraceTestSmc_Init(signed char forward);
extern int LineTraceTestSmc_Smc(UINT seqNo, float distanceAbs, int* courseJointDetected, float* distanceCorrection);

extern void LineTraceTestSmc_BTSendLapTime();

//#endif // #ifdef USE_SAMPLE_COURCE
