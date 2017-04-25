// 2012/07/21 created by s.sahara

#include "LineTraceControl.h"

extern int sonar_alert(int alert_distance);

extern void LineTrace_Init();
extern int LineTraceMain(int courseSide, int part, float distanceAbs, int* detectedStageId);

// 2012/08/12 add by t.akiyama 段差検知 >>>>
extern void BalanceControlTurn(char forward, char turn, char rotate_power, char rotate_LR);
// 2012/08/12 add by t.akiyama <<<<

extern void LineTrace_BTSendLapTime(); // IN側ベーシックコースSEQ区間タイムBT送信 2012/08/13 add by s.sahara
