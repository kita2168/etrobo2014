/* 2012/07/21 created by s.sahara 難所クリア単体テスト用 */

extern int LookUpGateStageMain();
extern int TailAngleCheck(int reqAngle);
extern void LookupGate_tail_control(signed int angle);

/* 尻尾の角度定義 */
#define GATE_TAIL_ANGLE_UNDER_GATE	65	// 完全停止時の角度[度]
#define GATE_TAIL_ANGLE_DRIVE		TAIL_ANGLE_TAILRUN	// バランス走行時の角度[度]
#define GATE_TAIL_ANGLE_SONAR		100
