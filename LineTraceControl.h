// 2012/08/13 created by s.sahara

// ライントレース共通制御処理

#include "ecrobot_interface.h"
#include "balancer.h" /* 倒立振子制御用ヘッダファイル */

/* ジャイロセンサオフセット値(角速度0[deg/sec]時) */
#ifdef STEC_EAST // 個体差切り替え
#define GYRO_OFFSET  605
#else
#define GYRO_OFFSET  608
#endif
#define FALL_DETECTION_ANLE_RATE 200 // 転倒検知平均角速度(level/sec)	…実測値を元に設定する。ここで使うのは平均角速度なので、検知を鈍らせたければ平均回数を増やすこと。ただしメモリ使用量に注意。絶対角度は安定して測定できない。

// ↓キャリブレーションした値を使用すること（変数の初期化やデバッグでのみ使用可）
#define LIGHT_WHITE	 610 /* 白色の光センサ値 */	// 2012/08/12 mod by s.sahara 白側曲線ゆるすぎる為、黒側へ少しシフト。※ライン横切り測定して波形見て決定すること。
//#define LIGHT_WHITE	 580 /* 白色の光センサ値 */
#define LIGHT_BLACK	 680 /* 黒色の光センサ値 */ // 2012/08/12 mod by s.sahara 速度上げるとコーナでアウトに膨らんで線中心乗り越えてしまうため下げる。※ライン横切り測定して波形見て決定すること。
//#define LIGHT_BLACK	 700 /* 黒色の光センサ値 */
// ↑キャリブレーションした値を使用すること（変数の初期化やデバッグでのみ使用可）

#define SONAR_ALERT_DISTANCE 20 /* 超音波センサによる障害物検知距離[cm] */
#define TAIL_ANGLE_STAND_UP 108 /* 完全停止時の角度[度] */
#define TAIL_ANGLE_DRIVE	  3 /* バランス走行時の角度[度] */
#define P_GAIN			   2.5F /* 完全停止用モータ制御比例係数 */
#define PWM_ABS_MAX 		 60 /* 完全停止用モータ制御PWM絶対最大値 */

// 高速進入マーカ検知（速度を出してマーカ通過させ、光センサ入力レベル変化率の閾値で検知）
// マーカ検知レベル（グレー基準値と入力の偏差の、前回との差に対する閾値）
// ※↓倒立走行速度80，光センサ入力３回平均で調整。速度変更時は要調整。
// ↓キャリブレーションした値を使用すること（変数の初期化やデバッグでのみ使用可）
#define LIGHT_STANDARD ((LIGHT_WHITE + LIGHT_BLACK) / 2.0F)	// 白と黒の中間がマーカとしておく
// ↑キャリブレーションした値を使用すること（変数の初期化やデバッグでのみ使用可）
#define MARKER_ON_THRESHOLD_LEVEL (-0.4f)	// [%] OUTヘアピン後spd40で検知できないので使用場所注意
//#define MARKER_ON_THRESHOLD_LEVEL (-0.8f)	// [%] 尻尾走行 spd80で検知できず
#define MARKER_OFF_THRESHOLD_LEVEL (0.2f)	// [%]

// 低速進入マーカ検知（速度が出せないヘアピン立ち上がりのマーカ等で使用）
#define LOWSPEED_MARKER_DETECTION__SPD (20) // [%]
// ↓キャリブレーションした値の相対値を使用すること
#define LOWSPEED_MARKER_DETECTION__LIGHT_THRESHOLD (635) // 黒→グレー，グレー→黒レベル判定用光センサ入力レベル閾値 ※補正必要
// ↑キャリブレーションした値の相対値を使用すること
#define LOWSPEED_MARKER_DETECTION__LIGHT_THRESHOLD_P (0.5f) // グレーの明るさの、白黒幅に対する比率
#define MARKER_IN_DETECTION (-15.0f)
#define MARKER_OUT_DETECTION (25.0f)

extern void BalanceControl(char forward, char turn);
extern void BalanceControl1(char forward, char turn, float addGyroOffset);
extern void TailRunControl(char forward, char turn);

extern void LineTraceControl_InitLightThreshold(U16 lightBlack, U16 lightWhite);
extern U16 LineTraceControl_LightBlackThreshold();
extern U16 LineTraceControl_LightWhiteThreshold();
extern void LineTraceControl_InitLightThreshold2(U16 lightBlack, U16 lightWhite);
extern U16 LineTraceControl_LightBlackThreshold2();
extern U16 LineTraceControl_LightWhiteThreshold2();

//光範囲計算用
extern U16 LineTraceControl_getLED_Band_Width();
extern U8 LineTraceControl_getLED_Band_Width_Per();
extern float LineTraceControl_getLight_Band_Rel_P(U16 lightlevel, U16 prevlightlevel);

//マーカー判定用
extern void LineTraceControl_MakerDetect_Init();
extern S8 LineTraceControl_getMaker_detect();
extern S8 LineTraceControl_MakerDetect();

extern void LineTraceControl_ChangeLightThreshold(U16 lightBlack, U16 lightWhite, float* pHensa0, float* pIntegral); // 制御中の閾値変更
extern signed char LineTraceControl_DirectionPID(float kp, float ki, float kd, float* pHensa0, float* pIntegral, float max, float min);

#if 1 // 2012/09/16 add by s.sahara ライン逸脱検知
extern U8 IsDeviateFromLine();
extern void ResetDetectDeviateFromLine();
extern float GetLightLevelOnLastDetectDeviate();
extern float GetNormalizedLightLevel();
#endif

extern void tail_control(signed int angle);
extern void tail_pid_control_reset();
extern void tail_pid_control(float kp, float ki, float kd, float angle, float max, float min);

extern void LinTraceControl_Periodic();
extern float LinTraceControl_GetCurveAngle();

extern void InitBalanceControlStraightP();
extern void BalanceControlStraightP(char forward, float kp);
extern void InitTailRunControlStraightP();
extern void TailRunControlStraightP(char forward, float kp);
extern void BalanceControl1StraightP(char forward, float kp, float offset);
