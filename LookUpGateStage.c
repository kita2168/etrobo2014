/* 2012/07/21 created by s.sahara 難所クリア単体テスト用 */
#include "LookUpGateStage.h"

#include "kernel.h"
#include "kernel_id.h"
#include "ecrobot_interface.h"

#include "SensorIn.h"
#include "Control.h"

#include "Calc.h"

#include "LineTrace.h"
#include "LineTraceParam.h"

#include "BalanceToTail.h"
#include "TailToBalance.h"

static signed char forward; 	 /* 前後進命令 */
static signed char turn;		 /* 旋回命令 */

#if 1 // add by s.sahara
static signed char forwardTarget = 0;		// 目標速度指令
static signed char forwardTargetPrev = 0;		// 前回目標速度指令
static float accelT = 0.0f;
static float accelTmax = 0;

static signed char angleTarget = 0;
static signed char angleTargetPrev = 0;
static float angleAccelT = 0.0f;
static float angleAccelTmax = 0;
#endif

/* シーケンス番号 */
#define BALANCE_TO_TAIL_SEQ_NO	0	/* 尻尾走行へ */
#define BEFORE_GATE_SEQ_NO		1	/* ルックアップゲート直前 */
#define BALANCE_OFF_SEQ_NO		2	/* 走行体傾け中 */
#define UNDER_GATE_SEQ_NO		3	/* 尻尾走行中 */
#define BRAKE_SEQ_NO			4	/* 静止 */
#define UNDER_RETURN_SEQ_NO		5	/* リターン走行 */
#define BRAKE2_SEQ_NO			6	/* 静止 */
#define UNDER_GATE2_SEQ_NO		7	/* 尻尾走行２ */
#define BALANCE_ON_SEQ_NO		8	/* 走行体復帰中 */
#define AFTER_GATE_SEQ_NO		9	/* ゴールへ */
#define END_GATE_SEQ_NO			10	/* テスト用 */
static signed char SeqNo = BALANCE_TO_TAIL_SEQ_NO;


/* ジャイロ検出 */
#define STOP_THRESHOLD (0.05f) //停止と判断する閾値
static int gyroInitflg = 0; // ジャイロ検出初期化フラグ

/* 尻尾走行移行シーケンス */
static int countBalanceToTailSeq = 0;

/* ルックアップゲート直前シーケンス */
#define INIT_SONAR_SENSOR 25 // 超音波センサの最初を無視する
static int countBeforGateSeq = 0;
static int prevSonarAlertValue = -1;

/* 走行体傾け中シーケンス */
#define ASSIST_BLANCE_OFF_COUNT 100 // 傾きのアシスト(傾き1段階目と同時に行う)
#define ONE_BLANCE_OFF_COUNT 250 // 傾き1段階目(尻尾を倒立状態と傾き状態の中間にする)

#define TWO_BLANCE_OFF_COUNT 250 // 傾き2段階目(尻尾を傾き状態にする)

#define THREE_BLANCE_OFF_COUNT 450 // 静止の検出(GyroDetect()==0で進む)
static int countBalanceOffSeq = 0;
#define ASSIST_MOTOR_SPEED 30 // 傾きをアシストする力の大きさ

/* 尻尾走行中 */
#define UNDER_BAR_RUN_CM 25  //35
#define UNDER_BAR_RUN_RET_CM 30 //45
static int countUnderGateSeq = 0;
static int countReturnGateSeq = 0;

/* 走行体復帰中 */
#define ASSIST_BLANCE_ON_COUNT 149 // 復帰のアシスト(復帰1段階目と同時に行う)
#define ZERO_BALANCE_ON_COUNT 250 // しばらく静止
#define ONE_BLANCE_ON_COUNT 500 // 復帰1段階目(尻尾を倒立状態と傾き状態の中間にする)

/* ゲート通過後 */
static int countAfterGateSeq = 0;

#define TWO_BLANCE_ON_COUNT 400 // 復帰2段階目(尻尾を傾き状態にする)
#define THREE_BLANCE_ON_COUNT 550 // 静止の検出(GyroDetect()==0で進む)
static int countBalanceOnSeq = 0;
#define ASSIST_STAND_MOTOR_SPEED (-30) // 復帰をアシストする力の大きさ

// 走行体自転ステータス
#define TURN_LEFT 0
#define TURN_RIGHT 1

/* 通常尻尾走行制御用ゲイン */
#define DIRECTION_KP_TEST 			(0.5f)

#define DIRECTION_MV_MAX 			(100.0f)
#define DIRECTION_MV_MIN 			(-100.0f)

/* キャリブレーション光センサ値 */
static U16 black;
static U16 white;

//static int isreturn = 0;

//#################################################################
// kuniライントレース用の関数の宣言 add by kunitake <2014/08/13>
//#################################################################
extern void init_model();
extern void ctrl_direction(float ref_vel);
extern signed char get_model_forward();
extern signed char get_model_turn();
extern float get_model_mu();
extern float get_model_y();
extern void change_model_black_to_gray();
extern void change_model_gray_to_black();
extern void change_model_black_to_tail();
extern void change_model_tail_to_black();
extern void reset_model_state();
extern U16 get_model_center();
extern U16 get_model_light();


void LookUpGateStageStm_ChangeSpeed(float spd, float Tmax)
{
	forwardTargetPrev = forward;
	forwardTarget = spd;
	accelT = 0.0f;
	accelTmax = Tmax;
}

void LookUpGateStageStm_ChangeAngle(float ang, float Tmax)
{
	angleTargetPrev = nxt_motor_get_count(NXT_PORT_A);
	angleTarget = ang;
	angleAccelT = 0.0f;
	angleAccelTmax = Tmax;
}

int LookUpGateStageMain()
{
	static char hensa0Initialized = 0;
	static float hensa0;
	static float integral;

	float dirKp = TAILRUN_DIRECTION_KP;
	float dirKi = TAILRUN_DIRECTION_KI;
	float dirKd = TAILRUN_DIRECTION_KD;
	float Kstr = 0.6f; // 直線走行時ゲイン調整係数

	// 尻尾PIDとりあえず適当
	float tailKp = TAIL_KP;
	float tailKi = TAIL_KI;
	float tailKd = TAIL_KD;
	float tailMaxMv = 100.0f;
	float tailMinMv = -100.0f;

	U16 lightValue = GetLightInLPF();
	signed int angle;

	if(!hensa0Initialized)
	{
		// 前回偏差保持値の初期化
		hensa0 = 0.0f;
		integral = 0.0f;
		hensa0Initialized = 1;
	}

	switch(SeqNo)
	{
		case BALANCE_TO_TAIL_SEQ_NO:

			if(BalanceToTailMain() == 1)
			{
				SeqNo = BEFORE_GATE_SEQ_NO;
			}

			break;
		case BEFORE_GATE_SEQ_NO:		//ゲートの検出

			if(0 == countBeforGateSeq)
			{
				LookUpGateStageStm_ChangeAngle(GATE_TAIL_ANGLE_SONAR, 0.5f);
 				LookUpGateStageStm_ChangeSpeed(20, 1.0f);
 			}
			forward = (signed char)SCurve((float)forwardTarget, (float)forwardTargetPrev, accelTmax, &accelT);
			// 方向陀PID
			ctrl_direction((float)forward);
			turn = get_model_turn();
			forward = get_model_forward();
			TailRunControl(forward, turn);

			angle = (signed char)SCurve((float)angleTarget, (float)angleTargetPrev, angleAccelTmax, &angleAccelT);
			tail_pid_control(tailKp, tailKi, tailKd, angle, tailMaxMv, tailMinMv);

			//切り替え（ゲートの検出）
			int SonarAlertValue = sonar_alert(12); // 12cm
			if (prevSonarAlertValue == -1)
			{
				prevSonarAlertValue = SonarAlertValue;
			}

			//超音波センサが10cm以内になった直後を判定、ただし最初の100msは無視する
			if (SonarAlertValue == 1 && prevSonarAlertValue == 0 && countBeforGateSeq == INIT_SONAR_SENSOR) // 障害物検知
			{
				forward = turn = 0; // 障害物を検知したら停止
				SeqNo = BALANCE_OFF_SEQ_NO;
				countBalanceOffSeq = 0;
			}

			if(countBeforGateSeq < INIT_SONAR_SENSOR)
			{
				countBeforGateSeq++;
			}

			prevSonarAlertValue = SonarAlertValue;

#ifdef USE_LOG
//			ecrobot_bt_data_logger((S8)(SeqNo * 10), SonarAlertValue); // BlueToothへログ書き出し(シーケンス番号*10, 超音波センサ検出)
#endif
			break;

		case BALANCE_OFF_SEQ_NO:	//ゲートを通れる角度に走行体を傾ける
			if(0 == countBalanceOffSeq)
			{
				LookUpGateStageStm_ChangeAngle(GATE_TAIL_ANGLE_UNDER_GATE, 1.0f);
				LookUpGateStageStm_ChangeSpeed(0, 0.5f);
				turn = 0;
			}
			forward = (signed char)SCurve((float)forwardTarget, (float)forwardTargetPrev, accelTmax, &accelT);
			TailRunControl(forward, turn);
			angle = (signed char)SCurve((float)angleTarget, (float)angleTargetPrev, angleAccelTmax, &angleAccelT);
			tail_pid_control(tailKp, tailKi, tailKd, angle, tailMaxMv, tailMinMv);

			/* 静止の検出 */
			if(TWO_BLANCE_OFF_COUNT <= countBalanceOffSeq && countBalanceOffSeq < THREE_BLANCE_OFF_COUNT)
			{
				// ジャイロ検出の初期化
				if(gyroInitflg == 0)
				{
					InitGyroDetect(STOP_THRESHOLD);
					gyroInitflg = 1;
				}
				else
				{
					if(GyroDetect() == 0)
					{
						countBalanceOffSeq++;
					}
				}
			}
			else
			{
				countBalanceOffSeq++;
			}

			/* 静止状態で一定時間たてば次のシーケンス */
			if(countBalanceOffSeq == THREE_BLANCE_OFF_COUNT)
			{
				hensa0 = 0;
				integral = 0;
				InitTailRunControlStraightP();
				SeqNo = UNDER_GATE_SEQ_NO;
				countUnderGateSeq = 0;
//				change_model_black_to_tail();
//				reset_model_state();
			}
			break;

		case UNDER_GATE_SEQ_NO:		//ゲート下を前進走行する


			forward = (signed char)SCurve((float)forwardTarget, (float)forwardTargetPrev, accelTmax, &accelT);
			// 方向陀PID
//			ctrl_direction((float)forward);
//			turn = get_model_turn();
//			forward = get_model_forward();
//			TailRunControl(forward, turn);
			TailRunControlStraightP(forward, 10.0f);
			tail_pid_control(tailKp, tailKi, tailKd, GATE_TAIL_ANGLE_UNDER_GATE, tailMaxMv, tailMinMv);

			if(countUnderGateSeq == 0)
			{
				init_DistanceFromPoint();
				LookUpGateStageStm_ChangeSpeed(30, 0.5f);
			}
			// 角度で走行した距離を判断
			if(calc_getDistanceFromPoint() * 100.0f >= UNDER_BAR_RUN_CM)
			{
				//切り替え（一定距離を走行した）
				SeqNo = BRAKE_SEQ_NO;
				countBalanceOffSeq = 0;
				gyroInitflg = 0;
			}

			countUnderGateSeq++;

#ifdef USE_LOG
//			ecrobot_bt_data_logger((S8)(SeqNo * 10), (S8)(countUnderGateSeq / 10));
#endif
			break;

		case BRAKE_SEQ_NO:		//静止する
			if(0 == countBalanceOffSeq)
			{
				LookUpGateStageStm_ChangeAngle(GATE_TAIL_ANGLE_UNDER_GATE, 1.0f);
				LookUpGateStageStm_ChangeSpeed(0, 0.5f);
				turn = 0;
			}
			forward = (signed char)SCurve((float)forwardTarget, (float)forwardTargetPrev, accelTmax, &accelT);
			TailRunControl(forward, turn);

			tail_pid_control(tailKp, tailKi, tailKd, GATE_TAIL_ANGLE_UNDER_GATE, tailMaxMv, tailMinMv);

			/* 静止の検出 */
			countBalanceOffSeq++;

			/* 静止状態で一定時間たてば次のシーケンス */
			if(countBalanceOffSeq == ASSIST_BLANCE_OFF_COUNT)
			{
				hensa0 = 0;
				integral = 0;
				SeqNo = UNDER_RETURN_SEQ_NO;
				countBalanceOnSeq = 0;
				countReturnGateSeq = 0;
				InitTailRunControlStraightP();
			}
			break;


		case UNDER_RETURN_SEQ_NO:		//ゲート下を後進走行する　（直進走行）
			turn = 0;
			forward = (signed char)SCurve((float)forwardTarget, (float)forwardTargetPrev, accelTmax, &accelT);
			// 方向陀PID
//			ctrl_direction((float)forward);
//			turn = get_model_turn();
//			forward = get_model_forward();
//			TailRunControl(forward, turn);
			TailRunControlStraightP(forward, -10.0f);
			tail_pid_control(tailKp, tailKi, tailKd, GATE_TAIL_ANGLE_UNDER_GATE, tailMaxMv, tailMinMv);

			if(countReturnGateSeq == 0)
			{
				init_DistanceFromPoint();

				LookUpGateStageStm_ChangeSpeed(-30, 0.5f);
				InitTailRunControlStraightP();
			}

			//走行完了判定
			if((calc_getDistanceFromPoint() * 100.0f * (-1.0)) >= UNDER_BAR_RUN_RET_CM)
			{
				turn = 0;
				SeqNo = BRAKE2_SEQ_NO;
				countBalanceOffSeq =0;
			}

			countReturnGateSeq++;
#ifdef USE_LOG
//			ecrobot_bt_data_logger((S8)(SeqNo * 10), (S8)(countReturnGateSeq / 10));
#endif
			break;

		case BRAKE2_SEQ_NO:		//静止する
			if(0 == countBalanceOffSeq)
			{
				LookUpGateStageStm_ChangeAngle(GATE_TAIL_ANGLE_UNDER_GATE, 1.0f);
				LookUpGateStageStm_ChangeSpeed(0, 0.5f);
				turn = 0;
			}
			forward = (signed char)SCurve((float)forwardTarget, (float)forwardTargetPrev, accelTmax, &accelT);
			TailRunControl(forward, turn);

			tail_pid_control(tailKp, tailKi, tailKd, GATE_TAIL_ANGLE_UNDER_GATE, tailMaxMv, tailMinMv);

			/* 静止の検出 */
			countBalanceOffSeq++;

			/* 静止状態で一定時間たてば次のシーケンス */
			if(countBalanceOffSeq == ASSIST_BLANCE_OFF_COUNT)
			{
				hensa0 = 0;
				integral = 0;
				SeqNo = UNDER_GATE2_SEQ_NO;
				countUnderGateSeq = 0;
//				InitTailRunControlStraightP();
				change_model_black_to_tail();
				reset_model_state();
			}
			break;


		case UNDER_GATE2_SEQ_NO:		//ゲート下を前進走行する （直進走行）


			turn = 0;
			forward = (signed char)SCurve((float)forwardTarget, (float)forwardTargetPrev, accelTmax, &accelT);
			// 方向陀PID
			ctrl_direction((float)forward);
			turn = get_model_turn();
			forward = get_model_forward();
			TailRunControl(forward, turn);
//			TailRunControlStraightP(forward, 10.0f);
			tail_pid_control(tailKp, tailKi, tailKd, GATE_TAIL_ANGLE_UNDER_GATE, tailMaxMv, tailMinMv);

			if(countUnderGateSeq == 0)
			{
				init_DistanceFromPoint();

				LookUpGateStageStm_ChangeSpeed(30, 0.5f);
//				InitTailRunControlStraightP();
			}

			//走行完了判定
			if((calc_getDistanceFromPoint() * 100.0f) >= UNDER_BAR_RUN_CM)
			{
				turn = 0;
				LookUpGateStageStm_ChangeSpeed(0, 1.0f);
				SeqNo = BALANCE_ON_SEQ_NO;
				countBalanceOnSeq =0;
				//isreturn = isreturn++;
			}

			countUnderGateSeq++;
#ifdef USE_LOG
//			ecrobot_bt_data_logger((S8)(SeqNo * 10), (S8)(countUnderGateSeq / 10));
#endif
			break;

		case BALANCE_ON_SEQ_NO: // 体を起こす
			if(0 == countBalanceOnSeq) LookUpGateStageStm_ChangeAngle(TAIL_ANGLE_TAILRUN, 1.0f);
			forward = (signed char)SCurve((float)forwardTarget, (float)forwardTargetPrev, accelTmax, &accelT);
			TailRunControl(forward, 0);
			angle = (signed char)SCurve((float)angleTarget, (float)angleTargetPrev, angleAccelTmax, &angleAccelT);
			tail_pid_control(tailKp, tailKi, tailKd, angle, tailMaxMv, tailMinMv);

			if(TWO_BLANCE_ON_COUNT <= countBalanceOnSeq && countBalanceOnSeq < THREE_BLANCE_ON_COUNT)
			{
				// ジャイロ検出の初期化
				if(gyroInitflg == 1)
				{
					InitGyroDetect(STOP_THRESHOLD);
					gyroInitflg = 2;
				}
				else
				{
					if(GyroDetect() == 0)
					{
						countBalanceOnSeq++;
					}
				}
			}
			else
			{
				countBalanceOnSeq++;
			}

			/* 静止状態で一定時間たてば次のシーケンス */
			if(countBalanceOnSeq == THREE_BLANCE_ON_COUNT)
			{
				reset_model_state();
				SeqNo = AFTER_GATE_SEQ_NO;
				countAfterGateSeq = 0;
				InitTailRunControlStraightP();
				change_model_tail_to_black();
				reset_model_state();
			}
			break;
		case AFTER_GATE_SEQ_NO:

			if(countAfterGateSeq == 0)
			{
				init_DistanceFromPoint();
				LookUpGateStageStm_ChangeSpeed(-20, 0.5f);
			}

			//走行完了判定
//			if((calc_getDistanceFromPoint() * 100.0f) >= 15)
			if((calc_getDistanceFromPoint() * 100.0f) <= -10)
			{
				return 1;
			}
			else
			{
				forward = (signed char)SCurve((float)forwardTarget, (float)forwardTargetPrev, accelTmax, &accelT);
//				// 方向陀PID
//				ctrl_direction((float)forward);
//				turn = get_model_turn();
//				forward = get_model_forward();
//				TailRunControl(forward, turn);
				TailRunControlStraightP(forward, -10.0f);
				tail_pid_control(tailKp, tailKi, tailKd, TAIL_ANGLE_TAILRUN, tailMaxMv, tailMinMv);

				countAfterGateSeq++;
			}
#if 0
			tail_pid_control(tailKp, tailKi, tailKd, TAIL_ANGLE_TAILRUN, tailMaxMv, tailMinMv);
#ifdef USE_LOG
			ecrobot_bt_data_logger((S8)(SeqNo * 10), 0);
#endif
			//defaultへ
			SeqNo++;
#else
			//尻尾走行のままゴールへ
			//if(TailToBalanceMain() == 1)
			//{
			//	return 1;
			//}
#endif
			break;
		case END_GATE_SEQ_NO:
			break;

		default:
			return 1;
			break;
	}
//	ecrobot_bt_data_logger((S8)SeqNo+10, (S8)(get_model_mu()*1000)); // マーカ検知閾確認用
//	ecrobot_debug1((UINT)(SeqNo), (UINT)0, (UINT)0);
	ecrobot_bt_data_logger((S8)SeqNo+10, (S8)(get_model_light()/8)); // マーカ検知閾確認用
	return 0;
}

#define ANGLE_CORRECT_VALUE 5 // 尻尾の角度判定の補正
/* 尻尾が目標角度に近いなら1を返す */
int TailAngleCheck(int reqAngle)
{
	int angle = nxt_motor_get_count(NXT_PORT_A);
	if(reqAngle - ANGLE_CORRECT_VALUE < angle && angle < reqAngle + ANGLE_CORRECT_VALUE)
	{
		return 1;
	}
	return 0;
}

