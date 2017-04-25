/* 2014/07/16 created by y.tanaka 難所クリア単体テスト用 */

#include "FigureLStage.h"

#include "kernel.h"
#include "kernel_id.h"
#include "ecrobot_interface.h"

#include "LineTrace.h"
#include "Calc.h"
#include "Control.h"
#include "SensorIn.h"
#include "LineTraceParam.h"
#include "SpeedoMeter.h"
#include "BalanceToTail.h"

/* シーケンス番号定義 */
#define FIGUREL_SEQ_NO_00 0	// 初期セット
#define FIGUREL_SEQ_NO_01 1 // 尻尾走行（ライントレースP）	板面の検出
#define FIGUREL_SEQ_NO_02 2 // 尻尾走行（固定操作）			板面への当て込み
#define FIGUREL_SEQ_NO_03 3 // 尻尾走行（直進制御）			一定距離後退
#define FIGUREL_SEQ_NO_04 4 // 走行なし						尻尾PID制御
#define FIGUREL_SEQ_NO_05 5 // 走行なし						倒立走行前リセット
#define FIGUREL_SEQ_NO_06 6 // 倒立走行（直進制御）			段差
#define FIGUREL_SEQ_NO_07 7 // 倒立走行（直進制御）			右に回転（ラインの右に出るため右を向く）
#define FIGUREL_SEQ_NO_08 8 // 倒立走行（直進制御）			右に少し前進
#define FIGUREL_SEQ_NO_09 9 // 倒立走行（直進制御）			左に回転（ラインの方を向く）
#define FIGUREL_SEQ_NO_10 10 // 倒立走行					板の上でライントレースし、スピン位置まで移動
#define FIGUREL_SEQ_NO_11 11 // 倒立走行→尻尾走行
#define FIGUREL_SEQ_NO_12 12 // 尻尾走行					スピン
#define FIGUREL_SEQ_NO_13 13 // 走行なし					尻尾PID制御
#define FIGUREL_SEQ_NO_14 14 // 走行なし					倒立走行前リセット
#define FIGUREL_SEQ_NO_15 15 // 板の上でライントレースを行いながら、段差を降りる
#define FIGUREL_SEQ_NO_15_1 25 // 板の上でライントレースを行いながら、段差を降りる
#define FIGUREL_SEQ_NO_16 16 // 倒立走行→尻尾走行
#define FIGUREL_SEQ_NO_17 17 // 尻尾走行					右に90度回転（ラインを探すため）
#define FIGUREL_SEQ_NO_18 18 // 倒立走行（直進制御）		ラインの右側に出る
#define FIGUREL_SEQ_NO_19 19 // 倒立走行（直進制御）		ラインを探す
#define FIGUREL_SEQ_NO_20 20 // 尻尾走行					左に回転（ライン復帰のため）

/* 前進値定義 */
#define SEQ_NO_00_FORWARD 			15
#define SEQ_NO_01_FORWARD 			15
#define SEQ_NO_02_FORWARD 			15 // 板面合わせ
#define SEQ_NO_03_FORWARD 			(-20) //一時後退
#define SEQ_NO_06_FORWARD 			20	//段差
#define SEQ_NO_08_FORWARD			10	//ライン検索のため、右に移動
#define SEQ_NO_10_FORWARD 			15	//ライン復帰し、スピン位置まで
#define SEQ_NO_15_FORWARD 			15	//段差を降りて、マーカ中央あたりまで
#define SEQ_NO_18_FORWARD 			15	//マーカの右に移動
#define SEQ_NO_19_FORWARD 			(-15)	//マーカを探す

/* 旋回値定義 */
#define SEQ_NO_02_TURN				15 //板面合わせ
#define SEQ_NO_07_TURN				30 //ライン検索のため、右に移動
#define SEQ_NO_09_TURN				30 //ライン復帰のため、左に移動
#define SEQ_NO_12_TURN				365 //スピン
#define SEQ_NO_17_TURN				90 //ライン検索のため、右に移動
#define SEQ_NO_20_TURN				65 //ライン復帰

/* 距離定義(cm) */
#define SEQ_NO_03_DISTANCE 			(-15.0f) // 後退距離
#define SEQ_NO_06_DISTANCE 			(35.0f) // 減速する距離まで
#define SEQ_NO_07_DISTANCE 			(20.0f) // スピンする距離まで
#define SEQ_NO_08_DISTANCE 			(5.0f) // ライン検索のため、右に移動
#define SEQ_NO_15_DISTANCE 			(5.0f) // スピン位置から段を降りるまで
#define SEQ_NO_15_1_DISTANCE 			(25.0f) // スピン位置から段を降りるまで
#define SEQ_NO_18_DISTANCE 			(10.0f) // ラインに移動する距離

/* 時間 */
#define SEQ_NO_06_TIMER_S 			250		// 板にあがるための加速開始
#define SEQ_NO_06_TIMER_E 			525		// 板にあがるための加速終了

/* 灰色マーカ検知係数 */
#define SEQ_NO_19_GRAY				(0.95)

// 走行体自転ステータス
#define TURN_LEFT 0
#define TURN_RIGHT 1

/* ジャイロオフセット定義 */
#define SEQ_NO_06_OFFSET 			10	// 段差登り用

/* 現在シーケンス番号 */
static S8 figurelSeqNo = FIGUREL_SEQ_NO_00;

/* 光センサ現在値 */
static U16 lightValue;		/* 0-1023 */

/* キャリブレーション光センサ値 */
static U16 black;
static U16 white;
	
/* 走行用パラメータ */
static signed char forward;			/* 前後進命令 */
static signed char turn;			/* 旋回命令 */

static signed char forwardTarget = 0;		// 目標速度指令
static signed char forwardTargetPrev = 0;	// 前回目標速度指令
static float accelT = 0.0f;
static float accelTmax = 0;

/* 尻尾角度 */
signed int angle;
static signed char angleTarget = 0;
static signed char angleTargetPrev = 0;
static float angleAccelT = 0.0f;
static float angleAccelTmax = 0;

/* 時間判定用パラメータ */
static U32 count = 0;		/* シーケンスに入ってからのカウント値 */
static U16 timer_1s = 250;
static U16 timer_2s = 500;

/* 速度LPF用パラメータ */
static S16 velocityInLPF = 0;	/* 速度現在値(cm/s)（ローパス付） */
static float vel_omega0;
static float vel_a;
static float vel_b0;
static float vel_b1;
static float vel_b2;
static float vel_a1;
static float vel_a2;
static float vel_pre1;
static float vel_pre2;

/* ジャイロオフセット走行用 */
static S8 offset;

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
extern void reset_model_state();
extern U16 get_model_center();
extern U16 get_model_light();

void InitFigureLStageStm_ChangeSpeed()
{
	forward = 0.0f;
	forwardTargetPrev = 0.0f;
	accelT = 0.0f;
}
void FigureLStageStm_ChangeSpeed(float spd, float Tmax)
{
	forwardTargetPrev = forward;
	forwardTarget = spd;
	accelT = 0.0f;
	accelTmax = Tmax;
}

void BalanceToTail_ChangeAngle(float ang, float Tmax)
{
	angleTargetPrev = nxt_motor_get_count(NXT_PORT_A);
	angleTarget = ang;
	angleAccelT = 0.0f;
	angleAccelTmax = Tmax;
}

int FigureLStageMain(void)
{
	static char hensa0Initialized = 0;
	static float hensa0;
	static float integral;
	
	// 尻尾PIDとりあえず適当
	float tailKp = TAIL_KP;
	float tailKi = TAIL_KI;
	float tailKd = TAIL_KD;
	float tailMaxMv = 100.0f;
	float tailMinMv = -100.0f;
	
	// 光センサ値取得
	lightValue = ecrobot_get_light_sensor(NXT_PORT_S3);
	
	// キャリブレーション値取得
	black = LineTraceControl_LightBlackThreshold();
	white = LineTraceControl_LightWhiteThreshold();
	
	// LPFをかけた速度を常に計算する（最初の計算に時間がかかるため）
	float velocity = SpeedoMeter();
	velocityInLPF = LPF((S16)(velocity * 100.0f), vel_omega0, vel_a, vel_b0, vel_b1, vel_b2, vel_a1, vel_a2, &vel_pre1, &vel_pre2);

	// 前進値計算
	forward = (signed char)SCurve((float)forwardTarget, (float)forwardTargetPrev, accelTmax, &accelT);
			
	if(!hensa0Initialized)
	{
		// 前回偏差保持値の初期化
		hensa0 = 0.0f;
		integral = 0.0f;
		hensa0Initialized = 1;
		change_model_gray_to_black();
		init_DistanceFromPoint();
	}
	
	switch(figurelSeqNo)
	{
		case FIGUREL_SEQ_NO_00:		// 初期セット
		{
			
			FigureLStageStm_ChangeSpeed(SEQ_NO_00_FORWARD, 0.3f);
			figurelSeqNo = FIGUREL_SEQ_NO_01;
			
			#ifdef USE_LOG
			ecrobot_debug1((UINT)figurelSeqNo, (UINT)(forward), 0);
			ecrobot_bt_data_logger((S8)(figurelSeqNo), (S8)(forward));
			#endif
			break;
		}
		case FIGUREL_SEQ_NO_01: // 尻尾走行（ライントレースP）	板面の検出
		{
			
			// 方向陀PID
			ctrl_direction((float)forward);
			turn = get_model_turn();
			forward = get_model_forward();	
			
			TailRunControl(forward, turn);

			tail_pid_control(tailKp, tailKi, tailKd, TAIL_ANGLE_TAILRUN, tailMaxMv, tailMinMv);

			if(timer_1s / 2 == count)
			{
				float distance = calc_getDistanceFromPoint();
				if(distance <= 0.01f)
				{
					figurelSeqNo = FIGUREL_SEQ_NO_02;
				}
				
				count = 0;
				
			}
			else
			{
				if(count == 0)
				{
					init_DistanceFromPoint();
				}
				
				count++;
			}
			#ifdef USE_LOG
			ecrobot_debug1((UINT)figurelSeqNo, (UINT)(forward), 0);
			ecrobot_bt_data_logger((S8)(figurelSeqNo), (S8)(forward));
			#endif
			break;
		}
		case FIGUREL_SEQ_NO_02: // 尻尾走行（固定操作）			板面への当て込み
		{
			
			if(count == 0)
			{
				FigureLStageStm_ChangeSpeed(SEQ_NO_02_FORWARD, 0.1f);
			}
			
			// 板の端にタイヤを合わせる動作
			if(timer_1s / 2 > count)
			{
				turn = SEQ_NO_02_TURN;
				count++;
			}
			else if(timer_1s > count)
			{
				turn = -SEQ_NO_02_TURN;
				count++;
			}
			else
			{
				turn = 0;
				
				figurelSeqNo = FIGUREL_SEQ_NO_03;
				count = 0;
			}
			
			
			TailRunControl(forward, turn);
			tail_pid_control(tailKp, tailKi, tailKd, TAIL_ANGLE_TAILRUN, tailMaxMv, tailMinMv);

			#ifdef USE_LOG
			ecrobot_debug1((UINT)figurelSeqNo, (UINT)(forward), 0);
			ecrobot_bt_data_logger((S8)(figurelSeqNo), (S8)(forward));
			#endif
			break;
		}
		case FIGUREL_SEQ_NO_03: // 尻尾走行（直進制御）			一定距離後退
		{
			if(count == 0)
			{
				init_DistanceFromPoint();
				InitTailRunControlStraightP();
				
				FigureLStageStm_ChangeSpeed(SEQ_NO_03_FORWARD, 0.3f);
			}
			
			// 後退した距離(cm)
			float distance = calc_getDistanceFromPoint() * 100.0f;
			
			if(distance > SEQ_NO_03_DISTANCE)
			{
				TailRunControlStraightP(forward, -2.0f);
				tail_pid_control(tailKp, tailKi, tailKd, TAIL_ANGLE_TAILRUN, tailMaxMv, tailMinMv);

				count++;
			}
			else
			{
				forward=0;
				turn = 0;
				FigureLStageStm_ChangeSpeed(0, 0.0f);
				
				figurelSeqNo = FIGUREL_SEQ_NO_04;
				count = 0;
			}
			#ifdef USE_LOG
			ecrobot_debug1((UINT)figurelSeqNo, (UINT)(forward), 0);
			ecrobot_bt_data_logger((S8)(figurelSeqNo), (S8)(forward));
			#endif
			break;
		}
		case FIGUREL_SEQ_NO_04: // 走行なし						尻尾PID制御
		{
			if(timer_1s > count)
			{
				if(count == 0)
				{
					
					//倒立前にモータ角度をリセット
					nxt_motor_set_count(NXT_PORT_B, 0);
					nxt_motor_set_count(NXT_PORT_C, 0);
					
					balance_init();
					
					init_DistanceFromPoint();
				}
				tail_pid_control(tailKp, tailKi, tailKd, TAIL_ANGLE_TAILRUN, tailMaxMv, tailMinMv);
				count++;
			}
			else
			{
				forward=0;
				turn = 0;
				
				figurelSeqNo = FIGUREL_SEQ_NO_05;
				count = 0;
			}
			#ifdef USE_LOG
			ecrobot_debug1((UINT)figurelSeqNo, (UINT)(forward), 0);
			ecrobot_bt_data_logger((S8)(figurelSeqNo), (S8)(forward));
			#endif
			break;
		}
		case FIGUREL_SEQ_NO_05: // 走行なし						倒立走行前リセット
		{
			if(count == 0)
			{
				InitBalanceControlStraightP();
			}
			
			if(timer_1s > count)
			{
				tail_pid_control(tailKp, tailKi, tailKd, TAIL_ANGLE_TAILRUN, tailMaxMv, tailMinMv);
				BalanceControlStraightP(forward, 3.0f);
				count++;
			}
			else if(timer_2s > count)
			{
				if(timer_1s == count)
				{
					BalanceToTail_ChangeAngle(TAIL_ANGLE_BALANCE_RUN, 1.0f);
				}
				angle = (signed char)SCurve((float)angleTarget, (float)angleTargetPrev, angleAccelTmax, &angleAccelT);
				tail_pid_control(tailKp, tailKi, tailKd, angle, tailMaxMv, tailMinMv);
				
				BalanceControlStraightP(forward, 3.0f);
				
				count++;
			}
			else
			{
				figurelSeqNo = FIGUREL_SEQ_NO_06;
				count = 0;
			}
			#ifdef USE_LOG
			ecrobot_debug1((UINT)figurelSeqNo, (UINT)(forward), 0);
			ecrobot_bt_data_logger((S8)(figurelSeqNo), (S8)(forward));
			#endif
			break;
		}
		case FIGUREL_SEQ_NO_06: // 倒立走行（直進制御）			段差
		{
			// 倒立開始からの距離(cm)
			float distance = calc_getDistanceFromPoint() * 100.0f;
			
			if(count == 0)
			{
				FigureLStageStm_ChangeSpeed(SEQ_NO_06_FORWARD, 0.3f);
			}
			
			offset = 0;
			if(distance >= SEQ_NO_06_DISTANCE)
			{
				figurelSeqNo = FIGUREL_SEQ_NO_07;
				
				FigureLStageStm_ChangeSpeed(0, 0.1f);
				
				count = 0;
			}
			else
			{
				if(SEQ_NO_06_TIMER_S < count && count < SEQ_NO_06_TIMER_E)	//250 ～ 525
				{
					offset = SEQ_NO_06_OFFSET;
				}
				
				tail_pid_control(tailKp, tailKi, tailKd, TAIL_ANGLE_BALANCE_RUN, tailMaxMv, tailMinMv);
				BalanceControl1StraightP(forward, 10.0f, offset);
				
				count++;
			}
			#ifdef USE_LOG
			ecrobot_debug1((UINT)figurelSeqNo, (UINT)(forward), 0);
			ecrobot_bt_data_logger((S8)(figurelSeqNo), (S8)(forward));
			#endif
			break;
		}
		case FIGUREL_SEQ_NO_07: // 倒立走行（直進制御）			右に回転（ラインの右に出るため右を向く）
		{
			turn = 70;
			forward = 0;
	
			if(count == 0)
			{
				init_rotateDistance(TURN_RIGHT);		/* 回転距離測定初期化 */
				
			}
			
			//ターン完了判定
			if(check_rotateComplete(SEQ_NO_07_TURN))
			{
				
				figurelSeqNo = FIGUREL_SEQ_NO_08;
				
				InitBalanceControlStraightP();
				init_DistanceFromPoint();
				
				FigureLStageStm_ChangeSpeed(SEQ_NO_08_FORWARD, 0.2f);
				turn = 0;
				
				count = 0;
			}
			else
			{
				BalanceControl(forward, turn);
				tail_pid_control(tailKp, tailKi, tailKd, TAIL_ANGLE_BALANCE_RUN, tailMaxMv, tailMinMv); 
				count++;	
			}
			#ifdef USE_LOG
			ecrobot_debug1((UINT)figurelSeqNo, (UINT)(forward), 0);
			ecrobot_bt_data_logger((S8)(figurelSeqNo), (S8)(forward));
			#endif
			break;
		}
		case FIGUREL_SEQ_NO_08: // 倒立走行（直進制御）			右に少し前進
		{
			// 倒立開始からの距離(cm)
			float distance = calc_getDistanceFromPoint() * 100.0f;
			
			if(distance >= SEQ_NO_08_DISTANCE)	// 5.0f
			{
				figurelSeqNo = FIGUREL_SEQ_NO_09;
				count = 0;
			}
			else
			{
				turn = 0;
				tail_pid_control(tailKp, tailKi, tailKd, TAIL_ANGLE_BALANCE_RUN, tailMaxMv, tailMinMv);
				BalanceControlStraightP(forward, 3.0f);
				
				count++;
			}
			#ifdef USE_LOG
			ecrobot_debug1((UINT)figurelSeqNo, (UINT)(forward), 0);
			ecrobot_bt_data_logger((S8)(figurelSeqNo), (S8)(forward));
			#endif
			break;
		}
		case FIGUREL_SEQ_NO_09: // 倒立走行（直進制御）			左に回転（ラインの方を向く）
		{
			turn = -70;
			forward = 0;
	
			if(count == 0)
			{
				init_rotateDistance(TURN_LEFT);		/* 回転距離測定初期化 */
				
			}
			
			//ターン完了判定
			if(check_rotateComplete(SEQ_NO_09_TURN))
			{
				forward=0;
				turn = 0;
				InitFigureLStageStm_ChangeSpeed();
				
				figurelSeqNo = FIGUREL_SEQ_NO_10;
				
				init_DistanceFromPoint();
				
				count = 0;
			}
			else
			{
				BalanceControl(forward, turn);
				tail_pid_control(tailKp, tailKi, tailKd, TAIL_ANGLE_BALANCE_RUN, tailMaxMv, tailMinMv); 
				count++;	
			}
			#ifdef USE_LOG
			ecrobot_debug1((UINT)figurelSeqNo, (UINT)(forward), 0);
			ecrobot_bt_data_logger((S8)(figurelSeqNo), (S8)(forward));
			#endif
			break;
		}
		case FIGUREL_SEQ_NO_10: // 倒立走行			板の上でライントレースし、スピン位置まで移動
		{
			// 倒立開始からの距離(cm)
			float distance = calc_getDistanceFromPoint() * 100.0f;

			// 方向陀PID
			ctrl_direction((float)forward);
			turn = get_model_turn();
			forward = get_model_forward();	
			
			if(count == 0)
			{
				FigureLStageStm_ChangeSpeed(SEQ_NO_10_FORWARD, 0.1f);
			}
			
			if(distance >= SEQ_NO_07_DISTANCE)
			{
				figurelSeqNo = FIGUREL_SEQ_NO_11;
				count = 0;
			}
			else
			{
				tail_pid_control(tailKp, tailKi, tailKd, TAIL_ANGLE_BALANCE_RUN, tailMaxMv, tailMinMv);
				BalanceControl(forward, turn);
				
				count++;
			}
			#ifdef USE_LOG
			ecrobot_debug1((UINT)figurelSeqNo, (UINT)(forward), 0);
			ecrobot_bt_data_logger((S8)(figurelSeqNo), (S8)(forward));
			#endif
			break;
		}
		case FIGUREL_SEQ_NO_11: // 倒立走行→尻尾走行
		{
			if(count == 0)
			{
				FigureLStageStm_ChangeSpeed(0, 0.0f);
				
				BalanceToTail_Init();
			}
			
			if(BalanceToTailMain() == 1)
			{
				figurelSeqNo = FIGUREL_SEQ_NO_12;
				forward=0;
				turn = 0;
				count = 0;
			
			}else{
				count++;
			}
			#ifdef USE_LOG
			ecrobot_debug1((UINT)figurelSeqNo, (UINT)(forward), 0);
			ecrobot_bt_data_logger((S8)(figurelSeqNo), (S8)(forward));
			#endif
			break;
		}
		case FIGUREL_SEQ_NO_12: // 尻尾走行		スピン
		{
			turn = -70;
			forward = 0;
	
			if(count == 0)
			{
				init_rotateDistance(TURN_LEFT);		/* 回転距離測定初期化 */
				
			}
			
			//スピン完了判定
			if(check_rotateComplete(SEQ_NO_12_TURN))	// 365
			{
				forward=0;
				FigureLStageStm_ChangeSpeed(0, 0.1f);
				turn = 0;
				
				figurelSeqNo = FIGUREL_SEQ_NO_13;
				count = 0;
			}
			else
			{
				TailRunControl(forward, turn);
				tail_pid_control(tailKp, tailKi, tailKd, TAIL_ANGLE_TAILRUN, tailMaxMv, tailMinMv); 
				count++;	
			}
			#ifdef USE_LOG
			ecrobot_debug1((UINT)figurelSeqNo, (UINT)(forward), 0);
			ecrobot_bt_data_logger((S8)(figurelSeqNo), (S8)(forward));
			#endif
			break;
		}
		case FIGUREL_SEQ_NO_13: // 走行なし						尻尾PID制御
		{
			
			if(timer_1s > count)
			{
				//停止
				forward=0;
				turn = 0;
				
				TailRunControl(forward, turn);
				tail_pid_control(tailKp, tailKi, tailKd, TAIL_ANGLE_TAILRUN, tailMaxMv, tailMinMv);
			
				count++;
			}
			else if(timer_2s > count)
			{
				if(count == timer_1s)
				{
					//倒立前にモータ角度をリセット
					nxt_motor_set_count(NXT_PORT_B, 0);
					nxt_motor_set_count(NXT_PORT_C, 0);
					
					balance_init();
					
					init_DistanceFromPoint();
				}
				
				tail_pid_control(tailKp, tailKi, tailKd, TAIL_ANGLE_TAILRUN, tailMaxMv, tailMinMv);
			
				count++;
			}
			else
			{
				forward=0;
				turn = 0;
				
				figurelSeqNo = FIGUREL_SEQ_NO_14;
				count = 0;
			}
			#ifdef USE_LOG
			ecrobot_debug1((UINT)figurelSeqNo, (UINT)(forward), 0);
			ecrobot_bt_data_logger((S8)(figurelSeqNo), (S8)(forward));
			#endif
			break;
		}
		case FIGUREL_SEQ_NO_14: // 走行なし						倒立走行前リセット
		{
			forward=0;
			turn = 0;
			
			if(timer_2s > count)
			{
				if(count == timer_1s)
				{
					BalanceToTail_ChangeAngle(TAIL_ANGLE_BALANCE_RUN, 1.0f);
				}
				
				angle = (signed char)SCurve((float)angleTarget, (float)angleTargetPrev, angleAccelTmax, &angleAccelT);
				tail_pid_control(tailKp, tailKi, tailKd, angle, tailMaxMv, tailMinMv);
				
				BalanceControl(forward, turn);
				
				count++;
			}
			else
			{
				figurelSeqNo = FIGUREL_SEQ_NO_15;
				
				count = 0;
			}
			#ifdef USE_LOG
			ecrobot_debug1((UINT)figurelSeqNo, (UINT)(forward), 0);
			ecrobot_bt_data_logger((S8)(figurelSeqNo), (S8)(forward));
			#endif
			break;
		}
		case FIGUREL_SEQ_NO_15: // 板の上でライントレースを行いながら、段差を降りる
		{
			// スピン位置からの距離(cm)
			float distance = calc_getDistanceFromPoint() * 100.0f;
			
			// 方向陀PID
			ctrl_direction((float)forward);
			turn = get_model_turn();
			forward = get_model_forward();	
			
			if(count == 0)
			{
				FigureLStageStm_ChangeSpeed(SEQ_NO_15_FORWARD, 0.1f);
			}
			
			if(distance > SEQ_NO_15_DISTANCE)
			{
				figurelSeqNo = FIGUREL_SEQ_NO_15_1;
				
				InitBalanceControlStraightP();
				init_DistanceFromPoint();
				
				count++;
			}
			else
			{	
				tail_pid_control(tailKp, tailKi, tailKd, TAIL_ANGLE_BALANCE_RUN, tailMaxMv, tailMinMv);
				BalanceControl(forward, turn);
				
				count++;
			}
			#ifdef USE_LOG
			ecrobot_debug1((UINT)figurelSeqNo, (UINT)(forward), 0);
			ecrobot_bt_data_logger((S8)(figurelSeqNo), (S8)(forward));
			#endif
			break;
		}
		case FIGUREL_SEQ_NO_15_1: // 板の上で直進走行で段差を降りる
		{
			// スピン位置からの距離(cm)
			float distance = calc_getDistanceFromPoint() * 100.0f;
			
			if(count == 0)
			{
				FigureLStageStm_ChangeSpeed(SEQ_NO_15_FORWARD, 0.1f);
			}
			
			if(distance > SEQ_NO_15_1_DISTANCE)
			{
				figurelSeqNo = FIGUREL_SEQ_NO_16;
				forward = 0;
				count = 0;
			}
			else
			{	
				tail_pid_control(tailKp, tailKi, tailKd, TAIL_ANGLE_BALANCE_RUN, tailMaxMv, tailMinMv);
				BalanceControlStraightP(forward, 3.0f);
				
				count++;
			}
			#ifdef USE_LOG
			ecrobot_debug1((UINT)figurelSeqNo, (UINT)(forward), 0);
			ecrobot_bt_data_logger((S8)(figurelSeqNo), (S8)(forward));
			#endif
			break;
		}
		case FIGUREL_SEQ_NO_16: // 倒立走行→尻尾走行
		{
			
			if(count == 0)
			{
				BalanceToTail_Init();
			}
			
			if(BalanceToTailMain() == 1)
			{
				figurelSeqNo = FIGUREL_SEQ_NO_17;
				forward = 0;
				count = 0;
			
			}else{
				count++;
			}
			#ifdef USE_LOG
			ecrobot_debug1((UINT)figurelSeqNo, (UINT)(forward), 0);
			ecrobot_bt_data_logger((S8)(figurelSeqNo), (S8)(forward));
			#endif
			break;
		}
		case FIGUREL_SEQ_NO_17: // 尻尾走行		右に90度回転（ラインを探すため）
		{
			turn = 70;
			forward = 0;
	
			if(count == 0)
			{
				init_rotateDistance(TURN_RIGHT);		/* 回転距離測定初期化 */
				
			}
			
			//ターン完了判定
			if(check_rotateComplete(SEQ_NO_17_TURN))
			{
				
				figurelSeqNo = FIGUREL_SEQ_NO_18;
				
				InitTailRunControlStraightP();
				init_DistanceFromPoint();
				
				turn = 0;
				
				count = 0;
			}
			else
			{
				TailRunControl(forward, turn);
				tail_pid_control(tailKp, tailKi, tailKd, TAIL_ANGLE_TAILRUN, tailMaxMv, tailMinMv); 
				count++;	
			}
			#ifdef USE_LOG
			ecrobot_debug1((UINT)figurelSeqNo, (UINT)(forward), 0);
			ecrobot_bt_data_logger((S8)(figurelSeqNo), (S8)(forward));
			#endif
			break;
		}
		case FIGUREL_SEQ_NO_18: // 倒立走行（直進制御）			ラインの右側に出る
		{
			// 倒立開始からの距離(cm)
			float distance = calc_getDistanceFromPoint() * 100.0f;
			
			if(distance >= SEQ_NO_18_DISTANCE)
			{
				figurelSeqNo = FIGUREL_SEQ_NO_19;
				
				forward = 0;
				
				count = 0;
			}
			else
			{
				forward = SEQ_NO_18_FORWARD;	// 15
				turn = 0;
				tail_pid_control(tailKp, tailKi, tailKd, TAIL_ANGLE_TAILRUN, tailMaxMv, tailMinMv);
				TailRunControlStraightP(forward, 3.0f);
				
				count++;
			}
			#ifdef USE_LOG
			ecrobot_debug1((UINT)figurelSeqNo, (UINT)(forward), 0);
			ecrobot_bt_data_logger((S8)(figurelSeqNo), (S8)(forward));
			#endif
			break;
		}
		case FIGUREL_SEQ_NO_19: // 倒立走行（直進制御）			ラインを探す
		{
			U16 lightSearchLine = (white + black) / 2;
			if(lightValue >= (lightSearchLine * SEQ_NO_19_GRAY))
			{
				figurelSeqNo = FIGUREL_SEQ_NO_20;
				count = 0;
			}
			else
			{
				forward = SEQ_NO_19_FORWARD;
				turn = 0;
				tail_pid_control(tailKp, tailKi, tailKd, TAIL_ANGLE_TAILRUN, tailMaxMv, tailMinMv);
				TailRunControlStraightP(forward, -3.0f);
				
				count++;
			}
			#ifdef USE_LOG
			ecrobot_debug1((UINT)figurelSeqNo, (UINT)(forward), 0);
			ecrobot_bt_data_logger((S8)(lightValue / 10), (S8)(lightSearchLine / 10));
			#endif
			break;
		}
		case FIGUREL_SEQ_NO_20: // 尻尾走行		左に回転（ライン復帰のため）
		{
			turn = -70;
			forward = 0;
	
			if(count == 0)
			{
				init_rotateDistance(TURN_LEFT);		/* 回転距離測定初期化 */
				
			}
			
			//ターン完了判定
			if(check_rotateComplete(SEQ_NO_20_TURN))
			{
				return 1;
			}
			else
			{
				TailRunControl(forward, turn);
				tail_pid_control(tailKp, tailKi, tailKd, TAIL_ANGLE_TAILRUN, tailMaxMv, tailMinMv); 
				count++;	
			}
			#ifdef USE_LOG
			ecrobot_debug1((UINT)figurelSeqNo, (UINT)(calc_getRotateDistance() * 100.0f), 0);
			ecrobot_bt_data_logger((S8)(figurelSeqNo), (S8)(calc_getRotateDistance() * 100.0f));
			#endif
			break;
		}
		default:
		{
			forward = 0;
			turn = 0;
			break;
		}
	}
	
	/* 全体ロギング */
	//ecrobot_bt_data_logger((S8)(figurelSeqNo), (S8)(forward));
	ecrobot_debug1((UINT)figurelSeqNo, 0, 0);
#ifdef USE_LOG
	//ecrobot_bt_data_logger((S8)(figurelSeqNo), (S8)(forward));
	//ecrobot_debug1((UINT)figurelSeqNo, 0, 0);
#endif

	return 0;
}
