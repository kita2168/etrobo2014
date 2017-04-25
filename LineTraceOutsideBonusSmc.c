// 2012/08/14 created by s.sahara

// OUT側ボーナス走行状態遷移マシン

#include "LineTraceOutsideBonusSmc.h"

#include "SensorIn.h"
#include "Control.h"

#include "LineTraceControl.h"
#include "LineTraceParam.h"

#include "CourseDef.h"
#include "LineTraceOutsideBasicSmc.h"

#include "Timer.h"
#include "BalanceToTail.h"

#if 0
// SEQ区間タイム測定
static UINT mT = 0;
static S8 mSeqTime[LT_OUT2_SEQ_MAX];
static S8 mSeqDist[LT_OUT2_SEQ_MAX];
#endif

static signed char forward;		// 前後進命令
static signed char turn;		// 旋回命令

static signed char forwardTarget = 0;		// 目標速度指令
static signed char forwardTargetPrev = 0;		// 前回目標速度指令
static float accelT = 0.0f;
static float accelTmax = 0;

static char onMarker = 0;
//static char onMarkerPrev = 0;

static float hensa0;
static float integral;

static float Kspd = 1.0f;
#if 0 //del by k.nakagawa 区間ごとのゲイン変更を削除
static float dirKp = BALANCERUN_DIRECTION_KP;
static float dirKi = BALANCERUN_DIRECTION_KI;
static float dirKd = BALANCERUN_DIRECTION_KD;
#endif

static int seqNoPrv = -1;

/* 通常尻尾走行制御用ゲイン */
#define DIRECTION_KP_TEST 			(0.5f)

#define DIRECTION_MV_MAX 			(100.0f)
#define DIRECTION_MV_MIN 			(-100.0f)

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

void LineTraceOutsideBonusSmc_Init(signed char arg_forward)
{
#if 0
	U16 black = LineTraceControl_LightBlackThreshold();
	U16 white = LineTraceControl_LightWhiteThreshold();
#endif

	onMarker = 0;
	//onMarkerPrev = 0;

#if 0
	mT = 0;
	for(int i=0;i<LT_OUT2_SEQ_MAX;i++)
	{
		mSeqTime[i] = 0;
		mSeqDist[i] = 0;
	}
#endif

	forward = arg_forward;
	forwardTarget = arg_forward; // 初期目標速度指令
	forwardTargetPrev = arg_forward;

	// 前回偏差保持値の初期化
	hensa0 = 0.0f;
	integral = 0;

	seqNoPrv = -1;
}

#if 0
// ※PU測定用
void LineTraceOutsideBonusSmc_logTurn(signed char turn)
{
	static S32 integralTurn = 0;
	//
	if(turn > 0){
		if(integralTurn < 0)integralTurn = 0;
	}
	else if(turn < 0){
		if(integralTurn > 0)integralTurn = 0;
	}
	integralTurn += turn;
#ifdef USE_LOG
	//ecrobot_bt_data_logger(turn, (S8)integralTurn);
#endif
	//ecrobot_debug1((U16)turn, (U16)integral, (U16)(integral * -1));
}

void SaveSeqEndTime(UINT seqNo, UINT absTime, float distanceAbs)
{
	mSeqTime[seqNo] = (S8)(absTime * 0.004 * 10.0); // [1/10sec]
	mSeqDist[seqNo] = (S8)(distanceAbs * 10.0);	// [1/10seq]
}

void LineTraceOutsideBonusSmc_BTSendLapTime()
{
	for(int i=0;i<LT_OUT2_SEQ_MAX;i++)
	{
#ifdef USE_LOG
		ecrobot_bt_data_logger(mSeqDist[i], mSeqTime[i]);
#endif
		systick_wait_ms(10);
	}
}
#endif

void LineTraceOutsideBonusSmc_ChangeSpeed(float spd, float Tmax)
{
	forwardTargetPrev = forward;
	forwardTarget = spd;
	accelT = 0.0f;
	accelTmax = Tmax;
}

#if 0 //del by k.nakagawa 区間ごとのゲイン変更を削除
void LineTraceOutsideBonusSmc_ChangeDirGain(float k)
{
	dirKp = BALANCERUN_DIRECTION_KP * k;
	dirKi = BALANCERUN_DIRECTION_KI * k;
	dirKd = BALANCERUN_DIRECTION_KD * k;
}
#endif

// OUTコース後半(ボーナスコース)シーケンス
// seqNo : シーケンス番号
// distanceAbs : 開始からの絶対距離[m]
// courseJointDetected : コース継ぎ目検知出力
// distanceCorrection : 開始からの絶対距離補正値出力
int LineTraceOutsideBonusSmc_Smc(UINT seqNo, float distanceAbs, int* courseJointDetected, float* distanceCorrection)
{
	static int onMarkerCounter = 0;
	static float distanceAbsMarkerStart = 0.0f;
	static float initDistance = 0.0f;

	UINT seqNoNext = seqNo;

#if 0
	U16 black = LineTraceControl_LightBlackThreshold();
	U16 white = LineTraceControl_LightWhiteThreshold();
#endif

	// 尻尾PIDとりあえず適当
	float tailKp = TAIL_KP;
	float tailKi = TAIL_KI;
	float tailKd = TAIL_KD;
	float tailMaxMv = 100.0f;
	float tailMinMv = -100.0f;


	UINT str = 0;
	float mumu;

	switch(seqNo)
	{
	case LT_OUT2_SEQ_00: //最後の直線に入った後に、速度を中速に落として、マーカー探索シーケンスに入る準備
		if(seqNoPrv != seqNo){
			LineTraceOutsideBonusSmc_ChangeSpeed(40, 0.3f);
			initDistance = distanceAbs;
		}

		// 安定するまで0.10mライントレース走行
		if(distanceAbs - initDistance > 0.10f)
		{
			seqNoNext = LT_OUT2_SEQ_00_1;
		}
		break;
	case LT_OUT2_SEQ_00_1: // 中速で、マーカー探索し、ルックアップゲート前マーカー前端を検出

		mumu = get_model_mu();
		if(mumu<-0.04f || 0.04f<mumu)
		{
			distanceAbsMarkerStart = distanceAbs;
			change_model_black_to_gray();
			onMarker = 1;
			seqNoNext = LT_OUT2_SEQ_01;
		}
		break;
	case LT_OUT2_SEQ_01: //ルックアップゲート前マーカー検知後
	{
		if(seqNoPrv != seqNo)
		{
			LineTraceOutsideBonusSmc_ChangeSpeed(10, 0.1f);
			BalanceToTail_Init();
			onMarkerCounter = 0;
		}

		if(BalanceToTailMain() == 1)
		{
			seqNoNext = LT_OUT2_SEQ_01_1;
		}
		break;
	}
	case LT_OUT2_SEQ_01_1: //少しバック
	{
		if(seqNoPrv != seqNo)
		{
			LineTraceOutsideBonusSmc_ChangeSpeed(-20, 0.1f);
			initDistance = distanceAbs;
			change_model_black_to_gray();
			reset_model_state();
			InitTailRunControlStraightP();
		}

		tail_pid_control(tailKp, tailKi, tailKd, TAIL_ANGLE_TAILRUN, tailMaxMv, tailMinMv);

		if(distanceAbs - initDistance < -0.1f)
		{
			seqNoNext = LT_OUT2_SEQ_02;
		}
		break;
	}
	case LT_OUT2_SEQ_02: //マーカー内をライントレース
	{
		if(seqNoPrv != seqNo)
		{
			LineTraceOutsideBonusSmc_ChangeSpeed(30, 0.5f);
			initDistance = distanceAbs;
		}

		tail_pid_control(tailKp, tailKi, tailKd, TAIL_ANGLE_TAILRUN, tailMaxMv, tailMinMv);

		if(distanceAbs - initDistance > 0.02f)
		{
			seqNoNext = LT_OUT2_SEQ_02_1;
		}
		break;
	}
	case LT_OUT2_SEQ_02_1://ルックアップゲート開始まで
	{
		tail_pid_control(tailKp, tailKi, tailKd, TAIL_ANGLE_TAILRUN, tailMaxMv, tailMinMv);

		mumu = get_model_mu();
		if(mumu<-0.04f || 0.04f<mumu)
		{
			change_model_gray_to_black();
			reset_model_state();
			onMarker = 2;
			*courseJointDetected = OUT_COURSE_PART_ON_LOOKUPGATE;
			seqNoNext = LT_OUT2_SEQ_03;
		}
		break;
	}
	case LT_OUT2_SEQ_03: //6 走行を安定させるライントレース
	{
		if(seqNoPrv != seqNo)
		{
			LineTraceOutsideBonusSmc_ChangeSpeed(30, 1.0f);
			initDistance = distanceAbs;
			change_model_tail_to_black();
		}

		tail_pid_control(tailKp, tailKi, tailKd, TAIL_ANGLE_TAILRUN, tailMaxMv, tailMinMv);

		if(distanceAbs - initDistance > 0.02f)
		{
			seqNoNext = LT_OUT2_SEQ_03_1;
		}
		break;
	}

	case LT_OUT2_SEQ_03_1: //7 ゴール前マーカー前端を検知

		tail_pid_control(tailKp, tailKi, tailKd, TAIL_ANGLE_TAILRUN, tailMaxMv, tailMinMv);

		mumu = get_model_mu();
//		if(mumu<-0.04f || 0.04f<mumu)
		if(mumu<-0.015f)
		{
			change_model_black_to_gray();
			seqNoNext = LT_OUT2_SEQ_03_2;
		}

		break;
	case LT_OUT2_SEQ_03_2: //8 ゴール前マーカー内をライントレース走行

		tail_pid_control(tailKp, tailKi, tailKd, TAIL_ANGLE_TAILRUN, tailMaxMv, tailMinMv);
		if(seqNoPrv != seqNo)
		{
			LineTraceOutsideBonusSmc_ChangeSpeed(30, 0.3f);
			onMarker = 1;
			initDistance = distanceAbs;
			reset_model_state();
		}

		if(distanceAbs - initDistance > 0.08f)
		{
			seqNoNext = LT_OUT2_SEQ_04;
		}
		break;
	case LT_OUT2_SEQ_04: //9 マーカー内を走行、マーカー終端を検知前
	{
		tail_pid_control(tailKp, tailKi, tailKd, TAIL_ANGLE_TAILRUN, tailMaxMv, tailMinMv);

		mumu = get_model_mu();
		if(mumu<-0.04f || 0.04f<mumu)
		{
			change_model_gray_to_black();
			seqNoNext = LT_OUT2_SEQ_05;
		}
		break;
	}
	case LT_OUT2_SEQ_05: //10 入庫
	{
		tail_pid_control(tailKp, tailKi, tailKd, TAIL_ANGLE_TAILRUN, tailMaxMv, tailMinMv);

		if(seqNoPrv != seqNo)
		{
			LineTraceOutsideBonusSmc_ChangeSpeed(20, 0.5f);
			initDistance = distanceAbs;
			reset_model_state();
		}


		if(distanceAbs - initDistance > 0.25f)
		{
			seqNoNext = LT_OUT2_SEQ_06;
		}
		break;
	}
	case LT_OUT2_SEQ_06: //ゴール
	{
		TailRunControl(0, 0);
		tail_pid_control(tailKp, tailKi, tailKd, TAIL_ANGLE_TAILRUN, tailMaxMv, tailMinMv);
		break;
	}

	default:
		#if 1
		// その場停止等何もしない場合
		#else
		// そのまま進む場合（データ取りしたい場合）
		str = 1;
		#endif
		// ecrobot_debug1((UINT)seqNo, (UINT)(distanceAbs * 1000.0f), onMarker);
		break;
	}

	forward = (signed char)SCurve((float)forwardTarget, (float)forwardTargetPrev, accelTmax, &accelT);
	forward = (S8)(forward * Kspd);	// debug用補正


	// 走行体制御
	//TurnRecord(turn); //turn値を記録するために、制御毎に必ず呼ぶこと
	if(str) turn = 0;

	if(seqNo < LT_OUT2_SEQ_01)
	{
		if(1 == onMarker && onMarkerCounter <= 50)
		{
			BalanceControlStraightP(forward, 3.0f);
		}
		else
		{
			// 方向陀PID
			ctrl_direction((float)forward);
			turn = get_model_turn();
			forward = get_model_forward();
			BalanceControl(forward, turn);
		}
	}
	else if(seqNo >= LT_OUT2_SEQ_02 && seqNo <= LT_OUT2_SEQ_05)
	{
		// 方向陀PID
		ctrl_direction((float)forward);
		turn = get_model_turn();
		forward = get_model_forward();
		TailRunControl(forward, turn);
	}
	else if(seqNo == LT_OUT2_SEQ_01_1)
	{
		TailRunControlStraightP(forward, -10.0f);
	}

#ifdef USE_LOG
	//ecrobot_bt_data_logger((S8)seqNo, (S8)(get_model_mu()*1000)); // マーカ検知閾確認用
	//ecrobot_bt_data_logger((S8)seqNo, (S8)turn);
	ecrobot_bt_data_logger((S8)seqNo, (S8)(get_model_light()/8));
#endif

	seqNoPrv = seqNo;
	return seqNoNext;
}

