// 2012/08/13 created by s.sahara

// IN側ベーシック走行状態遷移マシン

// 2013/07/17～27 mod by s.sahara ベーシックステージでは尻尾走行が禁止になった為、尻尾走行関連の処理を倒立走行に変更または削除

#include "LineTraceInsideBasicSmc.h"

#include "SensorIn.h"
#include "Control.h"

#include "LineTraceControl.h"
#include "LineTraceParam.h"

#include "CourseDef.h"

#define abs(X)	((X) < 0 ? -(X) : (X))

// SEQ区間タイム測定
#if 0
static UINT mT = 0;
static S8 mSeqTime[LT_IN1_SEQ_MAX];
static S8 mSeqDist[LT_IN1_SEQ_MAX];
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

//#################################################################
// kuniライントレース用の関数の宣言 add by kunitake <2014/08/13>
//#################################################################
extern void init_model();
extern void ctrl_direction(float ref_vel);
extern signed char get_model_forward();
extern signed char get_model_turn();
//extern float get_model_mu();


void LineTraceInsideBasicSmc_Init(signed char forward)
{
#if 0
	U16 black = LineTraceControl_LightBlackThreshold();
	U16 white = LineTraceControl_LightWhiteThreshold();
#endif

	onMarker = 0;
//	onMarkerPrev = 0;

#if 0
	mT = 0;
	for(int i=0;i<LT_IN1_SEQ_MAX;i++)
	{
		mSeqTime[i] = 0;
		mSeqDist[i] = 0;
	}
#endif

	forwardTarget = forward; // 初期目標速度指令
	forwardTargetPrev = forward;

	// 前回偏差保持値の初期化
	hensa0 = 0.0f;
	integral = 0;

	seqNoPrv = -1;

	init_model();
}

signed char LineTraceInsideBasicSmc_GetForward()
{
	return forward;
}

#if 0
// ※PU測定用
void LineTraceInsideBasicSmc_logTurn(signed char turn)
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

void LineTraceInsideBasicSmc_BTSendLapTime()
{
	for(int i=0;i<LT_IN1_SEQ_MAX;i++)
	{
#ifdef USE_LOG
		ecrobot_bt_data_logger(mSeqDist[i], mSeqTime[i]);
#endif
		systick_wait_ms(10);
	}
}
#endif

void ChangeSpeed(float spd, float Tmax)
{
	forwardTargetPrev = forward;
	forwardTarget = spd;
	accelT = 0.0f;
	accelTmax = Tmax;
}

#if 0 //del by k.nakagawa 区間ごとのゲイン変更を削除
void LineTraceInsideBasicSmc_ChangeDirGain(float k)
{
	dirKp = BALANCERUN_DIRECTION_KP * k;
	dirKi = BALANCERUN_DIRECTION_KI * k;
	dirKd = BALANCERUN_DIRECTION_KD * k;
}
#endif

// 2012/08/12 add by s.sahara >>>>
// INコース前半(ベーシックコース)シーケンス
// seqNo : シーケンス番号
// distanceAbs : 開始からの絶対距離[m]
// courseJointDetected : コース継ぎ目検知出力
// distanceCorrection : 開始からの絶対距離補正値出力
int LineTraceInsideBasicSmc_Smc(UINT seqNo, float distanceAbs, int* courseJointDetected, float* distanceCorrection)
{
#if 0 //del by k.nakagawa マーカー検知用定義の削除
	static UINT lastStraightdetected;
	static float distanceAbsMarkerStart = 0.0f;

	U16 black = LineTraceControl_LightBlackThreshold();
	U16 white = LineTraceControl_LightWhiteThreshold();

	U16 lightValue = GetLightInLPF();
	U16 md_mid_lightValue = md_mid_GetLightInLPF();
	U16 md_slow_lightValue = md_slow_GetLightInLPF();
	float md = ((float)(md_mid_lightValue) / (float)(md_slow_lightValue) - 1.0f) * 1000.0f;
#endif

#if 0 //del by k.nakagawa 安全機能の削除
	U8 autoBrake = false;
#endif

#if 0 //del by k.nakagawa
	UINT outLpf = 0;
#endif

	UINT str = 0;
	UINT seqNoNext = seqNo;

	switch(seqNo)
	{
	case LT_IN1_SEQ_00: // 通常のライントレース
		#if 1
		//autoBrake = true;
		if(seqNoPrv != seqNo) ChangeSpeed(100, 0.3f);
		#else
		if(seqNoPrv != seqNo) ChangeSpeed(BALANCERUN_FORWARD_SPEED, 0.3f);
		#endif
		//outLpf = 1;

		if(distanceAbs > IN1_NORMAL_DISTANCE)
		{
			seqNoNext = LT_IN1_SEQ_13;
		}
		// ecrobot_debug1((UINT)seqNo, (UINT)(accelT * 1000.0f), (UINT)(forward));
		// ecrobot_debug1((UINT)seqNo, (UINT)(distanceAbs * 1000.0f), (UINT)(lightValue));
		break;

	case LT_IN1_SEQ_13: // FigureLのシーケンスに入る
		*courseJointDetected = IN_COURSE_PART_BEFORE_FIGUREL;
		ecrobot_debug1((UINT)seqNo, (UINT)(distanceAbs * 1000.0f), onMarker);
		break;

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


#if 0 //del by k.nakagawa 安全機能の削除
	// 2013/09/21 add by s.sahara 安全機能
	if(autoBrake)
	{
		#if 0 // 2013/09/21時点、ライン逸脱時自動ブレーキのみで十分
		// 曲線走行自動ブレーキ
		static float integralDirAng;
		float dirAngle = LinTraceControl_GetCurveAngle();
		float Kspdc = 1.0f - ((dirAngle / 90.0f) * (dirAngle / 90.0f) * integralDirAng);
		integralDirAng += (dirAngle / 90.0f) * (dirAngle / 90.0f); // 抵抗
		integralDirAng *= 0.975f;	// 漸減
		if(Kspdc < 0.1f) Kspdc = 0.1f; // 飽和
		#endif

		// ライン逸脱時自動ブレーキ
		float lightnorm = abs(GetNormalizedLightLevel()) * 0.01f;
		if(lightnorm <= 1.0f) lightnorm = 0.0f;
		else lightnorm -= 1.0f;
		float Kspdl = 1.0f - lightnorm * lightnorm * 0.5f;
		if(Kspdl < 0.2f) Kspdl = 0.2f; // 飽和

		//forward = (S8)(forward * Kspdc * Kspdl);
		forward = (S8)(forward * 1.0f * Kspdl);
		//ecrobot_debug1((UINT)(lightnorm*100), forward, (UINT)(Kspdl*100));
	}
#endif

	// SEQ区間タイム計測
	//if(seqNo != seqNoPrv) SaveSeqEndTime(seqNoPrv, mT, distanceAbs);

	// 方向陀PID
#if 0
	//turn = LineTraceControl_DirectionPID1(dirKp, dirKi, dirKd, &hensa0, &integral, BALANCERUN_DIRECTION_MV_MAX, BALANCERUN_DIRECTION_MV_MIN, outLpf); // out-LPF付き
	turn = LineTraceControl_DirectionPID(dirKp, dirKi, dirKd, &hensa0, &integral, BALANCERUN_DIRECTION_MV_MAX, BALANCERUN_DIRECTION_MV_MIN);
	//ecrobot_debug1((UINT)seqNo, (UINT)(distanceAbs * 1000.0f), onMarker);
#else
	ctrl_direction((float)forward);
	turn = get_model_turn();
	forward = get_model_forward();
#endif

	// 走行体制御
	//TurnRecord(turn); //turn値を記録するために、制御毎に必ず呼ぶこと
	if(str) turn = 0;
	//if(onMarker) turn -= 4; // 右に行くので左
	//if(onMarker) turn -= 3; // 右に行くので左

	BalanceControl(forward, turn);

#ifdef USE_LOG
	//ecrobot_bt_data_logger((S8)(seqNo), (S8)(integral)); // マーカ検知閾確認用
	//ecrobot_bt_data_logger((S8)(md_mid_lightValue - 600), (S8)(md)); // マーカ検知閾確認用
	//ecrobot_bt_data_logger((S8)(get_model_mu()*100), (S8)(get_model_turn())); // マーカ検知閾確認用
	//ecrobot_bt_data_logger((S8)(md_mid_lightValue - 600), (S8)(md_slow_lightValue - 600)); // マーカ検知閾確認用
	//ecrobot_bt_data_logger((S8)(lightValue*0.1), (S8)(md_mid_lightValue*0.1));
	//ecrobot_bt_data_logger((S8)seqNo, (S8)turn);
	//ecrobot_bt_data_logger((S8)seqNo, (S8)forward);
#endif

	//LineTraceInsideBasicSmc_logTurn(turn); // PU測定用

	//mT++;
	seqNoPrv = seqNo;
	return seqNoNext;
}
// 2012/08/12 add by s.sahara <<<<

