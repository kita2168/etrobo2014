// 2012/08/13 created by s.sahara

// ライントレース共通方向陀PID処理

#include "LineTraceControl.h"
#include "LineTraceParam.h"

#include "Calc.h"
#include "SensorIn.h"
#include "Control.h"

static U16 mLightBlack = LIGHT_BLACK;
static U16 mLightWhite = LIGHT_WHITE;
static U16 mLightBlack2 = LIGHT_BLACK;
static U16 mLightWhite2 = LIGHT_WHITE;

// balancerより
extern F32 K_PHIDOT;
#define rt_SATURATE(sig,ll,ul)         (((sig) >= (ul)) ? (ul) : (((sig) <= (ll)) ? (ll) : (sig)) )

#if 0
// 2012/09/14 add by s.sahara >>>>
// ライントレースの出力用LPF用
static float omega0;
static float a;
static float b0;
static float b1;
static float b2;
static float a1;
static float a2;
static float pre1;
static float pre2;
static U16 ltOutLPF = 0;
// 2012/09/14 add by s.sahara <<<<
#endif

static S32 prev_motor_left_value;
static S32 prev_motor_right_value;
static S32 motor_left_value;
static S32 motor_right_value;
static float curveAngle;

void LinTraceControl_Periodic()
{
	prev_motor_left_value = motor_left_value;
	prev_motor_right_value = motor_right_value;
	motor_left_value = nxt_motor_get_count(NXT_PORT_C);
	motor_right_value = nxt_motor_get_count(NXT_PORT_B);
	// 各車輪の移動距離[m]
	float lLd = (prev_motor_left_value - motor_left_value) * DEG2RAD * WHEEL_D/2.0;
	float lRd = (prev_motor_right_value - motor_right_value) * DEG2RAD * WHEEL_D/2.0;
	// 走行カーブ角度[deg]（左回転が＋）
	curveAngle = (lRd - lLd) / DEG2RAD;
}

float LinTraceControl_GetCurveAngle()
{
	return curveAngle * 1000 / CONTROL_CYCLE; // deg/sec
}

// 2012/08/12 mod by s.sahara >>>>
void BalanceControl(char forward, char turn)
{
	BalanceControl1(forward, turn, 0.0f);
}

void BalanceControl1(char forward, char turn, float addGyroOffset)
{
	signed char pwm_L, pwm_R; /* 左右モータPWM出力 */

	/* 倒立振子制御(forward = 0, turn = 0で静止バランス) */
	balance_control(
		(float)forward,									/* 前後進命令(+:前進, -:後進) */
		(float)(turn),									/* 旋回命令(+:右旋回, -:左旋回) */
		(float)ecrobot_get_gyro_sensor(NXT_PORT_S1),	/* ジャイロセンサ値 */
		(float)GYRO_OFFSET + addGyroOffset,				/* ジャイロセンサオフセット値 */
		(float)nxt_motor_get_count(NXT_PORT_C),			/* 左モータ回転角度[deg] */
		(float)nxt_motor_get_count(NXT_PORT_B),			/* 右モータ回転角度[deg] */
		(float)ecrobot_get_battery_voltage(),			/* バッテリ電圧[mV] */
		&pwm_L,											/* 左モータPWM出力値 */
		&pwm_R);										/* 右モータPWM出力値 */
	nxt_motor_set_speed(NXT_PORT_C, pwm_L, 1);			/* 左モータPWM出力セット(-100～100) */
	nxt_motor_set_speed(NXT_PORT_B, pwm_R, 1);			/* 右モータPWM出力セット(-100～100) */
}
// 2012/08/12 mod by s.sahara <<<<


// 2012/08/13 add by s.sahara >>>>
void TailRunControl(char forward, char turn)
{
	// balancerを参考に作成
	F32 tmp_pwm_turn = (rt_SATURATE((F32)turn, -100.0f, 100.0f) / CMD_MAX) * K_PHIDOT;
	F32 tmp_limitter_p = 100.0f;
	F32 tmp_limitter_m = -100.0f;
	F32 tmp_pwm_r = rt_SATURATE(forward, -100.0f, 100.0f);
	F32 tmp_pwm_l = rt_SATURATE(forward, -100.0f, 100.0f);

	if(forward >= 0)
	{
		if(turn > 0) tmp_limitter_p = 100.0f - tmp_pwm_turn;
		if(turn < 0) tmp_limitter_p = 100.0f + tmp_pwm_turn;
		if(turn != 0)
		{
			tmp_pwm_r = rt_SATURATE(forward, -100.0f, tmp_limitter_p);
			tmp_pwm_l = rt_SATURATE(forward, -100.0f, tmp_limitter_p);
		}
		tmp_pwm_l += tmp_pwm_turn;
		tmp_pwm_r -= tmp_pwm_turn;
	}
	else
	{
		if(turn > 0) tmp_limitter_m = -100.0f + tmp_pwm_turn * -1;
		if(turn < 0) tmp_limitter_m = -100.0f - tmp_pwm_turn * -1;
		if(turn != 0)
		{
			tmp_pwm_r = rt_SATURATE(forward, tmp_limitter_m, 100.0f);
			tmp_pwm_l = rt_SATURATE(forward, tmp_limitter_m, 100.0f);
		}
		tmp_pwm_l -= tmp_pwm_turn;
		tmp_pwm_r += tmp_pwm_turn;
	}

	nxt_motor_set_speed(NXT_PORT_C, (S8)tmp_pwm_l, 1);			/* 左モータPWM出力セット(-100～100) */
	nxt_motor_set_speed(NXT_PORT_B, (S8)tmp_pwm_r, 1);			/* 右モータPWM出力セット(-100～100) */
}
// 2012/08/13 add by s.sahara <<<<

void LineTraceControl_InitLightThreshold(U16 lightBlack, U16 lightWhite)
{
	mLightBlack = lightBlack;
	mLightWhite = lightWhite;

	//InitLPF(LIGHT_LPF_HZ, &omega0, &a, &b0, &b1, &b2, &a1, &a2);
}

U16 LineTraceControl_LightBlackThreshold()
{
	return mLightBlack;
}

U16 LineTraceControl_LightWhiteThreshold()
{
	return mLightWhite;
}

void LineTraceControl_InitLightThreshold2(U16 lightBlack, U16 lightWhite)
{
	mLightBlack2 = lightBlack;
	mLightWhite2 = lightWhite;

	//InitLPF(LIGHT_LPF_HZ, &omega0, &a, &b0, &b1, &b2, &a1, &a2);
}

U16 LineTraceControl_LightBlackThreshold2()
{
	return mLightBlack2;
}

U16 LineTraceControl_LightWhiteThreshold2()
{
	return mLightWhite2;
}

U16 LineTraceControl_getLED_Band_Width()
{
	return mLightBlack - mLightWhite;
}

U8 LineTraceControl_getLED_Band_Width_Per()
{
	return (U8)((float)(mLightBlack - mLightWhite) / (float)(mLightBlack) * 100.0f);
}

//入力１：今回値の光センサレベル
//入力２：前回値の光センサレベル（1周期前）
//返り値：白が0%、黒が100%としたときの変化速度（%）
float LineTraceControl_getLight_Band_Rel_P(U16 lightlevel, U16 prevlightlevel)
{
	//今回値-前回値（白→黒は+、黒→白は-)
	return ((float)(lightlevel - prevlightlevel) / (float)(mLightBlack - mLightWhite) * 100.0f);
}

static U8 makerdetect_initflg = 0;
static S8 maker_detect = 0;
static U16 init_Maker_detect_count = 0;
void LineTraceControl_MakerDetect_Init()
{
	makerdetect_initflg = 0;
	maker_detect = 0;
	init_Maker_detect_count = 0;

	return;
}

S8 LineTraceControl_getMaker_detect()
{
	return maker_detect;
}

static float minDelta=0;
static float minAccel=0;
static float maxDelta=0;
static float maxAccel=0;
static float minPer=0;
static float maxPer=0;

#include "BtDatalog.h"
#include "CourseDef.h"
//1ms周期のマーカー判定を行う
//光センサの値、光センサの変化速度、変化加速度で判定
//判定結果はmaker_detectに入れる
S8 LineTraceControl_MakerDetect()
{
	if(init_Maker_detect_count >= 1000)
	{
		static U16 prevLightIn;
		static float prevDelta;

		U16 lightIn = GetLightIn();
		if(makerdetect_initflg == 0)
		{
			prevLightIn = lightIn;
		}

		//前回値からの光の変化量(%)を求める
		float delta = LineTraceControl_getLight_Band_Rel_P(lightIn, prevLightIn);

		if(makerdetect_initflg == 0)
		{
			prevDelta = delta;
			makerdetect_initflg = 1;
		}

		float accel = delta - prevDelta;
		//ecrobot_debug1(123, (UINT)(delta * 10.0f), (UINT)(accel * 10.0f));
		//ecrobot_debug1(234, (UINT)(lightIn), (UINT)(prevLightIn));
		if(delta < minDelta)
		{
			minDelta = delta;
		}
		if(accel < minAccel)
		{
			minAccel = accel;
		}
		if(delta > maxDelta)
		{
			maxDelta = delta;
		}
		if(accel > maxAccel)
		{
			maxAccel = accel;
		}

		prevLightIn = lightIn;
		prevDelta = delta;

		U16 band_width = LineTraceControl_getLED_Band_Width();
		//現在値が黒色から何％離れているか計算
		float per = (float)(mLightBlack - lightIn) / (float)band_width * 100.0f;
		if(per < minPer)
		{
			minPer = per;
		}
		if(per > maxPer)
		{
			maxPer = per;
		}

		if(makerdetect_initflg == 1 && maker_detect == 0)
		{
			//マーカー入る前
//			if(per > 55.0f)
			{
//				if(delta < -2.0f && accel < -2.0f)
				if(delta < -10.0f && accel < -10.0f)
				{
					maker_detect = 1;
				}
			}
		}
#if 0 //マーカー離脱光レベルで検知
		else if(maker_detect == 1)
		{
			//マーカー出る前
			if(per > 75.0f)
			{
				if(delta > 4.0f && accel > 4.0f)
				{
					maker_detect = -1;
				}
			}
		}
#endif
#if 0
		U32 systick = ecrobot_get_systick_ms();
		if(systick % 4 == 0)
		{
			SINT var[16] = {0};
			var[0] = (SINT)(minDelta * 10.0f);
			var[1] = (SINT)(minAccel * 10.0f);
			var[2] = (SINT)(maxDelta * 10.0f);
			var[3] = (SINT)(maxAccel * 10.0f);
			var[4] = (SINT)(per);
			var[6] = (SINT)(minPer);
			var[7] = (SINT)(maxPer);
			ecrobot_sint_var_monitor(var);
		}
#endif
#ifdef USE_1MS_BT_LOG
		return bt_data_log_append(2, (S8)(delta * 10.0f), (S8)(accel * 10.0f), (U16)lightIn);
#else
		return 0;
#endif
	}
	else
	{
		init_Maker_detect_count++;
		return 0;
	}

}

void LineTraceControl_ChangeLightThreshold(U16 lightBlack, U16 lightWhite, float* pHensa0, float* pIntegral)
{
	mLightBlack = lightBlack;
	mLightWhite = lightWhite;

	// 閾値変わると制御用保持値も更新する必要あり。
	//U16 lightValue = GetLightIn3();
	//U16 lightValue = GetLightInLPF();
	#if 1 // 2012/09/11 mod by s.sahara 幅のminで-100, maxで+100に変更（光レベル幅が変わっても物理幅は同じとする）
	float command = (mLightWhite + mLightBlack) / 2.0f; // 白と黒の中間
	float lightLevel = ((float)(command - GetLightInLPF()) / (float)(mLightBlack - command)) * 100.0f * -1.0f;
	*pHensa0 = 0 - (((float)(command - lightLevel) / ((float)(mLightBlack - mLightWhite) / 2.0f)) * 100.0f);
	#endif
	*pIntegral = 0.0f;
}

// 2012/09/05 add by s.sahara >>>>
// -1～～+1の範囲に正規化する
float LineTraceControl_Normalize(float in)
{
	float widhth = mLightBlack - mLightWhite;
	//float in2 = in - mLightWhite;
	float norm = in / widhth * 2 - 1.0f; // +-1なので幅は2。中心オフセットが-1
	return norm;
}
// 2012/09/05 add by s.sahara <<<<

#if 1 // 2012/09/16 add by s.sahara ライン逸脱検知
#define DETECT_DEVIATE_FROM_LINE_MAX (120.0f)		// 何%外れたら検知するか
#define DETECT_DEVIATE_FROM_LINE_MIN (-120.0f)		// 何%外れたら検知するか
static float lightLevelOnLastDetectDeviate;		// ライン逸脱時光レベル保持値
static U8 holdLightLevelOnLastDetectDeviate;	// ライン逸脱時光レベル保持フラグ
U8 IsDeviateFromLine(){ return holdLightLevelOnLastDetectDeviate; }
void ResetDetectDeviateFromLine(){ holdLightLevelOnLastDetectDeviate = 0; }
float GetLightLevelOnLastDetectDeviate(){ return lightLevelOnLastDetectDeviate; }
static float normalizedLightLevel;
float GetNormalizedLightLevel(){ return normalizedLightLevel; }
#endif

signed char LineTraceControl_DirectionPID(float kp, float ki, float kd,
	float* pHensa0, float* pIntegral, float max, float min)
{
	// 方向舵 PID
	U16 lightValue = GetLightInLPF();
	float hensa1 = 0.0f;
	#if 1 // 2012/09/11 mod by s.sahara 幅のminで-100, maxで+100に変更（光レベル幅が変わっても物理幅は同じとする）
	float command = (mLightWhite + mLightBlack) / 2.0f; // 白と黒の中間
	normalizedLightLevel = ((float)(command - lightValue) / (float)(mLightBlack - command)) * 100.0f * -1.0f;
	#endif
	#if 1 // 2012/09/16 add by s.sahara ライン逸脱検知
	if(!holdLightLevelOnLastDetectDeviate){
		if( !(DETECT_DEVIATE_FROM_LINE_MIN <= normalizedLightLevel
				&& normalizedLightLevel <= DETECT_DEVIATE_FROM_LINE_MAX) ){
			lightLevelOnLastDetectDeviate = normalizedLightLevel;
			holdLightLevelOnLastDetectDeviate = 1;
		}
	}
	#endif
	#if 1
	// PIDはアナログ値ではなく光レベル相対量（±100の範囲）で制御するので、目標値は0。
	float mv = ControlPID(kp, ki, kd, 0, normalizedLightLevel,
		*pHensa0, &hensa1, pIntegral, max, min);	// 2012/09/13 mod by s.sahara 値をレベルに、目標を0に変更。（中心を目指すことには変わりない）
	#endif
	*pHensa0 = hensa1;
	//ltOutLPF = LPF(lightValue, omega0, a, b0, b1, b2, a1, a2, &pre1, &pre2);
	//if(enableLpf) return (signed char)ltOutLPF;
	//else return (signed char)mv;
#ifdef MAKE_INSIDE
	return (signed char)(-mv);
#else
	return (signed char)mv;
#endif
}

//*****************************************************************************
// 関数名 : tail_control
// 引数 : angle (モータ目標角度[度])
// 返り値 : 無し
// 概要 : 走行体完全停止用モータの角度制御
//*****************************************************************************
void tail_control(signed int angle)
{
	float pwm = (float)(angle - nxt_motor_get_count(NXT_PORT_A))*P_GAIN; /* 比例制御 */
	/* PWM出力飽和処理 */
	if (pwm > PWM_ABS_MAX)
	{
		pwm = PWM_ABS_MAX;
	}
	else if (pwm < -PWM_ABS_MAX)
	{
		pwm = -PWM_ABS_MAX;
	}

	nxt_motor_set_speed(NXT_PORT_A, (signed char)pwm, 1);
}

static float tail_hensa0;
static float tail_integral;

void tail_pid_control_reset()
{
	tail_hensa0 = 0;
	tail_integral = 0;
}

void tail_pid_control(float kp, float ki, float kd, float angle, float max, float min)
{
	float command = angle;
	float error = (float)(nxt_motor_get_count(NXT_PORT_A));
	float pwm = ControlPID(kp, ki, kd, command, error, tail_hensa0, &tail_hensa0, &tail_integral, max, min);
	nxt_motor_set_speed(NXT_PORT_A, (signed char)pwm, 1);
}

/*
 * 2012/09/08 直進
 * turn値は0から補正（P制御）
 */
static float bcs_p_left_value;
static float bcs_p_right_value;

void InitBalanceControlStraightP()
{
	bcs_p_left_value = nxt_motor_get_count(NXT_PORT_C);
	bcs_p_right_value = nxt_motor_get_count(NXT_PORT_B);
}

void BalanceControlStraightP(char forward, float kp)
{
	BalanceControl1StraightP(forward, kp, 0.0f);
}

void BalanceControl1StraightP(char forward, float kp, float offset)
{
	// 現在値 - 初期値
	float left_diff = nxt_motor_get_count(NXT_PORT_C) - bcs_p_left_value;
	float right_diff = nxt_motor_get_count(NXT_PORT_B) - bcs_p_right_value;

	float delta_turn = (float)(right_diff - left_diff) * kp;

#ifdef USE_LOG
	//ecrobot_bt_data_logger((S8)(deltaTurn), 0);
#endif

	BalanceControl1(forward, (signed char)delta_turn, offset);
}

static float trcs_p_left_value;
static float trcs_p_right_value;

void InitTailRunControlStraightP()
{
	trcs_p_left_value = nxt_motor_get_count(NXT_PORT_C);
	trcs_p_right_value = nxt_motor_get_count(NXT_PORT_B);
}

void TailRunControlStraightP(char forward, float kp)
{
	// 現在値 - 初期値
	float left_diff = nxt_motor_get_count(NXT_PORT_C) - trcs_p_left_value;
	float right_diff = nxt_motor_get_count(NXT_PORT_B) - trcs_p_right_value;
	float delta_turn = (float)(right_diff - left_diff) * kp;

	TailRunControl(forward, (signed char)delta_turn);
}
