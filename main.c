/* 2012/07/21 created by s.sahara
・ETロボコン公式サイトで配布のetrobo2012sample(nxtOSEK)内のsample_c4を元に
	難所クリア単体テスト用プログラムを作成。
・
*/

#include "Error.h"
#include "CourseDef.h"

/**
 ******************************************************************************
 **	ファイル名 : sample.c
 **
 **	概要 : 2輪倒立振子ライントレースロボットのTOPPERS/ATK1(OSEK)用Cサンプルプログラム
 **
 ** 注記 : sample_c4 (sample_c3にBluetooth通信リモートスタート機能を追加)
 ******************************************************************************
 **/
#include "kernel.h"
#include "kernel_id.h"
#include "ecrobot_interface.h"
#include "balancer.h" /* 倒立振子制御用ヘッダファイル */

#include "Calc.h"
#include "LineTraceControl.h"
#include "LineTraceParam.h"
#include "SensorIn.h"
#include "Control.h"

// 2012/07/21 add by s.sahara >>>>
#include "LineTrace.h"			// 通常走行
//#include "SlopeStage.h"			// 坂道
#include "LookUpGateStage.h"	// ルックアップゲート
// 2012/07/21 add by s.sahara <<<<
// 2012/08/09 add by t.akiyama >>>>
//#include "StepStage.h"
#include "FigureLStage.h"
// 2012/08/09 add by t.akiyama <<<<
// 2012/08/12 add by t.akiyama >>>>
//#include "DriftTurnStage.h"
// 2012/08/12 add by t.akiyama <<<<
#include "BalanceToTail.h"
#include "ReturnToLine.h"

#include "BtDatalog.h"

#include "SpeedoMeter.h"
#include "AngleMeter.h"
#include "LineTraceControl.h"

DeclareCounter(SysTimerCnt);
DeclareResource(resource1);

enum Mode
{
	BEFORE_INIT,
	BEFORE_RESET_TAIL_POS_ZERO,
	BEFORE_START_ENCODER_RESET,
	BEFORE_CALIBRATION,
	BEFORE_RESET_TAIL_POS_ZERO2,
	BEFORE_START_ENCODER_RESET2,
	BEFORE_CALIBRATION2,
	BEFORE_STARTLINE_SET,
	BEFORE_TOUCH,
	BEFORE_RUN_INIT,
	START_UP, // 倒立で走行開始時の走行安定待ち直進加速
	RUN,
	EXPLOR_LINE,	// ライン探索
	EXIT,			// 即停止
	STOP,			// 減速停止
	BT_SEND, //BTログの送信
};
static U8 mode = BEFORE_INIT;

/* sample_c4マクロ */
#ifdef STEC_EAST
#define DEVICE_NAME 	  "ET205-E"  /* Bluetooth通信用デバイス名 */
#define PASS_KEY		  "steceast" /* Bluetooth通信用パスキー */
#else
#define DEVICE_NAME 	  "ET205-W"  /* Bluetooth通信用デバイス名 */
#define PASS_KEY		  "stecwest" /* Bluetooth通信用パスキー */
#endif

#define CMD_START		  '1'	 /* リモートスタートコマンド(変更禁止) */

/* 関数プロトタイプ宣言 */
static int remote_start(void);

/* Bluetooth通信用データ受信バッファ */
char rx_buf[BT_MAX_RX_BUF_SIZE];

static UINT sw_prv;

#if 0	//スタート時直進加速の削除
static int iStartUp;
static signed char vMax;
static signed char vTarget;
static float accelT;
static float accelTmax;
#endif



// #################################################################################
// ### ここはmonoスタートアップ処理用の変数の宣言 add by mononobe <2014/09/06> #####
static signed char forward;		// 前後進命令
static signed char turn;		// 旋回命令
static int enc_l = 0;	//... Left エンコーダ現在値
static int enc_r = 0;	//... Rightエンコーダ現在値
static int enc_l0 = 0;	//... Left エンコーダ過去値
static int enc_r0 = 0;	//... Rightエンコーダ過去値

static int cnt_stup = 0;	//... スタートアップ待機カウント

// #### monoスタートアップ処理の変数宣言 終了 ######################################
// #################################################################################


// ###########################################################
// ここから、kuniライントレース用変数 add by kunitake <2014/08/13>
// ###########################################################
// ＜参考文献＞
// ・非ホロノミックDriftlessシステムのフィードバック制御　／　三平満司，石川将人
//#define __OUTPUT_KUNI__
extern U16 LineTraceControl_LightBlackThreshold();
extern U16 LineTraceControl_LightWhiteThreshold();
#define SAMP (0.004)
#define TREAD (100.0)
#define TIRE_RAD (20.0)
#define MAX_FORWARD (100.0)
#define MAX_TURN (5.0)
#define FREQ_CUTOFF (20.0)
#define Q_FACT (1.0)
#define FREQ_CUTOFF2 (20.0)
#define Q_FACT2 (1.0)
//#define RATIO_Y (0.15)				// ラインエッジからの距離[mm] / 輝度差  ※会議室で成功パターン
#ifdef MAKE_INSIDE
#define RATIO_Y (-0.05)
#define RATIO_Y_GRAY (-0.10)
#else
#define RATIO_Y (0.05)
#define RATIO_Y_GRAY (0.10)
#endif
//#define GAIN_K1 (0.002)  ※会議室で成功パターン
//#define GAIN_K2 (0.03)  ※会議室で成功パターン
//#define GAIN_K1 (0.003) // GAIN_K2=0.12のときの下限値
#define GAIN_K1 (0.006)
#define GAIN_K2 (0.12)

struct model_nxtway{
	signed char cmd_forward;		// 直進の指令値
	signed char cmd_turn;			// 旋回の指令値
	int fai_r;		int fai_l;		// 車輪の回転角度[deg]
	int fai_r_old;	int fai_l_old;	// １つ前の車輪の回転角度[deg]
	float dr0;	float dr1;	float dr2;	float dr3;	float dr4;	float dr5;	// スタートしてからの右車輪で進んだ距離（0～5まであるのはLPF用）
	float dl0;	float dl1;	float dl2;	float dl3;	float dl4;	float dl5;	// スタートしてからの左車輪で進んだ距離（0～5まであるのはLPF用）
	float vr;		float vl;		// 左右車輪の並進速度[mm/s]
	float forward;		// 直進速度
	float turn;		// 旋回速度

	float x0;	// ライン接線方向の距離
	float y0;	float y1;	float y2;	float y3;	float y4;	float y5;	// ラインからの横方向距離
	float z0;	// ラインに対する角度θ
	float y_dot;	// yの速度

	float a0, a1, a2;		// y値のローパスフィルタ
	float b0, b1, b2;		// y値のローパスフィルタ
	float c0, c1, c2;		// 車輪移動距離のローパスフィルタ
	float d0, d1, d2;		// 車輪移動距離のローパスフィルタ
	float mu;				// フィードバック量
	float gain_k1;			// フィードバックゲイン１
	float gain_k2;			// フィードバックゲイン２

	U16 light;				// 現在の光センサ値
	U16 light_c;			// 追従すべき光センサ値
	float ratio_y;			// ラインエッジからの距離[mm] / 輝度差
};
static struct model_nxtway model1;

#include <math.h>
// kuniライントレースに使う関数
float line_distance_table(U16 light_value)
{
	return (float)(-model1.ratio_y * (light_value - model1.light_c));
}

void init_model(){
	float w0, al;
	model1.fai_r = nxt_motor_get_count(NXT_PORT_B);
	model1.fai_l = nxt_motor_get_count(NXT_PORT_C);
	model1.fai_r_old = model1.fai_r;
	model1.fai_l_old = model1.fai_l;
	model1.dr0 = 0.0;	model1.dr1 = 0.0;	model1.dr2 = 0.0;	model1.dr3 = 0.0;	model1.dr4 = 0.0;	model1.dr5 = 0.0;
	model1.dl0 = 0.0;	model1.dl1 = 0.0;	model1.dl2 = 0.0;	model1.dl3 = 0.0;	model1.dl4 = 0.0;	model1.dl5 = 0.0;
	model1.vr = 0.0;
	model1.vl = 0.0;
	model1.forward = 0.0;
	model1.turn = 0.0;

	model1.x0 = 0.0;
	model1.y0 = 0.0;	model1.y1 = 0.0;	model1.y2 = 0.0;	model1.y3 = 0.0;	model1.y4 = 0.0;	model1.y5 = 0.0;
	model1.z0 = 0.0;
	model1.y_dot = 0.0;


	w0 = DEG2RAD*360.0*FREQ_CUTOFF / (1.0/SAMP);
	al = sin(w0) / Q_FACT;
	model1.a0 = 1.0 + al;
	model1.a1 = -2.0*cos(w0);
	model1.a2 = 1.0 - al;
	model1.b0 = (1.0 - cos(w0)) * 0.5;
	model1.b1 = 1.0 - cos(w0);
	model1.b2 = model1.b0;

	w0 = DEG2RAD*360.0*FREQ_CUTOFF2 / (1.0/SAMP);
	al = sin(w0) / Q_FACT2;
	model1.c0 = 1.0 + al;
	model1.c1 = -2.0*cos(w0);
	model1.c2 = 1.0 - al;
	model1.d0 = (1.0 - cos(w0)) * 0.5;
	model1.d1 = 1.0 - cos(w0);
	model1.d2 = model1.d0;

	model1.mu = 0.0;
	model1.gain_k1 = GAIN_K1;
	model1.gain_k2 = GAIN_K2;

	model1.light = ecrobot_get_light_sensor(NXT_PORT_S3);
	model1.light_c = (U16)(0.5*LineTraceControl_LightBlackThreshold() + 0.5*LineTraceControl_LightWhiteThreshold());
	model1.ratio_y = RATIO_Y;
	// 現在値からスタートするため
	model1.y0 = line_distance_table(model1.light);
	model1.y1 = model1.y0;	model1.y2 = model1.y0;	model1.y3 = model1.y0;	model1.y4 = model1.y0;	model1.y5 = model1.y0;
}

void change_model_black_to_gray()
{
//	model1.light_c = model1.light;
	model1.light_c = (U16)(0.25*LineTraceControl_LightBlackThreshold() + 0.75*LineTraceControl_LightWhiteThreshold());
	model1.ratio_y = RATIO_Y_GRAY;
}
void change_model_gray_to_black()
{
	model1.light_c = (LineTraceControl_LightBlackThreshold() + LineTraceControl_LightWhiteThreshold())/2;
	model1.ratio_y = RATIO_Y;
}
void change_model_black_to_tail()
{
	model1.light_c = (LineTraceControl_LightBlackThreshold2() + LineTraceControl_LightWhiteThreshold2())/2;
	model1.ratio_y = RATIO_Y * 4;
}
void change_model_tail_to_black()
{
	model1.light_c = (LineTraceControl_LightBlackThreshold() + LineTraceControl_LightWhiteThreshold())/2;
	model1.ratio_y = RATIO_Y;
}
void reset_model_state()
{
	// 現在値からスタートするため
	model1.y0 = line_distance_table(model1.light);
	model1.y1 = model1.y0;	model1.y2 = model1.y0;	model1.y3 = model1.y0;	model1.y4 = model1.y0;	model1.y5 = model1.y0;
	model1.z0 = 0.0;
}

void get_model_state(){
	// 更新
	model1.dr2 = model1.dr1;
	model1.dr1 = model1.dr0;
	model1.dr5 = model1.dr4;
	model1.dr4 = model1.dr3;
	model1.dl2 = model1.dl1;
	model1.dl1 = model1.dl0;
	model1.dl5 = model1.dl4;
	model1.dl4 = model1.dl3;

	model1.y2 = model1.y1;
	model1.y1 = model1.y0;
	model1.y5 = model1.y4;
	model1.y4 = model1.y3;


	// 車輪の角度を取得して、左右車輪の並進速度を計算
	model1.fai_r = nxt_motor_get_count(NXT_PORT_B);
	model1.fai_l = nxt_motor_get_count(NXT_PORT_C);
	model1.dr0 = TIRE_RAD*DEG2RAD*(model1.fai_r - model1.fai_r_old);
	model1.dl0 = TIRE_RAD*DEG2RAD*(model1.fai_l - model1.fai_l_old);
	model1.dr3 = (model1.d0 * model1.dr0 + model1.d1 * model1.dr1 + model1.d2 * model1.dr2
					- model1.c1 * model1.dr4 - model1.c2 * model1.dr5) / model1.c0;
	model1.dl3 = (model1.d0 * model1.dl0 + model1.d1 * model1.dl1 + model1.d2 * model1.dl2
					- model1.c1 * model1.dl4 - model1.c2 * model1.dl5) / model1.c0;

	model1.vr = (model1.dr3 - model1.dr4) / SAMP;
	model1.vl = (model1.dl3 - model1.dl4) / SAMP;

	// 車輪速度から、直進速度と旋回速度を求める
	model1.forward = (model1.vl + model1.vr) / 2.0;
	model1.turn = (model1.vr - model1.vl) / 2.0 / TREAD;

	// 観測量
	// センサ輝度値からy, y(0)を求め、過去データy(-1)とともにy_dotを求める
	// さらに上記の直進速度から、theta=asin(y_dot/forward)からtan(theta)を求める
	model1.light = ecrobot_get_light_sensor(NXT_PORT_S3);
	model1.y0 = line_distance_table(model1.light);

	model1.y3 = (model1.b0 * model1.y0 + model1.b1 * model1.y1 + model1.b2 * model1.y2
					- model1.a1 * model1.y4 - model1.a2 * model1.y5) / model1.a0;
	model1.y_dot = (model1.y3 - model1.y4) / SAMP;

	if(model1.forward<=-0.001 || 0.001<=model1.forward) model1.z0 = asin(model1.y_dot / model1.forward);
}

void ctrl_direction(float ref_vel){
	float mu1, mu2;
	float u1, u2;		// 速度指令値
	float tmp_f, tmp_t;

	// 現在の速度を取得＆計算
	mu1 = ref_vel;
	mu2 = (-model1.gain_k1) * model1.y0 + (-model1.gain_k2) * tan(model1.z0);
	model1.mu = mu2;

	u1 = mu1 / cos(model1.z0);
	u2 = pow(cos(model1.z0),2.0) * mu2 * mu1;

	// u1,u2をPWMに換算して、balance_controlへの入力とする
	tmp_f = 100.0 * u1/MAX_FORWARD;
	if(tmp_f<-100.0) model1.cmd_forward = -100;
	else if(tmp_f>100.0) model1.cmd_forward = 100;
	else model1.cmd_forward = (signed char)tmp_f;

	tmp_t = -100.0 * u2/MAX_TURN;
	if(tmp_t<-100.0) model1.cmd_turn = -100;
	else if(tmp_t>100.0) model1.cmd_turn = 100;
	else model1.cmd_turn = (signed char)tmp_t;

}
signed char get_model_forward()
{
	return model1.cmd_forward;
}
signed char get_model_turn()
{
	return model1.cmd_turn;
}
float get_model_mu()
{
	return model1.mu;
}
float get_model_y()
{
	return model1.y0;
}

U16 get_model_center()
{
	return model1.light_c;
}
U16 get_model_light()
{
	return model1.light;
}
//######################################################################



//*****************************************************************************
// 関数名 : ecrobot_device_initialize
// 引数 : なし
// 戻り値 : なし
// 概要 : ECROBOTデバイス初期化処理フック関数
//*****************************************************************************
void ecrobot_device_initialize()
{
	ecrobot_set_light_sensor_active(NXT_PORT_S3); /* 光センサ赤色LEDをON */
	ecrobot_init_sonar_sensor(NXT_PORT_S2); /* 超音波センサ(I2C通信)を初期化 */
	nxt_motor_set_count(NXT_PORT_A, 0); /* 完全停止用モータエンコーダリセット */
	ecrobot_init_bt_slave(PASS_KEY); /* Bluetooth通信初期化 */
}

//*****************************************************************************
// 関数名 : ecrobot_device_terminate
// 引数 : なし
// 戻り値 : なし
// 概要 : ECROBOTデバイス終了処理フック関数
//*****************************************************************************
void ecrobot_device_terminate()
{
	ecrobot_set_light_sensor_inactive(NXT_PORT_S3); /* 光センサ赤色LEDをOFF */
	ecrobot_term_sonar_sensor(NXT_PORT_S2); /* 超音波センサ(I2C通信)を終了 */
	ecrobot_term_bt_connection(); /* Bluetooth通信を終了 */
}

//*****************************************************************************
// 関数名 : user_1ms_isr_type2
// 引数 : なし
// 戻り値 : なし
// 概要 : 1msec周期割り込みフック関数(OSEK ISR type2カテゴリ)
//*****************************************************************************
void user_1ms_isr_type2(void)
{
	(void)SignalCounter(SysTimerCnt);
}

#ifdef MAKE_INSIDE
int Cource_InSide_Main()
{
	static int part = IN_COURSE_PART_ON_START;
	int courseJointDetected = 0;

	/* コース別走行処理 */
	int res;
	float distansAbs = calc_getDistanceFromStart();
	switch(part)
	{
		case IN_COURSE_PART_ON_FIGUREL:
			res = FigureLStageMain();
			if(1 == res)
			{
				ReturnToLine_Init();
				//part = IN_COURSE_PART_ON_RETURN_TO_LINE;
				part = IN_COURSE_PART_BEFORE_GOAL;
			}
			break;
		case IN_COURSE_PART_ON_GOAL:
			//LineTrace_StopAndStand(distansAbs);
			break;
		case IN_COURSE_PART_ON_RETURN_TO_LINE:
			res = ReturnToLineMain();
			if(1 == res)
			{
				part = IN_COURSE_PART_BEFORE_GOAL;
			}
			break;
		default:
			res = LineTraceMain(COURSE_IN_SIDE, part, distansAbs, &courseJointDetected);
			//if(distansAbs > STR_1_DISTANCE) courseJointDetected = IN_COURSE_PART_ON_GOAL; // 第一ストレート終端で止まってみる
			break;
	}

	/* コース切り替えイベント */
	switch(courseJointDetected)
	{
		case IN_COURSE_PART_ON_FIGUREL:		/* フィギュアL部へ遷移 */
			/* コース切り替えイベント処理をここに書く */

			/* 切り替え */
			part = IN_COURSE_PART_ON_FIGUREL;
			break;
		case IN_COURSE_PART_ON_GOAL:		/* ゴール部へ遷移 */
			/* コース切り替えイベント処理をここに書く */

			/* 切り替え */
			part = IN_COURSE_PART_ON_GOAL;
			break;
		default:
			break;
	}
	courseJointDetected = 0; // reset
	return part;
}
#endif // MAKE_INSIDE

#ifndef MAKE_INSIDE
int Cource_OutSide_Main()
{
	static int part = OUT_COURSE_PART_ON_START;
	int courseJointDetected = 0;
	int res;
	float distansAbs = calc_getDistanceFromStart();

	/* コース別走行処理 */
	switch(part)
	{

		case OUT_COURSE_PART_ON_LOOKUPGATE:
			res = LookUpGateStageMain();
			if(1 == res)
			{
				ReturnToLine_Init();
				part = OUT_COURSE_PART_BEFORE_GOAL;
			}
			break;

		/* 2012/08/09 add t.akiyama <<<< */
		case OUT_COURSE_PART_ON_GOAL:
			//LineTrace_StopAndStand(distansAbs);
			break;
		case OUT_COURSE_PART_ON_RETURN_TO_LINE:
			//res = ReturnToLineMain();
			//if(1 == res)
			//{
			//	part = OUT_COURSE_PART_BEFORE_GOAL;
			//}
			break;
		default:
			res = LineTraceMain(COURSE_OUT_SIDE, part, distansAbs, &courseJointDetected);
			break;
	}

	/* コース切り替えイベント */
	switch(courseJointDetected)
	{
		case OUT_COURSE_PART_ON_LOOKUPGATE:
			/* コース切り替えイベント処理をここに書く */

			part = OUT_COURSE_PART_ON_LOOKUPGATE;
			break;

		/* 2012/08/09 add t.akiyama <<<< */
		case OUT_COURSE_PART_ON_GOAL:		/* ゴール部へ遷移 */
			/* コース切り替えイベント処理をここに書く */

			part = OUT_COURSE_PART_ON_GOAL;
			break;
		default:
			break;
	}
	courseJointDetected = 0; // reset
	return part;
}
#endif // !MAKE_INSIDE

// リセットするために、尻尾を上げる（自分に当たるまで）
static U16 reset_tail_pos_zero_count = 0;
static UINT delta_zero_count = 0;
static int encA_prv = 0;

void tail_zero_count_clear()
{
	reset_tail_pos_zero_count = 0;
	delta_zero_count = 0;
	encA_prv = 0;
}
	

void reset_tail_pos_zero()
{
#if 0 //音は後で対応
	ecrobot_sound_tone(550U, 100U, 50U);
	systick_wait_ms(300);
	ecrobot_sound_tone(0U, 0U, 0U);
	systick_wait_ms(300);

	for(int i=0;i<10;i++)
	{
		nxt_motor_set_speed(NXT_PORT_A, (signed char)-20, 1);
		systick_wait_ms(10);
	}
#endif
	int encA = nxt_motor_get_count(NXT_PORT_A);

	if(0<=reset_tail_pos_zero_count && reset_tail_pos_zero_count<25)
	{
		nxt_motor_set_speed(NXT_PORT_A, (signed char)-20, 1);
	}
#if 0
	for(int i=0;i<200;i++) // 長い時間行き着かなければ諦めるようにしておく
	{
				systick_wait_ms(10);
	}
#endif

	if(25<=reset_tail_pos_zero_count && reset_tail_pos_zero_count<500)
	{
		if(encA == encA_prv)
		{
			delta_zero_count++;
//			if(delta_zero_count > 30) break;
			if(delta_zero_count > 75) mode++;
		}
		else
		{
			delta_zero_count = 0;
		}
		nxt_motor_set_speed(NXT_PORT_A, (signed char)-20, 1);
	}

	encA_prv = encA;

	reset_tail_pos_zero_count++;
}

// 白黒キャリブレーション
void LineDetectionlevelCalibration()
{
	static U8 LineDetectionlevelCalibration_mode = 0;

	// ライン横切り動作
	static signed char pwm = 20;		// できるだけ低速にする
	static U16 lightValueMax = 0;		// アナログmin値で初期化
	static U16 lightValueMin = 1023;	// アナログmax値で初期化

	// ※白黒をまたぐように、ラインに90°向けて置いて、スイッチを押すとn cmスキャンする
	if(LineDetectionlevelCalibration_mode == 0)
	{
		nxt_motor_set_count(NXT_PORT_C, 0); // 左モータエンコーダリセット
		nxt_motor_set_count(NXT_PORT_B, 0); // 右モータエンコーダリセット
		LineDetectionlevelCalibration_mode++;
	}
	else if(LineDetectionlevelCalibration_mode == 1)
	{
		// 2012/08/13 add by s.sahara >>>>
		//ecrobot_sound_tone(440U, 100U, 50U);
		//systick_wait_ms(300);
		//ecrobot_sound_tone(0U, 0U, 0U);
		//systick_wait_ms(300);

		// 尻尾出して待機
		//while(1)
		//{
//			tail_control(108);	// 108°(固定)で測定（補正用データの基準を108°とした為）

			if(mode==BEFORE_CALIBRATION2) tail_control(GATE_TAIL_ANGLE_UNDER_GATE);		// ゲートをくぐるときの角度
			else tail_control(TAIL_ANGLE_FOR_BALANCE_RUN_START);

			UINT sw = ecrobot_get_touch_sensor(NXT_PORT_S4);
			if ((1 == sw) && (0 == sw_prv))
			{
				//ecrobot_sound_tone(440U, 100U, 50U);
				//for(int i=0;i<30;i++)
				//{
				//	tail_control(TAIL_ANGLE_TAILRUN);	// 尻尾走行用角度で停止
				//	systick_wait_ms(10);
				//}
				//ecrobot_sound_tone(0U, 0U, 0U);
				//for(int i=0;i<30;i++)
				//{
				//	tail_control(TAIL_ANGLE_TAILRUN);	// 尻尾走行用角度で停止
				//	systick_wait_ms(10);
				//}
				//break; // タッチセンサが押された
				LineDetectionlevelCalibration_mode++;
			}
			sw_prv = sw;
#ifdef USE_LOG
			// ecrobot_bt_data_logger((S8)0, (S8)0);
#endif
			//systick_wait_ms(10);
		//}
	}
	else if(LineDetectionlevelCalibration_mode == 2)
	{
		//while(1)
		//{
			U16 lightValue = ecrobot_get_light_sensor(NXT_PORT_S3); // 0-1023
			float distanceAbs = calc_getDistanceFromStart();
#ifdef USE_LOG
//			ecrobot_bt_data_logger((U8)(distanceAbs*1000.0), (U8)(lightValue-500));
//...			ecrobot_bt_data_logger((U8)(distanceAbs*1000.0), (U8)(lightValue/4));
#endif

//			tail_control(TAIL_ANGLE_TAILRUN);	// 尻尾走行用角度で停止
			if(mode==BEFORE_CALIBRATION2) tail_control(GATE_TAIL_ANGLE_UNDER_GATE);		// ゲートをくぐるときの角度
			else tail_control(TAIL_ANGLE_FOR_BALANCE_RUN_START);

			nxt_motor_set_speed(NXT_PORT_B, pwm, 1);
			nxt_motor_set_speed(NXT_PORT_C, pwm, 1);

			if(lightValue > lightValueMax) lightValueMax = lightValue;
			if(lightValue < lightValueMin) lightValueMin = lightValue;
			//if(distanceAbs > 0.08f) break; // 8cm横切らせる
			if(distanceAbs > 0.08f) LineDetectionlevelCalibration_mode++; // 8cm横切らせる

			ecrobot_debug1(lightValueMax, lightValueMin, lightValue);
#ifdef USE_LOG
			//ecrobot_bt_data_logger((S8)(distanceAbs * 1000.0f), (S8)((lightValue - 580)*0.5f)); // 横切り波形測定用
#endif
		//	systick_wait_ms(4);
		//}
	}
	else if(LineDetectionlevelCalibration_mode == 3)
	{
		nxt_motor_set_speed(NXT_PORT_B, 0, 1);
		nxt_motor_set_speed(NXT_PORT_C, 0, 1);

		// 白黒の端は切る（max-min幅の端のn%をカット）
		int delta = lightValueMax - lightValueMin;

		if(mode==BEFORE_CALIBRATION2)
			LineTraceControl_InitLightThreshold2(lightValueMax - (U16)(delta * 0.2f), lightValueMin + (U16)(delta * 0.2f)); // 20%としておく
		else
			LineTraceControl_InitLightThreshold(lightValueMax - (U16)(delta * 0.2f), lightValueMin + (U16)(delta * 0.2f)); // 20%としておく

		LineDetectionlevelCalibration_mode = 0;
		sw_prv = 0;
		mode++; //スタートラインセット
	}

	//TODO キャリブレーション終了を音でスタータに示す
}

//設置位置決定用関数
//TODO キャリブレーション結果による最適な設置位置を音でスタータに示す
int SetRobotToStartLine()
{
	tail_control(TAIL_ANGLE_FOR_BALANCE_RUN_START);	// バランス走行用角度で停止

	if (remote_start() == 1)
	{
		return 1;
	}
	UINT sw = ecrobot_get_touch_sensor(NXT_PORT_S4);
	if ((1 == sw) && (0 == sw_prv))
	{
		sw_prv = sw;
		return 1;
	}
	sw_prv = sw;

	return 0;
}

#define abs(X)	((X) < 0 ? -(X) : (X))

// 障害検知
static U16 curveCounter;
int DetectFail()
{
	// 転倒検知（前後角速度が一定以上で検知。難所で角度変わるところでは使用しないこと）
	S32 angle = abs(AngleMeter());
	if(angle > FALL_DETECTION_ANLE_RATE) return ERROR_FALL;

	// 旋回連続検知
	LinTraceControl_Periodic();
	float curveAngle = abs(LinTraceControl_GetCurveAngle());
	if(curveAngle > 15) curveCounter++;
	else curveCounter = 0;
	if(curveCounter > (1000 / CONTROL_CYCLE)) return ERROR_LOST;
	// ecrobot_debug1(curveAngle, curveCounter, 0);

	//ecrobot_debug1(GetNormalizedLightLevel(), GetLightLevelOnLastDetectDeviate(), IsDeviateFromLine());
	if(IsDeviateFromLine()) return ERROR_DEVIATE;

	return 0;
}

// 異常検知有効
// returns 1:有効 0:無効
U8 IsDetectFailEnable(int courseSide, int part)
{
	if(COURSE_IN_SIDE == courseSide)
	{
		switch(part)
		{
//		case IN_COURSE_PART_BEFORE_SLOPE:
//		case IN_COURSE_PART_AFTER_SLOPE:
		case IN_COURSE_PART_BEFORE_FIGUREL:
		case IN_COURSE_PART_BEFORE_GOAL:
			return 1;
		case IN_COURSE_PART_ON_START:
//		case IN_COURSE_PART_ON_SLOPE:
		case IN_COURSE_PART_ON_FIGUREL:
		case IN_COURSE_PART_ON_GOAL:
		case IN_COURSE_PART_ON_RETURN_TO_LINE:
			break;
		}
	}
	else
	{
		switch(part)
		{
//		case OUT_COURSE_PART_BEFORE_SLOPE:
//		case OUT_COURSE_PART_AFTER_SLOPE:
		case OUT_COURSE_PART_BEFORE_LOOKUPGATE:
		case OUT_COURSE_PART_BEFORE_GOAL:
			return 1;
		case OUT_COURSE_PART_ON_START:
//		case OUT_COURSE_PART_ON_SLOPE:
		case OUT_COURSE_PART_ON_LOOKUPGATE:
		case OUT_COURSE_PART_ON_GOAL:
			break;
		}
	}
	return 0;
}


//*****************************************************************************
// タスク名 : TaskMain
// 概要 : メインタスク
//*****************************************************************************
TASK(TaskMain)
{
	GetResource(resource1);

	static int part = 0;
#if 0		//障害検知の削除
	U8 detectFailEnable = IsDetectFailEnable(STARTING_SIDE, part);
#endif

	//ecrobot_debug1(mode,0,0);
#ifdef USE_LOG
	//ecrobot_bt_data_logger(mode, 0);
#endif
	switch(mode)
	{
		case BEFORE_INIT:
			ecrobot_device_initialize();

			/**
			 * Bluetooth通信用デバイス名の変更は、Bluetooth通信接続が確立されていない場合のみ有効です。
			 * 通信接続確立時にはデバイス名は変更されません。(下記のAPIは何もしません)
			 */
			ecrobot_set_bt_device_name(DEVICE_NAME);

			tail_zero_count_clear();
			mode++;
			break;

		case BEFORE_RESET_TAIL_POS_ZERO:
		case BEFORE_RESET_TAIL_POS_ZERO2:
			reset_tail_pos_zero();	// 2012/08/17 add by s.sahara
			break;

		case BEFORE_START_ENCODER_RESET:
		case BEFORE_START_ENCODER_RESET2:
			// キャリブレーションの為ここでもリセット
			nxt_motor_set_count(NXT_PORT_C, 0); // 左モータエンコーダリセット
			nxt_motor_set_count(NXT_PORT_B, 0); // 右モータエンコーダリセット
			nxt_motor_set_count(NXT_PORT_A, 0); // 尻尾モータエンコーダリセット
			tail_zero_count_clear();
			mode++;
			break;

		case BEFORE_CALIBRATION:
		case BEFORE_CALIBRATION2:
			//TASK(Task_1ms)で白黒キャリブレーション中
			LineDetectionlevelCalibration();
			break;

		case BEFORE_STARTLINE_SET:
			// 走行開始位置へ設置
			if(1 == SetRobotToStartLine())
			{
				// セット完了
				initSpeedoMeter(4);
				initAngleMeter(4);
				mode++;
			}
			break;

		case BEFORE_TOUCH:
			// 開始トリガ待ち
			//LightIn();		// 光センサ入力
			#ifndef USE_SAMPLE_COURCE
			tail_control(TAIL_ANGLE_FOR_BALANCE_RUN_START);	// バランス走行用角度で停止
			//tail_control(TAIL_ANGLE_TAILRUN);	// 尻尾走行用角度で停止
			#else // #ifndef USE_SAMPLE_COURCE
			#ifdef TEST_SLOW_FORWARD_RUN
			tail_control(108);
			#else // #ifdef TEST_SLOW_FORWARD_RUN
			tail_control(TAIL_ANGLE_TAILRUN);	// 尻尾走行用角度で停止
			#endif // #ifdef TEST_SLOW_FORWARD_RUN
			#endif // #ifndef USE_SAMPLE_COURCE

			if (remote_start() == 1)
			{
				// リモートスタート
				mode++;
			}

			UINT sw = ecrobot_get_touch_sensor(NXT_PORT_S4);
			if ((1 == sw) && (0 == sw_prv))
			{
				// タッチセンサが押された
				sw_prv = sw;
				mode++;
			}
			sw_prv = sw;
			break;

		case BEFORE_RUN_INIT:
#if 0	//スタート時直進加速の削除
			// 直進制御付きで倒立走行が安定するまでスタート加速 -- 初期化 2013/08 add by s.sahara >>>>
			iStartUp = 0;
			accelT = 0.0f;
			accelTmax = 0.5f;
			vMax = 60; // 安定して加速できる目標速度（実機調整）
			InitBalanceControlStraightP();
			// <<<<
#endif
			balance_init();						/* 倒立振子制御初期化 */
			nxt_motor_set_count(NXT_PORT_C, 0); /* 左モータエンコーダリセット */
			nxt_motor_set_count(NXT_PORT_B, 0); /* 右モータエンコーダリセット */

			LineTrace_Init();

			//##########################################################
			// kuniライントレースの初期化 add by kunitake <2014/08/13>
			//##########################################################
			//init_model();  ←これは上の「LineTrace_Init()」の中で実行


			mode++;
			break;

		// 直進制御付きで倒立走行が安定するまでスタート加速 2013/08 add by s.sahara
		case START_UP:
#if 0	//障害検知の削除
			switch(DetectFail()){
				case ERROR_FALL: mode = EXIT; break;
				case ERROR_LOST:
					BalanceToTail_Init();
					mode = STOP;
					break;
			}
#endif
#if 0	//スタート時直進加速の削除
			if(iStartUp < (accelTmax*1000/4))
			{
				// 尻尾をたたむ
				tail_pid_control(TAIL_KP, TAIL_KI, TAIL_KD, 0, TAIL_MAX_MV, TAIL_MIN_MV);
				// 倒立直線制御で微速で前進開始
				vTarget = (signed char)SCurve((float)vMax, (float)0, accelTmax, &accelT);
				BalanceControl1StraightP(vTarget, 1, 10);	// 強制ジャイロオフセット付き
				iStartUp++;
			}
			else
			{
				ResetDetectDeviateFromLine();
				mode++;
			}
#endif

			//...#########################################################
			//... monoスタートアップ処理（走行開始時のライン逸脱の復帰）
			//... add by mononobe <2014/09/06>
			//...#########################################################
//...			ecrobot_sound_tone(100, 200, 30);	//... デバッグ用サウンド

			tail_control(TAIL_ANGLE_DRIVE);


#if 0	//... plan1 : kuniライントレースを使う
/*
			enc_l = nxt_motor_get_count(NXT_PORT_C); //... 左モータエンコーダ値取得
			enc_r = nxt_motor_get_count(NXT_PORT_B); //... 右モータエンコーダ値取得
#ifdef USE_LOG
			ecrobot_bt_data_logger((S8)(enc_l/10), (S8)(enc_r/10));
#endif

			//... kuniライントレース
			forward = 100;
			ctrl_direction((float)forward);
			turn = get_model_turn();
			forward = get_model_forward();

			//... スタートアップ直後の後退検知と、前進に切り替わったかの検知
			if( (enc_l-enc_l0)>0 || (enc_r-enc_r0)>0 ){		//... 左右どちらかの前進を検知したら通常処理へ	//...不感帯を設けたいけどね
				//... 何もしない（本番ではmode++）
			}
			else if((enc_l+enc_r)<0){		//... 後退検知したら、ライントレースturnを反転
				turn = -turn;
			}
*/

			BalanceControl(100, 0);

//...			enc_l0 = enc_l;		//... 次ループ差分用
//...			enc_r0 = enc_r;		//... 次ループ差分用

#else	//... plan2 : std倒立でやる
#if 0
			BalanceControl(100, 0);
#else
			BalanceControlStraightP(100, 3.0f);
#endif
			cnt_stup++;

			if(cnt_stup>=120){
				mode++;
			}
#endif

			//#######################################################################
			// kuniライントレース用にモデルの状態取得 add by kunitake <2014/08/13>
			//#######################################################################
			get_model_state();

			break;

		case RUN:
#if 0	//障害検知の削除
			if(detectFailEnable){
				switch(DetectFail()){
				case ERROR_FALL: mode = EXIT; break;
				case ERROR_LOST:
					BalanceToTail_Init();
					mode = STOP;
					break;
				}
			}
#endif

			tail_control(TAIL_ANGLE_DRIVE);

			//#######################################################################
			// kuniライントレース用にモデルの状態取得 add by kunitake <2014/08/13>
			//#######################################################################
			get_model_state();
#ifdef USE_LOG
			ecrobot_bt_data_logger((S8)(model1.forward*0.1), (S8)(model1.turn*100.0));
#endif
			if(COURSE_IN_SIDE == STARTING_SIDE)
			{
				#ifdef MAKE_INSIDE
				part = Cource_InSide_Main();
				#endif // #ifdef MAKE_INSIDE
			}
			else
			{
				#ifndef MAKE_INSIDE
				part = Cource_OutSide_Main();
				#endif // #ifndef MAKE_INSIDE
			}

			break;

		case EXIT:
			nxt_motor_set_speed(NXT_PORT_A,0,1);
			nxt_motor_set_speed(NXT_PORT_B,0,1);
			nxt_motor_set_speed(NXT_PORT_C,0,1);
			break;

		case STOP:
			BalanceToTailMain();
			break;

		case BT_SEND:
			if(bt_data_log_send() == -1)
			{
				mode = BEFORE_STARTLINE_SET;
			}
			break;

		default:
			break;
	}

	ReleaseResource(resource1);
	TerminateTask();
}

//*****************************************************************************
// タスク名 : Task_1ms
// 概要 : 1ms周期呼び出しタスク
//			・光センサの値を読み取って各変数に設定する
//			・走行時は光センサ値のLPF計算する
//			・ロボットがラインから外れたときにモータを停止する
//			・光センサのLEDを点滅させる
//*****************************************************************************
TASK(Task_1ms)
{
	GetResource(resource1);

	switch(mode)
	{
		case BEFORE_CALIBRATION:
		case BEFORE_CALIBRATION2:
			#ifndef TEST_SLOW_FORWARD_RUN
			// 白黒キャリブレーション
			//LineDetectionlevelCalibration();
			#endif
			break;
		case BEFORE_TOUCH:
			LightIn();
			LineTraceControl_MakerDetect_Init();
			break;
		case RUN:
			LightIn();
			if(LineTraceControl_MakerDetect() == -1)
			{
				mode = BT_SEND;
			}
			break;
		default:
			break;
	}

#ifdef USE_FAIL_SAFE
	static U16 count = 0;

	//白色検知すればモータを止める 白色は仮の固定値
	if(ecrobot_get_light_sensor(NXT_PORT_S3) < LineTraceControl_LightWhiteThreshold())
	{
		count++;
	}
	else
	{
		count = 0;
	}
/*
	if(mode == RUN && count >= 1000)
	{
		mode = EXIT;
	}
*/
#endif

#ifdef USE_LIGHT_BLINK
	static U8 on_off_flg = 1U;
	U32 systick = ecrobot_get_systick_ms();
	if(systick % LIGHT_BLINK_CYCLE == 0)
	{
		switch(mode)
		{
			case BEFORE_CALIBRATION:
			case BEFORE_CALIBRATION2:
				if(on_off_flg)
				{
					/* 赤色LEDを点灯する */
					ecrobot_set_light_sensor_active(NXT_PORT_S3);
					on_off_flg = 0U;
				}
				else
				{
					/* 赤色LEDを消灯する */
					ecrobot_set_light_sensor_inactive(NXT_PORT_S3);
					on_off_flg = 1U;
				}
				break;
			case BEFORE_STARTLINE_SET:
				/* 赤色LEDを点灯する */
				ecrobot_set_light_sensor_active(NXT_PORT_S3);
				break;
			default:
				/*何もしない*/
				break;
		}
	}
#endif

	ReleaseResource(resource1);
	TerminateTask();
}

//*****************************************************************************
// 関数名 : remote_start
// 引数 : 無し
// 返り値 : 1(スタート)/0(待機)
// 概要 : Bluetooth通信によるリモートスタート。 Tera Termなどのターミナルソフトから、
//		 ASCIIコードで1を送信すると、リモートスタートする。
//*****************************************************************************
static int remote_start(void)
{
	int i;
	unsigned int rx_len;
	unsigned char start = 0;

	for (i=0; i<BT_MAX_RX_BUF_SIZE; i++)
	{
		rx_buf[i] = 0; /* 受信バッファをクリア */
	}

	rx_len = ecrobot_read_bt(rx_buf, 0, BT_MAX_RX_BUF_SIZE);
	if (rx_len > 0)
	{
		/* 受信データあり */
		if (rx_buf[0] == CMD_START)
		{
			start = 1; /* 走行開始 */
		}
		ecrobot_send_bt(rx_buf, 0, rx_len); /* 受信データをエコーバック */
	}

	return start;
}
