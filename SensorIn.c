/* 2012/08/09 created by s.sahara 難所クリア単体テスト用 */

/* センサ入力関連処理 */

#include <math.h>

#include "kernel.h"
#include "kernel_id.h"
#include "ecrobot_interface.h"

#include "SensorIn.h"
#include "LineTraceParam.h"

static char mliInitialized = 0;

#if 0
// 光センサ入力関連
#define MAX_LIGHT_IN_COUNT1 3
#define MAX_LIGHT_IN_COUNT2 10
#define MAX_LIGHT_IN_COUNT3 20
static U16 lightIn1[MAX_LIGHT_IN_COUNT1];
static U16 lightIn2[MAX_LIGHT_IN_COUNT2];
static U16 lightIn3[MAX_LIGHT_IN_COUNT3];
static U16 idxLightIn1;
static U16 idxLightIn2;
static U16 idxLightIn3;
static U16 lightInAvg1;
static U16 lightInAvg2;
static U16 lightInAvg3;
#endif

// 光センサの値(LPFなし)
static U16 lightIn = 0;

// ライントレースの光センサ入力LPF用
static float omega0;
static float a;
static float b0;
static float b1;
static float b2;
static float a1;
static float a2;
static float pre1;
static float pre2;
static U16 lightInLPF = 0;

// ライントレースの光センサ入力LPF用(マーカ検知用)
// md_ : marker detection
static float md_mid_omega0;
static float md_mid_a;
static float md_mid_b0;
static float md_mid_b1;
static float md_mid_b2;
static float md_mid_a1;
static float md_mid_a2;
static float md_mid_pre1;
static float md_mid_pre2;
static U16 md_mid_lightInLPF = 0;

static float md_slow_omega0;
static float md_slow_a;
static float md_slow_b0;
static float md_slow_b1;
static float md_slow_b2;
static float md_slow_a1;
static float md_slow_a2;
static float md_slow_pre1;
static float md_slow_pre2;
static U16 md_slow_lightInLPF = 0;

// 段差検知関連
#define GD_RING_MAX 5
static float mgdThreshold;
static int mgdRingCount = 0; // 周期毎にカウントアップするリングバッファインデックス
static U16 mgdRing[GD_RING_MAX] = {0,0,0,0,0}; // ジャイロ値記録用
static U16 mgdPrevsum = 0;// 前回の角速度合計値

// f0 : カットオフ周波数
// omega0, a, b0, b1, b2, a1, a2 : パラメータ（計算結果が返される。呼び出し元で保持。）
void InitLPF(float f0, float* omega0, float* a, float* b0, float* b1, float* b2, float* a1, float* a2)
{
	// ω0
	*omega0 = tan(M_PI * f0 / LPF_FS);
	// a
	*a = sin(*omega0) / LPF_Q;
	// b0
	*b0 = (1.0 - cos(*omega0)) / (2.0 * (1.0 + *a));
	// b1
	*b1 = (1.0 - cos(*omega0)) / (1.0 + *a);
	// b2
	*b2 = (1.0 - cos(*omega0)) / (2.0 * (1.0 + *a));
	// a1
	*a1 = (2.0 * cos(*omega0)) / (1.0 + *a);
	// a2
	*a2 = (*a - 1.0) / (*a + 1.0);

	#if 0 // 計算結果確認用
	while(1)
	{
		ecrobot_debug1((UINT)(f0 * 100.0), (UINT)(omega0 * 1000.0), (UINT)(0 * 1000.0));
		systick_wait_ms(1000);
		ecrobot_debug1((UINT)(a * 100.0), (UINT)(a1 * 100.0), (UINT)(a2 * 100.0));
		systick_wait_ms(1000);
		ecrobot_debug1((UINT)(b0 * 10000.0), (UINT)(b1 * 10000.0), (UINT)(b2 * 10000.0));
		systick_wait_ms(1000);
		ecrobot_debug1((UINT)(0), (UINT)(0), (UINT)(0));
		systick_wait_ms(1000);
	}
	#endif
}

// LPF
// inp : LPFをかけたい値
// omega0, a, b0, b1, b2, a1, a2 : パラメータ（初期化結果のパラメータを渡す）
// pre1, pre2 : 過去値（呼び出し元で保持）
S16 LPF(S16 inp, float omega0, float a, float b0, float b1, float b2, float a1, float a2, float* pre1, float* pre2)
{
	float reg, out;
	reg = inp + a1 * (*pre1) + a2 * (*pre2);
	out = b0 * reg + b1 * (*pre1) + b2 * (*pre2);

	*pre2 = *pre1;
	*pre1 = reg;

	return (S16)out;
}

#if 0
// private
// 光センサ入力単純平均用リングバッファ初期化（現在値で埋める）
void InitLightInAvg(U16 inp, U16 ringBuf[], U16 sz)
{
	for(int i=0; i<sz; i++) ringBuf[i] = inp;
}

// private
// 光センサ入力単純平均
U16 LightInAvg(U16 inp, U16 ringBuf[], U16 sz, U16* pIdx)
{
	U32 inpSum = 0;
	ringBuf[*pIdx] = inp;
	for(int i=0; i<sz; i++)inpSum += ringBuf[i];
	*pIdx++;

	if(*pIdx >= sz) *pIdx = 0;
	return (U16)(inpSum / sz);
}
#endif

// 光センサ入力（制御周期毎に実行のこと）
void LightIn()
{
	lightIn = ecrobot_get_light_sensor(NXT_PORT_S3); // 0-1023

	if(!mliInitialized)
	{
		// 初期化、初回だけ実行
		InitLPF(LIGHT_LPF_HZ, &omega0, &a, &b0, &b1, &b2, &a1, &a2);
		InitLPF(MID_LPF_HZ, &md_mid_omega0, &md_mid_a, &md_mid_b0, &md_mid_b1, &md_mid_b2, &md_mid_a1, &md_mid_a2);
		InitLPF(SLOW_LPF_HZ, &md_slow_omega0, &md_slow_a, &md_slow_b0, &md_slow_b1, &md_slow_b2, &md_slow_a1, &md_slow_a2);
#if 0
		idxLightIn1 = 0;
		idxLightIn2 = 0;
		idxLightIn3 = 0;
		InitLightInAvg(lightIn, lightIn1, MAX_LIGHT_IN_COUNT1);
		InitLightInAvg(lightIn, lightIn2, MAX_LIGHT_IN_COUNT2);
		InitLightInAvg(lightIn, lightIn3, MAX_LIGHT_IN_COUNT3);
#endif
		mliInitialized = 1;
	}
#if 0
	// ※LPFの動作確認ができたら平均はいらない
	lightInAvg1 = LightInAvg(lightIn, lightIn1, MAX_LIGHT_IN_COUNT1, &idxLightIn1);
	lightInAvg2 = LightInAvg(lightIn, lightIn2, MAX_LIGHT_IN_COUNT2, &idxLightIn2);
	lightInAvg3 = LightInAvg(lightIn, lightIn3, MAX_LIGHT_IN_COUNT3, &idxLightIn3);
#endif
	lightInLPF = LPF(lightIn, omega0, a, b0, b1, b2, a1, a2, &pre1, &pre2);
	md_mid_lightInLPF = LPF(lightIn, md_mid_omega0, md_mid_a, md_mid_b0, md_mid_b1, md_mid_b2, md_mid_a1, md_mid_a2, &md_mid_pre1, &md_mid_pre2);
	md_slow_lightInLPF = LPF(lightIn, md_slow_omega0, md_slow_a, md_slow_b0, md_slow_b1, md_slow_b2, md_slow_a1, md_slow_a2, &md_slow_pre1, &md_slow_pre2);
#ifdef USE_LOG
	//ecrobot_bt_data_logger((S8)(lightIn*0.1f), (S8)(lightInLPF*0.1f));
#endif
}

#if 0
// 3回平均
U16 GetLightIn3()
{
	return lightInAvg1;
}

// 10回平均
U16 GetLightIn10()
{
	return lightInAvg2;
}

// 20回平均
U16 GetLightIn20()
{
	return lightInAvg3;
}
#endif

U16 GetLightIn()
{
	return lightIn;
}

U16 GetLightInLPF()
{
	return lightInLPF;
}

U16 md_mid_GetLightInLPF()
{
	return md_mid_lightInLPF;
}

U16 md_slow_GetLightInLPF()
{
	return md_slow_lightInLPF;
}

/*
段差検知の初期化
threshold : しきい値 1/100%
*/
void InitGyroDetect(float threshold)
{
	U16 sensIn = ecrobot_get_gyro_sensor(NXT_PORT_S1);
	mgdThreshold = threshold;

	mgdRingCount = 0;
	mgdPrevsum = 0;
	for(int i = 0; i < GD_RING_MAX; i++)
	{
		mgdRing[i] = sensIn;
		mgdPrevsum += sensIn;
	}
}

/*
段差検知（定周期実行）
戻り値 1 : 段差検知（振れ幅がしきい値以上に振れたらON）, 0 : 未検知
*/
int GyroDetect()
{
	int rvalue = 0; // 返り値
	double rate = 0; // 角速度合計の前回値と今回値のレート

	// 値の格納
	mgdRing[mgdRingCount] = ecrobot_get_gyro_sensor(NXT_PORT_S1);
	// 配列の尻尾にきたら次は頭へ
	mgdRingCount++;
	if(mgdRingCount == GD_RING_MAX)
	{
		mgdRingCount = 0;
	}

	// 過去のジャイロ検出値を累積する
	int sum = 0;
	for(int i = 0; i < GD_RING_MAX; i++)
	{
		sum += mgdRing[i];
	}

	// 速度が大きくなれば、変化量も大きくなるためレートを計算
	rate = (1.0 - (double)sum / (double)mgdPrevsum) * 10000.0; // 1/100%
	if(rate < 0.0)
	{
		rate = -1.0 * rate;
	}
	if (rate >= mgdThreshold)
	{
		rvalue = 1;
	}

#ifdef USE_LOG
	//BTへログ書き出し
	//ecrobot_bt_data_logger((S8)(mgdThreshold / 10), (S8)(rate / 10.0)); // btログがbyteなのでbyteに収まるように単位は1/10%とする
#endif

	mgdPrevsum = sum;

	return rvalue;
}
