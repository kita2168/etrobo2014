/* 2012/08/09 created by s.sahara ��N���A�P�̃e�X�g�p */

/* �Z���T���͊֘A���� */

#include <math.h>

#include "kernel.h"
#include "kernel_id.h"
#include "ecrobot_interface.h"

#include "SensorIn.h"
#include "LineTraceParam.h"

static char mliInitialized = 0;

#if 0
// ���Z���T���͊֘A
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

// ���Z���T�̒l(LPF�Ȃ�)
static U16 lightIn = 0;

// ���C���g���[�X�̌��Z���T����LPF�p
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

// ���C���g���[�X�̌��Z���T����LPF�p(�}�[�J���m�p)
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

// �i�����m�֘A
#define GD_RING_MAX 5
static float mgdThreshold;
static int mgdRingCount = 0; // �������ɃJ�E���g�A�b�v���郊���O�o�b�t�@�C���f�b�N�X
static U16 mgdRing[GD_RING_MAX] = {0,0,0,0,0}; // �W���C���l�L�^�p
static U16 mgdPrevsum = 0;// �O��̊p���x���v�l

// f0 : �J�b�g�I�t���g��
// omega0, a, b0, b1, b2, a1, a2 : �p�����[�^�i�v�Z���ʂ��Ԃ����B�Ăяo�����ŕێ��B�j
void InitLPF(float f0, float* omega0, float* a, float* b0, float* b1, float* b2, float* a1, float* a2)
{
	// ��0
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

	#if 0 // �v�Z���ʊm�F�p
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
// inp : LPF�����������l
// omega0, a, b0, b1, b2, a1, a2 : �p�����[�^�i���������ʂ̃p�����[�^��n���j
// pre1, pre2 : �ߋ��l�i�Ăяo�����ŕێ��j
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
// ���Z���T���͒P�����ϗp�����O�o�b�t�@�������i���ݒl�Ŗ��߂�j
void InitLightInAvg(U16 inp, U16 ringBuf[], U16 sz)
{
	for(int i=0; i<sz; i++) ringBuf[i] = inp;
}

// private
// ���Z���T���͒P������
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

// ���Z���T���́i����������Ɏ��s�̂��Ɓj
void LightIn()
{
	lightIn = ecrobot_get_light_sensor(NXT_PORT_S3); // 0-1023

	if(!mliInitialized)
	{
		// �������A���񂾂����s
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
	// ��LPF�̓���m�F���ł����畽�ς͂���Ȃ�
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
// 3�񕽋�
U16 GetLightIn3()
{
	return lightInAvg1;
}

// 10�񕽋�
U16 GetLightIn10()
{
	return lightInAvg2;
}

// 20�񕽋�
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
�i�����m�̏�����
threshold : �������l 1/100%
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
�i�����m�i��������s�j
�߂�l 1 : �i�����m�i�U�ꕝ���������l�ȏ�ɐU�ꂽ��ON�j, 0 : �����m
*/
int GyroDetect()
{
	int rvalue = 0; // �Ԃ�l
	double rate = 0; // �p���x���v�̑O��l�ƍ���l�̃��[�g

	// �l�̊i�[
	mgdRing[mgdRingCount] = ecrobot_get_gyro_sensor(NXT_PORT_S1);
	// �z��̐K���ɂ����玟�͓���
	mgdRingCount++;
	if(mgdRingCount == GD_RING_MAX)
	{
		mgdRingCount = 0;
	}

	// �ߋ��̃W���C�����o�l��ݐς���
	int sum = 0;
	for(int i = 0; i < GD_RING_MAX; i++)
	{
		sum += mgdRing[i];
	}

	// ���x���傫���Ȃ�΁A�ω��ʂ��傫���Ȃ邽�߃��[�g���v�Z
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
	//BT�փ��O�����o��
	//ecrobot_bt_data_logger((S8)(mgdThreshold / 10), (S8)(rate / 10.0)); // bt���O��byte�Ȃ̂�byte�Ɏ��܂�悤�ɒP�ʂ�1/10%�Ƃ���
#endif

	mgdPrevsum = sum;

	return rvalue;
}
