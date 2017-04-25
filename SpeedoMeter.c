/* 2012/08/10 created by k.nakagawa ���x�v */
#include "mytypes.h"

#include "Calc.h"

static int freqSpeedoMeter = 1;
static float prevDistance = 0.0F;

//*****************************************************************************
// �֐��� : SpeedoMeter
// ���� : �Ȃ�
// �Ԃ�l : ���݂̃��{�b�g�̑��x(m/s)
// �T�v : ���x�v�Ƃ��ėp����A�K��init���������ŌĂяo������
//*****************************************************************************
float SpeedoMeter(void)
{
	// ����̋����̎擾(m)
	float Distance = calc_getDistanceFromStart();
	
	// �O��Ƃ̋����̍�(m)
	float diff = Distance - prevDistance;
	
	prevDistance = Distance;
	
	return diff * (1000.0F / (float)freqSpeedoMeter);
}

//*****************************************************************************
// �֐��� : initSpeedoMeter
// ���� : freq=����(ms)(��0)
// �Ԃ�l : �Ȃ�
// �T�v : �����̃Z�b�g�A�O��l�̏�����
//*****************************************************************************
void initSpeedoMeter(U8 freq)
{
	freqSpeedoMeter = freq;
	prevDistance = 0.0F;
	return;
}
