/* 2013/09/15 created by s.sahara �p�x�v */
#include "mytypes.h"
#include "ecrobot_interface.h"
#include "LineTraceControl.h"
#include "Control.h"

static S16 angle;

// ���l�����肵�Ȃ��̂ŁA�ώZ���Đ�Ίp�x�𑪒肷��͓̂���B
// �p���x�𑪒肷�邪�A�΂�����傫���̂ŕ��ω�����B

#define ANGLE_COUNT 20 // ���ω� �c80ms
#define ANGLE_CYCLE (CONTROL_CYCLE*ANGLE_COUNT)
static S16 gyroBuf[ANGLE_COUNT];
static U8 idxGyroBuf;

//*****************************************************************************
// �֐��� : AngleMeter
// ���� : �Ȃ�
// �Ԃ�l : ���݂̃��{�b�g�̊p���x�ilevel/sec�j
// �T�v : �p�x�v�Ƃ��ėp����A�K���������(CONTROL_CYCLE)�ŌĂяo������
//*****************************************************************************
S32 AngleMeter(void)
{
	U16 gyro = ecrobot_get_gyro_sensor(NXT_PORT_S1);
	gyroBuf[idxGyroBuf++] = gyro - GYRO_OFFSET;
	if(idxGyroBuf >= ANGLE_COUNT) idxGyroBuf = 0;

	angle = 0;
	for(int i=0;i<ANGLE_COUNT;i++) angle += gyroBuf[i];
	angle = angle / (1000 / ANGLE_CYCLE); // level/sec

	// ecrobot_debug1(angle, gyro, GYRO_OFFSET);

	return angle;
}

//*****************************************************************************
// �֐��� : initAngleMeter
// ���� : freq=����(ms)(��0)
// �Ԃ�l : �Ȃ�
// �T�v : �����̃Z�b�g�A�O��l�̏�����
//*****************************************************************************
void initAngleMeter()
{
	angle = 0;
	for(int i=0;i<ANGLE_COUNT;i++) gyroBuf[ANGLE_COUNT] = 0;
	idxGyroBuf = 0;
	return;
}
