/* 2012/08/09 created by t.akiyama ��N���A�P�̃e�X�g�p */

#include "Calc.h"

#include "balancer.h"
#include "ecrobot_interface.h"

//*****************************************************************************
// �֐��� : calc_distance
// ���� : �p�xA , �p�xB
// �Ԃ�l : ���s����
// �T�v : 1�̂̃��[�^��]�p�x���瑖�s���������߂�
//*****************************************************************************
int calc_distance(int deg_A , int deg_B)
{
	unsigned int distance = 0;
	unsigned int diff_deg = 0;
	
	diff_deg = deg_A - deg_B;
	if(diff_deg < 0)
	{
		diff_deg = (-1) * diff_deg;
	}
	
	/* ���s�������Βl�ŕԂ� */
	distance = 85 * PI / (360 / diff_deg);
	
	return distance;
}

// �G���R�[�_�[�����Z�b�g����̋���(m)
float calc_getDistanceFromStart()
{
	float radL = nxt_motor_get_count(NXT_PORT_C) * DEG2RAD;	/* �����[�^��]�p�x[deg] */
	float radR = nxt_motor_get_count(NXT_PORT_B) * DEG2RAD;	/* �E���[�^��]�p�x[deg] */
	
	float distance = WHEEL_D * ((radL + radR) / 4.0f); // ���E�ړ��𕽋ς������̂����S�ړ��ʂƂ���(����Ɋp�x/2)
	return distance;
}


//*****************************************************************************
// ��]�����v���p
//*****************************************************************************
static float baseValue_radB;
static float baseValue_radC;
static unsigned char rotateLR;

/* �C�j�V��������̉�]���������߂� */
float calc_getRotateDistance()
{
	float currentValue_radB = nxt_motor_get_count(NXT_PORT_B) * DEG2RAD;
	float currentValue_radC = nxt_motor_get_count(NXT_PORT_C) * DEG2RAD;
	float distance = 0;
	
//	if(0 == rotateLR)
//	{
//		distance = WHEEL_D * PI * ((currentValue_radB - baseValue_radB) / (2 * PI));
//	}
//	else
//	{
//		distance = WHEEL_D * PI * ((currentValue_radC - baseValue_radC) / (2 * PI));
//	}
	
	/* �Зւł͌덷���傫�����ߗ��ւ̉�]�p�x���畽�ς����߂� */
	float aveRad = 0; 
	if(0 == rotateLR)
	{
		aveRad = ((currentValue_radB - baseValue_radB) + (-1) * (currentValue_radC - baseValue_radC)) / 2.0f;
	}
	else
	{
		aveRad = ((-1) * (currentValue_radB - baseValue_radB) + (currentValue_radC - baseValue_radC)) / 2.0f;
	}
	distance = WHEEL_D * PI * (aveRad / (2 * PI));
	return distance;
}

/* ������ check_rotateComplete���g�p����O�ɏ������K�{ */
void init_rotateDistance(unsigned char a_rotateLR)
{
	/* ���[�^�[B,C�̊�l��ێ� */
	baseValue_radB = nxt_motor_get_count(NXT_PORT_B) * DEG2RAD;
	baseValue_radC = nxt_motor_get_count(NXT_PORT_C) * DEG2RAD;
	rotateLR = a_rotateLR;
}

/* �����ŗ^����ꂽ�p�x�ɑ��s�̂���]���������肷�� */
int check_rotateComplete(unsigned int a_rotateDeg)
{
	int unsigned ret = 0;
	
	float targetRad = (float)(a_rotateDeg) * DEG2RAD;
	
	/* ���s�̂������̉�]�p�x�ɂȂ�܂łɕK�v�ȋ��� */
	float targetDistance = AXLE_D * PI * (targetRad / (2.0f * PI));
	float rotateDistance = calc_getRotateDistance();

	if(targetDistance <= rotateDistance)
	{
		ret = 1;	
	}
	else
	{
		ret = 0;
	}

	/* 0:��]������ 1:��]���� */
	return ret;
}

//*****************************************************************************
// �C�ӈʒu����̑��s�����v���p
//*****************************************************************************
static float pointValue_radL; 
static float pointValue_radR;

float calc_getDistanceFromPoint()
{
	float radL = (nxt_motor_get_count(NXT_PORT_C) - pointValue_radL) * DEG2RAD;	/* �����[�^��]�p�x[deg] */
	float radR = (nxt_motor_get_count(NXT_PORT_B) - pointValue_radR) * DEG2RAD;	/* �E���[�^��]�p�x[deg] */
	
	float distance = WHEEL_D * ((radL + radR) / 4.0f); // ���E�ړ��𕽋ς������̂����S�ړ��ʂƂ���(����Ɋp�x/2)
	return distance;
}

void init_DistanceFromPoint()
{
	pointValue_radL = nxt_motor_get_count(NXT_PORT_C);
	pointValue_radR = nxt_motor_get_count(NXT_PORT_B);
}

