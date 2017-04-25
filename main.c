/* 2012/07/21 created by s.sahara
�EET���{�R�������T�C�g�Ŕz�z��etrobo2012sample(nxtOSEK)����sample_c4������
	��N���A�P�̃e�X�g�p�v���O�������쐬�B
�E
*/

#include "Error.h"
#include "CourseDef.h"

/**
 ******************************************************************************
 **	�t�@�C���� : sample.c
 **
 **	�T�v : 2�֓|���U�q���C���g���[�X���{�b�g��TOPPERS/ATK1(OSEK)�pC�T���v���v���O����
 **
 ** ���L : sample_c4 (sample_c3��Bluetooth�ʐM�����[�g�X�^�[�g�@�\��ǉ�)
 ******************************************************************************
 **/
#include "kernel.h"
#include "kernel_id.h"
#include "ecrobot_interface.h"
#include "balancer.h" /* �|���U�q����p�w�b�_�t�@�C�� */

#include "Calc.h"
#include "LineTraceControl.h"
#include "LineTraceParam.h"
#include "SensorIn.h"
#include "Control.h"

// 2012/07/21 add by s.sahara >>>>
#include "LineTrace.h"			// �ʏ푖�s
//#include "SlopeStage.h"			// �⓹
#include "LookUpGateStage.h"	// ���b�N�A�b�v�Q�[�g
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
	START_UP, // �|���ő��s�J�n���̑��s����҂����i����
	RUN,
	EXPLOR_LINE,	// ���C���T��
	EXIT,			// ����~
	STOP,			// ������~
	BT_SEND, //BT���O�̑��M
};
static U8 mode = BEFORE_INIT;

/* sample_c4�}�N�� */
#ifdef STEC_EAST
#define DEVICE_NAME 	  "ET205-E"  /* Bluetooth�ʐM�p�f�o�C�X�� */
#define PASS_KEY		  "steceast" /* Bluetooth�ʐM�p�p�X�L�[ */
#else
#define DEVICE_NAME 	  "ET205-W"  /* Bluetooth�ʐM�p�f�o�C�X�� */
#define PASS_KEY		  "stecwest" /* Bluetooth�ʐM�p�p�X�L�[ */
#endif

#define CMD_START		  '1'	 /* �����[�g�X�^�[�g�R�}���h(�ύX�֎~) */

/* �֐��v���g�^�C�v�錾 */
static int remote_start(void);

/* Bluetooth�ʐM�p�f�[�^��M�o�b�t�@ */
char rx_buf[BT_MAX_RX_BUF_SIZE];

static UINT sw_prv;

#if 0	//�X�^�[�g�����i�����̍폜
static int iStartUp;
static signed char vMax;
static signed char vTarget;
static float accelT;
static float accelTmax;
#endif



// #################################################################################
// ### ������mono�X�^�[�g�A�b�v�����p�̕ϐ��̐錾 add by mononobe <2014/09/06> #####
static signed char forward;		// �O��i����
static signed char turn;		// ���񖽗�
static int enc_l = 0;	//... Left �G���R�[�_���ݒl
static int enc_r = 0;	//... Right�G���R�[�_���ݒl
static int enc_l0 = 0;	//... Left �G���R�[�_�ߋ��l
static int enc_r0 = 0;	//... Right�G���R�[�_�ߋ��l

static int cnt_stup = 0;	//... �X�^�[�g�A�b�v�ҋ@�J�E���g

// #### mono�X�^�[�g�A�b�v�����̕ϐ��錾 �I�� ######################################
// #################################################################################


// ###########################################################
// ��������Akuni���C���g���[�X�p�ϐ� add by kunitake <2014/08/13>
// ###########################################################
// ���Q�l������
// �E��z���m�~�b�NDriftless�V�X�e���̃t�B�[�h�o�b�N����@�^�@�O�����i�C�ΐ쏫�l
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
//#define RATIO_Y (0.15)				// ���C���G�b�W����̋���[mm] / �P�x��  ����c���Ő����p�^�[��
#ifdef MAKE_INSIDE
#define RATIO_Y (-0.05)
#define RATIO_Y_GRAY (-0.10)
#else
#define RATIO_Y (0.05)
#define RATIO_Y_GRAY (0.10)
#endif
//#define GAIN_K1 (0.002)  ����c���Ő����p�^�[��
//#define GAIN_K2 (0.03)  ����c���Ő����p�^�[��
//#define GAIN_K1 (0.003) // GAIN_K2=0.12�̂Ƃ��̉����l
#define GAIN_K1 (0.006)
#define GAIN_K2 (0.12)

struct model_nxtway{
	signed char cmd_forward;		// ���i�̎w�ߒl
	signed char cmd_turn;			// ����̎w�ߒl
	int fai_r;		int fai_l;		// �ԗւ̉�]�p�x[deg]
	int fai_r_old;	int fai_l_old;	// �P�O�̎ԗւ̉�]�p�x[deg]
	float dr0;	float dr1;	float dr2;	float dr3;	float dr4;	float dr5;	// �X�^�[�g���Ă���̉E�ԗւŐi�񂾋����i0�`5�܂ł���̂�LPF�p�j
	float dl0;	float dl1;	float dl2;	float dl3;	float dl4;	float dl5;	// �X�^�[�g���Ă���̍��ԗւŐi�񂾋����i0�`5�܂ł���̂�LPF�p�j
	float vr;		float vl;		// ���E�ԗւ̕��i���x[mm/s]
	float forward;		// ���i���x
	float turn;		// ���񑬓x

	float x0;	// ���C���ڐ������̋���
	float y0;	float y1;	float y2;	float y3;	float y4;	float y5;	// ���C������̉���������
	float z0;	// ���C���ɑ΂���p�x��
	float y_dot;	// y�̑��x

	float a0, a1, a2;		// y�l�̃��[�p�X�t�B���^
	float b0, b1, b2;		// y�l�̃��[�p�X�t�B���^
	float c0, c1, c2;		// �ԗֈړ������̃��[�p�X�t�B���^
	float d0, d1, d2;		// �ԗֈړ������̃��[�p�X�t�B���^
	float mu;				// �t�B�[�h�o�b�N��
	float gain_k1;			// �t�B�[�h�o�b�N�Q�C���P
	float gain_k2;			// �t�B�[�h�o�b�N�Q�C���Q

	U16 light;				// ���݂̌��Z���T�l
	U16 light_c;			// �Ǐ]���ׂ����Z���T�l
	float ratio_y;			// ���C���G�b�W����̋���[mm] / �P�x��
};
static struct model_nxtway model1;

#include <math.h>
// kuni���C���g���[�X�Ɏg���֐�
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
	// ���ݒl����X�^�[�g���邽��
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
	// ���ݒl����X�^�[�g���邽��
	model1.y0 = line_distance_table(model1.light);
	model1.y1 = model1.y0;	model1.y2 = model1.y0;	model1.y3 = model1.y0;	model1.y4 = model1.y0;	model1.y5 = model1.y0;
	model1.z0 = 0.0;
}

void get_model_state(){
	// �X�V
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


	// �ԗւ̊p�x���擾���āA���E�ԗւ̕��i���x���v�Z
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

	// �ԗ֑��x����A���i���x�Ɛ��񑬓x�����߂�
	model1.forward = (model1.vl + model1.vr) / 2.0;
	model1.turn = (model1.vr - model1.vl) / 2.0 / TREAD;

	// �ϑ���
	// �Z���T�P�x�l����y, y(0)�����߁A�ߋ��f�[�^y(-1)�ƂƂ���y_dot�����߂�
	// ����ɏ�L�̒��i���x����Atheta=asin(y_dot/forward)����tan(theta)�����߂�
	model1.light = ecrobot_get_light_sensor(NXT_PORT_S3);
	model1.y0 = line_distance_table(model1.light);

	model1.y3 = (model1.b0 * model1.y0 + model1.b1 * model1.y1 + model1.b2 * model1.y2
					- model1.a1 * model1.y4 - model1.a2 * model1.y5) / model1.a0;
	model1.y_dot = (model1.y3 - model1.y4) / SAMP;

	if(model1.forward<=-0.001 || 0.001<=model1.forward) model1.z0 = asin(model1.y_dot / model1.forward);
}

void ctrl_direction(float ref_vel){
	float mu1, mu2;
	float u1, u2;		// ���x�w�ߒl
	float tmp_f, tmp_t;

	// ���݂̑��x���擾���v�Z
	mu1 = ref_vel;
	mu2 = (-model1.gain_k1) * model1.y0 + (-model1.gain_k2) * tan(model1.z0);
	model1.mu = mu2;

	u1 = mu1 / cos(model1.z0);
	u2 = pow(cos(model1.z0),2.0) * mu2 * mu1;

	// u1,u2��PWM�Ɋ��Z���āAbalance_control�ւ̓��͂Ƃ���
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
// �֐��� : ecrobot_device_initialize
// ���� : �Ȃ�
// �߂�l : �Ȃ�
// �T�v : ECROBOT�f�o�C�X�����������t�b�N�֐�
//*****************************************************************************
void ecrobot_device_initialize()
{
	ecrobot_set_light_sensor_active(NXT_PORT_S3); /* ���Z���T�ԐFLED��ON */
	ecrobot_init_sonar_sensor(NXT_PORT_S2); /* �����g�Z���T(I2C�ʐM)�������� */
	nxt_motor_set_count(NXT_PORT_A, 0); /* ���S��~�p���[�^�G���R�[�_���Z�b�g */
	ecrobot_init_bt_slave(PASS_KEY); /* Bluetooth�ʐM������ */
}

//*****************************************************************************
// �֐��� : ecrobot_device_terminate
// ���� : �Ȃ�
// �߂�l : �Ȃ�
// �T�v : ECROBOT�f�o�C�X�I�������t�b�N�֐�
//*****************************************************************************
void ecrobot_device_terminate()
{
	ecrobot_set_light_sensor_inactive(NXT_PORT_S3); /* ���Z���T�ԐFLED��OFF */
	ecrobot_term_sonar_sensor(NXT_PORT_S2); /* �����g�Z���T(I2C�ʐM)���I�� */
	ecrobot_term_bt_connection(); /* Bluetooth�ʐM���I�� */
}

//*****************************************************************************
// �֐��� : user_1ms_isr_type2
// ���� : �Ȃ�
// �߂�l : �Ȃ�
// �T�v : 1msec�������荞�݃t�b�N�֐�(OSEK ISR type2�J�e�S��)
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

	/* �R�[�X�ʑ��s���� */
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
			//if(distansAbs > STR_1_DISTANCE) courseJointDetected = IN_COURSE_PART_ON_GOAL; // ���X�g���[�g�I�[�Ŏ~�܂��Ă݂�
			break;
	}

	/* �R�[�X�؂�ւ��C�x���g */
	switch(courseJointDetected)
	{
		case IN_COURSE_PART_ON_FIGUREL:		/* �t�B�M���AL���֑J�� */
			/* �R�[�X�؂�ւ��C�x���g�����������ɏ��� */

			/* �؂�ւ� */
			part = IN_COURSE_PART_ON_FIGUREL;
			break;
		case IN_COURSE_PART_ON_GOAL:		/* �S�[�����֑J�� */
			/* �R�[�X�؂�ւ��C�x���g�����������ɏ��� */

			/* �؂�ւ� */
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

	/* �R�[�X�ʑ��s���� */
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

	/* �R�[�X�؂�ւ��C�x���g */
	switch(courseJointDetected)
	{
		case OUT_COURSE_PART_ON_LOOKUPGATE:
			/* �R�[�X�؂�ւ��C�x���g�����������ɏ��� */

			part = OUT_COURSE_PART_ON_LOOKUPGATE;
			break;

		/* 2012/08/09 add t.akiyama <<<< */
		case OUT_COURSE_PART_ON_GOAL:		/* �S�[�����֑J�� */
			/* �R�[�X�؂�ւ��C�x���g�����������ɏ��� */

			part = OUT_COURSE_PART_ON_GOAL;
			break;
		default:
			break;
	}
	courseJointDetected = 0; // reset
	return part;
}
#endif // !MAKE_INSIDE

// ���Z�b�g���邽�߂ɁA�K�����グ��i�����ɓ�����܂Łj
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
#if 0 //���͌�őΉ�
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
	for(int i=0;i<200;i++) // �������ԍs�������Ȃ���Β��߂�悤�ɂ��Ă���
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

// �����L�����u���[�V����
void LineDetectionlevelCalibration()
{
	static U8 LineDetectionlevelCalibration_mode = 0;

	// ���C�����؂蓮��
	static signed char pwm = 20;		// �ł��邾���ᑬ�ɂ���
	static U16 lightValueMax = 0;		// �A�i���Omin�l�ŏ�����
	static U16 lightValueMin = 1023;	// �A�i���Omax�l�ŏ�����

	// ���������܂����悤�ɁA���C����90�������Ēu���āA�X�C�b�`��������n cm�X�L��������
	if(LineDetectionlevelCalibration_mode == 0)
	{
		nxt_motor_set_count(NXT_PORT_C, 0); // �����[�^�G���R�[�_���Z�b�g
		nxt_motor_set_count(NXT_PORT_B, 0); // �E���[�^�G���R�[�_���Z�b�g
		LineDetectionlevelCalibration_mode++;
	}
	else if(LineDetectionlevelCalibration_mode == 1)
	{
		// 2012/08/13 add by s.sahara >>>>
		//ecrobot_sound_tone(440U, 100U, 50U);
		//systick_wait_ms(300);
		//ecrobot_sound_tone(0U, 0U, 0U);
		//systick_wait_ms(300);

		// �K���o���đҋ@
		//while(1)
		//{
//			tail_control(108);	// 108��(�Œ�)�ő���i�␳�p�f�[�^�̊��108���Ƃ����ׁj

			if(mode==BEFORE_CALIBRATION2) tail_control(GATE_TAIL_ANGLE_UNDER_GATE);		// �Q�[�g��������Ƃ��̊p�x
			else tail_control(TAIL_ANGLE_FOR_BALANCE_RUN_START);

			UINT sw = ecrobot_get_touch_sensor(NXT_PORT_S4);
			if ((1 == sw) && (0 == sw_prv))
			{
				//ecrobot_sound_tone(440U, 100U, 50U);
				//for(int i=0;i<30;i++)
				//{
				//	tail_control(TAIL_ANGLE_TAILRUN);	// �K�����s�p�p�x�Œ�~
				//	systick_wait_ms(10);
				//}
				//ecrobot_sound_tone(0U, 0U, 0U);
				//for(int i=0;i<30;i++)
				//{
				//	tail_control(TAIL_ANGLE_TAILRUN);	// �K�����s�p�p�x�Œ�~
				//	systick_wait_ms(10);
				//}
				//break; // �^�b�`�Z���T�������ꂽ
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

//			tail_control(TAIL_ANGLE_TAILRUN);	// �K�����s�p�p�x�Œ�~
			if(mode==BEFORE_CALIBRATION2) tail_control(GATE_TAIL_ANGLE_UNDER_GATE);		// �Q�[�g��������Ƃ��̊p�x
			else tail_control(TAIL_ANGLE_FOR_BALANCE_RUN_START);

			nxt_motor_set_speed(NXT_PORT_B, pwm, 1);
			nxt_motor_set_speed(NXT_PORT_C, pwm, 1);

			if(lightValue > lightValueMax) lightValueMax = lightValue;
			if(lightValue < lightValueMin) lightValueMin = lightValue;
			//if(distanceAbs > 0.08f) break; // 8cm���؂点��
			if(distanceAbs > 0.08f) LineDetectionlevelCalibration_mode++; // 8cm���؂点��

			ecrobot_debug1(lightValueMax, lightValueMin, lightValue);
#ifdef USE_LOG
			//ecrobot_bt_data_logger((S8)(distanceAbs * 1000.0f), (S8)((lightValue - 580)*0.5f)); // ���؂�g�`����p
#endif
		//	systick_wait_ms(4);
		//}
	}
	else if(LineDetectionlevelCalibration_mode == 3)
	{
		nxt_motor_set_speed(NXT_PORT_B, 0, 1);
		nxt_motor_set_speed(NXT_PORT_C, 0, 1);

		// �����̒[�͐؂�imax-min���̒[��n%���J�b�g�j
		int delta = lightValueMax - lightValueMin;

		if(mode==BEFORE_CALIBRATION2)
			LineTraceControl_InitLightThreshold2(lightValueMax - (U16)(delta * 0.2f), lightValueMin + (U16)(delta * 0.2f)); // 20%�Ƃ��Ă���
		else
			LineTraceControl_InitLightThreshold(lightValueMax - (U16)(delta * 0.2f), lightValueMin + (U16)(delta * 0.2f)); // 20%�Ƃ��Ă���

		LineDetectionlevelCalibration_mode = 0;
		sw_prv = 0;
		mode++; //�X�^�[�g���C���Z�b�g
	}

	//TODO �L�����u���[�V�����I�������ŃX�^�[�^�Ɏ���
}

//�ݒu�ʒu����p�֐�
//TODO �L�����u���[�V�������ʂɂ��œK�Ȑݒu�ʒu�����ŃX�^�[�^�Ɏ���
int SetRobotToStartLine()
{
	tail_control(TAIL_ANGLE_FOR_BALANCE_RUN_START);	// �o�����X���s�p�p�x�Œ�~

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

// ��Q���m
static U16 curveCounter;
int DetectFail()
{
	// �]�|���m�i�O��p���x�����ȏ�Ō��m�B��Ŋp�x�ς��Ƃ���ł͎g�p���Ȃ����Ɓj
	S32 angle = abs(AngleMeter());
	if(angle > FALL_DETECTION_ANLE_RATE) return ERROR_FALL;

	// ����A�����m
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

// �ُ팟�m�L��
// returns 1:�L�� 0:����
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
// �^�X�N�� : TaskMain
// �T�v : ���C���^�X�N
//*****************************************************************************
TASK(TaskMain)
{
	GetResource(resource1);

	static int part = 0;
#if 0		//��Q���m�̍폜
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
			 * Bluetooth�ʐM�p�f�o�C�X���̕ύX�́ABluetooth�ʐM�ڑ����m������Ă��Ȃ��ꍇ�̂ݗL���ł��B
			 * �ʐM�ڑ��m�����ɂ̓f�o�C�X���͕ύX����܂���B(���L��API�͉������܂���)
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
			// �L�����u���[�V�����ׂ̈����ł����Z�b�g
			nxt_motor_set_count(NXT_PORT_C, 0); // �����[�^�G���R�[�_���Z�b�g
			nxt_motor_set_count(NXT_PORT_B, 0); // �E���[�^�G���R�[�_���Z�b�g
			nxt_motor_set_count(NXT_PORT_A, 0); // �K�����[�^�G���R�[�_���Z�b�g
			tail_zero_count_clear();
			mode++;
			break;

		case BEFORE_CALIBRATION:
		case BEFORE_CALIBRATION2:
			//TASK(Task_1ms)�Ŕ����L�����u���[�V������
			LineDetectionlevelCalibration();
			break;

		case BEFORE_STARTLINE_SET:
			// ���s�J�n�ʒu�֐ݒu
			if(1 == SetRobotToStartLine())
			{
				// �Z�b�g����
				initSpeedoMeter(4);
				initAngleMeter(4);
				mode++;
			}
			break;

		case BEFORE_TOUCH:
			// �J�n�g���K�҂�
			//LightIn();		// ���Z���T����
			#ifndef USE_SAMPLE_COURCE
			tail_control(TAIL_ANGLE_FOR_BALANCE_RUN_START);	// �o�����X���s�p�p�x�Œ�~
			//tail_control(TAIL_ANGLE_TAILRUN);	// �K�����s�p�p�x�Œ�~
			#else // #ifndef USE_SAMPLE_COURCE
			#ifdef TEST_SLOW_FORWARD_RUN
			tail_control(108);
			#else // #ifdef TEST_SLOW_FORWARD_RUN
			tail_control(TAIL_ANGLE_TAILRUN);	// �K�����s�p�p�x�Œ�~
			#endif // #ifdef TEST_SLOW_FORWARD_RUN
			#endif // #ifndef USE_SAMPLE_COURCE

			if (remote_start() == 1)
			{
				// �����[�g�X�^�[�g
				mode++;
			}

			UINT sw = ecrobot_get_touch_sensor(NXT_PORT_S4);
			if ((1 == sw) && (0 == sw_prv))
			{
				// �^�b�`�Z���T�������ꂽ
				sw_prv = sw;
				mode++;
			}
			sw_prv = sw;
			break;

		case BEFORE_RUN_INIT:
#if 0	//�X�^�[�g�����i�����̍폜
			// ���i����t���œ|�����s�����肷��܂ŃX�^�[�g���� -- ������ 2013/08 add by s.sahara >>>>
			iStartUp = 0;
			accelT = 0.0f;
			accelTmax = 0.5f;
			vMax = 60; // ���肵�ĉ����ł���ڕW���x�i���@�����j
			InitBalanceControlStraightP();
			// <<<<
#endif
			balance_init();						/* �|���U�q���䏉���� */
			nxt_motor_set_count(NXT_PORT_C, 0); /* �����[�^�G���R�[�_���Z�b�g */
			nxt_motor_set_count(NXT_PORT_B, 0); /* �E���[�^�G���R�[�_���Z�b�g */

			LineTrace_Init();

			//##########################################################
			// kuni���C���g���[�X�̏����� add by kunitake <2014/08/13>
			//##########################################################
			//init_model();  ������͏�́uLineTrace_Init()�v�̒��Ŏ��s


			mode++;
			break;

		// ���i����t���œ|�����s�����肷��܂ŃX�^�[�g���� 2013/08 add by s.sahara
		case START_UP:
#if 0	//��Q���m�̍폜
			switch(DetectFail()){
				case ERROR_FALL: mode = EXIT; break;
				case ERROR_LOST:
					BalanceToTail_Init();
					mode = STOP;
					break;
			}
#endif
#if 0	//�X�^�[�g�����i�����̍폜
			if(iStartUp < (accelTmax*1000/4))
			{
				// �K����������
				tail_pid_control(TAIL_KP, TAIL_KI, TAIL_KD, 0, TAIL_MAX_MV, TAIL_MIN_MV);
				// �|����������Ŕ����őO�i�J�n
				vTarget = (signed char)SCurve((float)vMax, (float)0, accelTmax, &accelT);
				BalanceControl1StraightP(vTarget, 1, 10);	// �����W���C���I�t�Z�b�g�t��
				iStartUp++;
			}
			else
			{
				ResetDetectDeviateFromLine();
				mode++;
			}
#endif

			//...#########################################################
			//... mono�X�^�[�g�A�b�v�����i���s�J�n���̃��C����E�̕��A�j
			//... add by mononobe <2014/09/06>
			//...#########################################################
//...			ecrobot_sound_tone(100, 200, 30);	//... �f�o�b�O�p�T�E���h

			tail_control(TAIL_ANGLE_DRIVE);


#if 0	//... plan1 : kuni���C���g���[�X���g��
/*
			enc_l = nxt_motor_get_count(NXT_PORT_C); //... �����[�^�G���R�[�_�l�擾
			enc_r = nxt_motor_get_count(NXT_PORT_B); //... �E���[�^�G���R�[�_�l�擾
#ifdef USE_LOG
			ecrobot_bt_data_logger((S8)(enc_l/10), (S8)(enc_r/10));
#endif

			//... kuni���C���g���[�X
			forward = 100;
			ctrl_direction((float)forward);
			turn = get_model_turn();
			forward = get_model_forward();

			//... �X�^�[�g�A�b�v����̌�ތ��m�ƁA�O�i�ɐ؂�ւ�������̌��m
			if( (enc_l-enc_l0)>0 || (enc_r-enc_r0)>0 ){		//... ���E�ǂ��炩�̑O�i�����m������ʏ폈����	//...�s���т�݂��������ǂ�
				//... �������Ȃ��i�{�Ԃł�mode++�j
			}
			else if((enc_l+enc_r)<0){		//... ��ތ��m������A���C���g���[�Xturn�𔽓]
				turn = -turn;
			}
*/

			BalanceControl(100, 0);

//...			enc_l0 = enc_l;		//... �����[�v�����p
//...			enc_r0 = enc_r;		//... �����[�v�����p

#else	//... plan2 : std�|���ł��
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
			// kuni���C���g���[�X�p�Ƀ��f���̏�Ԏ擾 add by kunitake <2014/08/13>
			//#######################################################################
			get_model_state();

			break;

		case RUN:
#if 0	//��Q���m�̍폜
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
			// kuni���C���g���[�X�p�Ƀ��f���̏�Ԏ擾 add by kunitake <2014/08/13>
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
// �^�X�N�� : Task_1ms
// �T�v : 1ms�����Ăяo���^�X�N
//			�E���Z���T�̒l��ǂݎ���Ċe�ϐ��ɐݒ肷��
//			�E���s���͌��Z���T�l��LPF�v�Z����
//			�E���{�b�g�����C������O�ꂽ�Ƃ��Ƀ��[�^���~����
//			�E���Z���T��LED��_�ł�����
//*****************************************************************************
TASK(Task_1ms)
{
	GetResource(resource1);

	switch(mode)
	{
		case BEFORE_CALIBRATION:
		case BEFORE_CALIBRATION2:
			#ifndef TEST_SLOW_FORWARD_RUN
			// �����L�����u���[�V����
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

	//���F���m����΃��[�^���~�߂� ���F�͉��̌Œ�l
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
					/* �ԐFLED��_������ */
					ecrobot_set_light_sensor_active(NXT_PORT_S3);
					on_off_flg = 0U;
				}
				else
				{
					/* �ԐFLED���������� */
					ecrobot_set_light_sensor_inactive(NXT_PORT_S3);
					on_off_flg = 1U;
				}
				break;
			case BEFORE_STARTLINE_SET:
				/* �ԐFLED��_������ */
				ecrobot_set_light_sensor_active(NXT_PORT_S3);
				break;
			default:
				/*�������Ȃ�*/
				break;
		}
	}
#endif

	ReleaseResource(resource1);
	TerminateTask();
}

//*****************************************************************************
// �֐��� : remote_start
// ���� : ����
// �Ԃ�l : 1(�X�^�[�g)/0(�ҋ@)
// �T�v : Bluetooth�ʐM�ɂ�郊���[�g�X�^�[�g�B Tera Term�Ȃǂ̃^�[�~�i���\�t�g����A
//		 ASCII�R�[�h��1�𑗐M����ƁA�����[�g�X�^�[�g����B
//*****************************************************************************
static int remote_start(void)
{
	int i;
	unsigned int rx_len;
	unsigned char start = 0;

	for (i=0; i<BT_MAX_RX_BUF_SIZE; i++)
	{
		rx_buf[i] = 0; /* ��M�o�b�t�@���N���A */
	}

	rx_len = ecrobot_read_bt(rx_buf, 0, BT_MAX_RX_BUF_SIZE);
	if (rx_len > 0)
	{
		/* ��M�f�[�^���� */
		if (rx_buf[0] == CMD_START)
		{
			start = 1; /* ���s�J�n */
		}
		ecrobot_send_bt(rx_buf, 0, rx_len); /* ��M�f�[�^���G�R�[�o�b�N */
	}

	return start;
}
