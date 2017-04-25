// 2012/08/13 created by s.sahara

// ���C���g���[�X���ʐ��䏈��

#include "ecrobot_interface.h"
#include "balancer.h" /* �|���U�q����p�w�b�_�t�@�C�� */

/* �W���C���Z���T�I�t�Z�b�g�l(�p���x0[deg/sec]��) */
#ifdef STEC_EAST // �̍��؂�ւ�
#define GYRO_OFFSET  605
#else
#define GYRO_OFFSET  608
#endif
#define FALL_DETECTION_ANLE_RATE 200 // �]�|���m���ϊp���x(level/sec)	�c�����l�����ɐݒ肷��B�����Ŏg���͕̂��ϊp���x�Ȃ̂ŁA���m��݂点������Ε��ω񐔂𑝂₷���ƁB�������������g�p�ʂɒ��ӁB��Ίp�x�͈��肵�đ���ł��Ȃ��B

// ���L�����u���[�V���������l���g�p���邱�Ɓi�ϐ��̏�������f�o�b�O�ł̂ݎg�p�j
#define LIGHT_WHITE	 610 /* ���F�̌��Z���T�l */	// 2012/08/12 mod by s.sahara �����Ȑ���邷����ׁA�����֏����V�t�g�B�����C�����؂葪�肵�Ĕg�`���Č��肷�邱�ƁB
//#define LIGHT_WHITE	 580 /* ���F�̌��Z���T�l */
#define LIGHT_BLACK	 680 /* ���F�̌��Z���T�l */ // 2012/08/12 mod by s.sahara ���x�グ��ƃR�[�i�ŃA�E�g�ɖc���Ő����S���z���Ă��܂����߉�����B�����C�����؂葪�肵�Ĕg�`���Č��肷�邱�ƁB
//#define LIGHT_BLACK	 700 /* ���F�̌��Z���T�l */
// ���L�����u���[�V���������l���g�p���邱�Ɓi�ϐ��̏�������f�o�b�O�ł̂ݎg�p�j

#define SONAR_ALERT_DISTANCE 20 /* �����g�Z���T�ɂ���Q�����m����[cm] */
#define TAIL_ANGLE_STAND_UP 108 /* ���S��~���̊p�x[�x] */
#define TAIL_ANGLE_DRIVE	  3 /* �o�����X���s���̊p�x[�x] */
#define P_GAIN			   2.5F /* ���S��~�p���[�^������W�� */
#define PWM_ABS_MAX 		 60 /* ���S��~�p���[�^����PWM��΍ő�l */

// �����i���}�[�J���m�i���x���o���ă}�[�J�ʉ߂����A���Z���T���̓��x���ω�����臒l�Ō��m�j
// �}�[�J���m���x���i�O���[��l�Ɠ��͂̕΍��́A�O��Ƃ̍��ɑ΂���臒l�j
// �����|�����s���x80�C���Z���T���͂R�񕽋ςŒ����B���x�ύX���͗v�����B
// ���L�����u���[�V���������l���g�p���邱�Ɓi�ϐ��̏�������f�o�b�O�ł̂ݎg�p�j
#define LIGHT_STANDARD ((LIGHT_WHITE + LIGHT_BLACK) / 2.0F)	// ���ƍ��̒��Ԃ��}�[�J�Ƃ��Ă���
// ���L�����u���[�V���������l���g�p���邱�Ɓi�ϐ��̏�������f�o�b�O�ł̂ݎg�p�j
#define MARKER_ON_THRESHOLD_LEVEL (-0.4f)	// [%] OUT�w�A�s����spd40�Ō��m�ł��Ȃ��̂Ŏg�p�ꏊ����
//#define MARKER_ON_THRESHOLD_LEVEL (-0.8f)	// [%] �K�����s spd80�Ō��m�ł���
#define MARKER_OFF_THRESHOLD_LEVEL (0.2f)	// [%]

// �ᑬ�i���}�[�J���m�i���x���o���Ȃ��w�A�s�������オ��̃}�[�J���Ŏg�p�j
#define LOWSPEED_MARKER_DETECTION__SPD (20) // [%]
// ���L�����u���[�V���������l�̑��Βl���g�p���邱��
#define LOWSPEED_MARKER_DETECTION__LIGHT_THRESHOLD (635) // �����O���[�C�O���[�������x������p���Z���T���̓��x��臒l ���␳�K�v
// ���L�����u���[�V���������l�̑��Βl���g�p���邱��
#define LOWSPEED_MARKER_DETECTION__LIGHT_THRESHOLD_P (0.5f) // �O���[�̖��邳�́A�������ɑ΂���䗦
#define MARKER_IN_DETECTION (-15.0f)
#define MARKER_OUT_DETECTION (25.0f)

extern void BalanceControl(char forward, char turn);
extern void BalanceControl1(char forward, char turn, float addGyroOffset);
extern void TailRunControl(char forward, char turn);

extern void LineTraceControl_InitLightThreshold(U16 lightBlack, U16 lightWhite);
extern U16 LineTraceControl_LightBlackThreshold();
extern U16 LineTraceControl_LightWhiteThreshold();
extern void LineTraceControl_InitLightThreshold2(U16 lightBlack, U16 lightWhite);
extern U16 LineTraceControl_LightBlackThreshold2();
extern U16 LineTraceControl_LightWhiteThreshold2();

//���͈͌v�Z�p
extern U16 LineTraceControl_getLED_Band_Width();
extern U8 LineTraceControl_getLED_Band_Width_Per();
extern float LineTraceControl_getLight_Band_Rel_P(U16 lightlevel, U16 prevlightlevel);

//�}�[�J�[����p
extern void LineTraceControl_MakerDetect_Init();
extern S8 LineTraceControl_getMaker_detect();
extern S8 LineTraceControl_MakerDetect();

extern void LineTraceControl_ChangeLightThreshold(U16 lightBlack, U16 lightWhite, float* pHensa0, float* pIntegral); // ���䒆��臒l�ύX
extern signed char LineTraceControl_DirectionPID(float kp, float ki, float kd, float* pHensa0, float* pIntegral, float max, float min);

#if 1 // 2012/09/16 add by s.sahara ���C����E���m
extern U8 IsDeviateFromLine();
extern void ResetDetectDeviateFromLine();
extern float GetLightLevelOnLastDetectDeviate();
extern float GetNormalizedLightLevel();
#endif

extern void tail_control(signed int angle);
extern void tail_pid_control_reset();
extern void tail_pid_control(float kp, float ki, float kd, float angle, float max, float min);

extern void LinTraceControl_Periodic();
extern float LinTraceControl_GetCurveAngle();

extern void InitBalanceControlStraightP();
extern void BalanceControlStraightP(char forward, float kp);
extern void InitTailRunControlStraightP();
extern void TailRunControlStraightP(char forward, float kp);
extern void BalanceControl1StraightP(char forward, float kp, float offset);
