/* 2012/07/21 created by s.sahara */

/* �R�[�X��ԕʃR�[�h��` */
/*
�ύX����
2012/08/12 add by s.sahara �e��Ԃ̋�����`�ǉ��B
2012/08/09 mod by s.sahara IN��OUT�ʂɒ�`���������B
2013/08/xx mod by s.sahara 2013�N�x�R�[�X�ɑΉ��B
2014/09/05 mod by k.nakagawa 2014�N�x�R�[�X�ɑΉ��B��Ԃ��Ƃ̑��x�E�Q�C���̒������폜
*/

#ifndef __COURCEDEF_H__
#define __COURCEDEF_H__

// MAKE�؂�ւ�

// �@�̐؂�ւ�
//#define STEC_EAST ��make�t�@�C���Ő؂�ւ�
#ifdef STEC_EAST
#warning #### STEC_EAST machine selected ####
#else
#warning #### STEC_WEST machine selected ####
#endif

// make����IN or OUT �R�[�X�؂�ւ�
// #define MAKE_INSIDE ��make�t�@�C���Ő؂�ւ�
#ifdef MAKE_INSIDE
#warning #### INSIDE selected ####
#else
#warning #### OUTSIDE selected ####
#endif

#if 0
#define ENABLE_SLOPE // �⓹�g�p�iOFF�Ȃ畽�n�łő��s�j
#endif

// �㔼����m�F�p�X�C�b�`
// ��IN����OUT���Ƃ��ɁA�x�[�V�b�N�R�[�X�̃S�[���Q�[�g���瓮��m�F����ꍇ
#if 0
#define TEST_BONUS
#endif

// ����m�F�p�X�C�b�`
#if 0
#define USE_SAMPLE_COURCE // ��{�ԃR�[�X�i�T���v���R�[�X��R�[�X�����ł̓���m�F�j
#if 0 // �P�Ȃ�|���ᑬ�O�i
#define TEST_SLOW_FORWARD_RUN
#endif
#if 1 // ���C���g���[�XPID�Q�C�������p�i�K�����s��max���s����̂݁j
#define ADJUST_LT_PID
#endif
#endif

//1ms����BT���O�L�^�X�C�b�`
#if 0
#define USE_1MS_BT_LOG
#endif

//�t�F�[���Z�[�t�X�C�b�`
#if 0
#define USE_FAIL_SAFE
#endif

//���C�g�_�ŃX�C�b�`
#if 0
#define USE_LIGHT_BLINK
#define LIGHT_BLINK_CYCLE 20
#endif

//���O�擾�X�C�b�`
#if 0
#define USE_LOG
#endif

// �R�[�X�T�C�h�iIN or OUT�j
#define COURSE_IN_SIDE 0
#define COURSE_OUT_SIDE 1

#ifdef MAKE_INSIDE
#define STARTING_SIDE COURSE_IN_SIDE
#else
#define STARTING_SIDE COURSE_OUT_SIDE
#endif

// �R�[�X���ʁi���C���g���[�X or ��j
// IN
enum CourseIn
{
	IN_COURSE_PART_ON_START = 0,
//	IN_COURSE_PART_BEFORE_SLOPE,
//	IN_COURSE_PART_ON_SLOPE,
//	IN_COURSE_PART_AFTER_SLOPE,
	IN_COURSE_PART_BEFORE_FIGUREL,
	IN_COURSE_PART_ON_FIGUREL,
	IN_COURSE_PART_BEFORE_GOAL,
	IN_COURSE_PART_ON_GOAL,
	IN_COURSE_PART_ON_RETURN_TO_LINE,
};

// OUT
enum CourseOut
{
	OUT_COURSE_PART_ON_START = 0,
//	OUT_COURSE_PART_BEFORE_SLOPE,
//	OUT_COURSE_PART_ON_SLOPE,
//	OUT_COURSE_PART_AFTER_SLOPE,
	OUT_COURSE_PART_BEFORE_LOOKUPGATE,
	OUT_COURSE_PART_ON_LOOKUPGATE,
	OUT_COURSE_PART_BEFORE_GOAL,
	OUT_COURSE_PART_ON_GOAL,
	OUT_COURSE_PART_ON_RETURN_TO_LINE,
};


// �e��Ԃ̋����i���v���J�R�[�X�̃��C���̒��S�Ŏ����B���x��cm���x�Ȃ̂Ŏg�p���@���ӁB�j
// IN�R�[�X��ԋ���
// �x�[�V�b�N
#define IN1_NORMAL_DISTANCE (10.0f)	// ��܂ł̋���


// OUT�R�[�X��ԋ���
// �x�[�V�b�N
#define OUT1_NORMAL_DISTANCE (11.7f)	// ��܂ł̋���
//#define OUT1_NORMAL_DISTANCE (0.3f)	// ��܂ł̋���debug�p


#define MARKER_LENGTH (0.15f)

//�}�[�J�[���m�p�������l�i�P�ʎ��ԓ�������̌��Z���T�l�̕ω��ʁj
//�ω��ʂ͔��F��0%�A���F��100%�Ƃ������̕S�����i���j�Ŏw��
#define MARKER_IN_DETECT_DELTA (-5.5f)
#define MARKER_OUT_DETECT_DELTA (5.5f)

#endif // __COURCEDEF_H__
