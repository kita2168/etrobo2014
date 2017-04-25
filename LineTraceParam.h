/* 2012/08/12 created by s.sahara */

/* ���C���g���[�X�p�p�����[�^ */

#define BALANCERUN_FORWARD_SPEED (80) // �|�����s�p���e�ő呬�x 2013/07/27 add by s.sahara

#define BALANCERUN_DIRECTION_MV_MAX (100.0f)
#define BALANCERUN_DIRECTION_MV_MIN (-100.0f)

#define TAIL_ANGLE_FOR_BALANCE_RUN_START	113
#define TAIL_ANGLE_BALANCE_RUN			3

// 2012/08/13 add by s.sahara >>>>
// �K�����s
#define TAILRUN_ENABLE	1
// ���E�H�[���M�A�����ڒn����悤��������
// ���w��100�ł��A�X�^���h��95�A���s��80�ɂȂ�B
// �c��������@�����K�v�B�i�d���ቺ�ɔ�Ⴕ�Ďw�߂𑝂����B���̕����x�����邩������Ȃ��̂łł��邾�����ĂĐK���ɉ׏d�����Ȃ����ƁB�j
#define TAIL_ANGLE_TAILRUN				100		// ��~���X�^�[�g���ʏ푖�s�i���]�|���S�͈͓��łł��邾�����Ă�j(��c���Ŕ����͈͌v�Z���ʂ�690�`640������)
#define TAIL_ANGLE_TAILRUN_BFR_SLOPE	90		// �⓹��O���i���p
#define TAIL_ANGLE_TAILRUN_SLOPE_UP		108		// �⓹���p
#define TAIL_ANGLE_TAILRUN_SLOPE_DW		80		// �⓹����p(��c���Ŕ����͈͌v�Z���ʂ�690�`650������)
// 2012/08/13 add by s.sahara <<<<

#define TAILRUN_DIRECTION_MV_MAX (100.0f)
#define TAILRUN_DIRECTION_MV_MIN (-100.0f)

#define SOUKYOKU_K (20.0f)

/* ���x�p���[�p�X�t�B���^ */
#define VEL_LPF_HZ (LPF_FS * 0.25f)

/* ���x �Q�C��*/
#define VELOCITY_PID_MAX_FORWARD (100.0f)
#define VELOCITY_PID_MIN_FORWARD (-100.0f)

/* ���x�Q�C���͊e�\�[�X�Œ������� */
#define VELOCITY_KP (1.0f)		// Kp = 0.6 * KU = 0.60
#define VELOCITY_KI (0.00f)		// Ki = Kp / Ti = 1.20 ; Ti = 0.5 * PU = 0.50
#define VELOCITY_KD (0.00f)		// Kd = Kp * Td = 0.075 ; Td = 0.125 * PU = 0.125

#if 1
//�K�����[�^��PID��������
// KU=24.0, PU=0.05
#define TAIL_KP (14.4f) // Kp = 0.6 * KU = 14.4
#define TAIL_KI (0.36f)  // Ki = Kp * Ti = 576.0 ; Ti = 0.5 * PU = 0.025
#define TAIL_KD (0.09f * 2.0f)  // Kd = Kp * Td = 0.09 ; Td = 0.125 * PU = 0.00625 //�O�ɓ|���̂�2�{�ɂ���
#define TAIL_MAX_MV (100.0f)
#define TAIL_MIN_MV (-100.0f)
#elif 0
#define DIRECTION_TAIL_KP (24.0f) //�K���Ŏx���Ă���Ƃ���56ms�����Ŏ����U������
#define DIRECTION_TAIL_KI (0.0f)
#define DIRECTION_TAIL_KD (0.0f)
#endif

/* ������ �Q�C�� ���v���@���� */

#if 1

// KU=0.30, PU=1.00 �������������O�̐U������
#define BALANCERUN_DIRECTION_KP (0.18f * 1.1f)		// Kp = 0.6 * KU = 0.18
#define BALANCERUN_DIRECTION_KI (0.36f * 1.1f)		// Ki = Kp / Ti = 1.20 ; Ti = 0.5 * PU = 0.50
#define BALANCERUN_DIRECTION_KD (0.0225f * 1.1f)		// Kd = Kp * Td = 0.075 ; Td = 0.125 * PU = 0.125

#else
#endif


#if 1
#define LIGHT_LPF_HZ (LPF_FS * 0.25f)		// �J�b�g�I�t�����𐧌������4�{���x�ɂ��Ă���
#define TAILRUN_FORWARD_SPEED (100)

// KU=0.30, PU=1.00 �������������O�̐U������
#define TAILRUN_DIRECTION_KP (0.18f * 1.1f)		// Kp = 0.6 * KU = 0.18
#define TAILRUN_DIRECTION_KI (0.36f * 1.1f)		// Ki = Kp / Ti = 1.20 ; Ti = 0.5 * PU = 0.50
#define TAILRUN_DIRECTION_KD (0.0225f * 1.1f)		// Kd = Kp * Td = 0.075 ; Td = 0.125 * PU = 0.125

#elif 0
// 2012/09/13 �������ǉ��i臕���max�Ȃ�+100�Amin�Ȃ�-100�ƂȂ�悤�ɕύX�����B�Q�C���������Ȃ����B�j
// �K�����s KU����
// TAIL_ANGLE_							100
#define LIGHT_LPF_HZ (LPF_FS * 0.25f)		// �J�b�g�I�t�����𐧌������4�{���x�ɂ��Ă���
#define TAILRUN_FORWARD_SPEED (100)
// KU=1.00, PU=1.00 �������������O�̐U������
// ������������́A�����O�Ɠ����̃Q�C����ݒ肷��

#define TAILRUN_DIRECTION_KP (0.30f)	// �����@09/13 22;34 ��csv
//#define TAILRUN_DIRECTION_KP (0.35f)	// ����
//#define TAILRUN_DIRECTION_KP (0.4f)	// ����
//#define TAILRUN_DIRECTION_KP (0.6f)	// �����@22:25
#define TAILRUN_DIRECTION_KI (0.0f)
#define TAILRUN_DIRECTION_KD (0.0f)

/*
#define AFTER_HABA_HOSEI (0.1f)
#define TAILRUN_DIRECTION_KP (0.6f * AFTER_HABA_HOSEI)		// Kp = 0.6 * KU = 0.60
#define TAILRUN_DIRECTION_KI (1.2f * AFTER_HABA_HOSEI)		// Ki = Kp / Ti = 1.20 ; Ti = 0.5 * PU = 0.50
#define TAILRUN_DIRECTION_KD (0.075f * AFTER_HABA_HOSEI)		// Kd = Kp * Td = 0.075 ; Td = 0.125 * PU = 0.125
*/

#endif
