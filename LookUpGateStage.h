/* 2012/07/21 created by s.sahara ��N���A�P�̃e�X�g�p */

extern int LookUpGateStageMain();
extern int TailAngleCheck(int reqAngle);
extern void LookupGate_tail_control(signed int angle);

/* �K���̊p�x��` */
#define GATE_TAIL_ANGLE_UNDER_GATE	65	// ���S��~���̊p�x[�x]
#define GATE_TAIL_ANGLE_DRIVE		TAIL_ANGLE_TAILRUN	// �o�����X���s���̊p�x[�x]
#define GATE_TAIL_ANGLE_SONAR		100
