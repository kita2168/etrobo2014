/* 2012/08/09 created by t.akiyama ��N���A�P�̃e�X�g�p */

/*
�v�Z�֘A
*/

#define PI (3.141593f)
#define WHEEL_D (0.081f) // �ԗ֒��a[m] �����ɒ�K�ő������B�^�C�����݁B
/* 2012/08/14 add by t.akiyama >>>> */
#define AXLE_D (0.162f) // �Ԏ��ԋ���[m] �����s�̂̉�]���a
/* 2012/08/14 add by t.akiyama <<<< */

/* ���s�����v�Z�i���[�^1�ɂ��P���v�Z
deg_A:���s�O�p�x deg_B:���݊p�x
�߂�l:���s�����i�P��:mm�j
*/
extern int calc_distance(int deg_A , int deg_B);


/* ���s�����v�Z�i�G���R�[�_���Z�b�g����̋����B�{�̒��S�ړ��ʁB
�߂�l:���s�����i�P��:m�j
*/
extern float calc_getDistanceFromStart();

extern void init_rotateDistance(unsigned char a_rotateLR);
extern float calc_getRotateDistance();
extern int check_rotateComplete(unsigned int a_rotateDeg);

extern void init_DistanceFromPoint();
extern float calc_getDistanceFromPoint();
