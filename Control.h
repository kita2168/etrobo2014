/* 2012/08/08 created by s.sahara ��N���A�P�̃e�X�g�p */

/*
��������n�v�Z�֘A
*/


#define DELTA_T 0.004f // 4ms
#define CONTROL_CYCLE 4

/*
P����
kp:���Q�C�� command:�ڕW�l error:�덷 max:�����max min:�����min
�߂�l:�����
*/
extern float ControlP(float kp, float command, float error, float max, float min);

/*
PD����
kp:���Q�C�� command:�ڕW�l error:�덷 hensa0:�O��΍� hensa1:����΍� max:�����max min:�����min
�߂�l:�����
*/
extern float ControlPD(float kp, float kd, float command, float error, float hensa0, float* hensa1, float max, float min);


/*
PID����
kp:���Q�C�� command:�ڕW�l error:�덷 hensa0:�O��΍� hensa1:����΍� integral:�ϕ��l max:�����max min:�����min
�߂�l:�����
*/
extern float ControlPID(float kp, float ki, float kd, float command, float error, float hensa0, float* hensa1, float* integral, float max, float min);

extern float SCurve(float Vaft, float Vbfr, float Tmax, float* T);
