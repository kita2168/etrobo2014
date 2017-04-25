/* 2012/08/08 created by s.sahara 難所クリア単体テスト用 */

/*
自動制御系計算関連
*/


#define DELTA_T 0.004f // 4ms
#define CONTROL_CYCLE 4

/*
P制御
kp:比例ゲイン command:目標値 error:誤差 max:制御量max min:制御量min
戻り値:制御量
*/
extern float ControlP(float kp, float command, float error, float max, float min);

/*
PD制御
kp:比例ゲイン command:目標値 error:誤差 hensa0:前回偏差 hensa1:今回偏差 max:制御量max min:制御量min
戻り値:制御量
*/
extern float ControlPD(float kp, float kd, float command, float error, float hensa0, float* hensa1, float max, float min);


/*
PID制御
kp:比例ゲイン command:目標値 error:誤差 hensa0:前回偏差 hensa1:今回偏差 integral:積分値 max:制御量max min:制御量min
戻り値:制御量
*/
extern float ControlPID(float kp, float ki, float kd, float command, float error, float hensa0, float* hensa1, float* integral, float max, float min);

extern float SCurve(float Vaft, float Vbfr, float Tmax, float* T);
