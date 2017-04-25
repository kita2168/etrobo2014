/* 2012/08/09 created by t.akiyama 難所クリア単体テスト用 */

/*
計算関連
*/

#define PI (3.141593f)
#define WHEEL_D (0.081f) // 車輪直径[m] ※仮に定規で測った。タイヤ込み。
/* 2012/08/14 add by t.akiyama >>>> */
#define AXLE_D (0.162f) // 車軸間距離[m] ※走行体の回転直径
/* 2012/08/14 add by t.akiyama <<<< */

/* 走行距離計算（モータ1個による単純計算
deg_A:走行前角度 deg_B:現在角度
戻り値:走行距離（単位:mm）
*/
extern int calc_distance(int deg_A , int deg_B);


/* 走行距離計算（エンコーダリセットからの距離。本体中心移動量。
戻り値:走行距離（単位:m）
*/
extern float calc_getDistanceFromStart();

extern void init_rotateDistance(unsigned char a_rotateLR);
extern float calc_getRotateDistance();
extern int check_rotateComplete(unsigned int a_rotateDeg);

extern void init_DistanceFromPoint();
extern float calc_getDistanceFromPoint();
