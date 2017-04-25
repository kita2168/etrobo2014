/* 2012/08/09 created by t.akiyama 難所クリア単体テスト用 */

#include "Calc.h"

#include "balancer.h"
#include "ecrobot_interface.h"

//*****************************************************************************
// 関数名 : calc_distance
// 引数 : 角度A , 角度B
// 返り値 : 走行距離
// 概要 : 1体のモータ回転角度から走行距離を求める
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
	
	/* 走行距離を絶対値で返す */
	distance = 85 * PI / (360 / diff_deg);
	
	return distance;
}

// エンコーダゼロリセットからの距離(m)
float calc_getDistanceFromStart()
{
	float radL = nxt_motor_get_count(NXT_PORT_C) * DEG2RAD;	/* 左モータ回転角度[deg] */
	float radR = nxt_motor_get_count(NXT_PORT_B) * DEG2RAD;	/* 右モータ回転角度[deg] */
	
	float distance = WHEEL_D * ((radL + radR) / 4.0f); // 左右移動を平均したものが中心移動量とする(さらに角度/2)
	return distance;
}


//*****************************************************************************
// 回転距離計測用
//*****************************************************************************
static float baseValue_radB;
static float baseValue_radC;
static unsigned char rotateLR;

/* イニシャルからの回転距離を求める */
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
	
	/* 片輪では誤差が大きいため両輪の回転角度から平均を求める */
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

/* 初期化 check_rotateCompleteを使用する前に初期化必須 */
void init_rotateDistance(unsigned char a_rotateLR)
{
	/* モーターB,Cの基準値を保持 */
	baseValue_radB = nxt_motor_get_count(NXT_PORT_B) * DEG2RAD;
	baseValue_radC = nxt_motor_get_count(NXT_PORT_C) * DEG2RAD;
	rotateLR = a_rotateLR;
}

/* 引数で与えられた角度に走行体が回転したか判定する */
int check_rotateComplete(unsigned int a_rotateDeg)
{
	int unsigned ret = 0;
	
	float targetRad = (float)(a_rotateDeg) * DEG2RAD;
	
	/* 走行体が引数の回転角度になるまでに必要な距離 */
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

	/* 0:回転未完了 1:回転完了 */
	return ret;
}

//*****************************************************************************
// 任意位置からの走行距離計測用
//*****************************************************************************
static float pointValue_radL; 
static float pointValue_radR;

float calc_getDistanceFromPoint()
{
	float radL = (nxt_motor_get_count(NXT_PORT_C) - pointValue_radL) * DEG2RAD;	/* 左モータ回転角度[deg] */
	float radR = (nxt_motor_get_count(NXT_PORT_B) - pointValue_radR) * DEG2RAD;	/* 右モータ回転角度[deg] */
	
	float distance = WHEEL_D * ((radL + radR) / 4.0f); // 左右移動を平均したものが中心移動量とする(さらに角度/2)
	return distance;
}

void init_DistanceFromPoint()
{
	pointValue_radL = nxt_motor_get_count(NXT_PORT_C);
	pointValue_radR = nxt_motor_get_count(NXT_PORT_B);
}

