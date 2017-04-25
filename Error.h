/* 2013/09/15 created by s.sahara */

enum ErrorDef
{
	NO_ERROR = 0,
	ERROR_FALL,		// 転倒
	ERROR_LOST,		// 場所を見失う（旋回しっぱなし）
	ERROR_DEVIATE,	// ライン逸脱（制御範囲から外れた）
};
