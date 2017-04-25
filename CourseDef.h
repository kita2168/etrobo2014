/* 2012/07/21 created by s.sahara */

/* コース区間別コード定義 */
/*
変更履歴
2012/08/12 add by s.sahara 各区間の距離定義追加。
2012/08/09 mod by s.sahara INとOUT別に定義し直した。
2013/08/xx mod by s.sahara 2013年度コースに対応。
2014/09/05 mod by k.nakagawa 2014年度コースに対応。区間ごとの速度・ゲインの調整を削除
*/

#ifndef __COURCEDEF_H__
#define __COURCEDEF_H__

// MAKE切り替え

// 機体切り替え
//#define STEC_EAST →makeファイルで切り替え
#ifdef STEC_EAST
#warning #### STEC_EAST machine selected ####
#else
#warning #### STEC_WEST machine selected ####
#endif

// makeするIN or OUT コース切り替え
// #define MAKE_INSIDE →makeファイルで切り替え
#ifdef MAKE_INSIDE
#warning #### INSIDE selected ####
#else
#warning #### OUTSIDE selected ####
#endif

#if 0
#define ENABLE_SLOPE // 坂道使用（OFFなら平地版で走行）
#endif

// 後半動作確認用スイッチ
// ※IN側とOUT側ともに、ベーシックコースのゴールゲートから動作確認する場合
#if 0
#define TEST_BONUS
#endif

// 動作確認用スイッチ
#if 0
#define USE_SAMPLE_COURCE // 非本番コース（サンプルコースやコース無しでの動作確認）
#if 0 // 単なる倒立低速前進
#define TEST_SLOW_FORWARD_RUN
#endif
#if 1 // ライントレースPIDゲイン調整用（尻尾走行でmax走行するのみ）
#define ADJUST_LT_PID
#endif
#endif

//1ms周期BTログ記録スイッチ
#if 0
#define USE_1MS_BT_LOG
#endif

//フェールセーフスイッチ
#if 0
#define USE_FAIL_SAFE
#endif

//ライト点滅スイッチ
#if 0
#define USE_LIGHT_BLINK
#define LIGHT_BLINK_CYCLE 20
#endif

//ログ取得スイッチ
#if 0
#define USE_LOG
#endif

// コースサイド（IN or OUT）
#define COURSE_IN_SIDE 0
#define COURSE_OUT_SIDE 1

#ifdef MAKE_INSIDE
#define STARTING_SIDE COURSE_IN_SIDE
#else
#define STARTING_SIDE COURSE_OUT_SIDE
#endif

// コース部位（ライントレース or 難所）
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


// 各区間の距離（レプリカコースのラインの中心で実測。精度はcm程度なので使用方法注意。）
// INコース区間距離
// ベーシック
#define IN1_NORMAL_DISTANCE (10.0f)	// 難所までの距離


// OUTコース区間距離
// ベーシック
#define OUT1_NORMAL_DISTANCE (11.7f)	// 難所までの距離
//#define OUT1_NORMAL_DISTANCE (0.3f)	// 難所までの距離debug用


#define MARKER_LENGTH (0.15f)

//マーカー検知用しきい値（単位時間当あたりの光センサ値の変化量）
//変化量は白色を0%、黒色を100%とした時の百分率（％）で指定
#define MARKER_IN_DETECT_DELTA (-5.5f)
#define MARKER_OUT_DETECT_DELTA (5.5f)

#endif // __COURCEDEF_H__
