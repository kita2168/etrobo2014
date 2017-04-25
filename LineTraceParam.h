/* 2012/08/12 created by s.sahara */

/* ライントレース用パラメータ */

#define BALANCERUN_FORWARD_SPEED (80) // 倒立走行用許容最大速度 2013/07/27 add by s.sahara

#define BALANCERUN_DIRECTION_MV_MAX (100.0f)
#define BALANCERUN_DIRECTION_MV_MIN (-100.0f)

#define TAIL_ANGLE_FOR_BALANCE_RUN_START	113
#define TAIL_ANGLE_BALANCE_RUN			3

// 2012/08/13 add by s.sahara >>>>
// 尻尾走行
#define TAILRUN_ENABLE	1
// ※ウォームギア部が接地するよう調整する
// ※指令100でも、スタンド時95、走行中80になる。
// …これも実機調整必要。（電圧低下に比例して指令を増す等。その分速度落ちるかもしれないのでできるだけ立てて尻尾に荷重かけないこと。）
#define TAIL_ANGLE_TAILRUN				100		// 停止→スタート→通常走行（※転倒安全範囲内でできるだけ立てる）(会議室で白黒範囲計算結果が690～640だった)
#define TAIL_ANGLE_TAILRUN_BFR_SLOPE	90		// 坂道手前側段差用
#define TAIL_ANGLE_TAILRUN_SLOPE_UP		108		// 坂道上り用
#define TAIL_ANGLE_TAILRUN_SLOPE_DW		80		// 坂道下り用(会議室で白黒範囲計算結果が690～650だった)
// 2012/08/13 add by s.sahara <<<<

#define TAILRUN_DIRECTION_MV_MAX (100.0f)
#define TAILRUN_DIRECTION_MV_MIN (-100.0f)

#define SOUKYOKU_K (20.0f)

/* 速度用ローパスフィルタ */
#define VEL_LPF_HZ (LPF_FS * 0.25f)

/* 速度 ゲイン*/
#define VELOCITY_PID_MAX_FORWARD (100.0f)
#define VELOCITY_PID_MIN_FORWARD (-100.0f)

/* 速度ゲインは各ソースで調整する */
#define VELOCITY_KP (1.0f)		// Kp = 0.6 * KU = 0.60
#define VELOCITY_KI (0.00f)		// Ki = Kp / Ti = 1.20 ; Ti = 0.5 * PU = 0.50
#define VELOCITY_KD (0.00f)		// Kd = Kp * Td = 0.075 ; Td = 0.125 * PU = 0.125

#if 1
//尻尾モータのPID調整測定
// KU=24.0, PU=0.05
#define TAIL_KP (14.4f) // Kp = 0.6 * KU = 14.4
#define TAIL_KI (0.36f)  // Ki = Kp * Ti = 576.0 ; Ti = 0.5 * PU = 0.025
#define TAIL_KD (0.09f * 2.0f)  // Kd = Kp * Td = 0.09 ; Td = 0.125 * PU = 0.00625 //前に倒れるので2倍にした
#define TAIL_MAX_MV (100.0f)
#define TAIL_MIN_MV (-100.0f)
#elif 0
#define DIRECTION_TAIL_KP (24.0f) //尻尾で支えているときに56ms周期で持続振動する
#define DIRECTION_TAIL_KI (0.0f)
#define DIRECTION_TAIL_KD (0.0f)
#endif

/* 方向舵 ゲイン ※要実機調整 */

#if 1

// KU=0.30, PU=1.00 ※幅調整実装前の振動測定
#define BALANCERUN_DIRECTION_KP (0.18f * 1.1f)		// Kp = 0.6 * KU = 0.18
#define BALANCERUN_DIRECTION_KI (0.36f * 1.1f)		// Ki = Kp / Ti = 1.20 ; Ti = 0.5 * PU = 0.50
#define BALANCERUN_DIRECTION_KD (0.0225f * 1.1f)		// Kd = Kp * Td = 0.075 ; Td = 0.125 * PU = 0.125

#else
#endif


#if 1
#define LIGHT_LPF_HZ (LPF_FS * 0.25f)		// カットオフ周期を制御周期の4倍程度にしておく
#define TAILRUN_FORWARD_SPEED (100)

// KU=0.30, PU=1.00 ※幅調整実装前の振動測定
#define TAILRUN_DIRECTION_KP (0.18f * 1.1f)		// Kp = 0.6 * KU = 0.18
#define TAILRUN_DIRECTION_KI (0.36f * 1.1f)		// Ki = Kp / Ti = 1.20 ; Ti = 0.5 * PU = 0.50
#define TAILRUN_DIRECTION_KD (0.0225f * 1.1f)		// Kd = Kp * Td = 0.075 ; Td = 0.125 * PU = 0.125

#elif 0
// 2012/09/13 幅調整追加（閾幅のmaxなら+100、minなら-100となるように変更した。ゲイン調整しなおす。）
// 尻尾走行 KU測定
// TAIL_ANGLE_							100
#define LIGHT_LPF_HZ (LPF_FS * 0.25f)		// カットオフ周期を制御周期の4倍程度にしておく
#define TAILRUN_FORWARD_SPEED (100)
// KU=1.00, PU=1.00 ※幅調整実装前の振動測定
// 幅調整実装後は、実装前と同等のゲインを設定する

#define TAILRUN_DIRECTION_KP (0.30f)	// 強い　09/13 22;34 のcsv
//#define TAILRUN_DIRECTION_KP (0.35f)	// 強い
//#define TAILRUN_DIRECTION_KP (0.4f)	// 強い
//#define TAILRUN_DIRECTION_KP (0.6f)	// 強い　22:25
#define TAILRUN_DIRECTION_KI (0.0f)
#define TAILRUN_DIRECTION_KD (0.0f)

/*
#define AFTER_HABA_HOSEI (0.1f)
#define TAILRUN_DIRECTION_KP (0.6f * AFTER_HABA_HOSEI)		// Kp = 0.6 * KU = 0.60
#define TAILRUN_DIRECTION_KI (1.2f * AFTER_HABA_HOSEI)		// Ki = Kp / Ti = 1.20 ; Ti = 0.5 * PU = 0.50
#define TAILRUN_DIRECTION_KD (0.075f * AFTER_HABA_HOSEI)		// Kd = Kp * Td = 0.075 ; Td = 0.125 * PU = 0.125
*/

#endif
