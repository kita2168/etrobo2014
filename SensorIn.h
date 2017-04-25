/* 2012/08/09 created by s.sahara 難所クリア単体テスト用 */

/* センサ入力関連処理 */

//#define LPF_FS (250.0f) // 1/0.004 = 250   ...first
#define LPF_FS (1000.0f) // 1/0.001 = 1000   ...first
#define LPF_Q (1.0f)
//#define LPF_Q (0.707f) // 安定するまで右に行く？
#define MID_LPF_HZ (LPF_FS * 0.0078125f)			// 1/128
#define SLOW_LPF_HZ (LPF_FS * 0.0078125f * 0.25f)	// 1/4

extern void LightIn();
#if 0
extern U16 GetLightIn3();
extern U16 GetLightIn10();
extern U16 GetLightIn20();
#endif
extern U16 GetLightIn();
extern U16 GetLightInLPF();
extern U16 md_mid_GetLightInLPF();
extern U16 md_slow_GetLightInLPF();

extern void InitGyroDetect(float threshold);
extern int GyroDetect();

extern void InitLPF(float cutOffHz, float* omega0, float* a, float* b0, float* b1, float* b2, float* a1, float* a2);
extern S16 LPF(S16 inp, float omega0, float a, float b0, float b1, float b2, float a1, float a2, float* pre1, float* pre2);
