//実装予定
//優先度の高いやつから
//1．コースログの取り方を見直す（現状，距離が正確に計測できていない）
//2．曲率ごとに速度を変える
//3．ジャイロを使用した制御
//4．ショートカット

#include <machine.h>
#include "iodefine.h"

#define FLD_SUCCESS 1 
#define FLD_ERROR 2
#define FLD_BLANK 3
#define FLD_OK 4
#define FLD_TMOUT 5
#define FLD_NOBLANK 6
#define BUZZ_permit 1
#define ON 1
#define OFF 0 
#define SW_ON 0
#define SW_OFF 1
#define corner 6
#define corner2 7
#define goal 8
#define START_A 1
#define STOP 0
#define ST 1
#define curve 0
#define Tred 123
#define R 100.0
//#define mmpp 0.00934524
//#define cmpp 0.000934524
#define mmpp 0.0086019
#define cmpp 0.00086019
#define mmps 8.6019
#define max_thre 400.0
#define length_P_25 18189
#define length_P_10 7576
#define length_25 250
#define length_10 100
#define PWM_MAX 1499.0
#define R10 0
#define R15 1
#define R20 2
#define R25 3
#define R30 4
#define R35 5
#define R40 6
#define R45 7
#define R50 8
#define R55 9
#define R60 10
#define R65 11
#define R70 12
#define R75 13
#define R80 14
#define R85 15
#define R90 16
#define R95 17

/**********ポートマクロ***********/
#define buzz PORTB.DR.BIT.B7
#define SW PORTE.PORT.BIT.B0
#define LED1 PORTD.DR.BIT.B1	//●○○○ 
#define LED2 PORTD.DR.BIT.B2	//○●○○
#define LED3 PORT9.DR.BIT.B2	//○○●○
#define LED4 PORT9.DR.BIT.B3	//○○○●
#define V_LED1  PORT8.DR.BIT.B0	//    12345
#define V_LED2  PORT7.DR.BIT.B4	// 67        8
#define V_LED3  PORT7.DR.BIT.B6	//	   9 10
#define V_LED4  PORT8.DR.BIT.B1  
#define V_LED5  PORT7.DR.BIT.B5
#define V_LED6  PORT1.DR.BIT.B0
#define V_LED7  PORT8.DR.BIT.B2
#define V_LED8  PORT9.DR.BIT.B1
#define V_LED9  PORT1.DR.BIT.B1
#define V_LED10  PORT9.DR.BIT.B0
#define MOT_ctr_L1 PORTA.DR.BIT.B0
#define	MOT_ctr_L2 PORTA.DR.BIT.B1
#define MOT_ctr_R1 PORTA.DR.BIT.B2
#define	MOT_ctr_R2 PORTA.DR.BIT.B3
#define MOT_STBY   PORTA.DR.BIT.B4		
#define ENC_R MTU1.TCNT
#define ENC_L MTU2.TCNT

/***************カウンタ***************/
unsigned long TimeCount=0;
unsigned long micro_time_count=0;
unsigned short TimeNow=0;
unsigned short StartTimeCount=0;
unsigned short CrossTimeCount=0;
unsigned short stop_count=0;
unsigned short curve_count=0;
		 short circl_count=1;
		 float AccelTimeCount=0.0;
		 char  goal_count=0;
		 char  Total_goal_count=0;
		 short CornerTimeCount=0;
		 char shortcut_area=0;
/***************床センサ***************/
short SEN_OFF[11];
short SEN[11];
short cari_SEN[11];
short sum_SEN[11];
short MAX_SEN[11];
short MIN_SEN[11];
short ave_SEN[11];
short SEN_min[11];
long SEN_max[11];
short SEN_thre[11];
long SEN_minmax[11];
short ave_count=0;
float ERR=0;
float sub_ERR=0;
float PRE_ERR=0;
float PRE_senc=0;
short thre_max=350;	//300
float sens_dis[6]={0.0, 480.0, 300.0, 0.0 , -300.0, -480.0};
char  pre_max_number=3;
short max_ERR=300;
short min_ERR=-300;
float now_dis=0.0;
char sens_check_count=0;
short sens_check=0;

/***************ジャイロ***************/
float GYRO_ERR=0.0;
short zero_gyro=0;
short sum_gyro=0;
short ave_gyro=0;
short curve_check=0;
short gyro_data[200];
long gyro_data_sum=0;
long data_count=0;
short MAX_gyro=0;
short MIN_gyro=32000;
char gyro_count=0;
short lowpass_window[5];
short lowpass_gyro=0;
short lowpass_gyro_sum=0;
float Desire_gyro=0;

/***************エンコーダ*************/
long now_ENC_R=0;
long now_ENC_L=0;
long NowEnc=0;
long SUM_ENC=0;
long curvature=0;
long ENC_dis[200];
short course_data[200];
short ENC_count=0;
short now_curvature=0;
char now_comp=0;
char continue_st[200];
char shortcut_frag=0;


/***************PIDゲイン**************/
float K_P=1.0;
float K_P_sub=1.0;
float K_D=1.0;
float K_I=1.0;
float K_G=1.0;
float first_K_P=1.0;
float first_K_D=1.0;
float first_K_G=0.0;
float K_P_SPEED=0.05;
float K_I_SPEED=0.025;
float speed=130.0;
float now_vel_R=0.0;
float now_vel_L=0.0;
float now_vel=0.0;
float Desire_vel=0.0;
float Desire_vel_nomal=0.0;
float Accel=0.0;
float SPEED_ERR=0.0;
float SPEED_SUM_ERR=0.0;
float speed_para[3];
float accel_dis[3];
float first_speed=0.0;
float curve_speed_para[18];
float Desire_next_speed=0.0;

/***************フラグ*****************/
char goal_frag1=0;
char goal_frag2=0;
char first_check=0;
char stop_frag=0;
char start0=0;
char mot_frag=0;
char corner_frag=0;
char cari_frag=0;
char max_frag=0;
char min_frag=0;
char get_sens_frag=0;
short EG_stop_count=0;
char  circle_number=1;
char debug_mode_flag=0;
char gyro_frag=0;
char ch_frag=0;
char fail_safe_frag=0;

/*************プロトタイプ宣言**********/
void set();
void wait();
short get_Batt(); 
short get_gyro();
void get_senc();
int calc_senc();
void mot_STB(int start_stop);
void search_max();
void stop_manager();
void goal_check(long RIGHT_ENC, long LEFT_ENC, short err);
void TIMER_CALL();
void TIMER_WAIT(unsigned short ms);
void mot_drive(float mot_in_p, float mot_in_p_sub, float mot_in_d, float mot_in_g, float mot_in_speed, float mot_in_sum_speed, float SP);
void Battery_Check();
void countdown();
void trace();
void out_senc();
void ch_mode(int mode);
void check_curve();
void mot_brake();	
void calc_accel_dis(float Initial_velocity);

void BUZZ_ON()
{	
	short i=0;
	short buzz_hz=500;

#ifdef BUZZ_permit	
	for(i=0; i<25; i++){
		buzz=ON;
		_wait_usec_(buzz_hz);
		buzz=OFF;
		_wait_usec_(buzz_hz);
		if(i%2==0)
			buzz_hz/=2;
		else{
			buzz_hz*=2;
		}
	}
#endif	
}

void BUZZ_ON2()
{	
	short i=0;
	short buzz_hz=500;

#ifdef BUZZ_permit	
	for(i=0; i<10; i++){
		buzz=ON;
		_wait_usec_(buzz_hz);
		buzz=OFF;
		_wait_usec_(buzz_hz);
		if(i%2==0)
			buzz_hz/=2;
		else{
			buzz_hz*=2;
		}
	}
#endif	
}

void LED_manager(short led)
{
	switch(led){
		case 1:	LED1=ON; LED2=OFF; LED3=OFF; LED4=OFF;	break;
		case 2:	LED1=OFF; LED2=ON; LED3=OFF; LED4=OFF;	break;
		case 3:	LED1=ON; LED2=ON; LED3=OFF;	 LED4=OFF;	break;
		case 4:	LED1=OFF; LED2=OFF; LED3=ON; LED4=OFF;	break;
		case 5:	LED1=ON; LED2=OFF; LED3=ON;	 LED4=OFF;	break;
		case 6:	LED1=OFF; LED2=ON; LED3=ON;	 LED4=OFF;	break;
		case 7:	LED1=ON; LED2=ON; LED3=ON;	 LED4=OFF;	break;
		case 8:	LED1=OFF; LED2=OFF; LED3=OFF; LED4=ON;	break;
		case 9:	LED1=ON; LED2=OFF; LED3=OFF; LED4=ON;	break;
		case 10:LED1=OFF; LED2=ON; LED3=OFF; LED4=ON;	break;
		case 11:LED1=ON; LED2=ON; LED3=OFF;	 LED4=ON;	break;
		case 12:LED1=OFF; LED2=OFF; LED3=ON; LED4=ON;	break;
		case 13:LED1=ON; LED2=OFF; LED3=ON;	 LED4=ON;	break;
		case 14:LED1=OFF; LED2=ON; LED3=ON;	 LED4=ON;	break;
		case 15:LED1=ON; LED2=ON; LED3=ON;	 LED4=ON;	break;

	}
}

void set()	//各種機能初期設定
{
	PORTD.DDR.BYTE = 0x06;
	PORTA.DDR.BYTE = 0x1f;
	PORTB.DDR.BYTE = 0xf0;

	PORT7.DDR.BYTE = 0x70;
	PORT8.DDR.BYTE = 0x07;
	PORT9.DDR.BYTE = 0x0f;
	PORTE.DDR.BYTE = 0x00;
	PORT1.DDR.BYTE = 0x03;
	
	SYSTEM.SCKCR.BIT.PCK=1;		//モジュールクロックの変更　12M Hz を4逓倍
	SYSTEM.SCKCR.BIT.ICK=0;		//システムクロックの変更　12M Hz を8逓倍
	init_sci1();
	
	MSTP_CMT0=0;				//CMT モジュールスタンバイ解除
    CMT0.CMCR.BIT.CMIE = 1; 	//CMTの割り込み許可
    CMT0.CMCR.BIT.CKS = 0;  	//φ／8 = 6 MHz
    CMT0.CMCOR = 6000;       	//1ms周期で割り込み
	ICU.IER[0x03].BIT.IEN4=1;	//割り込み要求許可
	ICU.IPR[0x04].BIT.IPR=10;	//割り込み優先レベル変更
    CMT.CMSTR0.BIT.STR0 = 1; 	//CMTカウントスタート
		
	MSTP_CMT1=0;				//CMT モジュールスタンバイ解除
    CMT1.CMCR.BIT.CMIE = 1; 	//CMTの割り込み許可
    CMT1.CMCR.BIT.CKS = 0;  	//φ／8 = 6 MHz
    CMT1.CMCOR = 3000;       	//0.5ms周期で割り込み
	ICU.IER[0x03].BIT.IEN5=1;	//割り込み要求許可
	ICU.IPR[0x05].BIT.IPR=15;	//割り込み優先レベル変更
    CMT.CMSTR0.BIT.STR1 = 1; 	//CMTカウントスタート
	
	MSTP_AD0=0;					//10bitAD変換 モジュールストップ状態の解除
  	AD0.ADCSR.BIT.ADST  = 0;	// AD変換停止
  	AD0.ADCR.BIT.MODE   = 0;	// AD変換モード選択 シングルモード
  	AD0.ADCSR.BIT.ADIE  = 0;	// 割込み禁止
  	AD0.ADCR.BIT.CKS    = 0;	// 周辺動作クロック 48MHz
  	AD0.ADDPR.BIT.DPPRC = 0;	//10bitで変換
	AD0.ADDPR.BIT.DPSEL = 1;	//MSB詰め
	
	SYSTEM.MSTPCRA.BIT.MSTPA16=0;	//12ビットA/D変換モジュールユニット1の使用を許可
	SYSTEM.MSTPCRA.BIT.MSTPA17=0;	//12ビットA/D変換モジュールユニット0の使用を許可
	SYSTEM.MSTPCRA.BIT.MSTPA24=0;	//12ビットA/D変換モジュールユニット制御部の使用を許可
	S12AD0.ADCSR.BIT.ADST=0;		//A/D変換停止
	S12AD1.ADCSR.BIT.ADST=0;		//A/D変換停止
	S12AD0.ADCER.BIT.SHBYP=1;		//サンプル&ホールド回路を使用しない
	S12AD1.ADCER.BIT.SHBYP=1;		//サンプル&ホールド回路を使用しない
	S12AD0.ADANS.BIT.CH=2;			//AN002をA/D変換
	S12AD1.ADANS.BIT.CH=3;			//AN103をA/D変換
	S12AD0.ADCSR.BIT.CKS=3;			//変換クロックを96MHzに設定
	S12AD1.ADCSR.BIT.CKS=3;			//変換クロックを96MHzに設定
	S12AD0.ADCSR.BIT.ADCS=0;		//シングルモード
	S12AD1.ADCSR.BIT.ADCS=0;		//シングルモード
	S12AD1.ADCSR.BIT.ADST=1;		//A/D変換開始
	while(S12AD1.ADCSR.BIT.ADST==1);//A/D変換が終わるまで待つ	
	
	SYSTEM.MSTPCRA.BIT.MSTPA9=0;
	MTU.TSTRA.BIT.CST0=0;		//カウント停止
	MTU0.TCR.BIT.CCLR=2;
	MTU0.TCR.BIT.TPSC=0;		//PWM周期を96kHzに設定
	MTU0.TMDR.BIT.MD=3;
	MTU0.TIORH.BIT.IOA=5;
	MTU0.TIORL.BIT.IOC=5;		
	MTU0.TGRB=1499;
	MTU0.TGRA=0;				//コンペアマッチの値を0に設定
	MTU0.TGRC=0;				
	MTU.TSTRA.BIT.CST0=1;		//カウントスタート

	PORT3.ICR.BYTE = 0x0f;
	MTU.TSTRA.BIT.CST1=0;		//カウント停止
	MTU.TSTRA.BIT.CST2=0;	
	MTU1.TMDR.BIT.MD=4;			//位相計数モードに設定
	MTU2.TMDR.BIT.MD=4;
	MTU.TSTRA.BIT.CST1=1;		//カウント開始
	MTU.TSTRA.BIT.CST2=1;	
	
	fld_init_fcu_ram();
	(void)fld_init_pclk_notification(15);		//マップ用
	(void)fld_init_pclk_notification(14);		//ログ用
	(void)fld_init_pclk_notification(13);		//ログ用
	(void)fld_init_pclk_notification(12);		//ログ用
	(void)fld_init_pclk_notification(11);		//ログ用
	(void)fld_init_pclk_notification(0);		//パラメータ用
	(void)fld_init_pclk_notification(1);		//パラメータ用
	(void)fld_init_pclk_notification(2);		//パラメータ用
	(void)fld_init_pclk_notification(3);		//パラメータ用
	(void)fld_init_pclk_notification(4);		//パラメータ用
	(void)fld_init_pclk_notification(5);		//パラメータ用
	(void)fld_init_pclk_notification(6);		//センサ値保持用
}

void reset()
{
	LED1=OFF;
	LED2=OFF;
	LED3=OFF;
}
	
void wait()
{
	int i,j;
	for(i=0; i<10000; i++){
		for(j=0; j<10000; j++){}
	}
}

short get_gyro()	//ジャイロ値取得
{
	short ad_gyro;
	
	S12AD1.ADCSR.BIT.ADST=0;			//A/D変換停止		
	S12AD1.ADANS.BIT.CH=1;				//AN101をA/D変換
	S12AD1.ADCSR.BIT.ADST=1;			//A/D変換開始
	while(S12AD1.ADCSR.BIT.ADST==1);	//A/D変換が終わるまで待つ
	ad_gyro=S12AD1.ADDR1-zero_gyro;					//LEDを消した状態での値を格納
	
	return ad_gyro;
}

void get_senc()
{
	short count=0;
	
	V_LED1=OFF;
	V_LED2=OFF;
	V_LED3=OFF;
	V_LED4=OFF;
	V_LED5=OFF;
	V_LED6=OFF;
	V_LED7=OFF;
	V_LED8=OFF;
	V_LED9=OFF;
	V_LED10=OFF;

//	for(count=0; count<201; count++){}
	S12AD0.ADCSR.BIT.ADST=0;			//A/D変換停止
	S12AD0.ADANS.BIT.CH=1;				//AN001をA/D変換
	S12AD0.ADCSR.BIT.ADST=1;			//A/D変換開始
	while(S12AD0.ADCSR.BIT.ADST==1);	//A/D変換が終わるまで待つ
	SEN_OFF[1]=S12AD0.ADDR1;			
	
	S12AD0.ADCSR.BIT.ADST=0;			//A/D変換停止
	S12AD0.ADANS.BIT.CH=2;				//AN002をA/D変換
	S12AD0.ADCSR.BIT.ADST=1;			//A/D変換開始
	while(S12AD0.ADCSR.BIT.ADST==1);	//A/D変換が終わるまで待つ
	SEN_OFF[2]=S12AD0.ADDR2;			

	S12AD1.ADCSR.BIT.ADST=0;			//A/D変換停止
	S12AD1.ADANS.BIT.CH=0;				//AN100をA/D変換
	S12AD1.ADCSR.BIT.ADST=1;			//A/D変換開始
	while(S12AD1.ADCSR.BIT.ADST==1);	//A/D変換が終わるまで待つ
	SEN_OFF[3]=S12AD1.ADDR0A;			

	S12AD1.ADCSR.BIT.ADST=0;			//A/D変換停止
	S12AD1.ADANS.BIT.CH=3;				//AN103をA/D変換
	S12AD1.ADCSR.BIT.ADST=1;			//A/D変換開始
	while(S12AD1.ADCSR.BIT.ADST==1);	//A/D変換が終わるまで待つ
	SEN_OFF[4]=S12AD1.ADDR3;			

	S12AD0.ADCSR.BIT.ADST=0;			//A/D変換停止
	S12AD0.ADANS.BIT.CH=3;				//AN003をA/D変換
	S12AD0.ADCSR.BIT.ADST=1;			//A/D変換開始
	while(S12AD0.ADCSR.BIT.ADST==1);	//A/D変換が終わるまで待つ
	SEN_OFF[5]=S12AD0.ADDR3;			

	AD0.ADCSR.BIT.CH=8;				//AD変換するアナログ入力チャンネルを設定  AD0.ADDRI
	AD0.ADCSR.BIT.ADST = 1;			// AD変換開始
	while(AD0.ADCSR.BIT.ADST==1);	//AD変換終了まで待機
	AD0.ADCSR.BIT.ADST  = 0;		// AD停止
	SEN_OFF[6]=AD0.ADDRI;

	AD0.ADCSR.BIT.CH=7;				//AD変換するアナログ入力チャンネルを設定  AD0.ADDRI
	AD0.ADCSR.BIT.ADST = 1;			// AD変換開始
	while(AD0.ADCSR.BIT.ADST==1);	//AD変換終了まで待機
	AD0.ADCSR.BIT.ADST  = 0;		// AD停止
	SEN_OFF[7]=AD0.ADDRH;

	AD0.ADCSR.BIT.CH=5;				//AD変換するアナログ入力チャンネルを設定  AD0.ADDRI
	AD0.ADCSR.BIT.ADST = 1;			// AD変換開始
	while(AD0.ADCSR.BIT.ADST==1);	//AD変換終了まで待機
	AD0.ADCSR.BIT.ADST  = 0;		// AD停止
	SEN_OFF[8]=AD0.ADDRF;

//	S12AD0.ADCSR.BIT.ADST=0;			//A/D変換停止
//	S12AD0.ADANS.BIT.CH=0;				//AN000をA/D変換
//	S12AD0.ADCSR.BIT.ADST=1;			//A/D変換開始
//	while(S12AD0.ADCSR.BIT.ADST==1);	//A/D変換が終わるまで待つ
//	SEN_OFF[9]=S12AD0.ADDR0A;			
//
//	S12AD1.ADCSR.BIT.ADST=0;			//A/D変換停止
//	S12AD1.ADANS.BIT.CH=2;				//AN102をA/D変換
//	S12AD1.ADCSR.BIT.ADST=1;			//A/D変換開始
//	while(S12AD1.ADCSR.BIT.ADST==1);	//A/D変換が終わるまで待つ
//	SEN_OFF[10]=S12AD1.ADDR2;			

/***************************************************************************/

	V_LED1=ON;		
	for(count=0; count<201; count++){}	//LEDが点灯しきるまで少し待機
	S12AD0.ADCSR.BIT.ADST=0;			//A/D変換停止
	S12AD0.ADANS.BIT.CH=1;				//AN001をA/D変換
	S12AD0.ADCSR.BIT.ADST=1;			//A/D変換開始
	while(S12AD0.ADCSR.BIT.ADST==1);	//A/D変換が終わるまで待つ
	SEN[1]=S12AD0.ADDR1-SEN_OFF[1];			
	V_LED1=OFF;		
	V_LED2=ON;		
	for(count=0; count<201; count++){}

	S12AD0.ADCSR.BIT.ADST=0;			//A/D変換停止
	S12AD0.ADANS.BIT.CH=2;				//AN002をA/D変換
	S12AD0.ADCSR.BIT.ADST=1;			//A/D変換開始
	while(S12AD0.ADCSR.BIT.ADST==1);	//A/D変換が終わるまで待つ
	SEN[2]=S12AD0.ADDR2-SEN_OFF[2];			
	V_LED2=OFF;		
	V_LED3=ON;		
	for(count=0; count<201; count++){}

	S12AD1.ADCSR.BIT.ADST=0;			//A/D変換停止
	S12AD1.ADANS.BIT.CH=0;				//AN100をA/D変換
	S12AD1.ADCSR.BIT.ADST=1;			//A/D変換開始
	while(S12AD1.ADCSR.BIT.ADST==1);	//A/D変換が終わるまで待つ
	SEN[3]=S12AD1.ADDR0A-SEN_OFF[3];			
	V_LED3=OFF;		
	V_LED4=ON;		
	for(count=0; count<201; count++){}

	S12AD1.ADCSR.BIT.ADST=0;			//A/D変換停止
	S12AD1.ADANS.BIT.CH=3;				//AN103をA/D変換
	S12AD1.ADCSR.BIT.ADST=1;			//A/D変換開始
	while(S12AD1.ADCSR.BIT.ADST==1);	//A/D変換が終わるまで待つ
	SEN[4]=S12AD1.ADDR3-SEN_OFF[4];			
	V_LED4=OFF;		
	V_LED5=ON;		
	for(count=0; count<201; count++){}

	S12AD0.ADCSR.BIT.ADST=0;			//A/D変換停止
	S12AD0.ADANS.BIT.CH=3;				//AN003をA/D変換
	S12AD0.ADCSR.BIT.ADST=1;			//A/D変換開始
	while(S12AD0.ADCSR.BIT.ADST==1);	//A/D変換が終わるまで待つ
	SEN[5]=S12AD0.ADDR3-SEN_OFF[5];			
	V_LED5=OFF;		
	V_LED6=ON;		
	for(count=0; count<201; count++){}

	AD0.ADCSR.BIT.CH=8;				//AD変換するアナログ入力チャンネルを設定  AD0.ADDRI
	AD0.ADCSR.BIT.ADST = 1;			// AD変換開始
	while(AD0.ADCSR.BIT.ADST==1);	//AD変換終了まで待機
	AD0.ADCSR.BIT.ADST  = 0;		// AD停止
	SEN[6]=AD0.ADDRI-SEN_OFF[6];
	V_LED6=OFF;		
	V_LED7=ON;		
	for(count=0; count<201; count++){}

	AD0.ADCSR.BIT.CH=7;				//AD変換するアナログ入力チャンネルを設定  AD0.ADDRI
	AD0.ADCSR.BIT.ADST = 1;			// AD変換開始
	while(AD0.ADCSR.BIT.ADST==1);	//AD変換終了まで待機
	AD0.ADCSR.BIT.ADST  = 0;		// AD停止
	SEN[7]=AD0.ADDRH-SEN_OFF[7];
	V_LED7=OFF;		
	V_LED8=ON;		
	for(count=0; count<201; count++){}

	AD0.ADCSR.BIT.CH=5;				//AD変換するアナログ入力チャンネルを設定  AD0.ADDRI
	AD0.ADCSR.BIT.ADST = 1;			// AD変換開始
	while(AD0.ADCSR.BIT.ADST==1);	//AD変換終了まで待機
	AD0.ADCSR.BIT.ADST  = 0;		// AD停止
	SEN[8]=AD0.ADDRF-SEN_OFF[8];
	V_LED8=OFF;		

//	V_LED9=ON;		
//	S12AD0.ADCSR.BIT.ADST=0;			//A/D変換停止
//	S12AD0.ADANS.BIT.CH=0;				//AN000をA/D変換
//	S12AD0.ADCSR.BIT.ADST=1;			//A/D変換開始
//	while(S12AD0.ADCSR.BIT.ADST==1);	//A/D変換が終わるまで待つ
//	SEN[9]=S12AD0.ADDR0A-SEN_OFF[9];			
//	V_LED9=OFF;		
//	for(count=0; count<201; count++){}
//
//	V_LED10=ON;		
//	S12AD1.ADCSR.BIT.ADST=0;			//A/D変換停止
//	S12AD1.ADANS.BIT.CH=2;				//AN102をA/D変換
//	S12AD1.ADCSR.BIT.ADST=1;			//A/D変換開始
//	while(S12AD1.ADCSR.BIT.ADST==1);	//A/D変換が終わるまで待つ
//	SEN[10]=S12AD1.ADDR2-SEN_OFF[10];			
//	V_LED10=OFF;		
//	for(count=0; count<201; count++){}
}

void calc_gyro()
{
	short tmp,i=0;
	
	tmp=get_gyro();
	sum_gyro+=tmp;
	if(tmp > MAX_gyro)	MAX_gyro=tmp;
	if(tmp < MIN_gyro)	MIN_gyro=tmp;
	gyro_count++;
	
	if(gyro_count==5){
		sum_gyro=sum_gyro-MAX_gyro-MIN_gyro;
		ave_gyro=sum_gyro/=3;
		
//		lowpass_gyro_sum-=lowpass_window[0];
//		for(i=0; i<4; i++)
//			lowpass_window[i]=lowpass_window[i+1];
//		lowpass_window[4]=ave_gyro;
//		lowpass_gyro_sum+=lowpass_window[4];
//		lowpass_gyro=lowpass_gyro_sum/5;
		
		sum_gyro=0;
		gyro_count=0;
		MAX_gyro=0;
		MIN_gyro=32000;
		gyro_data_sum+=ave_gyro;	//区間ごとの平均
//		gyro_data_sum+=lowpass_gyro;	//区間ごとの平均
//		data_count++;
	}
}

int calc_senc()
{
	short i;
		
	get_senc();
	for(i=1; i<6; i++){
		if(SEN[i]<0)
			SEN[i]*=(-1);
			
		ave_SEN[i] = 400*(SEN[i]-SEN_min[i])/SEN_minmax[i];	
	}

	for(i=6; i<9; i++){
		if(SEN[i]<0)
			SEN[i]*=(-1);
			
		cari_SEN[i] = 400*(SEN[i]-SEN_min[i])/SEN_minmax[i];	
		sum_SEN[i] += cari_SEN[i];
		
		if(MAX_SEN[i] < cari_SEN[i])
			MAX_SEN[i] = cari_SEN[i];
		if(MIN_SEN[i] > cari_SEN[i])
			MIN_SEN[i] = cari_SEN[i];
	}

	ave_count++;
	if(ave_count==5){	
		for(i=6; i<9; i++){
			ave_SEN[i]=(sum_SEN[i]-MAX_SEN[i]-MIN_SEN[i])/3;
			sum_SEN[i]=0;
			MAX_SEN[i]=0;
			MIN_SEN[i]=ave_SEN[i]*2;
		}		
		ave_count=0;
	}	
	return 0;
}

void calc_mot_in_senc()
{
		short max;
		short max_number=1;
		short max_number_err=0;
		short i=0;
		
		max=ave_SEN[1];
		PRE_senc=ERR;
		
		for(i=2; i<6; i++){			
			if(max < ave_SEN[i]){
				max=ave_SEN[i];
				max_number=i;
			}
		}	
		
		max_number_err=max_number-pre_max_number;
		if(max_number_err > 1)
			max_number=max_number-(max_number_err-1);
		if(max_number_err < -1)
			max_number=max_number-(max_number_err+1);

		if((ave_SEN[1] > thre_max) && (ave_SEN[2] < thre_max && ave_SEN[3] < thre_max && ave_SEN[4] < thre_max)){
			max_frag=1;
		}
		if(ave_SEN[2] > thre_max || ave_SEN[3] > thre_max || ave_SEN[4] > thre_max){
			max_frag=0;
			LED2=OFF;
		}
		
		if((ave_SEN[2] < thre_max && ave_SEN[3] < thre_max && ave_SEN[4] < thre_max) && (ave_SEN[5] > thre_max)){
			min_frag=1;
		}
		if(ave_SEN[2] > thre_max || ave_SEN[3] > thre_max || ave_SEN[4] > thre_max){
			min_frag=0;
			LED2=OFF;
		}

		if(max_number==2){
			if(ave_SEN[1] > ave_SEN[3]){
				now_dis=((sens_dis[1]*ave_SEN[1])+(sens_dis[2]*ave_SEN[2]))/(ave_SEN[1]+ave_SEN[2]);
				ERR=now_dis;
			}
			else{
				now_dis=(float)(ave_SEN[2]-ave_SEN[4]);	
				ERR=now_dis;
			}	
		}

		if(max_number==3){
			now_dis=(float)(ave_SEN[2]-ave_SEN[4]);	
			ERR=now_dis;
		}

		if(max_number==4){
			if(ave_SEN[3] > ave_SEN[5]){
				now_dis=(float)(ave_SEN[2]-ave_SEN[4]);	
				ERR=now_dis;
			}
			else{
				now_dis=((sens_dis[5]*ave_SEN[5])+(sens_dis[4]*ave_SEN[4]))/(ave_SEN[5]+ave_SEN[4]);
				ERR=now_dis;
			}	
		}
		if(max_number==1){
			now_dis=((sens_dis[1]*ave_SEN[1])+(sens_dis[2]*ave_SEN[2]))/(ave_SEN[1]+ave_SEN[2]);
			ERR=now_dis;
		}
		if(max_number==5){
			now_dis=((sens_dis[5]*ave_SEN[5])+(sens_dis[4]*ave_SEN[4]))/(ave_SEN[5]+ave_SEN[4]);
			ERR=now_dis;
		}
			
		if(max_frag==1){
			LED2=ON;
			now_dis=450.0;
			ERR=now_dis;
		}

		if(min_frag==1){
			LED2=ON;
			now_dis= (-450.0);					
			ERR=now_dis;
		}
		pre_max_number=max_number;
		PRE_ERR=ERR-PRE_senc;
}

void mot_STB(int start_stop)
{
	if(start_stop==START_A)
		MOT_STBY=1;
	else{
		MOT_STBY=0;
	}
}

void search_max()
{
	int i;
	short loop_frag=0;
	
	while(loop_frag==0){
		if(SW == SW_ON)
			loop_frag=1;
		else{}
	}
	TIMER_WAIT(1000);
	LED1=OFF;
	ENC_R=0;
	ENC_L=0;
	mot_STB(START_A);
	LED3=ON;
	
	for(i=1; i<11; i++){
		SEN_max[i]=1;
		SEN_min[i]=10;
	}
	cari_frag=1;
	while((float)ENC_R*mmpp < 150.0){
		MOT_ctr_R1=1;
		MOT_ctr_R2=0;
		MOT_ctr_L1=0;
		MOT_ctr_L2=1;
		MTU0.TGRC=150 - (0.1*(ENC_R-ENC_L));
		MTU0.TGRA=150 + (0.1*(ENC_R-ENC_L));
		if((float)ENC_R*mmpp > 20.0){
			for(i=1; i<11; i++){
				if(SEN_max[i] < SEN[i])
					SEN_max[i]=SEN[i];
				if(SEN[i] > 0 && SEN_min[i] > SEN[i])
					SEN_min[i]=SEN[i];
			}	
		}
	}
	cari_frag=0;
	mot_brake();
	TIMER_WAIT(200);
	ENC_R=40000;
	ENC_L=40000;
	
	MOT_ctr_R1=0;
	MOT_ctr_R2=1;
	MOT_ctr_L1=1;
	MOT_ctr_L2=0;
	cari_frag=1;
	while((float)(40000-ENC_R)*mmpp < 150.0){
		MTU0.TGRC=150 - (0.001*(ENC_R-ENC_L));
		MTU0.TGRA=150 + (0.001*(ENC_R-ENC_L));
		if((float)(40000-ENC_R)*mmpp > 20.0){
			for(i=1; i<11; i++){
				if(SEN_max[i] < SEN[i])
					SEN_max[i]=SEN[i];
				if(SEN[i] > 0 && SEN_min[i] > SEN[i])
					SEN_min[i]=SEN[i];
			}	
		}
	}
	
	cari_frag=0;
	mot_brake();
	ENC_R=0;
	ENC_L=0;
	MTU0.TGRC=0;
	MTU0.TGRA=0;
	TIMER_WAIT(1000);
	LED3=OFF;
	mot_STB(STOP);
	reset();

	for(i=1; i<11; i++)
		SEN_minmax[i]=SEN_max[i]-SEN_min[i];
}

void init_goal_frag()		//ゴール処理のフラグ初期化
{
	stop_count=0;
	reset();
	start0=0;
	stop_frag=0;
	goal_frag1=0;
	goal_frag2=0;
	curve_count=0;
	ENC_count=0;
	first_check=0;
	StartTimeCount=0;
	CrossTimeCount=0;
	AccelTimeCount=0.0;
	CornerTimeCount=0;
	Desire_vel=Desire_vel_nomal;
	Desire_next_speed=Desire_vel_nomal;
	now_vel_R=0.0;
	now_vel_L=0.0;
	SPEED_ERR=0.0;
	SPEED_SUM_ERR=0.0;
	max_frag=0;
	min_frag=0;
	now_comp=0;
	corner_frag=0;
	EG_stop_count=0;
	gyro_frag=0;
	ch_frag=0;
	shortcut_frag=0;
	fail_safe_frag=0;
}

void init_ENC()
{
	ENC_R=15000;
	ENC_L=15000;
}

void STB_MODE()
{
	LED4=ON;
	TIMER_WAIT(100);
	LED4=OFF;
	TIMER_WAIT(100);
}

void mot_brake()		//モータをブレーキ状態に
{
	MOT_ctr_L1=1;
	MOT_ctr_L2=1;
	MOT_ctr_R1=1;
	MOT_ctr_R2=1;
}

void EG_stop()
{
	short i=0;
	
	mot_brake();
	init_goal_frag();	
	LED3=ON;
	for(i=0; i<50; i++)
		BUZZ_ON();
	reset();	
	mot_STB(STOP);

}

void make_shortcut1()
{
	short i=0;
	short j=0;
	short k=0;
	short st_count=1;
	shortcut_area=0;
	
	for(i=0; i<200; i++){
		if(course_data[i]<0) course_data[i]*=(-1);
		continue_st[i]=0;
	}

	for(i=0; i<200; i++){
		if(abs(course_data[i]) > 100){
			for(j=i; j<200; j++){
				if(abs(course_data[j+1]) > 100){
					ENC_dis[i]+=ENC_dis[j+1];
					gyro_data[i]+=gyro_data[j+1];
					st_count++;
				}
				else{
					if(st_count>1){
						gyro_data[i]/=st_count;
						for(k=i; k<200; k++){
							ENC_dis[k+1]=ENC_dis[k+st_count];
							course_data[k+1]=course_data[k+st_count];
							gyro_data[k+1]=gyro_data[k+st_count];
						}
						continue_st[i]=st_count-1;
						st_count=1;	
					}
					break;
				}
			}				
		}
		else{}
	}
}

void make_shortcut2()
{
	short i=0;
	short j=0;
	short k=0;
	short st_count=1;
	shortcut_area=0;
	shortcut_frag=0;
	
	for(i=0; i<200; i++){
		if(course_data[i]<0) course_data[i]*=(-1);
		continue_st[i]=0;
	}

	for(i=0; i<200-4; i++){
		if(course_data[i] > 100 && course_data[i+1] < 100 && course_data[i+2] < 100 && course_data[i+3] < 100 && course_data[i+4] > 100){	//ST1 C3 ST1の検出
			continue_st[i]=1;
			for(j=i; j<200; j++)
				ENC_dis[j]+=(ENC_dis[j+2]+ENC_dis[j+3]+(ENC_dis[j+4]/2));

			for(j=i; j<200; j++){
				ENC_dis[j+1]=ENC_dis[j+5];
				course_data[j+1]=course_data[j+5];
			}
		}
		shortcut_area=0;
	}
	for(i=0; i<200; i++){
		shortcut_area+=(continue_st[i]*4);
	}
}

void stop_manager()		//停止処理
{	
	short i=0;
	 
	while(stop_count!=100){
		LED4=ON;
		mot_drive(ERR, sub_ERR, PRE_ERR, GYRO_ERR, 0.0, 0.0, speed);
	}
	
	for(i=0; i<200; i++){
		if(course_data[i]<0) course_data[i]*=(-1);
	}

	mot_brake();
	init_goal_frag();
	TIMER_WAIT(1000);
	mot_STB(STOP);

}	

void Accel_manager(short counter, short now_R)
{	
	float tmp = ENC_dis[counter]*mmpp;
	
	if((tmp > 2.0*accel_dis[0]) && (tmp < 2.0*accel_dis[1]))
		now_comp=0;

	else if((tmp > 2.0*accel_dis[1]) && (tmp < 2.0*accel_dis[2]))
		now_comp=1;
	
	else if((tmp > 2.0*accel_dis[2]))
		now_comp=2;
}

void goal_check(long RIGHT_ENC, long LEFT_ENC, short err)	//カーブ判定
{
	long distance=(RIGHT_ENC+LEFT_ENC)/2;
	float distance2=(float)distance*mmpp;
	long angl_vel=RIGHT_ENC-LEFT_ENC;
		
	if(first_check==0){
		if(ave_SEN[goal] > SEN_thre[goal] && goal_frag1==0){						//スタート時 白線上
			goal_frag1=1;
			now_ENC_L=0;
			now_ENC_R=0;
			ENC_count=0;
		}
	
		if(goal_frag1==1 && (ave_SEN[goal] < (SEN_thre[goal]))){		//スタート時 白線から外れたとき
			goal_frag2=1;
			first_check=2;
			StartTimeCount=0;
			CrossTimeCount=0;
			BUZZ_ON();
			SPEED_SUM_ERR=0.0;
			goal_count=0;
			first_speed=now_vel;
		}
	}
	if(first_check==2){		
		if((ave_SEN[1] > thre_max) && 	//クロス処理
			(ave_SEN[5] > thre_max) && goal_frag2==1){
			goal_frag2=0;
		}

		if(goal_frag2==0){
			CrossTimeCount++;
		}
			
		if(CrossTimeCount==100){	
			goal_frag2=1;
			CrossTimeCount=0;
			gyro_frag=1;
		}
	
	/********************コーナー処理*******************************/	
		if(circl_count==1){			
			if(ave_SEN[corner2] > SEN_thre[corner2] && corner_frag==0 && goal_frag2==1){
				LED3=ON;
				corner_frag=1;
				if(angl_vel == 0)
					angl_vel=1;
				curvature=(RIGHT_ENC+LEFT_ENC)*Tred/(2*angl_vel);	//曲率計算
						
				now_curvature=(short)curvature/10;					
				ENC_dis[ENC_count]=distance;				
				if(ENC_dis[ENC_count]<0)
					ENC_dis[ENC_count]*=(-1);
				
				course_data[ENC_count]=now_curvature;
			//	gyro_data[ENC_count]=SUM_ENC/data_count;
				
				ENC_count++;
				now_ENC_R=0;
				now_ENC_L=0;
				distance2=0.0;
				//SPEED_SUM_ERR=0.0;
				gyro_data_sum=0;
				data_count=0;
			}
						
			if(corner_frag==1){
				buzz=~buzz;
				CornerTimeCount++;
				if(ave_SEN[corner2] < SEN_thre[corner2]){	//チャタリング対策
					LED3=OFF;
					corner_frag=0;
					now_ENC_R=0;
					now_ENC_L=0;
					buzz=OFF;
					CornerTimeCount=0;
				}
			}
		}
		
		if(circl_count==2 && goal_frag1==1 && fail_safe_frag==0){
			if(abs(now_dis-(ENC_dis[ENC_count]*mmpp)) > 100)
				fail_safe_frag=1;
			
			if(corner_frag==0){
				if(course_data[ENC_count] > 100){
					LED1=ON;
					if(ch_frag==0){
						if(ENC_dis[ENC_count+1]*mmpp > 157){
							if(course_data[ENC_count+1] < 15) 										Desire_next_speed=curve_speed_para[R10];
							if(course_data[ENC_count+1] >= 15 && course_data[ENC_count+1] < 20) 	Desire_next_speed=curve_speed_para[R15];
							if(course_data[ENC_count+1] >= 20 && course_data[ENC_count+1] < 25) 	Desire_next_speed=curve_speed_para[R20];
							if(course_data[ENC_count+1] >= 25 && course_data[ENC_count+1] < 30){	Desire_next_speed=curve_speed_para[R25];	LED_manager(10);}
							if(course_data[ENC_count+1] >= 30 && course_data[ENC_count+1] < 35) 	Desire_next_speed=curve_speed_para[R30];
							if(course_data[ENC_count+1] >= 35 && course_data[ENC_count+1] < 40) 	Desire_next_speed=curve_speed_para[R35];
							if(course_data[ENC_count+1] >= 40 && course_data[ENC_count+1] < 45) 	Desire_next_speed=curve_speed_para[R40];
							if(course_data[ENC_count+1] >= 45 && course_data[ENC_count+1] < 50) 	Desire_next_speed=curve_speed_para[R45];
							if(course_data[ENC_count+1] >= 50 && course_data[ENC_count+1] < 55) 	Desire_next_speed=curve_speed_para[R50];
							if(course_data[ENC_count+1] >= 55 && course_data[ENC_count+1] < 60) 	Desire_next_speed=curve_speed_para[R55];
							if(course_data[ENC_count+1] >= 60 && course_data[ENC_count+1] < 65) 	Desire_next_speed=curve_speed_para[R60];
							if(course_data[ENC_count+1] >= 65 && course_data[ENC_count+1] < 70) 	Desire_next_speed=curve_speed_para[R65];
							if(course_data[ENC_count+1] >= 70 && course_data[ENC_count+1] < 75) 	Desire_next_speed=curve_speed_para[R70];
							if(course_data[ENC_count+1] >= 75 && course_data[ENC_count+1] < 80) 	Desire_next_speed=curve_speed_para[R75];
							if(course_data[ENC_count+1] >= 80 && course_data[ENC_count+1] < 85) 	Desire_next_speed=curve_speed_para[R80];
							if(course_data[ENC_count+1] >= 85 && course_data[ENC_count+1] < 90) 	Desire_next_speed=curve_speed_para[R85];
							if(course_data[ENC_count+1] >= 90 && course_data[ENC_count+1] < 95) 	Desire_next_speed=curve_speed_para[R90];
							if(course_data[ENC_count+1] >= 95) 										Desire_next_speed=curve_speed_para[R95];
							else{}
						}
						else{
							Desire_next_speed=curve_speed_para[R10];
						}
						ch_frag=1;
					}
					if(continue_st[ENC_count]==1 && shortcut_frag==1){
						LED2=ON;
						Desire_gyro=0.0;
						GYRO_ERR=Desire_gyro-ave_gyro;	
						
						K_P=0.0;
						K_D=0.0;
						K_G=0.5;
					}
					else if(continue_st[ENC_count]==0 && shortcut_frag==1){}
					
					if(accel_dis[now_comp] > distance2){
						AccelTimeCount+=0.001;
						Desire_vel=first_speed+(Accel*AccelTimeCount);
						if(Desire_vel > speed_para[now_comp])
							Desire_vel=speed_para[now_comp];
					}
					if(course_data[ENC_count+1]==0){
						LED_manager(7);
						AccelTimeCount+=0.001;
						Desire_vel=first_speed+(Accel*AccelTimeCount);
						if(Desire_vel > speed_para[2])
							Desire_vel=speed_para[2];
					}

					if(accel_dis[now_comp] < distance2 && distance2 < (((float)ENC_dis[ENC_count]*mmpp)-(accel_dis[now_comp]))){
						Desire_vel=speed_para[now_comp];
						AccelTimeCount=0.0;
					}
					if(distance2 > (((float)ENC_dis[ENC_count]*mmpp)-(accel_dis[now_comp]))){
						LED2=OFF;
						AccelTimeCount+=0.0015;
						Desire_vel=speed_para[now_comp]-(Accel*AccelTimeCount);
						if(Desire_vel < Desire_next_speed)
							Desire_vel=Desire_next_speed;
//						if(Desire_vel < Desire_vel_nomal)
//							Desire_vel=Desire_vel_nomal;
					}					
				}
				else{						
					if(ch_frag==0){
						if(ENC_dis[ENC_count]*mmpp > 157){
							if(course_data[ENC_count] < 15) 									Desire_next_speed=curve_speed_para[R10];
							if(course_data[ENC_count] >= 15 && course_data[ENC_count] < 20) 	Desire_next_speed=curve_speed_para[R15];
							if(course_data[ENC_count] >= 20 && course_data[ENC_count] < 25) 	Desire_next_speed=curve_speed_para[R20];
							if(course_data[ENC_count] >= 25 && course_data[ENC_count] < 30){	Desire_next_speed=curve_speed_para[R25];	LED_manager(10);}
							if(course_data[ENC_count] >= 30 && course_data[ENC_count] < 35) 	Desire_next_speed=curve_speed_para[R30];
							if(course_data[ENC_count] >= 35 && course_data[ENC_count] < 40) 	Desire_next_speed=curve_speed_para[R35];
							if(course_data[ENC_count] >= 40 && course_data[ENC_count] < 45) 	Desire_next_speed=curve_speed_para[R40];
							if(course_data[ENC_count] >= 45 && course_data[ENC_count] < 50) 	Desire_next_speed=curve_speed_para[R45];
							if(course_data[ENC_count] >= 50 && course_data[ENC_count] < 55) 	Desire_next_speed=curve_speed_para[R50];
							if(course_data[ENC_count] >= 55 && course_data[ENC_count] < 60) 	Desire_next_speed=curve_speed_para[R55];
							if(course_data[ENC_count] >= 60 && course_data[ENC_count] < 65) 	Desire_next_speed=curve_speed_para[R60];
							if(course_data[ENC_count] >= 65 && course_data[ENC_count] < 70) 	Desire_next_speed=curve_speed_para[R65];
							if(course_data[ENC_count] >= 70 && course_data[ENC_count] < 75) 	Desire_next_speed=curve_speed_para[R70];
							if(course_data[ENC_count] >= 75 && course_data[ENC_count] < 80) 	Desire_next_speed=curve_speed_para[R75];
							if(course_data[ENC_count] >= 80 && course_data[ENC_count] < 85) 	Desire_next_speed=curve_speed_para[R80];
							if(course_data[ENC_count] >= 85 && course_data[ENC_count] < 90) 	Desire_next_speed=curve_speed_para[R85];
							if(course_data[ENC_count] >= 90 && course_data[ENC_count] < 95) 	Desire_next_speed=curve_speed_para[R90];
							if(course_data[ENC_count] >= 95) 										Desire_next_speed=curve_speed_para[R95];
							else{}
						}
						else{
							Desire_next_speed=curve_speed_para[R10];
						}
						ch_frag=1;
					}
					Desire_vel=Desire_next_speed;
				}
			}
				
			if(ave_SEN[corner2] > SEN_thre[corner2] && distance2 > ((float)(ENC_dis[ENC_count])*mmpp-50.0) && corner_frag==0 && goal_frag2==1){
				LED3=ON;
				LED1=OFF;
				LED4=OFF;
				corner_frag=1;
				ch_frag=0;
				distance2=0.0;
				Desire_vel=Desire_vel_nomal;
				K_P=first_K_P;
				K_D=first_K_D;
				K_G=first_K_G;

				ENC_count++;
				first_speed=now_vel;
				calc_accel_dis(first_speed);
				Accel_manager(ENC_count, course_data[ENC_count]);
				now_ENC_R=0;
				now_ENC_L=0;
				sub_ERR=0.0;
				K_P_sub=0.0;
			}
			if(corner_frag==1 && ave_SEN[corner2] < SEN_thre[corner2]){	//チャタリング対策
				LED3=OFF;
				corner_frag=0;
				now_ENC_R=0;
				now_ENC_L=0;
			}		
		}

		/*********************ゴール処理*******************/				
		if(circl_count==1){
			if((ave_SEN[goal] > SEN_thre[goal]) && goal_frag2==1){
				goal_count++;
				goal_frag2=0;
			}
			if(goal_count==circle_number){
				angl_vel=RIGHT_ENC-LEFT_ENC;		
				if(angl_vel >= 0){
					if(angl_vel == 0)
						angl_vel=1;
					curvature=(RIGHT_ENC+LEFT_ENC)*Tred/(2*angl_vel);
				}	
				else if(angl_vel < 0){
					angl_vel*=(-1);
					curvature=(RIGHT_ENC+LEFT_ENC)*Tred/(2*angl_vel);
				}
				course_data[ENC_count]=(short)curvature/10;
				ENC_dis[ENC_count]=distance;
				Total_goal_count=ENC_count;
				stop_frag=1;
				goal_count=0;
			}
		}
		if(circl_count==2){
			if((ave_SEN[goal] > SEN_thre[goal]) && goal_frag2==1 && course_data[ENC_count+5] == 0){
				goal_count++;
				goal_frag2=0;
			}
			if(goal_count==circle_number){
				stop_frag=1;
				goal_count=0;
			}
		
		}
		if(circl_count==3){
			EG_stop_count++;
			if(EG_stop_count==5000)
				stop_frag=1;
		}

		/*********セーフティ処理**********/
		if(ave_SEN[1]<150 && ave_SEN[2]<150 && ave_SEN[3]<150 && ave_SEN[4]<150 && ave_SEN[5]<150){
			EG_stop_count++;
		}
		if(EG_stop_count==300 &&  ave_SEN[1]<150 && ave_SEN[2]<150 && ave_SEN[3]<150 && ave_SEN[4]<150 && ave_SEN[5]<150){
			EG_stop();
			stop_frag=1;
		}
		if(EG_stop_count<300 &&  (ave_SEN[1]>150 || ave_SEN[2]>150 || ave_SEN[3]>150 || ave_SEN[4]>150 || ave_SEN[5]>150)){
			EG_stop_count=0;
		}

	}
	else{}
}

void TIMER_CALL()
{
	TimeCount++;
	if(cari_frag==1)
		get_senc();		
	if(get_sens_frag==1){
		calc_senc();
		calc_mot_in_senc();
	}
	
	if(start0==1){				
				
		now_ENC_R+=(ENC_R-15000);
		now_ENC_L+=(ENC_L-15000);
		NowEnc=ENC_R-ENC_L;
		SUM_ENC+=NowEnc;
		data_count++;
		
//		now_vel_R=(float)(ENC_R-15000)*mmps;	// mm/s
//		now_vel_L=(float)(ENC_L-15000)*mmps;	// mm/s			
		now_vel=(ENC_R+ENC_L-30000)*mmps/2.0;
		
		Desire_gyro=(now_dis/400.0)*Desire_vel_nomal*2.0;	//240は調整
		SPEED_ERR=now_vel-Desire_vel;
		SPEED_SUM_ERR+=SPEED_ERR;
		GYRO_ERR=Desire_gyro-(now_vel_R-now_vel_L);	

		calc_senc();					
		calc_mot_in_senc();
		//calc_gyro();
		goal_check(now_ENC_R, now_ENC_L, (short)ERR);
				
		mot_drive(ERR, sub_ERR, PRE_ERR, GYRO_ERR, SPEED_ERR, SPEED_SUM_ERR,speed);
		init_ENC();
	}

	if(stop_frag==1)
		stop_count++;
	
}

void TIMER_CALL2()
{
//	if(gyro_frag==1)
//		calc_gyro();

}

void TIMER_WAIT(unsigned short ms)
{
	unsigned long TimeNow = TimeCount;
	while((TimeCount-TimeNow) <= ms );
}

void mot_drive(float mot_in_p, float mot_in_p_sub, float mot_in_d, float mot_in_g,
				float mot_in_speed, float mot_in_sum_speed, float SP)
{
	float MOT_IN_R=SP;
	float MOT_IN_L=SP;
	short battery=get_Batt();
	
	if(circl_count==3)
		mot_in_p=-350.0;

	MOT_IN_R+=K_P*mot_in_p;
	MOT_IN_R+=K_D*mot_in_d;
//	MOT_IN_R+=K_P_sub*mot_in_p_sub;
	MOT_IN_R+=K_G*mot_in_g;
	MOT_IN_R-=K_P_SPEED*mot_in_speed;
	MOT_IN_R-=K_I_SPEED*mot_in_sum_speed;
	
	MOT_IN_L-=K_P*mot_in_p;
	MOT_IN_L-=K_D*mot_in_d;
//	MOT_IN_L-=K_P_sub*mot_in_p_sub;
	MOT_IN_L-=K_G*mot_in_g;
	MOT_IN_L-=K_P_SPEED*mot_in_speed;
	MOT_IN_L-=K_I_SPEED*mot_in_sum_speed;
	
//	MOT_IN_R=MOT_IN_R*PWM_MAX/get_Batt;
//	MOT_IN_L=MOT_IN_L*PWM_MAX/get_Batt;
	
	if(MOT_IN_R > 0.0){
		MOT_ctr_R1=1;
		MOT_ctr_R2=0;			
		MTU0.TGRC=MOT_IN_R;
	}
	else if(MOT_IN_R < 0.0){

		MOT_IN_R*=(-1.0);
		MOT_ctr_R1=0;
		MOT_ctr_R2=1;
		MTU0.TGRC=MOT_IN_R;
	}

	if(MOT_IN_L > 0.0){	
		MOT_ctr_L1=0;
		MOT_ctr_L2=1;
		MTU0.TGRA=MOT_IN_L;
	}
	else if(MOT_IN_L < 0.0){	
		MOT_IN_L*=(-1.0);
		MOT_ctr_L1=1;
		MOT_ctr_L2=0;
		MTU0.TGRA=MOT_IN_L;
	}
}

short get_Batt()	//バッテリ残量取得
{
	short Battery;
	
	AD0.ADCSR.BIT.CH=6;				//AD変換するアナログ入力チャンネルを設定 バッテリ AD0.ADDRG
	AD0.ADCSR.BIT.ADST = 1;			// AD変換開始
	while(AD0.ADCSR.BIT.ADST==1);	//AD変換終了まで待機
	AD0.ADCSR.BIT.ADST  = 0;		// AD停止
	Battery=AD0.ADDRG>>6;

	return Battery*9765/1000;
}

void Battery_Check()
{
	short battery;
	battery=get_Batt();
	
	if(battery >= 7400){	//mv
		LED_manager(7);
		LED1=ON; LED2=ON; LED3=ON;
	}
	if(battery < 7400 && battery >= 7100){
		LED_manager(3);	
	}
	if(battery < 7100){								//7.4V未満になったら未満強制停止
		LED_manager(4);
		while(1){
			mot_STB(STOP);
		}
	}
	TIMER_WAIT(1500);
	reset();
}

void countdown()
{

	BUZZ_ON();
	reset();
	LED1=ON; LED2=ON;
	TIMER_WAIT(300);
	LED1=OFF; LED2=OFF;
	TIMER_WAIT(300);
	LED1=ON; LED2=ON;
	TIMER_WAIT(300);
	
	LED3=ON;
	TIMER_WAIT(1000);
	reset();
}

short select_mode()
{
	short para_frag=0;
 	short para_count=1;

 	ENC_L=3201;
	
 	while(para_frag==0){
		if(ENC_L > 1600*2 && ENC_L < 1600*3){
			LED_manager(1);
			para_count=1;
		}
		if(ENC_L > 1600*3 && ENC_L < 1600*4){
			LED_manager(2);
			para_count=2;
		}
		if(ENC_L > 1600*4 && ENC_L < 1600*5){
			LED_manager(3);
			para_count=3;
		}
		if(ENC_L > 1600*5 && ENC_L < 1600*6){
			LED_manager(4);
			para_count=4;
		}
		if(ENC_L > 1600*6 && ENC_L < 1600*7){
			LED_manager(5);
			para_count=5;
		}
		if(ENC_L > 1600*7 && ENC_L < 1600*8){
			LED_manager(6);
			para_count=6;
		}
		if(ENC_L > 1600*8 && ENC_L < 1600*9){
			LED_manager(7);
			para_count=7;
		}
		if(ENC_L > 1600*9 && ENC_L < 1600*10){
			LED_manager(8);
			para_count=8;
		}
		if(ENC_L > 1600*10 && ENC_L < 1600*11){
			LED_manager(9);
			para_count=9;
		}
		if(ENC_L > 1600*11 && ENC_L < 1600*12){
			LED_manager(10);
			para_count=10;
		}

		if(ENC_L > 1600*12)
			ENC_L=(1600*2)+1;
			
		if(SW == SW_ON){
			BUZZ_ON();
			TIMER_WAIT(500);
			reset();
			para_frag=1;
		}
	}
	return para_count;
}

void calc_accel_dis(float Initial_velocity)
{	
	accel_dis[0]=((speed_para[0]*speed_para[0])-(Initial_velocity*Initial_velocity))/(2.0*Accel);
	accel_dis[1]=((speed_para[1]*speed_para[1])-(Initial_velocity*Initial_velocity))/(2.0*Accel);
	accel_dis[2]=((speed_para[2]*speed_para[2])-(Initial_velocity*Initial_velocity))/(2.0*Accel);	
}

void ch_para()
{
	switch(select_mode()){
		case 1: speed=500.0;	//220
				K_P=4.0;		//13.5
				K_P_sub=0.0;
				K_D=15.0;		//0.0002
				K_I=0.0;	//12
				K_G=0.0;
				first_K_G=K_G;
				
				Accel=5000.0;
				speed_para[0]=1500.0;
				speed_para[1]=2200.0;
				speed_para[2]=2500.0;
								
				sens_dis[1]=480.0;
				sens_dis[2]=300.0;
				sens_dis[4]=(-300.0);
				sens_dis[5]=(-480.0);

				break;
		
		case 2: speed=500.0;	//220
				K_P=4.0;		//13.5
				K_P_sub=0.0;
				K_D=15.0;		//0.0002
				K_I=0.0;	//12
				K_G=0.0;
				first_K_G=K_G;
				
				Accel=8000.0;
				speed_para[0]=1800.0;
				speed_para[1]=2200.0;
				speed_para[2]=2500.0;
				
				sens_dis[1]=480.0;
				sens_dis[2]=300.0;
				sens_dis[4]=(-300.0);
				sens_dis[5]=(-480.0);

				break;
				
		case 3: speed=500.0;	//220
				K_P=4.0;		//13.5
				K_P_sub=0.0;
				K_D=15.0;		//0.0002
				K_I=0.0;	//12
				K_G=0.0;
				first_K_G=K_G;
				
				Accel=8000.0;
				speed_para[0]=1900.0;
				speed_para[1]=2200.0;
				speed_para[2]=2600.0;
				
				sens_dis[1]=480.0;
				sens_dis[2]=300.0;
				sens_dis[4]=(-300.0);
				sens_dis[5]=(-480.0);

				break;
		
		case 4: speed=500.0;	//220
				K_P=4.0;		//13.5
				K_P_sub=0.0;
				K_D=15.0;		//0.0002
				K_I=0.0;	//12
				K_G=0.0;
				first_K_G=K_G;
				
				Accel=8000.0;
				speed_para[0]=2000.0;
				speed_para[1]=2500.0;
				speed_para[2]=3500.0;
				
				sens_dis[1]=480.0;
				sens_dis[2]=300.0;
				sens_dis[4]=(-300.0);
				sens_dis[5]=(-480.0);
				break;
	}
	reset();
}

void select_corner()
{
	int select_frag=0;
	short selecter=1000;

	while(select_frag==0){
		calc_senc();
		if(SW == SW_ON){
			TIMER_WAIT(200);
			BUZZ_ON();
			LED2=~LED2;
			selecter+=100;
		}
		
		if(cari_SEN[1] > 180.0){
			SEN_thre[corner2]=selecter;
			select_frag=1;
			LED_manager(7);
		}
	}
	TIMER_WAIT(1000);
	reset();
}

void select_P()
{
	int select_frag=0;
	float selecter=1.0; //1.0

	while(select_frag==0){
		calc_senc();
		if(SW == SW_ON){
			TIMER_WAIT(200);
			BUZZ_ON();
			LED2=~LED2;
			selecter+=0.1;
		}
		
		if(ave_SEN[1] > 180.0){
			K_P=selecter;
			first_K_P=K_P;
			select_frag=1;
			LED_manager(7);
		}
	}
	TIMER_WAIT(1000);
	reset();
}

void select_sub()
{
	int select_frag=0;
	float selecter=0.00;

	while(select_frag==0){
		calc_senc();
		if(SW == SW_ON){
			TIMER_WAIT(200);
			LED2=~LED2;
			selecter+=0.10;
		}
		
		if(ave_SEN[1] > 180.0){
			K_P_sub=selecter;
			select_frag=1;
			LED_manager(7);
		}
	}
	TIMER_WAIT(1000);
	reset();
}

void select_I()
{
	int select_frag=0;
	float selecter=0.0;

	while(select_frag==0){
		calc_senc();
		if(SW == SW_ON){
			TIMER_WAIT(200);
			LED2=~LED2;
			selecter+=0.00010;
		}
		
		if(ave_SEN[1] > 180.0){
			K_I=selecter;
			select_frag=1;
			LED_manager(7);
		}
	}
	TIMER_WAIT(1000);
	reset();
}

void select_D()
{
	int select_frag=0;
	float selecter=10.0;

	while(select_frag==0){
		calc_senc();
		if(SW == SW_ON){
			TIMER_WAIT(200);
			BUZZ_ON();
			LED2=~LED2;
			selecter+=5.0;
		}
		
		if(ave_SEN[1] > 180.0){
			K_D=selecter;
			first_K_D=K_D;
			select_frag=1;
			LED_manager(7);
		}
	}
	TIMER_WAIT(1000);
	reset();
}

void select_G()
{
	int select_frag=0;
	float selecter=0.0;

	while(select_frag==0){
		calc_senc();
		if(SW == SW_ON){
			TIMER_WAIT(200);
			BUZZ_ON();
			LED2=~LED2;
			selecter+=0.1;
		}
		
		if(ave_SEN[1] > 180.0){
			K_G=selecter;
			first_K_G=K_G;
			select_frag=1;
			LED_manager(7);
		}
	}
	TIMER_WAIT(1000);
	reset();
}

void select_speed()
{
	int select_frag=0;
	float selecter=1000.0;

	while(select_frag==0){
		calc_senc();
		if(SW == SW_ON){
			TIMER_WAIT(200);
			LED2=~LED2;
			BUZZ_ON();
			selecter+=50.0;
		}
		
		if(ave_SEN[1] > 180.0){
			Desire_vel_nomal=selecter;
			Desire_vel=Desire_vel_nomal;
			curve_speed_para[R10]=Desire_vel_nomal;
			curve_speed_para[R15]=Desire_vel_nomal;1.1;
			curve_speed_para[R20]=Desire_vel_nomal*1.2;
			curve_speed_para[R25]=Desire_vel_nomal*1.3;
			curve_speed_para[R30]=Desire_vel_nomal*1.4;
			curve_speed_para[R35]=Desire_vel_nomal*1.4;
			curve_speed_para[R40]=Desire_vel_nomal*1.5;
			curve_speed_para[R45]=Desire_vel_nomal*1.5;
			curve_speed_para[R50]=Desire_vel_nomal*1.5;
			curve_speed_para[R55]=Desire_vel_nomal*1.5;
			curve_speed_para[R60]=Desire_vel_nomal*1.5;
			curve_speed_para[R65]=Desire_vel_nomal*1.5;
			curve_speed_para[R70]=Desire_vel_nomal*1.5;
			curve_speed_para[R75]=Desire_vel_nomal*1.5;
			curve_speed_para[R80]=Desire_vel_nomal*1.5;
			curve_speed_para[R85]=Desire_vel_nomal*1.5;
			curve_speed_para[R90]=Desire_vel_nomal*1.5;
			curve_speed_para[R95]=Desire_vel_nomal*1.5;
			
			select_frag=1;
			LED_manager(7);
		}
	}
	TIMER_WAIT(1000);
	reset();
}

void select_circle()
{
	int select_frag=0;
	char selecter=1;

	while(select_frag==0){
		calc_senc();
		if(SW == SW_ON){
			TIMER_WAIT(200);
			LED2=~LED2;
			BUZZ_ON();
			selecter+=1;
		}
		
		if(ave_SEN[1] > 180.0){
			circle_number=selecter;
			select_frag=1;
			LED_manager(7);
		}
	}
	TIMER_WAIT(1000);
	reset();
}

void select_debugmode()
{
	int select_frag=0;
	char selecter=0;

	while(select_frag==0){
		calc_senc();
		if(SW == SW_ON){
			TIMER_WAIT(200);
			LED2=~LED2;
			BUZZ_ON();
			selecter=1;
		}
		
		if(ave_SEN[1] > 180.0){
			debug_mode_flag=selecter;
			select_frag=1;
			LED_manager(7);
		}
	}
	TIMER_WAIT(1000);
	reset();
}

void trace3()	//テストモード
{
	short trace_roop_frag=0;
	short i=0;
	
	circl_count=3;
	LED3=ON;
	TIMER_WAIT(500);
	LED3=OFF;

	ch_para();
	select_P();
	select_D();
	select_speed();
	calc_accel_dis(Desire_vel_nomal);
	Accel_manager(0,course_data[0]);
	init_ENC();
	
	while(trace_roop_frag==0){
		STB_MODE();
		if(SW == SW_ON){
			countdown();
			TIMER_WAIT(1000);
				
			zero_gyro=get_gyro();
			sum_gyro=0;
			for(i=0; i<5; i++)
				lowpass_window[i]=0;				
			lowpass_gyro_sum=0;
			lowpass_gyro=0;

			init_ENC();
			ENC_count=0;		
			mot_STB(START_A);
		
			start0=1;
			mot_frag=1;
			while(stop_frag==0){}
					
			stop_manager();
			trace_roop_frag=1;			
		}
		if(ENC_R < 10000){
			trace_roop_frag=1;
			for(i=0; i<25; i++)
				BUZZ_ON();
		}			

		else{}
	}
}

void trace2()	//二週目以降
{
	short trace_roop_frag=0;
	short i=0;
	
	circl_count=2;
	LED2=ON;
	TIMER_WAIT(500);
	LED2=OFF;

	ch_para();
	select_P();
	select_D();
	select_G();
	select_speed();
	
	while(1){
		if(SW == SW_ON){
			calc_senc;
			calc_mot_in_senc();
			mot_STB(START_A);
			TIMER_WAIT(500);
			
			while(1){
				i++;
				if(i>10000){
					LED4=~LED4;
					i=0;
				}
				get_sens_frag=1;
				mot_drive(ERR,0.0,0.0,0.0,0.0,0.0,0.0);
				if(SW==SW_ON)
					break;
			}
			mot_brake();
			get_sens_frag=0;
			break;
		}
		if(ENC_R < 10000){
			trace_roop_frag=1;
			for(i=0; i<10; i++)
				BUZZ_ON();
			break;	
		}			

		else{
			LED3=ON;
		}
	}
	
	calc_accel_dis(Desire_vel_nomal);
	Accel_manager(0,course_data[0]);
	init_ENC();
	
	while(trace_roop_frag==0){
		countdown();
		TIMER_WAIT(1000);
			
		zero_gyro=get_gyro();	
		sum_gyro=0;
		for(i=0; i<5; i++)
			lowpass_window[i]=0;				
		lowpass_gyro_sum=0;
		lowpass_gyro=0;
		init_ENC();
		ENC_count=0;		
		mot_STB(START_A);
		
		start0=1;
		mot_frag=1;
		while(stop_frag==0){}
				
		stop_manager();
		trace_roop_frag=1;			
	}
}

void trace()	//一週目
{
	short trace_roop_frag=0;
	short i=0;
	
	circl_count=1;	
	LED1=ON;
	TIMER_WAIT(500);
	LED1=OFF;
	
	ch_para();
	select_P();
	select_D();
	select_G();
	select_speed();
	select_circle();
	init_thre();			
	calc_accel_dis(Desire_vel_nomal);
	init_ENC();	

	while(1){
		if(SW == SW_ON){
			calc_senc;
			calc_mot_in_senc();
			mot_STB(START_A);
			TIMER_WAIT(500);
			
			while(1){
				i++;
				if(i>10000){
					LED4=~LED4;
					i=0;
				}
				get_sens_frag=1;
				mot_drive(ERR,0.0,0.0,0.0,0.0,0.0,0.0);
				if(SW==SW_ON)
					break;
			}
			mot_brake();
			get_sens_frag=0;
			K_P=first_K_P;
			break;
		}
		else{LED3=ON;}

		if(ENC_R < 10000){
			trace_roop_frag=1;
			for(i=0; i<10; i++)
				BUZZ_ON();
			break;
		}			
	}

	while(trace_roop_frag==0){	
		countdown();
		TIMER_WAIT(1000);
		for(i=0; i<201; i++){
			ENC_dis[i]=0;
			course_data[i]=0;
		}
		
		zero_gyro=get_gyro();
		sum_gyro=0;
		for(i=0; i<5; i++)
			lowpass_window[i]=0;				
		lowpass_gyro_sum=0;
		lowpass_gyro=0;
		
		init_ENC();
		ENC_count=0;
		shortcut_area=0;
		mot_STB(START_A);
		
		start0=1;
		mot_frag=1;
		//gyro_frag=1;
		LED1=OFF;
		while(stop_frag==0){}
		
		stop_manager();
		trace_roop_frag=1;
	}
}

void out_sens()
{
	TIMER_WAIT(2000);
	//get_sens_frag=1;	
	while(1){
//		get_senc();
		calc_senc();
		calc_mot_in_senc();
		dec_out(ave_SEN[1], 6);	outs(" ");
		dec_out(ave_SEN[2], 6);	outs(" ");
		dec_out(ave_SEN[3], 6);	outs(" ");
		dec_out(ave_SEN[4], 6);	outs(" ");
		dec_out(ave_SEN[5], 6);	outs(" ");
		dec_out(ave_SEN[6], 6);	outs(" ");
		dec_out(ave_SEN[corner], 6);	outs(" ");
		dec_out(ave_SEN[corner2], 6);	outs(" ");
		dec_out(ave_SEN[goal], 6);	outs(" ");
		
		dec_out((short)now_dis, 6);	outs(" ");

		outs("\n");

	}
} 

void out_sens2()
{
	short i;	
	TIMER_WAIT(2000);
	get_sens_frag=1;
	mot_STB(START_A);
	MOT_ctr_R1=1;
	MOT_ctr_R2=0;
	MOT_ctr_L1=0;
	MOT_ctr_L2=1;
	MTU0.TGRC=200;
	MTU0.TGRA=200;

	for(i=1; i<9; i++){
		dec_out(SEN_min[i], 5);	outs(" ");
		dec_out(SEN_max[i], 5);	outs("\n");
	}

	while(1){
		for(i=1; i<9; i++){
			if(ave_SEN[i]<0)
				ave_SEN[i]=0;
			dec_out(ave_SEN[i], 4);	outs(" ");
		}
		dec_out(SEN[corner], 4);	outs(" ");
		dec_out(SEN[corner2], 4);	outs(" ");
		dec_out(SEN[goal], 4);	outs(" ");	
		outs("\n");
	}
} 

void out_course()
{
	short i=0;
	float tmp=0.0;
					
	for(i=0; i<201; i++){
		tmp=(float)ENC_dis[i]*mmpp;
	
		dec_out(course_data[i], 6); outs(" ");
		dec_out(ENC_dis[i], 6); outs(" ");
		dec_out(gyro_data[i], 8); outs(" ");
		dec_out((long)tmp, 6); outs(" ");
		dec_out(continue_st[i], 3); outs(" ");
		dec_out(zero_gyro, 6); outs(" ");
		outs("\n");
	}
	dec_out(now_vel, 10); outs(" ");
	dec_out((short)accel_dis[0], 6); outs(" ");
	dec_out((short)accel_dis[1], 6); outs(" ");
	dec_out((short)accel_dis[2], 6); outs("\n");
	
}

void out_ERR()
{
	short i,j=0;
					
	for(i=0; i<200; i++){
		for(j=0; j<2; j++){
	//		dec_out(ERR_check[j][i], 6); outs(" ");
		}
	//	dec_out(Max_ERR_dis[i], 6); outs(" ");
		outs("\n");
	}
}

void out_gyro()
{
	TIMER_WAIT(2000);	
//	gyro_frag=1;
	zero_gyro=get_gyro();
	while(1){
		calc_gyro();
		dec_out(ave_gyro, 4);
		outs("\n");
	}
} 

void out_gyro2()
{		
	TIMER_WAIT(2000);	
	mot_STB(START_A);
	MOT_ctr_R1=1;
	MOT_ctr_R2=0;
	MOT_ctr_L1=0;
	MOT_ctr_L2=1;
	MTU0.TGRC=200;
	MTU0.TGRA=200;

	gyro_frag=1;
	while(1){
		dec_out(ave_gyro, 4);
		outs("\n");
	}
} 

int init_thre(){
	
	SEN_thre[1]     =400;
	SEN_thre[2]     =400;
	SEN_thre[3]     =400;
	SEN_thre[4]     =400;
	SEN_thre[5]     =400;
	
	if(debug_mode_flag==0){
		SEN_thre[goal]  =250;//2700;//max_thre*0.3;//TargetGoal*K_goal;//max_thre*0.3;//90;
		SEN_thre[corner]=250;//1200;//max_thre*0.3;//60;
		SEN_thre[corner2]=170;//1000;//max_thre*0.3;//60;
	}
	else if(debug_mode_flag==1){
		SEN_thre[goal]  =250;//1000;//max_thre*0.3;//TargetGoal*K_goal;//max_thre*0.3;//90;
		SEN_thre[corner]=250;//1200;//max_thre*0.3;//60;
		SEN_thre[corner2]=170;//2700;//max_thre*0.3;//60;
	}
	
	return 0;
}

void sens_save(void)
{
	short buf[4];
	short i=0;
	
	fld_erase_2KB(6);
	buf[0]=0;
	for(i=1; i<9; i++){
		buf[1]=SEN_max[i];
		buf[2]=SEN_min[i];
		buf[3]=SEN_minmax[i];
		Date_flash_write(6,i,buf);
	}
}

void course_save(void)
{
	short buf[4];
	long tmp=0;
	short i=0;
	
	fld_erase_2KB(6);
	fld_erase_2KB(5);

	buf[2]=0;
	buf[3]=0;
	for(i=0; i<200; i++){
		tmp=ENC_dis[i];
		buf[0] = (tmp>>16)&0xFFFF;
		buf[1] = (tmp)&0xFFFF;
		Date_flash_write(6,i,buf);
	}
	
	buf[0]=0;
	buf[1]=0;
	buf[2]=0;
	for(i=0; i<200; i++){
		buf[3]=course_data[i];
		Date_flash_write(5,i,buf);
	}
}

void sens_load(void)
{
	unsigned short *load;
	short i=0;
	
	for(i=1; i<9; i++){
		load=Data_flash_read(6,i);
		SEN_max[i]=load[1];
		SEN_min[i]=load[2];
		SEN_minmax[i]=load[3];
	}
}

void course_load(void)
{
	unsigned short *load;
	long tmp=0;
	short i=0;
	
	for(i=0; i<200; i++){
		load=Data_flash_read(6,i);
		tmp = load[0];
		tmp = (tmp<<16)&0xFFFF0000;
		tmp |= load[1]&0xFFFF;
		ENC_dis[i]=tmp;
	}
	for(i=0; i<200; i++){
		load=Data_flash_read(5,i);
		course_data[i]=load[3];
	}
}

void v_LED_ON()
{
	while(1){
		V_LED6=ON;
		V_LED7=ON;
		V_LED8=ON;
		TIMER_WAIT(100);
		V_LED6=OFF;
		V_LED7=OFF;
		V_LED8=OFF;
		TIMER_WAIT(100);
	}
}
	
void ch_mode(int mode)
{
	reset();
	switch(mode){
		case 1: search_max();	break;
		case 2: trace();		break;
		case 3: trace2();		break;
//		case 4: trace3();		break;
		case 4: course_save();	break;
		case 5: course_load();	break;
		case 6: out_sens();		break;
		case 7: v_LED_ON();		break;
		case 8: out_course();	break;
		case 9: make_shortcut1(); break;
		case 10: make_shortcut2(); break;

	}
}

void main()
{			
	set();
	BUZZ_ON();
	init_goal_frag();
	init_thre();
	mot_STB(STOP);
	Battery_Check();
	ENC_R=3300;

	while(1)
		ch_mode(select_mode());
}
