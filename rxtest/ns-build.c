#include <machine.h>
#include "iodefine.h"
/*
typedef unsigned int unsigned long;
typedef unsigned int long;
typedef unsigned int unsigned short;
typedef unsigned int unsigned char;
*/

#define FLD_SUCCESS 1 
#define FLD_ERROR 2
#define FLD_BLANK 3
#define FLD_OK 4
#define FLD_TMOUT 5
#define FLD_NOBLANK 6

#define BUZZ_permit 1

#define ON 1
#define OFF 0 

#define corner 6
#define corner2 7
#define goal 8

#define START_A 1
#define STOP 0

#define ST 1
#define curve 0

#define Tred 123

#define right 1
#define left 0

#define mmpp 0.0137375

#define buzz PORTB.DR.BIT.B7

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

#define MOT_ctr_L1 PORTA.DR.BIT.B0
#define	MOT_ctr_L2 PORTA.DR.BIT.B1
#define MOT_ctr_R1 PORTA.DR.BIT.B2
#define	MOT_ctr_R2 PORTA.DR.BIT.B3
#define MOT_STBY   PORTA.DR.BIT.B4		

#define ENC_R MTU1.TCNT
#define ENC_L MTU2.TCNT

#define max_thre 400.0

#define senc_dis1 26.0
#define senc_dis2 13.0
#define senc_dis3 0.0
#define senc_dis4 -13.0
#define senc_dis5 -26.0

short sens_dis[6]={0, 600, 300, 0 , -300, -600};

short max_ERR=300;
short min_ERR=-300;

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
		 short fail_safe_count=0;
		 short ST_curve_check=0;
		 short goal_count=0;
		 short u_count=0;
		 
/***************床センサ***************/
short SEN_OFF[9];
short SEN[9];
short cari_SEN[9];
short sum_SEN[9];
short MAX_SEN[9];
short MIN_SEN[9];
short ave_SEN[9];
short SEN_min[9];
short SEN_max[9];
short SEN_thre[9];
short SEN_minmax[9];
short ave_count=0;
short ERR=0;
short ERR2=0;				//debug
short sub_ERR=0;
short PRE_ERR=0;
short PRE_senc=0;
short SUM_ERR=0;
short thre_max=150;

/***************ジャイロ***************/
float GYRO_ERR=0.0;
float filter_gyro=0.0;
float gyro2=0.0;
float GYRO_speed=0.0;
float zero_gyro=0.0;
float target_gyro=0;
float sum_gyro=0;
float ave_gyro=0;
float MAX_gyro=0;
float MIN_gyro=0;
short gyro_count=0;
short curve_check=0;

/***************エンコーダ*************/
long now_ENC_R=0;
long now_ENC_L=0;
long NowEnc=0;
long diff_ENC_R=0;
long diff_ENC_L=0;
long curvature=0;
long angl_vel=0;
float ENC_dis[200];
short ENC_check[200];
short EncTimeCount=0;
short ENC_count=0;
short RL_check[200];

/***************PIDゲイン**************/
float K_P=1.0;
float K_P_sub=1.0;
float first_K_P=0.0;
float K_D=1.0;
float K_I=1.0;
float K_G=1.0;
float first_K_G=0.05;
float K_P_SPEED=0.001;
float K_I_SPEED=0.001;
float speed=130.0;
float first_speed=0.0;
float first_target_speed=0.0;
float now_vel_R=0.0;
float now_vel_L=0.0;
float now_vel=0.0;
float Desire_vel=0.0;
float Desire_vel_nomal=0.0;
float Target_vel=0.0;
float Accel=0.0;
float Accel_in=0.0;
float SPEED_ERR_R=0.0;
float SPEED_ERR_L=0.0;
float SPEED_ERR=0.0;
float SPEED_SUM_ERR_R=0.0;
float SPEED_SUM_ERR_L=0.0;
float SPEED_SUM_ERR=0.0;
float speed_para[3];

/***************フラグ*****************/
int goal_frag1=0;
int goal_frag2=0;
int first_check=0;
int stop_frag=0;
int start0=0;
int sci_frag=0;
int curve_frag=0;
int gyro_first=0;
int dis_count=0;

int mot_frag=0;
int curve_one=0;
int speed_check_frag=0;
short SEN_frag=0;
short ST_frag=0;
short ST_count=0;
short curve_frag2=0;
short corner_frag=0;
short cari_frag=0;
short Dispermit_goal=0;
short gyro_frag=0;
short gyro_permit=0;
short CcwPermit=0;
short ST_ENC_frag=0;
short sub_frag=0;
short StCurveCheck2=curve;
short first_target_frag=0;
short first_target_frag2=0;
short max_frag=0;
short min_frag=0;
short turn_frag=0;
short right_curve_frag=0;
short left_curve_frag=0;
short micro_time_frag=0;
short get_sens_frag=0;

/*************プロトタイプ宣言**********/
void set();
void wait();
short get_Batt(); 
float get_gyro();
void get_senc();
int calc_senc();
void mot_STB(int start_stop);
void search_max();
void stop_manager();
short goal_check(long RIGHT_ENC, long LEFT_ENC);
void TIMER_CALL();
void TIMER_WAIT(unsigned short ms);
void mot_drive(float mot_in_p, float mot_in_d, float mot_in_i, float mot_in_g, float mot_in_speed_R, float mot_in_sum_speed_R, float mot_in_speed_L, float mot_in_sum_speed_L, float mot_in_speed, float mot_in_sum_speed);
void Battery_Check();
void countdown();
void trace();
void out_senc();
void ch_mode(int mode);
void check_curve();
void mot_brake();	

void BUZZ_ON()
{	
	short i=0;
	short buzz_hz=500;

#ifdef BUZZ_permit	
	for(i=0; i<50; i++){
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
    CMT1.CMCOR = 600;       	//1us周期で割り込み
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
	MTU0.TGRB=1500;
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

float get_gyro()	//ジャイロ値取得
{
	float gyro=0.0;
	
	S12AD1.ADCSR.BIT.ADST=0;			//A/D変換停止		
	S12AD1.ADANS.BIT.CH=1;				//AN101をA/D変換
	S12AD1.ADCSR.BIT.ADST=1;			//A/D変換開始
	while(S12AD1.ADCSR.BIT.ADST==1);	//A/D変換が終わるまで待つ
	gyro=S12AD1.ADDR1;					//LEDを消した状態での値を格納
	//gyro=AD0.ADDRC>>6;
	
	return gyro;
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

	for(count=0; count<201; count++){}
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

/***************************************************************************/

	V_LED1=ON;		
	for(count=0; count<201; count++){}	//LEDが点灯しきるまで少し待機
	S12AD0.ADCSR.BIT.ADST=0;			//A/D変換停止
	S12AD0.ADANS.BIT.CH=1;				//AN001をA/D変換
	S12AD0.ADCSR.BIT.ADST=1;			//A/D変換開始
	while(S12AD0.ADCSR.BIT.ADST==1);	//A/D変換が終わるまで待つ
	SEN[1]=S12AD0.ADDR1-SEN_OFF[1];			
	V_LED1=OFF;		
	for(count=0; count<101; count++){}

	V_LED2=ON;		
	for(count=0; count<201; count++){}	//LEDが点灯しきるまで少し待機
	S12AD0.ADCSR.BIT.ADST=0;			//A/D変換停止
	S12AD0.ADANS.BIT.CH=2;				//AN002をA/D変換
	S12AD0.ADCSR.BIT.ADST=1;			//A/D変換開始
	while(S12AD0.ADCSR.BIT.ADST==1);	//A/D変換が終わるまで待つ
	SEN[2]=S12AD0.ADDR2-SEN_OFF[2];			
	V_LED2=OFF;		
	for(count=0; count<101; count++){}

	V_LED3=ON;		
	for(count=0; count<201; count++){}	//LEDが点灯しきるまで少し待機
	S12AD1.ADCSR.BIT.ADST=0;			//A/D変換停止
	S12AD1.ADANS.BIT.CH=0;				//AN100をA/D変換
	S12AD1.ADCSR.BIT.ADST=1;			//A/D変換開始
	while(S12AD1.ADCSR.BIT.ADST==1);	//A/D変換が終わるまで待つ
	SEN[3]=S12AD1.ADDR0A-SEN_OFF[3];			
	V_LED3=OFF;		
	for(count=0; count<101; count++){}

	V_LED4=ON;		
	for(count=0; count<201; count++){}	//LEDが点灯しきるまで少し待機
	S12AD1.ADCSR.BIT.ADST=0;			//A/D変換停止
	S12AD1.ADANS.BIT.CH=3;				//AN103をA/D変換
	S12AD1.ADCSR.BIT.ADST=1;			//A/D変換開始
	while(S12AD1.ADCSR.BIT.ADST==1);	//A/D変換が終わるまで待つ
	SEN[4]=S12AD1.ADDR3-SEN_OFF[4];			
	V_LED4=OFF;		
	for(count=0; count<101; count++){}

	V_LED5=ON;		
	for(count=0; count<201; count++){}	//LEDが点灯しきるまで少し待機
	S12AD0.ADCSR.BIT.ADST=0;			//A/D変換停止
	S12AD0.ADANS.BIT.CH=3;				//AN003をA/D変換
	S12AD0.ADCSR.BIT.ADST=1;			//A/D変換開始
	while(S12AD0.ADCSR.BIT.ADST==1);	//A/D変換が終わるまで待つ
	SEN[5]=S12AD0.ADDR3-SEN_OFF[5];			
	V_LED5=OFF;		
	for(count=0; count<101; count++){}

	V_LED6=ON;		
	for(count=0; count<201; count++){}	//LEDが点灯しきるまで少し待機
	AD0.ADCSR.BIT.CH=8;				//AD変換するアナログ入力チャンネルを設定  AD0.ADDRI
	AD0.ADCSR.BIT.ADST = 1;			// AD変換開始
	while(AD0.ADCSR.BIT.ADST==1);	//AD変換終了まで待機
	AD0.ADCSR.BIT.ADST  = 0;		// AD停止
	SEN[6]=AD0.ADDRI-SEN_OFF[6];
	V_LED6=OFF;		
	for(count=0; count<101; count++){}

	V_LED7=ON;		
	for(count=0; count<201; count++){}	//LEDが点灯しきるまで少し待機
	AD0.ADCSR.BIT.CH=7;				//AD変換するアナログ入力チャンネルを設定  AD0.ADDRI
	AD0.ADCSR.BIT.ADST = 1;			// AD変換開始
	while(AD0.ADCSR.BIT.ADST==1);	//AD変換終了まで待機
	AD0.ADCSR.BIT.ADST  = 0;		// AD停止
	SEN[7]=AD0.ADDRH-SEN_OFF[6];
	V_LED7=OFF;		
	for(count=0; count<101; count++){}

	V_LED8=ON;		
	for(count=0; count<201; count++){}	//LEDが点灯しきるまで少し待機
	AD0.ADCSR.BIT.CH=5;				//AD変換するアナログ入力チャンネルを設定  AD0.ADDRI
	AD0.ADCSR.BIT.ADST = 1;			// AD変換開始
	while(AD0.ADCSR.BIT.ADST==1);	//AD変換終了まで待機
	AD0.ADCSR.BIT.ADST  = 0;		// AD停止
	SEN[8]=AD0.ADDRF-SEN_OFF[6];
	V_LED8=OFF;		
	for(count=0; count<101; count++){}
}

int calc_senc()
{
	int i;
	
	get_senc();
	for(i=1; i<9; i++){
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
		
		for(i=1; i<9; i++){
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
		short now_dis[2];
		short line_place=0;
		short i=0;
		
		max=ave_SEN[1];
		
		for(i=2; i<6; i++){			
			if(max < ave_SEN[i]){
				max=ave_SEN[i];
				max_number=i;
			}
		}	
		
		if((ave_SEN[1] > thre_max) && (ave_SEN[2] < thre_max && ave_SEN[3] < thre_max && ave_SEN[4] < thre_max)){
			max_frag=1;
		}
		if(ave_SEN[2] > thre_max || ave_SEN[3] > thre_max || ave_SEN[4] > thre_max){
			LED1=OFF;
			max_frag=0;
		}
		
		if((ave_SEN[2] < thre_max && ave_SEN[3] < thre_max && ave_SEN[4] < thre_max) && (ave_SEN[5] > thre_max)){
			min_frag=1;
		}
		if(ave_SEN[2] > thre_max || ave_SEN[3] > thre_max || ave_SEN[4] > thre_max){
			min_frag=0;
			LED1=OFF;
		}

		if(max_number==2){
			if(ave_SEN[1] > ave_SEN[3])
				ERR=((sens_dis[1]*ave_SEN[1])+(sens_dis[2]*ave_SEN[2]))/(ave_SEN[1]+ave_SEN[2]);
			else{
//				ERR=((sens_dis[3]*ave_SEN[3])+(sens_dis[2]*ave_SEN[2]))/(ave_SEN[3]+ave_SEN[2]);
				ERR=ave_SEN[2]-ave_SEN[4];	
			}	
		}
		if(max_number==3){
#if 0
			if(ave_SEN[2] > ave_SEN[4])
				ERR=((sens_dis[3]*ave_SEN[3])+(sens_dis[2]*ave_SEN[2]))/(ave_SEN[3]+ave_SEN[2]);
			else{
				ERR=((sens_dis[3]*ave_SEN[3])+(sens_dis[4]*ave_SEN[4]))/(ave_SEN[3]+ave_SEN[4]);
			}	
#endif
			ERR=ave_SEN[2]-ave_SEN[4];	
		}
		if(max_number==4){
			if(ave_SEN[3] > ave_SEN[5])
//				ERR=((sens_dis[3]*ave_SEN[3])+(sens_dis[4]*ave_SEN[4]))/(ave_SEN[3]+ave_SEN[4]);
				ERR=ave_SEN[2]-ave_SEN[4];	
			else{
				ERR=((sens_dis[5]*ave_SEN[5])+(sens_dis[4]*ave_SEN[4]))/(ave_SEN[5]+ave_SEN[4]);
			}	
		}
		if(max_number==1){
			ERR=((sens_dis[1]*ave_SEN[1])+(sens_dis[2]*ave_SEN[2]))/(ave_SEN[1]+ave_SEN[2]);
		}
		if(max_number==5){
			ERR=((sens_dis[5]*ave_SEN[5])+(sens_dis[4]*ave_SEN[4]))/(ave_SEN[5]+ave_SEN[4]);
		}
	//	ERR=ave_SEN[2]-ave_SEN[4];	
/*						
		if(((ave_SEN[2] < thre_max) && (ave_SEN[5] < thre_max) && (ave_SEN[1] < thre_max) && (ave_SEN[6] < thre_max))){
			ERR=ave_SEN[3]-ave_SEN[4];
			K_P=first_K_P;
		}

		else if((max==ave_SEN[2]) && (ave_SEN[2] > thre_max) && goal_frag2==1){				
			ERR=max_sens;
			LED2=ON;
		}

		else if((max==ave_SEN[5]) && (ave_SEN[5] > thre_max) && goal_frag2==1){
			ERR=max_sens*(-1);
			LED2=ON;
		}			

		else if((max==ave_SEN[1]) && (ave_SEN[1] > thre_max) && goal_frag2==1){
			ERR=max_sens;
		}
		else if((max==ave_SEN[6]) && (ave_SEN[6] > thre_max) && goal_frag2==1){
			ERR=max_sens*(-1);		
		}

		if((goal_frag1==1 && max_frag==1 && (ave_SEN[2] < thre_max && ave_SEN[3] < thre_max && ave_SEN[4] < thre_max))){
			ERR=max_ERR;
			LED1=ON;
		}
				
		if((goal_frag1==1 && min_frag==1 && (ave_SEN[3] < thre_max && ave_SEN[4] < thre_max && ave_SEN[5] < thre_max))){
			ERR=min_ERR;
			LED1=ON;
		}
*/		
		PRE_ERR=ERR-PRE_senc;													
		SUM_ERR+=ERR;					
		PRE_senc=ERR;					
		
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
		if(PORTE.PORT.BIT.B0 == 0)
			loop_frag=1;
		else{}
	}
	TIMER_WAIT(1000);
	LED1=OFF;
	ENC_R=0;
	ENC_L=0;
	mot_STB(START_A);
	LED3=ON;
	
	for(i=1; i<9; i++){
		SEN_max[i]=1;
		SEN_min[i]=10;
	}
	cari_frag=1;
	while(ENC_R < 10000){
		MOT_ctr_R1=1;
		MOT_ctr_R2=0;
		MOT_ctr_L1=0;
		MOT_ctr_L2=1;
		MTU0.TGRC=100 - (0.2*(ENC_R-ENC_L));
		MTU0.TGRA=100 + (0.2*(ENC_R-ENC_L));
		if(ENC_R > 2000){
			for(i=1; i<9; i++){
				if(SEN_max[i] < SEN[i])
					SEN_max[i]=SEN[i];
				if(SEN[i] > 0 && SEN_min[i] > SEN[i])
					SEN_min[i]=SEN[i];
			}	
		}
	}
	cari_frag=0;
	mot_brake();
	ENC_R=40000;
	ENC_L=40000;
	TIMER_WAIT(200);
	
	MOT_ctr_R1=0;
	MOT_ctr_R2=1;
	MOT_ctr_L1=1;
	MOT_ctr_L2=0;
	cari_frag=1;
	while((40000-ENC_R) < 10000){
		MTU0.TGRC=100 - (0.0001*(ENC_R-ENC_L));
		MTU0.TGRA=100 + (0.0001*(ENC_R-ENC_L));
		if((40000-ENC_R) > 2000){
			for(i=1; i<9; i++){
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
	
	SEN_max[6]=SEN_max[1];
	SEN_max[7]=SEN_max[1];
	SEN_max[8]=SEN_max[1];
	
	SEN_min[6]=SEN_min[1];
	SEN_min[7]=SEN_min[1];
	SEN_min[8]=SEN_min[1];

	for(i=1; i<9; i++)
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
	curve_frag=0;
	curve_count=0;
	dis_count=0;
	ENC_count=0;
	first_check=0;
	StartTimeCount=0;
	CrossTimeCount=0;
	Dispermit_goal=0;
	AccelTimeCount=0.0;
	Desire_vel=0.0;
	now_vel_R=0.0;
	now_vel_L=0.0;
	SPEED_ERR=0.0;
	SPEED_SUM_ERR=0.0;
	SPEED_SUM_ERR_R=0.0;
	SPEED_SUM_ERR_L=0.0;
	fail_safe_count=0;
	ST_curve_check=0;
	first_target_frag2=0;
	first_target_speed=0;
	max_frag=0;
	min_frag=0;
	turn_frag=0;
}

void mot_brake()		//モータをブレーキ状態に
{
	MOT_ctr_L1=1;
	MOT_ctr_L2=1;
	MOT_ctr_R1=1;
	MOT_ctr_R2=1;
}

void stop_manager()		//停止処理
{
	while(stop_count!=200){
		mot_drive((float)ERR, (float)PRE_ERR, (float)SUM_ERR, GYRO_ERR, SPEED_ERR_R, SPEED_SUM_ERR_R, SPEED_ERR_L, SPEED_SUM_ERR_L, 0.0, 0.0);
	}

	init_goal_frag();
	mot_brake();
	TIMER_WAIT(1000);
	mot_STB(STOP);
}	

short Accel_manager(short counter, float N_E_R, float N_E_L)
{	
	short StCurveCheck=curve;

#if 1
	if(ENC_dis[counter] > 300.0){		
		if(ENC_check[counter] < 20){	//加速しない
			LED1=ON;
			LED2=OFF;
			LED3=OFF;
			K_P=first_K_P;
		}
		if(ENC_check[counter] > 20 && ENC_check[counter] < 50){	//加速しない
			LED1=ON;
			LED2=OFF;
			LED3=OFF;
			if(first_speed < 700)
				Desire_vel_nomal=first_speed*1.5;
		}
		else if(ENC_check[counter] > 50){							//加速する
			StCurveCheck=ST;
			K_P=first_K_P*1.5;
			LED1=OFF;
			LED2=ON;
			LED3=OFF;
			
			if(N_E_R > 100.0 || N_E_L > 100.0){
				if(ENC_dis[counter] < 400.0){
					if(RL_check[counter]==right && N_E_R<=(ENC_dis[counter] - 120.0)){
						Target_vel=speed_para[0];
						K_G=first_K_G*1.5;
					}
					else if(RL_check[counter]==right && N_E_R>(ENC_dis[counter] - 120.0)){
						if(first_target_frag==0)
							Desire_vel=Target_vel;
						
						first_target_frag=1;
						Target_vel=first_speed;
						Accel=(-1.0)*(Accel_in+3000.0);
					}
				
					if(RL_check[counter]==left && N_E_L<=(ENC_dis[counter] - 120.0)){
						Target_vel=speed_para[0];
						K_G=first_K_G*1.5;
					}
					else if(RL_check[counter]==left && N_E_L>(ENC_dis[counter] - 120.0)){
						if(first_target_frag==0)
							Desire_vel=Target_vel;
						
						first_target_frag=1;
						Target_vel=first_speed;
						Accel=(-1.0)*(Accel_in+3000.0);
					}
				}	
	
				if(ENC_dis[counter] > 400.0 && ENC_dis[counter] < 1020.0){
					if(RL_check[counter]==right && N_E_R<(ENC_dis[counter] - 280.0)){
						Target_vel=speed_para[1];
						K_G=first_K_G*1.5;
					}
					else if(RL_check[counter]==right && N_E_R>(ENC_dis[counter] - 280.0)){
						if(first_target_frag==0)
							Desire_vel=Target_vel;
					
						first_target_frag=1;
						Target_vel=first_speed;
						Accel=(-1.0)*(Accel_in+6000.0);
					}

					if(RL_check[counter]==left && N_E_L<(ENC_dis[counter] - 280.0)){
						Target_vel=speed_para[1];
						K_G=first_K_G*1.5;
					}		
					else if(RL_check[counter]==left && N_E_L>(ENC_dis[counter] - 280.0)){
						if(first_target_frag==0)
							Desire_vel=Target_vel;
						
						first_target_frag=1;
						Target_vel=first_speed;
						Accel=(-1.0)*(Accel_in+6000.0);
					}
				}

				if(ENC_dis[counter] > 1020.0){
					if(RL_check[counter]==right && N_E_R<(ENC_dis[counter] - 450.0)){
						Target_vel=speed_para[2];
						K_G=first_K_G*1.5;
					}
					else if(RL_check[counter]==right && N_E_R>(ENC_dis[counter] - 450.0)){
						if(first_target_frag==0)
							Desire_vel=Target_vel;
						
						first_target_frag=1;
						Target_vel=first_speed;
						Accel=(-1.0)*(Accel_in+6000.0);
					}

					if(RL_check[counter]==left && N_E_L<(ENC_dis[counter] - 450.0)){
						Target_vel=speed_para[2];
						K_G=first_K_G*1.5;
					}
					else if(RL_check[counter]==left && N_E_L>(ENC_dis[counter] - 450.0)){
						if(first_target_frag==0)
							Desire_vel=Target_vel;
					
						first_target_frag=1;
						Target_vel=first_speed;
						Accel=(-1.0)*(Accel_in+6000.0);
					}
				}	
			}
		}
	}
	else if(ENC_dis[counter] < 300.0){								//加速しない カーブでは速度落とすかも
		LED1=OFF;
		LED2=OFF;
		LED3=ON;
	}
#endif
	return StCurveCheck;
}

short goal_check(long RIGHT_ENC, long LEFT_ENC)
{
	short StCurveCheck=curve;
	
	if(first_check==0){
		if(ave_SEN[goal] > SEN_thre[goal]){						//スタート時 白線上
			goal_frag1=1;
			LED3=ON;
		}
	
		if(goal_frag1==1 && (ave_SEN[goal] < (SEN_thre[goal]))){		//スタート時 白線から外れたとき
			LED3=OFF;
			goal_frag2=1;
			first_check=1;
			StartTimeCount=0;
			CrossTimeCount=0;
			BUZZ_ON();
			SPEED_SUM_ERR=0.0;
		}
	}
	if(first_check==1){
		StartTimeCount++;
		if(StartTimeCount==300){
			first_check=2;
			now_ENC_L=0;
			now_ENC_R=0;
		}
	}
			
	if(first_check==2){		
		if((ave_SEN[1] > thre_max) && 
			(ave_SEN[5] > thre_max) && goal_frag2==1){
			goal_frag2=0;
		}

		if(goal_frag2==0){
			CrossTimeCount++;
			LED2=ON;
		}
			
		if(CrossTimeCount==200){	
			goal_frag2=1;
			CrossTimeCount=0;
			LED2=OFF;
			BUZZ_ON();
		}
	
	/********************コーナー処理*******************************/	
		if(circl_count==1){	
			if(((ave_SEN[corner] > SEN_thre[corner]) || (ave_SEN[corner2] > SEN_thre[corner2])) && goal_frag2==1 && curve_frag==0){
				SPEED_SUM_ERR=0.0;
				LED3=ON;
				curve_frag=1;
				right_curve_frag=0;
				left_curve_frag=0;
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
			
				ENC_check[ENC_count]=(short)curvature/10;
				if(ENC_check[ENC_count] < 0)
					ENC_check[ENC_count]*=(-1);
					
				if(RIGHT_ENC > LEFT_ENC){
					RL_check[ENC_count]=right;
					ENC_dis[ENC_count]=(float)RIGHT_ENC*mmpp;
				}
				else if(RIGHT_ENC < LEFT_ENC){
					RL_check[ENC_count]=left;
					ENC_dis[ENC_count]=(float)LEFT_ENC*mmpp;
				}
				
//				ST_check[ENC_count]=ST_curve_check;
				ST_curve_check=0;	
				ENC_count++;
				now_ENC_R=0;
				now_ENC_L=0;
				BUZZ_ON();
			}

			if(curve_frag==1)	//複数回認識することへの対策
				curve_count++;
			if(curve_frag==1 && curve_count==80){
			//if(curve_frag==1 && ((float)(RIGHT_ENC+LEFT_ENC)*mmpp > 35.0)){
				LED3=OFF;
				curve_frag=0;
				curve_count=0;
			}
		}
		
		else if(circl_count==2 || circl_count==3){
			if(RL_check[ENC_count]==right){
				if((((float)RIGHT_ENC*mmpp) > (ENC_dis[ENC_count] - 80.0) && (ave_SEN[corner] > SEN_thre[corner]))){
					ENC_count++;
					now_ENC_R=0;
					now_ENC_L=0;
					K_P=first_K_P;
					Desire_vel=0.0;
					AccelTimeCount=0.0;
					Accel=Accel_in;
					first_target_frag=0;
					first_target_frag2=0;
					SPEED_ERR_R=0.0;
					//SPEED_SUM_ERR_R=0.0;					
					SPEED_ERR_L=0.0;
					//SPEED_SUM_ERR_L=0.0;					
	//				SPEED_ERR=0.0;
	//				SPEED_SUM_ERR=0.0;
					Desire_vel_nomal=first_speed;
//					K_D=10.1;
					K_G=first_K_G;
				}
			}
			else if(RL_check[ENC_count]==left){
				if((((float)LEFT_ENC*mmpp) > (ENC_dis[ENC_count] - 80.0) && (ave_SEN[corner] > SEN_thre[corner]))){
					ENC_count++;
					now_ENC_R=0;
					now_ENC_L=0;
					K_P=first_K_P;
					Desire_vel=0.0;
					AccelTimeCount=0.0;
					Accel=Accel_in;
					first_target_frag=0;
					first_target_frag2=0;
					SPEED_ERR_R=0.0;
				//	SPEED_SUM_ERR_R=0.0;					
					SPEED_ERR_L=0.0;
				//	SPEED_SUM_ERR_L=0.0;
	//				SPEED_ERR=0.0;
	//				SPEED_SUM_ERR=0.0;
					Desire_vel_nomal=first_speed;
//					K_D=10.1;
					K_G=first_K_G;
				}
			}
			
			
			StCurveCheck=Accel_manager(ENC_count, (float)RIGHT_ENC*mmpp, (float)LEFT_ENC*mmpp);	
		}
	/*********************ゴール処理*******************/				
	
		if(turn_frag==0 && (ave_SEN[goal] > SEN_thre[goal]) && goal_frag2==1 && Dispermit_goal!=1){
			if(circl_count==1){
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
				ENC_check[ENC_count]=(short)curvature/10;
				if(RIGHT_ENC > LEFT_ENC){
					RL_check[ENC_count]=right;
					ENC_dis[ENC_count]=(float)RIGHT_ENC*mmpp;
				}
				else if(RIGHT_ENC < LEFT_ENC){
					RL_check[ENC_count]=left;
					ENC_dis[ENC_count]=(float)LEFT_ENC*mmpp;
				}
				goal_count=ENC_count;
				LED3=OFF;
				stop_frag=1;
			}

			if( (ENC_count > goal_count-3) && (circl_count==2 || circl_count==3)){
				LED3=OFF;
				stop_frag=1;
			}
		}
/*
		if((ave_SEN[1] > SEN_thre[1]) && (ave_SEN[2] > SEN_thre[2]) && (ave_SEN[3] > SEN_thre[3]) && (ave_SEN[4] > SEN_thre[4]) && (ave_SEN[5] > SEN_thre[5]) && (ave_SEN[6] > SEN_thre[6]))
			fail_safe_count++;
		if((ave_SEN[1] < SEN_thre[1]) || (ave_SEN[2] < SEN_thre[2]) || (ave_SEN[3] < SEN_thre[3]) || (ave_SEN[4] < SEN_thre[4]) || (ave_SEN[5] < SEN_thre[5]) || (ave_SEN[6] < SEN_thre[6]))
			fail_safe_count=0;	
			
		if(fail_safe_count > 300){
			stop_frag=1;
			fail_safe_count=0;
		}
*/
	}
	else{}
	
	return StCurveCheck;
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
		NowEnc=now_ENC_R-now_ENC_L;
		
		now_vel_R=(float)(ENC_R-15000)*13.7375;	// mm/s
		now_vel_L=(float)(ENC_L-15000)*13.7375;	// mm/s			
		now_vel=(now_vel_R+now_vel_L)/2.0;
		
		calc_senc();					
		StCurveCheck2=goal_check(now_ENC_R, now_ENC_L);
		calc_mot_in_senc();
		
		if(mot_frag==1){
			if((circl_count==2 || circl_count==3) && StCurveCheck2==ST){
				if(Accel > 0.0){
					if(first_target_frag2==0){
						first_target_frag2=1;
						first_target_speed=(now_vel_R+now_vel_L)/2.0;
					}
					if(Desire_vel < Target_vel){
						AccelTimeCount+=0.001;
						Desire_vel=first_target_speed+(Accel*AccelTimeCount);
					}
					else if(Desire_vel >= Target_vel ){
						Desire_vel=Target_vel;
					}
				}
				else if(Accel < 0.0){
					if(Desire_vel > Target_vel){
						AccelTimeCount+=0.0017;
						Desire_vel=Desire_vel+(Accel*AccelTimeCount);
					}
					else if(Desire_vel < Target_vel ){
						Desire_vel=Target_vel;
					}	
				}
				
				SPEED_ERR_R=now_vel_R-Desire_vel;
				SPEED_SUM_ERR_R+=SPEED_ERR_R;		
				SPEED_ERR_L=now_vel_L-Desire_vel;			
				SPEED_SUM_ERR_L+=SPEED_ERR_L;
			}
			
			SPEED_ERR=now_vel-Desire_vel_nomal;
			SPEED_SUM_ERR+=SPEED_ERR;
			mot_drive((float)ERR, (float)PRE_ERR, (float)SUM_ERR, GYRO_ERR, SPEED_ERR_R, SPEED_SUM_ERR_R, SPEED_ERR_L, SPEED_SUM_ERR_L, SPEED_ERR, SPEED_SUM_ERR);
		}
		ENC_R=15000;
		ENC_L=15000;
	}

	if(stop_frag==1)
		stop_count++;
	
}

void TIMER_CALL2()
{
}

void TIMER_WAIT(unsigned short ms)
{
	unsigned long TimeNow = TimeCount;
	while((TimeCount-TimeNow) <= ms );
}

void mot_drive(float mot_in_p, float mot_in_d, float mot_in_i, float mot_in_g,
				float mot_in_speed_R, float mot_in_sum_speed_R,
				float mot_in_speed_L, float mot_in_sum_speed_L,
				float mot_in_speed, float mot_in_sum_speed)
{
	float MOT_IN_R=speed;
	float MOT_IN_L=speed;
	
	MOT_IN_R+=(K_P*mot_in_p);
//	MOT_IN_R+=K_P_sub*mot_in_p_sub;
	MOT_IN_R+=(K_I*mot_in_i);
	MOT_IN_R+=K_D*mot_in_d;
//	MOT_IN_R+=K_G*mot_in_g;

	MOT_IN_L-=(K_P*mot_in_p);
//	MOT_IN_L-=K_P_sub*mot_in_p_sub;
	MOT_IN_L-=(K_I*mot_in_i);
	MOT_IN_L-=K_D*mot_in_d;
//	MOT_IN_L-=K_G*mot_in_g;

	if(circl_count==1){
		if(goal_frag2==1){
			MOT_IN_R-=K_P_SPEED*mot_in_speed;
			MOT_IN_R-=K_I_SPEED*mot_in_sum_speed;
			MOT_IN_L-=K_P_SPEED*mot_in_speed;
			MOT_IN_L-=K_I_SPEED*mot_in_sum_speed;
		}
	}

	if(((circl_count==2 || circl_count==3)) && StCurveCheck2==ST){
		MOT_IN_R-=K_P_SPEED*mot_in_speed_R;
		MOT_IN_R-=K_I_SPEED*mot_in_sum_speed_R;
		MOT_IN_L-=K_P_SPEED*mot_in_speed_L;
		MOT_IN_L-=K_I_SPEED*mot_in_sum_speed_L;
	}

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

	return Battery;
}

void Battery_Check()
{
	short battery;
	battery=get_Batt();
	
	if(battery >= 757){
		LED_manager(7);
		LED1=ON; LED2=ON; LED3=ON;
	}
	if(battery < 757 && battery >= 720){
		LED_manager(3);	
	}
	if(battery < 720){								//7.4V未満になったら未満強制停止
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

		if(ENC_L > 1600*10)
			ENC_L=(1600*2)+1;
			
		if(PORTE.PORT.BIT.B0 == 0){
			BUZZ_ON();
			TIMER_WAIT(500);
			reset();
			para_frag=1;
		}
	}
	return para_count;
}

void ch_para()
{
	switch(select_mode()){
		case 1: speed=300.0;	//220
				K_P=4.0;		//13.5
				first_K_P=K_P;
				K_P_sub=0.0;
				K_D=5.0;		//0.0002
				K_I=0.0;	//12
				K_G=0.3;
				first_K_G=K_G;
				
				first_speed=600.0;
				Accel=200.0;
				Accel_in=Accel+3800.0;
				Desire_vel_nomal=first_speed;
				Target_vel=1200.0;
				speed_para[0]=1000.0;
				speed_para[1]=1500.0;
				speed_para[2]=2500.0;
				break;
		
		case 2: speed=400.0;	//220
				K_P=4.0;		//13.5
				first_K_P=K_P;
				K_P_sub=0.0;
				K_D=10.0;		//0.0002
				K_I=0.0;	//12
				K_G=0.3;
				first_K_G=K_G;
				
				first_speed=600.0;
				Accel=200.0;
				Accel_in=Accel+3800.0;
				Desire_vel_nomal=first_speed;
				Target_vel=1200.0;
				speed_para[0]=1000.0;
				speed_para[1]=1500.0;
				speed_para[2]=2500.0;
				break;
				
		case 3: speed=300.0;	//220
				K_P=4.0;		//13.5
				first_K_P=K_P;
				K_P_sub=0.0;
				K_D=20.0;		//0.0002
				K_I=0.0;	//12
				K_G=0.3;
				first_K_G=K_G;
				
				first_speed=700.0;
				max_ERR=80.0;
				min_ERR=-80.0;
				Accel=200.0;
				Accel_in=Accel+3800.0;
				Desire_vel_nomal=first_speed;
				Target_vel=1200.0;
				speed_para[0]=1000.0;
				speed_para[1]=1800.0;
				speed_para[2]=2300.0;
				break;
		
		case 4: speed=150.0;	//220
				K_P=4.5;		//13.5
				first_K_P=K_P;
				K_P_sub=1.00;
				K_D=3.0;		//0.0002
				K_I=0.000003;	//12
				K_G=0.9;
				first_K_G=K_G;
							
				first_speed=800.0;
				Accel=1500.0;
				Accel_in=Accel;
				Target_vel=1200.0;
				speed_para[0]=1000.0;
				speed_para[1]=1200.0;
				speed_para[2]=1800.0;
				break;
	}
	reset();
}

void select_P()
{
	int select_frag=0;
	float selecter=0.50;

	while(select_frag==0){
		calc_senc();
		if(PORTE.PORT.BIT.B0 == 0){
			TIMER_WAIT(200);
			BUZZ_ON();
			LED2=~LED2;
			selecter+=0.1;
		}
		
		if(cari_SEN[1] > 180.0){
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
		if(PORTE.PORT.BIT.B0 == 0){
			TIMER_WAIT(200);
			LED2=~LED2;
			selecter+=0.10;
		}
		
		if(cari_SEN[1] > 180.0){
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
		if(PORTE.PORT.BIT.B0 == 0){
			TIMER_WAIT(200);
			LED2=~LED2;
			selecter+=0.00000010;
		}
		
		if(cari_SEN[1] > 180.0){
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
		if(PORTE.PORT.BIT.B0 == 0){
			TIMER_WAIT(200);
			BUZZ_ON();
			LED2=~LED2;
			selecter+=1.0;
		}
		
		if(cari_SEN[1] > 180.0){
			K_D=selecter;
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
		if(PORTE.PORT.BIT.B0 == 0){
			TIMER_WAIT(200);
			LED2=~LED2;
			selecter+=0.01;
		}
		
		if(cari_SEN[1] > 180.0){
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
	float selecter=300.0;

	while(select_frag==0){
		calc_senc();
		if(PORTE.PORT.BIT.B0 == 0){
			TIMER_WAIT(200);
			LED2=~LED2;
			BUZZ_ON();
			selecter+=100.0;
		}
		
		if(cari_SEN[1] > 180.0){
			Desire_vel_nomal=selecter;
			first_speed=Desire_vel_nomal;
			select_frag=1;
			LED_manager(7);
		}
	}
	TIMER_WAIT(1000);
	reset();
}

void trace2()	//二週目
{
	short trace_roop_frag=0;

	circl_count=2;
	LED2=ON;
	TIMER_WAIT(1000);
	LED2=OFF;

//	ch_para2(0);
	ch_para();
	select_P();
//	select_I();
	select_D();
//	select_G();
//	select_goal();
//	select_corner();
	select_speed();
	
	while(trace_roop_frag==0){
		if(PORTE.PORT.BIT.B0 == 0){
			countdown();
			TIMER_WAIT(1000);
				
			zero_gyro=get_gyro();
			ENC_R=15000;
			ENC_L=15000;
			EncTimeCount=0;
			ENC_count=0;
			dis_count=0;
				
			mot_STB(START_A);
		
			start0=1;
			mot_frag=1;
			while(stop_frag==0){}
					
			stop_manager();
			trace_roop_frag=1;			
		}
		else{}
	}
}

void trace()	//一週目
{
	short trace_roop_frag=0;
	short i=0;
	circl_count=1;	
	LED1=ON;
	TIMER_WAIT(1000);
	LED1=OFF;
	
	ch_para();
	//ch_para2(0);
	select_P();
//	select_sub();
//	select_I();
	select_D();
//	select_G();
//	select_goal();
//	select_corner();
	select_speed();
	init_thre();			
	
	while(trace_roop_frag==0){	
		if(PORTE.PORT.BIT.B0 == 0){
			countdown();
			TIMER_WAIT(1000);
			for(i=0; i<201; i++){
				ENC_dis[i]=0.0;
				ENC_check[i]=0;
				RL_check[i]=3;

			}
			
			zero_gyro=get_gyro();
			ENC_R=15000;
			ENC_L=15000;
			EncTimeCount=0;
			ENC_count=0;
			dis_count=0;
			
			mot_STB(START_A);
			
			start0=1;
			mot_frag=1;
			LED1=OFF;
			while(stop_frag==0){
				if(PORTE.PORT.BIT.B0 == 0){
					stop_frag=1;
					TIMER_WAIT(500);
				}
			}
			
			stop_manager();
			trace_roop_frag=1;
		}
		else{}
	}
}

void out_sens()
{
	short i;	
	TIMER_WAIT(2000);
	get_sens_frag=1;
	
	while(1){
		for(i=1; i<9; i++){
			dec_out(ave_SEN[i], 4);	outs(" ");
		}
		dec_out(SEN[corner], 4);	outs(" ");
		dec_out(SEN[corner2], 4);	outs(" ");
		dec_out(SEN[goal], 4);	outs(" ");

		dec_out(ERR, 4);	outs(" ");
	
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
	
	for(i=0; i<201; i++){
		dec_out((short)ENC_dis[i], 6);
		outs("\n");
	}
}

int init_thre(){
	
	SEN_thre[1]     =400;
	SEN_thre[2]     =400;
	SEN_thre[3]     =400;
	SEN_thre[4]     =400;
	SEN_thre[5]     =400;
	
	SEN_thre[goal]  =3000;//max_thre*0.3;//TargetGoal*K_goal;//max_thre*0.3;//90;
	SEN_thre[corner]=1200;//max_thre*0.3;//60;
	SEN_thre[corner2]=3000;//max_thre*0.3;//60;
	
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
	
void ch_mode(int mode)
{
	reset();
	switch(mode){
		case 1: search_max();	break;
		case 2: trace();		break;
		case 3: trace2();		break;
		case 4: sens_save();	break;
		case 5: sens_load();	break;
		case 6: out_sens();		break;
		case 7: out_sens2();	break;
		case 8: out_course();	break;

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
