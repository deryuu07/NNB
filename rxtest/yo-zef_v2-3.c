#include <machine.h>
#include "iodefine.h"

#define ON 1
#define OFF 0 

#define corner 7
#define goal 8

#define START_A 1
#define STOP 0

#define ST 1
#define curve 0

#define Tred 123

#define right 1
#define left 0

#define mmpp 0.0137375

#define LED1 PORTD.DR.BIT.B1	//●○○ 
#define LED2 PORTD.DR.BIT.B2	//○●○
#define LED3 PORT9.DR.BIT.B2	//○○●

#define V_LED1  PORT1.DR.BIT.B1	//   123  456
#define V_LED2  PORT8.DR.BIT.B0	//  7        8  
#define V_LED3  PORT8.DR.BIT.B1	//	   9 10
#define V_LED4  PORT7.DR.BIT.B5
#define V_LED5  PORT7.DR.BIT.B6
#define V_LED6  PORT9.DR.BIT.B0
#define V_LED7  PORT8.DR.BIT.B2
#define V_LED8  PORT9.DR.BIT.B1
#define V_LED9  PORT1.DR.BIT.B0
#define V_LED10 PORT7.DR.BIT.B4

#define MOT_ctr_L1 PORTA.DR.BIT.B0
#define	MOT_ctr_L2 PORTA.DR.BIT.B1
#define MOT_ctr_R1 PORTA.DR.BIT.B2
#define	MOT_ctr_R2 PORTA.DR.BIT.B3
#define MOT_STBY   PORTA.DR.BIT.B4		

#define ENC_R MTU1.TCNT
#define ENC_L MTU2.TCNT

#define max_thre 300.0

double max_ERR=300.0;
double min_ERR=-300.0;

/***************カウンタ***************/
unsigned long TimeCount=0;
unsigned short TimeNow=0;
unsigned short StartTimeCount=0;
unsigned short CrossTimeCount=0;
unsigned short stop_count=0;
unsigned short curve_count=0;
unsigned short mot_count=0;
		 short circl_count=1;
		 long vel_count=0;
		 double AccelTimeCount=0.0;
		 short fail_safe_count=0;
		 short ST_curve_check=0;
		 
/***************床センサ***************/
double SEN_ON[11];
double SEN_OFF[11];
double SEN[11];
double cari_SEN[11];
double sum_SEN[11];
double MAX_SEN[11];
double MIN_SEN[11];
double ave_SEN[11];
double SEN_min[11];
double SEN_max[11];
double SEN_thre[11];
double SEN_minmax[11];
short ave_count=0.0;
double ERR=0.0;
double sub_ERR=0.0; 
double PRE_ERR=0.0;
double SUM_ERR=0.0;
short ErrTimeCount=0;
double diff_SEN[8];
double ThretSumCorner=0.0;
double ThretSumGoal=0.0;
double TargetCorner=100.0;
double TargetGoal=100.0;
//double sub_SEN_check[200];
double thre_max=90.0;

/***************ジャイロ***************/
double GYRO_ERR=0.0;
double filter_gyro=0.0;
double gyro2=0.0;

double zero_gyro=0.0;
double target_gyro=0;
double sum_gyro=0;
double ave_gyro=0;
double MAX_gyro=0;
double MIN_gyro=0;
short gyro_count=0;
double check_gyro[4];
short curve_check=0;

/***************エンコーダ*************/
long now_ENC_R=0;
long ENC_sub_R=0;
long now_ENC_L=0;
long ENC_sub_L=0;
long NowEnc=0;
long pre_ENC_R=0;
long pre_ENC_L=0;
long diff_ENC_R=0;
long diff_ENC_L=0;
long curvature=0;
long angl_vel=0;
double ENC_dis[200];
short ENC_check[200];
short EncTimeCount=0;
short ENC_count=0;
short RL_check[200];
//short vel_check[200];
//short ST_check[200];

/***************PIDゲイン**************/
double K_P=1.0;
double K_P_sub=1.0;
double K_P_add1=1.0;
double K_P_add2=2.0;
double first_K_P=0.0;
double first_sub=0.0;
double K_D=1.0;
double K_I=1.0;
double K_G=1.0;
double K_E=1.0;
double first_K_G=0.05;
double K_goal=3.0;
double K_corner=3.0;
double K_P_SPEED=0.01;
double K_I_SPEED=0.001;
double first_corner=3.0;
double speed=130.0;
double first_speed=0.0;
double first_target_speed=0.0;
double now_vel_R=0.0;
double now_vel_L=0.0;
double now_vel_ave=0.0;
double Desire_vel=0.0;
double Target_vel=0.0;
double Accel=0.0;
double Accel_in=0.0;
double SPEED_ERR_R=0.0;
double SPEED_ERR_L=0.0;
double SPEED_SUM_ERR_R=0.0;
double SPEED_SUM_ERR_L=0.0;
double SPEED_ERR=0.0;
double SPEED_SUM_ERR=0.0;

double speed_para[3];

/***************フラグ*****************/
int goal_frag1=0;
int goal_frag2=0;
int first_check=0;
int cross_frag=0;
int cross_frag2=0;
int stop_frag=0;
int start0=0;
int sci_frag=0;
int curve_frag=0;
int u_count=0;
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
short no_line_contorol_frag=0;
short max_frag=0;
short min_frag=0;
short max_const_frag=0;
short min_const_frag=0;

/*************プロトタイプ宣言**********/
void set();
void reset();
void wait();
short get_Batt(); 
double get_gyro();
void get_senc();
int calc_senc();
void mot_STB(int start_stop);
void search_max();
void stop_manager();
short goal_check(long RIGHT_ENC, long LEFT_ENC);
void TIMER_CALL();
void TIMER_WAIT(unsigned short ms);
void mot_drive(double mot_in_p, double mot_in_p_sub, double mot_in_d, double mot_in_i, double mot_in_g, double mot_in_speed, double mot_in_sum_speed);
void Battery_Check();
void countdown();
void trace();
void out_senc();
void ch_mode(int mode);
void check_curve();
void mot_brake();

void LED_manager(short led)
{
	if(led > 7)
		led-=7;
	
	switch(led){
		case 1:	LED1=ON; LED2=OFF; LED3=OFF;	break;
		case 2:	LED1=OFF; LED2=ON; LED3=OFF;	break;
		case 3:	LED1=ON; LED2=ON; LED3=OFF;		break;
		case 4:	LED1=OFF; LED2=OFF; LED3=ON;	break;
		case 5:	LED1=ON; LED2=OFF; LED3=ON;		break;
		case 6:	LED1=OFF; LED2=ON; LED3=ON;		break;
		case 7:	LED1=ON; LED2=ON; LED3=ON;		break;
	}

}

void set()	//各種機能初期設定
{
	PORTD.DDR.BYTE = 0x06;
	PORTA.DDR.BYTE = 0x1f;
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
    CMT1.CMCOR = 3000;       	//1ms周期で割り込み
	ICU.IER[0x03].BIT.IEN5=1;	//割り込み要求許可
	ICU.IPR[0x05].BIT.IPR=15;	//割り込み優先レベル変更
    CMT.CMSTR0.BIT.STR1 = 1; 	//CMTカウントスタート
	
	MSTP_AD0=0;					//10bitAD変換 モジュールストップ状態の解除
  	AD0.ADCSR.BIT.ADST  = 0;	// AD変換停止
  	AD0.ADCR.BIT.MODE   = 0;	// AD変換モード選択 シングルモード
  	AD0.ADCSR.BIT.ADIE  = 0;	// 割込み禁止
  	AD0.ADCR.BIT.CKS    = 0;	// 周辺動作クロック φ/8
  	AD0.ADDPR.BIT.DPPRC = 0;	//10bitで変換
	AD0.ADDPR.BIT.DPSEL = 1;	//MSB詰め
		
	SYSTEM.MSTPCRA.BIT.MSTPA9=0;
	MTU.TSTRA.BIT.CST0=0;		//カウント停止
	MTU0.TCR.BIT.CCLR=2;
	MTU0.TCR.BIT.TPSC=0;		//PWM周期を96kHzに設定
	MTU0.TMDR.BIT.MD=3;
	MTU0.TIORH.BIT.IOA=5;
	MTU0.TIORL.BIT.IOC=5;		
	MTU0.TGRB=1000;
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

double get_gyro()	//ジャイロ値取得
{
	double gyro=0.0;
	
	AD0.ADCSR.BIT.CH=2;				//AD変換するアナログ入力チャンネルを設定 ジャイロ AD0.ADDRC
	AD0.ADCSR.BIT.ADST = 1;			// AD変換開始
	while(AD0.ADCSR.BIT.ADST==1);	//AD変換終了まで待機
	AD0.ADCSR.BIT.ADST  = 0;		// AD停止
	gyro=AD0.ADDRC>>6;
	
	filter_gyro=gyro;	
	return filter_gyro;
}

void get_senc()
{
	int count, i;
	
	/*************************LEDがONの時のセンサ値取得***********************/
	V_LED1=ON;
	V_LED2=ON;
	V_LED3=ON;	
	V_LED4=ON;
	V_LED5=ON;
	V_LED6=ON;	
	V_LED7=ON;
	V_LED8=ON;
	V_LED9=ON;	
	V_LED10=ON;
	
	for(count=0; count<801; count++){}	//LEDが点灯しきるまで少し待機
		
	AD0.ADCSR.BIT.CH=8;				//AD変換するアナログ入力チャンネルを設定  AD0.ADDRI
	AD0.ADCSR.BIT.ADST = 1;			// AD変換開始
	while(AD0.ADCSR.BIT.ADST==1);	//AD変換終了まで待機
	AD0.ADCSR.BIT.ADST  = 0;		// AD停止
	SEN_ON[1]=AD0.ADDRI;
	
	AD0.ADCSR.BIT.CH=9;				//AD変換するアナログ入力チャンネルを設定   AD0.ADDRJ
	AD0.ADCSR.BIT.ADST = 1;			// AD変換開始
	while(AD0.ADCSR.BIT.ADST==1);	//AD変換終了まで待機
	AD0.ADCSR.BIT.ADST  = 0;		// AD停止
	SEN_ON[2]=AD0.ADDRJ;
	
	AD0.ADCSR.BIT.CH=10;				//AD変換するアナログ入力チャンネルを設定  AD0.ADDRK
	AD0.ADCSR.BIT.ADST = 1;			// AD変換開始
	while(AD0.ADCSR.BIT.ADST==1);	//AD変換終了まで待機
	AD0.ADCSR.BIT.ADST  = 0;		// AD停止	
	SEN_ON[3]=AD0.ADDRK;
	
	AD0.ADCSR.BIT.CH=11;				//AD変換するアナログ入力チャンネルを設定  AD0.ADDRL
	AD0.ADCSR.BIT.ADST = 1;			// AD変換開始
	while(AD0.ADCSR.BIT.ADST==1);	//AD変換終了まで待機
	AD0.ADCSR.BIT.ADST  = 0;		// AD停止
	SEN_ON[4]=AD0.ADDRL;
	
	AD0.ADCSR.BIT.CH=0;				//AD変換するアナログ入力チャンネルを設定  AD0.ADDRA
	AD0.ADCSR.BIT.ADST = 1;			// AD変換開始
	while(AD0.ADCSR.BIT.ADST==1);	//AD変換終了まで待機
	AD0.ADCSR.BIT.ADST  = 0;		// AD停止
	SEN_ON[5]=AD0.ADDRA;

	AD0.ADCSR.BIT.CH=4;				//AD変換するアナログ入力チャンネルを設定  AD0.ADDRE
	AD0.ADCSR.BIT.ADST = 1;			// AD変換開始
	while(AD0.ADCSR.BIT.ADST==1);	//AD変換終了まで待機
	AD0.ADCSR.BIT.ADST  = 0;		// AD停止
	SEN_ON[6]=AD0.ADDRE;
			
	AD0.ADCSR.BIT.CH=7;				//AD変換するアナログ入力チャンネルを設定 AD0.ADDRH
	AD0.ADCSR.BIT.ADST = 1;			// AD変換開始
	while(AD0.ADCSR.BIT.ADST==1);	//AD変換終了まで待機
	AD0.ADCSR.BIT.ADST  = 0;		// AD停止
	SEN_ON[corner]=AD0.ADDRH;
	
	AD0.ADCSR.BIT.CH=5;				//AD変換するアナログ入力チャンネルを設定  AD0.ADDRF
	AD0.ADCSR.BIT.ADST = 1;			// AD変換開始
	while(AD0.ADCSR.BIT.ADST==1);	//AD変換終了まで待機
	AD0.ADCSR.BIT.ADST  = 0;		// AD停止
	SEN_ON[goal]=AD0.ADDRF;
	
	AD0.ADCSR.BIT.CH=1;				//AD変換するアナログ入力チャンネルを設定  AD0.ADDRB
	AD0.ADCSR.BIT.ADST = 1;			// AD変換開始
	while(AD0.ADCSR.BIT.ADST==1);	//AD変換終了まで待機
	AD0.ADCSR.BIT.ADST  = 0;		// AD停止
	SEN_ON[9]=AD0.ADDRB;
		
	AD0.ADCSR.BIT.CH=3;				//AD変換するアナログ入力チャンネルを設定  AD0.ADDRD
	AD0.ADCSR.BIT.ADST = 1;			// AD変換開始
	while(AD0.ADCSR.BIT.ADST==1);	//AD変換終了まで待機
	AD0.ADCSR.BIT.ADST  = 0;		// AD停止
	SEN_ON[10]=AD0.ADDRD;

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
	
	for(count=0; count<801; count++){}

	/*****************************LEDがOFFの時のセンサ値取得**************************/

	AD0.ADCSR.BIT.CH=8;				//AD変換するアナログ入力チャンネルを設定  AD0.ADDRI
	AD0.ADCSR.BIT.ADST = 1;			// AD変換開始
	while(AD0.ADCSR.BIT.ADST==1);	//AD変換終了まで待機
	AD0.ADCSR.BIT.ADST  = 0;		// AD停止
	SEN_OFF[1]=AD0.ADDRI;
	
	AD0.ADCSR.BIT.CH=9;				//AD変換するアナログ入力チャンネルを設定   AD0.ADDRJ
	AD0.ADCSR.BIT.ADST = 1;			// AD変換開始
	while(AD0.ADCSR.BIT.ADST==1);	//AD変換終了まで待機
	AD0.ADCSR.BIT.ADST  = 0;		// AD停止
	SEN_OFF[2]=AD0.ADDRJ;
	
	AD0.ADCSR.BIT.CH=10;				//AD変換するアナログ入力チャンネルを設定  AD0.ADDRK
	AD0.ADCSR.BIT.ADST = 1;			// AD変換開始
	while(AD0.ADCSR.BIT.ADST==1);	//AD変換終了まで待機
	AD0.ADCSR.BIT.ADST  = 0;		// AD停止	
	SEN_OFF[3]=AD0.ADDRK;
	
	AD0.ADCSR.BIT.CH=11;				//AD変換するアナログ入力チャンネルを設定  AD0.ADDRL
	AD0.ADCSR.BIT.ADST = 1;			// AD変換開始
	while(AD0.ADCSR.BIT.ADST==1);	//AD変換終了まで待機
	AD0.ADCSR.BIT.ADST  = 0;		// AD停止
	SEN_OFF[4]=AD0.ADDRL;
	
	AD0.ADCSR.BIT.CH=0;				//AD変換するアナログ入力チャンネルを設定  AD0.ADDRA
	AD0.ADCSR.BIT.ADST = 1;			// AD変換開始
	while(AD0.ADCSR.BIT.ADST==1);	//AD変換終了まで待機
	AD0.ADCSR.BIT.ADST  = 0;		// AD停止
	SEN_OFF[5]=AD0.ADDRA;

	AD0.ADCSR.BIT.CH=4;				//AD変換するアナログ入力チャンネルを設定  AD0.ADDRE
	AD0.ADCSR.BIT.ADST = 1;			// AD変換開始
	while(AD0.ADCSR.BIT.ADST==1);	//AD変換終了まで待機
	AD0.ADCSR.BIT.ADST  = 0;		// AD停止
	SEN_OFF[6]=AD0.ADDRE;
			
	AD0.ADCSR.BIT.CH=7;				//AD変換するアナログ入力チャンネルを設定 AD0.ADDRH
	AD0.ADCSR.BIT.ADST = 1;			// AD変換開始
	while(AD0.ADCSR.BIT.ADST==1);	//AD変換終了まで待機
	AD0.ADCSR.BIT.ADST  = 0;		// AD停止
	SEN_OFF[corner]=AD0.ADDRH;
	
	AD0.ADCSR.BIT.CH=5;				//AD変換するアナログ入力チャンネルを設定  AD0.ADDRF
	AD0.ADCSR.BIT.ADST = 1;			// AD変換開始
	while(AD0.ADCSR.BIT.ADST==1);	//AD変換終了まで待機
	AD0.ADCSR.BIT.ADST  = 0;		// AD停止
	SEN_OFF[goal]=AD0.ADDRF;
	
	AD0.ADCSR.BIT.CH=1;				//AD変換するアナログ入力チャンネルを設定  AD0.ADDRB
	AD0.ADCSR.BIT.ADST = 1;			// AD変換開始
	while(AD0.ADCSR.BIT.ADST==1);	//AD変換終了まで待機
	AD0.ADCSR.BIT.ADST  = 0;		// AD停止
	SEN_OFF[9]=AD0.ADDRB;
		
	AD0.ADCSR.BIT.CH=3;				//AD変換するアナログ入力チャンネルを設定  AD0.ADDRD
	AD0.ADCSR.BIT.ADST = 1;			// AD変換開始
	while(AD0.ADCSR.BIT.ADST==1);	//AD変換終了まで待機
	AD0.ADCSR.BIT.ADST  = 0;		// AD停止
	SEN_OFF[10]=AD0.ADDRD;
/************************************************************/

	for(i=1; i<11; i++){
		SEN[i]=SEN_ON[i]-SEN_OFF[i];
		if(SEN[i]<0.0)
			SEN[i]=0.0;
	}
}

int calc_senc()
{
	int i;
	double gyro=0.0;
	double max=0.0;
	double max_main=0.0;
	
	get_senc();
	for(i=1; i<11; i++){
		cari_SEN[i] = 300.0*(SEN[i]-SEN_min[i])/SEN_minmax[i];	
		sum_SEN[i] += cari_SEN[i];
		
		if(MAX_SEN[i] < cari_SEN[i])
			MAX_SEN[i] = cari_SEN[i];
		
		if(MIN_SEN[i] > cari_SEN[i])
			MIN_SEN[i] = cari_SEN[i];
	}
	ave_count++;
	if(ave_count==5){
		sub_ERR=0.0;
		
		for(i=1; i<11; i++){
			ave_SEN[i]=(sum_SEN[i]-MAX_SEN[i]-MIN_SEN[i])/3.0;
			sum_SEN[i]=0.0;
			MAX_SEN[i]=0.0;
			MIN_SEN[i]=0.0;
		}
		ave_count=0;
	}	
	if(start0==1){
		gyro=get_gyro();
		gyro2=gyro-zero_gyro;

		if(gyro_first==0){			//基準値取得
			target_gyro=gyro2;
			MAX_gyro=gyro2;
			MIN_gyro=gyro2;
			gyro_first++;
		}

		sum_gyro += gyro2;
		if(MAX_gyro < gyro2)
			MAX_gyro = gyro2;
		
		if(MIN_gyro > gyro2)
			MIN_gyro = gyro2;
		
		gyro_count++;
		if(gyro_count==5){
			ave_gyro=(sum_gyro-MAX_gyro-MIN_gyro)/3.0;
			GYRO_ERR=target_gyro-ave_gyro;
		
			sum_gyro=0.0;
			MIN_gyro=MAX_gyro;
			MAX_gyro=0.0;
			gyro_count=0;
		}
	
		diff_SEN[0]=ave_SEN[3]-ave_SEN[4];
		diff_SEN[1]=ave_SEN[2]-ave_SEN[5];
		diff_SEN[2]=ave_SEN[1]-ave_SEN[6];
		diff_SEN[3]=ave_SEN[9]-ave_SEN[10];	
				
		if(ErrTimeCount < 200){
			ThretSumCorner+=ave_SEN[corner];
			ThretSumGoal+=ave_SEN[goal];
			ErrTimeCount++;
		}
		if(ErrTimeCount==200){
			TargetGoal=ThretSumGoal/200.0;
			TargetCorner=ThretSumCorner/200.0;
			init_thre();
			ErrTimeCount=201;
		}
		
		max_main=ave_SEN[3];
		max=ave_SEN[3];
		
		for(i=1; i<7; i++){
			if(max_main < ave_SEN[i])
				max_main=ave_SEN[i];
			
			if(max < ave_SEN[i])
				max=ave_SEN[i];
		}	
		
		if((ave_SEN[1] > thre_max) && (ave_SEN[1] < thre_max && ave_SEN[3] < thre_max && ave_SEN[4] < thre_max && ave_SEN[5] < thre_max && ave_SEN[6] < thre_max)){
			LED1=ON;
			max_frag=1;
		}
		if(ave_SEN[2] > thre_max || ave_SEN[3] > thre_max || ave_SEN[4] > thre_max || ave_SEN[5] > thre_max || ave_SEN[6] > thre_max)
			max_frag=0;
		
		if((ave_SEN[1] < thre_max && ave_SEN[2] < thre_max && ave_SEN[3] < thre_max && ave_SEN[4] < thre_max && ave_SEN[5] < thre_max) && (ave_SEN[6] > thre_max)){
			LED1=ON;
			min_frag=1;
		}
		if(ave_SEN[1] > thre_max || ave_SEN[2] > thre_max || ave_SEN[3] > thre_max || ave_SEN[4] > thre_max || ave_SEN[5] > thre_max)
			min_frag=0;
		
		PRE_ERR=ERR;					
		
		if((max==ave_SEN[3] || max==ave_SEN[4]) || ((ave_SEN[1] < thre_max) && (ave_SEN[2] < thre_max) && (ave_SEN[5] < thre_max) && (ave_SEN[6] < thre_max))){
			ERR=ave_SEN[3]-ave_SEN[4];	//○○● ●○○
			LED1=OFF;
			K_P=first_K_P;
		}

		else if((max==ave_SEN[2]) && (ave_SEN[2] > thre_max) && goal_frag2==1){
			ERR=(ave_SEN[2]-ave_SEN[4]);	//○●○ ●○○
			LED1=OFF;
			K_P=first_K_P*2.0;
		}
		else if((max==ave_SEN[5]) && (ave_SEN[5] > thre_max) && goal_frag2==1){
			ERR=(ave_SEN[3]-ave_SEN[5]);	//○○● ○●○
			LED1=OFF;
			K_P=first_K_P*2.0;
		}			
		
//		ERR=ave_SEN[3]-ave_SEN[4];	//○○● ●○○

		if(max_const_frag==1 || (goal_frag2==1 && max_frag==1 && (ave_SEN[1] < thre_max && ave_SEN[2] < thre_max && ave_SEN[3] < thre_max && ave_SEN[4] < thre_max && ave_SEN[5] < thre_max && ave_SEN[6] < thre_max))){
			ERR=max_ERR;
			max_const_frag=1;
		}
				
		if(min_const_frag==1 || (goal_frag2==1 && min_frag==1 && (ave_SEN[1] < thre_max && ave_SEN[2] < thre_max && ave_SEN[3] < thre_max && ave_SEN[4] < thre_max && ave_SEN[5] < thre_max && ave_SEN[6] < thre_max))){
			ERR=min_ERR;
			min_const_frag=1;
		}
		
		if(ave_SEN[4] > thre_max){
			LED1=OFF;
			max_const_frag=0;
		}
		if(ave_SEN[3] > thre_max){
			LED1=OFF;
			min_const_frag=0;
		}
				
		//ERR=diff_SEN[0];		
		SUM_ERR+=ERR;
		sub_ERR=ave_SEN[9]-ave_SEN[10];					
/*
		if(goal_frag2==1 && ave_SEN[10] > thre_max && (ave_SEN[5] > thre_max || ave_SEN[6] > thre_max) && (ave_SEN[3] < thre_max)){
			LED2=ON;
			K_P_sub=first_sub*10.0;
		}		
		if(ave_SEN[3] > thre_max){
			LED2=OFF;
			K_P_sub=first_K_P;
		}
		
		if(goal_frag2==1 && ave_SEN[9] > thre_max && (ave_SEN[1] > thre_max || ave_SEN[2] > thre_max) && (ave_SEN[4] < thre_max)){
			LED2=ON;
			K_P_sub=first_sub*10.0;
		}		
		if(ave_SEN[4] > thre_max){
			LED2=OFF;
			K_P_sub=first_K_P;
		}
*/
	}
	return 0;
}

void check_curve()		//カーブ判定処理
{
	int j;
		
	check_gyro[u_count]=GYRO_ERR;
	u_count++;
	if(u_count==5){
		for(j=0; j<6; j++){
			if(check_gyro[j] > 140.0 || check_gyro[j] < -140.0){
				curve_check++;
				check_gyro[j]=0.0;
			}
		}
		if(curve_one==0){
			if(curve_check>=4){				//カーブ時
				curve_one=1;
			}
			else{							//直線時
			}
		}
		u_count=0;
		curve_check=0;
	}
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
	
	for(i=1; i<11; i++){
		SEN_max[i]=1.0;
		SEN_min[i]=10.0;
	}
	cari_frag=1;
	while(ENC_R < 10000){
		MOT_ctr_R1=1;
		MOT_ctr_R2=0;
		MOT_ctr_L1=0;
		MOT_ctr_L2=1;
		MTU0.TGRC=100 - (0.2*(ENC_R-ENC_L));
		MTU0.TGRA=100 + (0.2*(ENC_R-ENC_L));

		for(i=1; i<11; i++){
			if(SEN_max[i] < SEN[i])
				SEN_max[i]=SEN[i];
			if(SEN[i] > 0.0 && SEN_min[i] > SEN[i])
				SEN_min[i]=SEN[i];
		}	
	}
	cari_frag=0;
	mot_brake();
	ENC_R=40000;
	ENC_L=40000;
	TIMER_WAIT(1000);
	
	MOT_ctr_R1=0;
	MOT_ctr_R2=1;
	MOT_ctr_L1=1;
	MOT_ctr_L2=0;
	cari_frag=1;
	while((40000-ENC_R) < 10000){
		MTU0.TGRC=100 - (0.0001*(ENC_R-ENC_L));
		MTU0.TGRA=100 + (0.0001*(ENC_R-ENC_L));

		for(i=1; i<11; i++){
			if(SEN_max[i] < SEN[i])
				SEN_max[i]=SEN[i];
			if(SEN[i] > 0.0 && SEN_min[i] > SEN[i])
				SEN_min[i]=SEN[i];
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
	no_line_contorol_frag=0;
	fail_safe_count=0;
	ST_curve_check=0;
	first_target_frag2=0;
	first_target_speed=0;
	max_frag=0;
	min_frag=0;
	max_const_frag=0;
    min_const_frag=0;

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
		mot_drive(ERR, sub_ERR, PRE_ERR, SUM_ERR, GYRO_ERR, SPEED_ERR, SPEED_SUM_ERR);
	}

	init_goal_frag();
	mot_brake();
	TIMER_WAIT(1000);
	mot_STB(STOP);
}	

short Accel_manager(short counter, double N_E_R, double N_E_L)
{	
	short StCurveCheck=curve;

#if 1
	if(ENC_dis[counter] > 300.0){		
		if(ENC_check[counter] < 50){	//加速しない
			LED1=ON;
			LED2=OFF;
			LED3=OFF;
			K_P=first_K_P;
		}
		else if(ENC_check[counter] > 50){							//加速する
			StCurveCheck=ST;
			K_P=first_K_P*1.5;
			LED1=OFF;
			LED2=ON;
			LED3=OFF;
		
			if(ENC_dis[counter] < 400.0){
				if(RL_check[counter]==right && N_E_R<=(ENC_dis[counter] - 150.0)){
					Target_vel=speed_para[0];
					K_D=0.00001;
					K_G=2.0;
				}
				else if(RL_check[counter]==right && N_E_R>(ENC_dis[counter] - 150.0)){
					if(first_target_frag==0)
						Desire_vel=Target_vel;
					
					first_target_frag=1;
					Target_vel=first_speed;
					Accel=(-1.0)*(Accel_in+3000.0);
				}
				
				if(RL_check[counter]==left && N_E_L<=(ENC_dis[counter] - 150.0)){
					Target_vel=speed_para[0];
					K_D=0.00001;
					K_G=2.0;
				}
				else if(RL_check[counter]==left && N_E_L>(ENC_dis[counter] - 150.0)){
					if(first_target_frag==0)
						Desire_vel=Target_vel;
					
					first_target_frag=1;
					Target_vel=first_speed;
					Accel=(-1.0)*(Accel_in+3000.0);
				}
			}

			if(ENC_dis[counter] > 400.0 && ENC_dis[counter] < 1020.0){
				if(RL_check[counter]==right && N_E_R<(ENC_dis[counter] - 250.0)){
					Target_vel=speed_para[1];
					K_D=0.000001;
					K_G=2.0;
				}
				else if(RL_check[counter]==right && N_E_R>(ENC_dis[counter] - 250.0)){
					if(first_target_frag==0)
						Desire_vel=Target_vel;
					
					first_target_frag=1;
					Target_vel=first_speed;
					Accel=(-1.0)*(Accel_in+4000.0);
				}

				if(RL_check[counter]==left && N_E_L<(ENC_dis[counter] - 250.0)){
					Target_vel=speed_para[1];
					K_D=0.000001;
					K_G=2.0;
				}		
				else if(RL_check[counter]==left && N_E_L>(ENC_dis[counter] - 250.0)){
					if(first_target_frag==0)
						Desire_vel=Target_vel;
					
					first_target_frag=1;
					Target_vel=first_speed;
					Accel=(-1.0)*(Accel_in+4000.0);
				}
			}

			if(ENC_dis[counter] > 1020.0){
				if(RL_check[counter]==right && N_E_R<(ENC_dis[counter] - 510.0)){
					Target_vel=speed_para[2];
					K_D=0.000001;
					K_G=2.0;
				}
				else if(RL_check[counter]==right && N_E_R>(ENC_dis[counter] - 510.0)){
					if(first_target_frag==0)
						Desire_vel=Target_vel;
					
					first_target_frag=1;
					Target_vel=first_speed;
					Accel=(-1.0)*(Accel_in+5000.0);
				}

				if(RL_check[counter]==left && N_E_L<(ENC_dis[counter] - 510.0)){
					Target_vel=speed_para[2];
					K_D=0.000001;
					K_G=2.0;
				}
				else if(RL_check[counter]==left && N_E_L>(ENC_dis[counter] - 510.0)){
					if(first_target_frag==0)
						Desire_vel=Target_vel;
					
					first_target_frag=1;
					Target_vel=first_speed;
					Accel=(-1.0)*(Accel_in+5000.0);
				}
			}
		}
	}
	else if(ENC_dis[counter] < 180.0){								//加速しない カーブでは速度落とすかも
		LED1=OFF;
		LED2=OFF;
		LED3=ON;
	}
#endif
#if 0
	if(ENC_dis[counter] > 180.0){		
		if(ENC_check[counter] < 50){	//加速しない
			LED1=ON;
			LED2=OFF;
			LED3=OFF;
		}
		else if(ENC_check[counter] > 50){
			LED1=OFF;
			LED2=ON;
			LED3=OFF;
		}
	}			
	else if(ENC_dis[counter] < 180.0){								//加速しない カーブでは速度落とすかも
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
		}
	}
	if(first_check==1){
		StartTimeCount++;
		if(StartTimeCount==100){
			first_check=2;
			now_ENC_L=0;
			now_ENC_R=0;
		}
	}
			
	if(first_check==2){		
		if(ave_SEN[2] > SEN_thre[2] && ave_SEN[5] > SEN_thre[5] &&goal_frag2==1){
			goal_frag2=0;
		}

		if(goal_frag2==0){
			CrossTimeCount++;
			no_line_contorol_frag=1;
			LED2=ON;
		}
			
		if(CrossTimeCount==150){	
			no_line_contorol_frag=0;
			goal_frag2=1;
			CrossTimeCount=0;
			LED2=OFF;
		}
	
	/********************コーナー処理*******************************/	
		if(circl_count==1){	
			if((ave_SEN[corner] > SEN_thre[corner]) && goal_frag2==1 && curve_frag==0){
				K_corner=first_corner;
				LED3=ON;
				curve_frag=1;

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
					ENC_dis[ENC_count]=(double)RIGHT_ENC*mmpp;
				}
				else if(RIGHT_ENC < LEFT_ENC){
					RL_check[ENC_count]=left;
					ENC_dis[ENC_count]=(double)LEFT_ENC*mmpp;
				}
				
//				ST_check[ENC_count]=ST_curve_check;
				ST_curve_check=0;	
				ENC_count++;
				now_ENC_R=0;
				now_ENC_L=0;
			}

			if(curve_frag==1)	//複数回認識することへの対策
				curve_count++;
			if(curve_frag==1 && curve_count==40){
				LED3=OFF;
				curve_frag=0;
				curve_count=0;
			}
		}
		
		else if(circl_count==2 || circl_count==3){
			if(RL_check[ENC_count]==right){
				if((((double)RIGHT_ENC*mmpp) > (ENC_dis[ENC_count] - 80.0) && (ave_SEN[corner] > SEN_thre[corner]))){
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
					SPEED_SUM_ERR_R=0.0;					
					SPEED_ERR_L=0.0;
					SPEED_SUM_ERR_L=0.0;					
					K_D=0.0;
					K_G=first_K_G;
				}
			}
			else if(RL_check[ENC_count]==left){
				if((((double)LEFT_ENC*mmpp) > (ENC_dis[ENC_count] - 80.0) && (ave_SEN[corner] > SEN_thre[corner]))){
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
					SPEED_SUM_ERR_R=0.0;					
					SPEED_ERR_L=0.0;
					SPEED_SUM_ERR_L=0.0;
					K_D=0.0;
					K_G=first_K_G;
				}
			}
			
			StCurveCheck=Accel_manager(ENC_count, (double)RIGHT_ENC*mmpp, (double)LEFT_ENC*mmpp);	
		}
	/*********************ゴール処理*******************/				
	
		if((ave_SEN[goal] > SEN_thre[goal]) && goal_frag2==1 && Dispermit_goal!=1){
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
					ENC_dis[ENC_count]=(double)RIGHT_ENC*mmpp;
				}
				else if(RIGHT_ENC < LEFT_ENC){
					RL_check[ENC_count]=left;
					ENC_dis[ENC_count]=(double)LEFT_ENC*mmpp;
				}
	//			ST_check[ENC_count]=ST_curve_check;	
			
			}
			LED3=OFF;
			stop_frag=1;
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
	
	if(start0==1){				
		now_ENC_R+=(ENC_R-15000);
		now_ENC_L+=(ENC_L-15000);
		NowEnc=now_ENC_R-now_ENC_L;
		
		now_vel_R=(double)(ENC_R-15000)*13.7375;	// mm/s
		now_vel_L=(double)(ENC_L-15000)*13.7375;	// mm/s			
		now_vel_ave=(now_vel_R+now_vel_L)/2.0;
		
		if(Accel > 0.0){
			if(Desire_vel < Target_vel){
				AccelTimeCount+=0.001;
				Desire_vel=(Accel*AccelTimeCount);
			}
			else if(Desire_vel >= Target_vel ){
				Desire_vel=Target_vel;
			}
		}
		else if(Accel < 0.0){
			if(Desire_vel > Target_vel){
				AccelTimeCount+=0.001;
				Desire_vel=Desire_vel+(Accel*AccelTimeCount);
			}
			else if(Desire_vel < Target_vel ){
				Desire_vel=Target_vel;
			}	
		}
				
		SPEED_ERR=now_vel_ave-Desire_vel;
		SPEED_SUM_ERR+=SPEED_ERR;		
					
		EncTimeCount++;
		if(EncTimeCount==50){
			EncTimeCount=0;

			angl_vel=now_ENC_R-now_ENC_L;		
			if(angl_vel >= 0){
				if(angl_vel == 0)
					angl_vel=1;
				curvature=(now_ENC_R+now_ENC_L)*Tred/(2*angl_vel);
				if(curvature<0)
					curvature*=(-1);
			}	
			else if(angl_vel < 0){
				angl_vel*=(-1);
				curvature=(now_ENC_R+now_ENC_L)*Tred/(2*angl_vel);
				if(curvature<0)
					curvature*=(-1);
			}
				
			if(curvature/10 < 40){
				if(circl_count==1){
					K_corner*=0.8;
				}
			}
			else if(curvature/10 > 40){}
		}	
		
		calc_senc();					
		StCurveCheck2=goal_check(now_ENC_R, now_ENC_L);
		
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
						AccelTimeCount+=0.001;
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
			mot_drive(ERR, sub_ERR, PRE_ERR, SUM_ERR, GYRO_ERR, SPEED_ERR, SPEED_SUM_ERR);
		}
		ENC_R=15000;
		ENC_L=15000;
	}

	if(stop_frag==1)
		stop_count++;
	
}

void TIMER_CALL2()
{
	if(cari_frag==1)
		get_senc();		
}

void TIMER_WAIT(unsigned short ms)
{
	unsigned long TimeNow = TimeCount;
	while((TimeCount-TimeNow) <= ms );
}

void mot_drive(double mot_in_p, double mot_in_p_sub, 
				double mot_in_d, double mot_in_i, double mot_in_g,
				double mot_in_speed, double mot_in_sum_speed)
{
	double MOT_IN_R=speed;
	double MOT_IN_L=speed;
	
	MOT_IN_R+=(K_P*mot_in_p);
	MOT_IN_R+=K_P_sub*mot_in_p_sub;
	MOT_IN_R+=(K_I*mot_in_i);
	MOT_IN_R+=K_D*mot_in_d;
	MOT_IN_R+=K_G*mot_in_g;
	MOT_IN_R-=K_P_SPEED*mot_in_speed;
	MOT_IN_R-=K_I_SPEED*mot_in_sum_speed;
		
	MOT_IN_L-=(K_P*mot_in_p);
	MOT_IN_L-=K_P_sub*mot_in_p_sub;
	MOT_IN_L-=(K_I*mot_in_i);
	MOT_IN_L-=K_D*mot_in_d;
	MOT_IN_L-=K_G*mot_in_g;
	MOT_IN_L-=K_P_SPEED*mot_in_speed;
	MOT_IN_L-=K_I_SPEED*mot_in_sum_speed;

	if(MOT_IN_R > 0.0){
		MOT_ctr_R1=1;
		MOT_ctr_R2=0;			
		if(MOT_IN_R > 1000)
			MOT_IN_R=999;
		MTU0.TGRC=MOT_IN_R;
	}
	else if(MOT_IN_R < 0.0){
		MOT_IN_R*=(-1.0);
		MOT_ctr_R1=0;
		MOT_ctr_R2=1;
		if(MOT_IN_R > 1000)
			MOT_IN_R=999;
		MTU0.TGRC=MOT_IN_R;
	}

	if(MOT_IN_L > 0.0){	
		MOT_ctr_L1=0;
		MOT_ctr_L2=1;
		if(MOT_IN_L > 1000)
			MOT_IN_L=999;
		MTU0.TGRA=MOT_IN_L;
	}
	else if(MOT_IN_L < 0.0){	
		MOT_IN_L*=(-1.0);
		MOT_ctr_L1=1;
		MOT_ctr_L2=0;
		if(MOT_IN_L > 1000)
			MOT_IN_L=999;
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
	if(battery < 757 && battery >= 737){
		LED_manager(3);	
	}
	if(battery < 737){								//7.4V未満になったら未満強制停止
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

void out_curve()	//ログチェック
{
	short i=0;
	
	TIMER_WAIT(2000);
	dec_out(ST_count, 8); outs("\n");

	for(i=0; i<201; i++){	
	//	dec_out(ST_check[i], 8); outs(" ");
//		dec_out((short)sub_SEN_check[i], 8); outs(" ");
	//	dec_out(vel_check[i], 8); outs(" ");
		dec_out(ENC_check[i], 8); outs(" ");
		dec_out((short)ENC_dis[i], 8); outs("\n");
	}
}

void speed_check()
{
	int i=0;

	while(1){
		if(PORTE.PORT.BIT.B0 == 0){
			countdown();
			ENC_R=0;
			ENC_sub_R=0;
			pre_ENC_R=0;
			now_ENC_R=0;
			
			ENC_L=0;
			ENC_sub_L=0;
			pre_ENC_L=0;
			now_ENC_L=0;
			
			mot_STB(START_A);
	
			MOT_ctr_R1=1;
			MOT_ctr_R2=0;
			MOT_ctr_L1=0;
			MOT_ctr_L2=1;
			speed=100.0;
			speed_check_frag=1;
			while(1){
				if(mot_count>=500)
					break;
			}
			mot_brake();
			while(1){
				if(PORTE.PORT.BIT.B0 == 0){
					while(1){
						for(i=0; i<1001; i++){
						}
					}
				}
				else{}
			}
		}
		else{}
	}
}		

void out_ENC()
{
	TIMER_WAIT(1000);
	ENC_R=0;	//max:65535
	ENC_L=0;
	start0=1;
	
	while(1){
		dec_out(now_ENC_R, 8); outs(" ");
		dec_out(now_ENC_L, 8); outs(" ");
		dec_out((now_ENC_R-now_ENC_L)/6400, 8); outs(" ");
		outs("\n");
		
	}

}

void ch_para()
{
	short para_frag=0;
 	short para_count=0;

 	ENC_R=1600;
	
 	while(para_frag==0){
		if(ENC_R > 1600*2 && ENC_R < 1600*3){
			LED_manager(1);
			para_count=0;
		}
		if(ENC_R > 1600*3 && ENC_R < 1600*4){
			LED_manager(2);
			para_count=1;
		}
		if(ENC_R > 1600*4 && ENC_R < 1600*5){
			LED_manager(3);
			para_count=2;
		}
		if(ENC_R > 1600*5 && ENC_R < 1600*6){
			LED_manager(4);
			para_count=3;
		}
		if(ENC_R > 1600*6)
			ENC_R=(1600*2)+1;
			
		if(PORTE.PORT.BIT.B0 == 0){
			TIMER_WAIT(500);
			reset();
			para_frag=1;
		}
	}
	
	switch(para_count){
		case 0: speed=200.0;	//220
				K_P=8.5;		//13.5
				first_K_P=K_P;
				K_P_sub=1.00;
				K_P_add1=9.0;
				K_P_add2=12.0;
				K_D=0.0;		//0.0002
				K_I=0.000003;	//12
				K_G=0.9;
				first_K_G=K_G;
				K_E=0.00001;
				K_goal=6.5;
				K_corner=2.8;
				
				first_speed=800.0;
				Accel=1500.0;
				Accel_in=Accel;
				Target_vel=1200.0;
				speed_para[0]=1000.0;
				speed_para[1]=1200.0;
				speed_para[2]=1800.0;
				break;
		
		case 1: speed=230.0;	//220
				K_P=8.5;		//13.5
				first_K_P=K_P;
				K_P_sub=1.00;
				K_P_add1=9.0;
				K_P_add2=12.0;
				K_D=0.0;		//0.0002
				K_I=0.000003;	//12
				K_G=0.9;
				first_K_G=K_G;
				K_E=0.00001;
				K_goal=6.5;
				K_corner=3.0;
				
				first_speed=900.0;
				Accel=2000.0;
				Accel_in=Accel;
				Target_vel=2000.0;
				speed_para[0]=1200.0;
				speed_para[1]=1500.0;
				speed_para[2]=2000.0;
				break;
				
		case 2: speed=200.0;	
				K_P=1.5;		
				first_K_P=K_P;
				K_P_sub=0.00;
				K_P_add1=9.0;
				K_P_add2=12.0;
				K_D=1.0;		
				K_I=0.0;	
				K_G=0.0;
				first_K_G=K_G;
				K_E=0.00001;
				K_goal=0.25;
				K_corner=2.8;
				
				first_speed=300.0;
				Accel=1500.0;
				Accel_in=Accel;
				Target_vel=1200.0;
				speed_para[0]=1000.0;
				speed_para[1]=1200.0;
				speed_para[2]=1800.0;
				break;
		
		case 3: speed=0.0;	
				K_P=2.0;		
				first_K_P=K_P;
				K_P_sub=0.0;
				K_P_add1=9.0;
				K_P_add2=12.0;
				K_D=0.30;		
				K_I=0.000;	
				K_G=0.00;
				first_K_G=K_G;
				K_E=0.00001;
				K_goal=0.25;
				K_corner=2.8;
				max_ERR=100.0;
				max_ERR=-100.0;
				
				first_speed=300.0;
				Accel=1000.0;
				Accel_in=Accel;
				Target_vel=800.0;
				speed_para[0]=1000.0;
				speed_para[1]=1200.0;
				speed_para[2]=1800.0;
				break;
	}
	reset();
}

void ch_para2(short para_count)
{	
	switch(para_count){
		case 0: speed=220.0;	//220
				K_P=8.5;		//13.5
				first_K_P=K_P;
				K_P_sub=1.00;
				K_P_add1=9.0;
				K_P_add2=12.0;
				K_D=0.0;		//0.0002
				K_I=0.00000050;	//12
				K_G=0.9;
				first_K_G=K_G;
				K_E=0.00001;
				K_goal=6.5;
				K_corner=3.0;
				
				first_speed=900.0;
				Accel=5000.0;
				Accel_in=Accel;
				Target_vel=2000.0;
				speed_para[0]=1000.0;
				speed_para[1]=1500.0;
				speed_para[2]=3000.0;
				break;
		
		case 1: speed=300.0;
				K_P=8.5;		//13.5
				first_K_P=K_P;
				K_P_sub=1.00;
				K_P_add1=9.0;
				K_P_add2=12.0;
				K_D=0.0;		//0.0002
				K_I=0.000003;	//12
				K_G=0.9;
				first_K_G=K_G;
				K_E=0.00001;
				K_goal=5.5;
				K_corner=3.0;
				first_speed=speed;
				break;
				
		case 2: speed=200.0;
				K_P=7.5;		//1.5
				first_K_P=K_P;
				K_P_sub=1.00;
				K_P_add1=8.0;
				K_P_add2=12.0;
				K_D=0.0;		//0.0002
				K_I=0.000003;	//0.00001
				K_G=0.9;
				first_K_G=K_G;
				K_E=0.00001;
				K_goal=5.5;
				K_corner=3.0;
				first_speed=speed;
				break;
		
		case 3: speed=300.0;
				K_P=8.5;		//13.5
				first_K_P=K_P;
				K_P_sub=1.00;
				K_P_add1=10.0;
				K_P_add2=12.0;
				K_D=0.0;		//0.0002
				K_I=0.000003;	//12
				K_G=0.9;
				first_K_G=K_G;
				K_E=0.00001;
				K_goal=4.7;
				K_corner=3.0;
				first_speed=speed;
				break;
	}
	reset();
}

void select_P()
{
	int select_frag=0;
	double selecter=0.1;

	while(select_frag==0){
		calc_senc();
		if(PORTE.PORT.BIT.B0 == 0){
			TIMER_WAIT(200);
			LED2=~LED2;
			selecter+=0.10;
		}
		
		if(cari_SEN[1] > 80.0){
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
	double selecter=0.1;

	while(select_frag==0){
		calc_senc();
		if(PORTE.PORT.BIT.B0 == 0){
			TIMER_WAIT(200);
			LED2=~LED2;
			selecter+=0.1;
		}
		
		if(cari_SEN[1] > 80.0){
			K_P_sub=selecter;
			first_sub=K_P_sub;
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
	double selecter=0.0000001;

	while(select_frag==0){
		calc_senc();
		if(PORTE.PORT.BIT.B0 == 0){
			TIMER_WAIT(200);
			LED2=~LED2;
			selecter+=0.00000050;
		}
		
		if(cari_SEN[1] > 80.0){
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
	double selecter=0.0;

	while(select_frag==0){
		calc_senc();
		if(PORTE.PORT.BIT.B0 == 0){
			TIMER_WAIT(200);
			LED2=~LED2;
			selecter+=0.1;
		}
		
		if(cari_SEN[1] > 80.0){
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
	double selecter=0.0;

	while(select_frag==0){
		calc_senc();
		if(PORTE.PORT.BIT.B0 == 0){
			TIMER_WAIT(200);
			LED2=~LED2;
			selecter+=0.1;
		}
		
		if(cari_SEN[1] > 80.0){
			K_G=selecter;
			first_K_G=K_G;
			select_frag=1;
			LED_manager(7);
		}
	}
	TIMER_WAIT(1000);
	reset();
}

void select_goal()
{
	int select_frag=0;
	double selecter=0.22;

	while(select_frag==0){
		calc_senc();
		if(PORTE.PORT.BIT.B0 == 0){
			TIMER_WAIT(200);
			LED2=~LED2;
			selecter+=0.01;
		}
		
		if(cari_SEN[1] > 80.0){
			K_goal=selecter;
			select_frag=1;
			LED_manager(7);
		}
	}
	TIMER_WAIT(1000);
	reset();
}

void select_corner()
{
	int select_frag=0;
	double selecter=2.5;

	while(select_frag==0){
		calc_senc();
		if(PORTE.PORT.BIT.B0 == 0){
			TIMER_WAIT(200);
			LED2=~LED2;
			selecter+=0.1;
		}
		
		if(cari_SEN[1] > 80.0){
			K_corner=selecter;
			first_corner=K_corner;
			select_frag=1;
			LED_manager(7);
		}
	}
	TIMER_WAIT(1000);
	reset();
}

void select_max()
{
	int select_frag=0;
	double selecter=100.0;

	while(select_frag==0){
		calc_senc();
		if(PORTE.PORT.BIT.B0 == 0){
			TIMER_WAIT(200);
			LED2=~LED2;
			selecter+=10.0;
		}
		
		if(cari_SEN[1] > 80.0){
			max_ERR=selecter;
			min_ERR=(-1)*selecter;
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
	double selecter=300.0;

	while(select_frag==0){
		calc_senc();
		if(PORTE.PORT.BIT.B0 == 0){
			TIMER_WAIT(200);
			LED2=~LED2;
			selecter+=100.0;
		}
		
		if(cari_SEN[1] > 80.0){
			Target_vel=selecter;
			select_frag=1;
			LED_manager(7);
		}
	}
	TIMER_WAIT(1000);
	reset();
}

void trace3()	//3週目
{
	int start_frag[3];
	int trace_roop_frag=0;
	int roop_frag=0;
	short para_frag=0;

	circl_count=3;
	LED3=ON;
	TIMER_WAIT(1000);
	LED3=OFF;
		
	while(trace_roop_frag==0){
		if(PORTE.PORT.BIT.B0 == 0){	
			LED1=OFF;
			TIMER_WAIT(500);
			
			while(roop_frag==0){
				if(para_frag==0){
					select_P();
					ch_para();
					para_frag=1;
				}
				
				calc_senc();
				if(cari_SEN[1] > 40){
					start_frag[1]=1;
					LED2=ON;
				}
		
				if(start_frag[1]==1){
					if(cari_SEN[6] > 40){
						start_frag[2]=1;
						LED3=ON;
						TIMER_WAIT(500);
					}
				}	
				if(start_frag[2]==1){
				
					countdown();
					TIMER_WAIT(1000);
					mot_STB(START_A);
			
					zero_gyro=get_gyro();
					ENC_R=0;
					ENC_sub_R=0;
					ENC_L=0;
					ENC_sub_L=0;
					
					start0=1;
					while(stop_frag==0){}
					
					stop_manager();
					trace_roop_frag=1;
					roop_frag=1;
					
				}
				else{}
			}
		}
		else{}
	}
}

void trace2()	//二週目
{
	short start_frag=0;
	short trace_roop_frag=0;
	short permit_goal=0;

	circl_count=2;
	LED2=ON;
	TIMER_WAIT(1000);
	LED2=OFF;

//	ch_para2(0);
	ch_para();
	select_P();
	select_I();
//	select_D();
//	select_G();
	select_goal();
//	select_corner();
	
	while(trace_roop_frag==0){
		init_thre();			
		calc_senc();
	
		if(cari_SEN[1] > 80.0){
			if(Dispermit_goal==1){
				start_frag=1;
				LED2=ON;
				TIMER_WAIT(500);
			}
			else{
				LED2=ON;
				permit_goal=1;
			}
		}
		
		if(cari_SEN[6] > 80.0 && permit_goal==1){
			start_frag=1;
			LED3=ON;
			TIMER_WAIT(500);
		}
		else if(cari_SEN[6] > 80.0 && permit_goal==0){	//ゴールさせない
			LED3=ON;
			Dispermit_goal=1;
		}	
	
		if(start_frag==1){
			countdown();
			TIMER_WAIT(1000);
				
			zero_gyro=get_gyro();
			ENC_R=8000;
			ENC_sub_R=0;
			ENC_L=8000;
			ENC_sub_L=0;
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
	short start_frag=0;
	short trace_roop_frag=0;
	short i=0;
	short permit_goal=0;
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
//	select_max();
	select_speed();
	
	while(trace_roop_frag==0){
		init_thre();			
		calc_senc();
	
		if(cari_SEN[1] > 80.0){
			if(Dispermit_goal==1){
				start_frag=1;
				LED2=ON;
				TIMER_WAIT(500);
			}
			else{
				LED2=ON;
				permit_goal=1;
			}
		}
		
		if(cari_SEN[6] > 80.0 && permit_goal==1){
			start_frag=1;
			LED3=ON;
			TIMER_WAIT(500);
		}
		else if(cari_SEN[6] > 80.0 && permit_goal==0){	//ゴールさせない
			LED3=ON;
			Dispermit_goal=1;
		}	
	
		if(start_frag==1){
			countdown();
			TIMER_WAIT(1000);

			zero_gyro=get_gyro();
			ENC_R=15000;
			ENC_sub_R=0;
			ENC_L=15000;
			ENC_sub_L=0;
			EncTimeCount=0;
			ENC_count=0;
			dis_count=0;
			
			mot_STB(START_A);
			
			start0=1;
			mot_frag=1;
			for(i=0; i<201; i++){
				ENC_dis[i]=0.0;
				ENC_check[i]=0;
				RL_check[i]=3;
//				vel_check[i]=0;
		//		sub_SEN_check[i]=0.0;
			//	ST_check[i]=0;
			}
			
			for(i=1; i<7; i++){
				diff_SEN[i]=0.0;
			}
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

void out_gyro()
{	
	short roop_frag=0;
	short deg=0;
	short Gy=0;
	short gyr=0;
	short ave_gyr=0;
	short i=0;
	
	reset();
	while(roop_frag==0){
		if(PORTE.PORT.BIT.B0 == 0){
			TIMER_WAIT(2000);
			for(i=0; i<100; i++)
				zero_gyro=get_gyro();
			start0=1;
			roop_frag=1;
		}
	}
	while(1){
		Gy=(short)GYRO_ERR;
		gyr=(short)gyro2;
		ave_gyr=(short)ave_gyro;
		
		dec_out(Gy, 5); outs(" ");
		dec_out(deg, 5); outs(" ");
		dec_out(gyr, 5); outs(" ");
		dec_out(ave_gyr, 5); outs(" ");
		outs("\n");
	}
}

void out_senc()
{
	short i;	
	TIMER_WAIT(2000);
	
	while(1){
		calc_senc();
		for(i=1; i<11; i++){
			dec_out((short)ave_SEN[i], 4);	outs(" ");
		}
		outs("\n");
	}
} 

void out_senc_drive_mot()
{
	short i;
	
	TIMER_WAIT(2000);
	search_max();
	start0=1;
	mot_STB(START_A);
	mot_frag=1;
	TIMER_WAIT(2000);
	while(1){
		MTU0.TGRA=100;
		MTU0.TGRC=100;
		for(i=1; i<11; i++){
			dec_out((short)ave_SEN[i], 4);	outs(" ");
		}
		outs("\n");
	}
} 

void line_check()
{
	short i;
	double max=0.0;
		
	TIMER_WAIT(2000);
	start0=1;
	
	while(1){
		max=ave_SEN[1];
		for(i=1; i<7; i++){			
			if(max < ave_SEN[i])
				max=ave_SEN[i];
		}	
		
		if(max==ave_SEN[1]){
			if(max > thre_max ){
				LED1=ON;
				max_frag=1;
			}
		}
		else if(max!=ave_SEN[1])
			max_frag=0;
		
		if(max==ave_SEN[6]){
			if(max > thre_max){
				LED1=ON;
				min_frag=1;
			}
		}
		else if(max!=ave_SEN[6])
			min_frag=0;

		if(max_frag==1 && (ave_SEN[1] < thre_max && ave_SEN[2] < thre_max && ave_SEN[3] < thre_max && ave_SEN[4] < thre_max && ave_SEN[5] < thre_max && ave_SEN[6] < thre_max)){
			LED1=OFF;
			ERR=max_ERR;
		}
				
		if(min_frag==1 && (ave_SEN[1] < thre_max && ave_SEN[2] < thre_max && ave_SEN[3] < thre_max && ave_SEN[4] < thre_max && ave_SEN[5] < thre_max && ave_SEN[6] < thre_max)){
			ERR=min_ERR;
			LED1=OFF;
		}

	}
}

int init_thre(){
	
	SEN_thre[1]     =max_thre*0.4;
	SEN_thre[2]     =max_thre*0.4;
	SEN_thre[3]     =max_thre*0.4;
	SEN_thre[4]     =max_thre*0.4;
	SEN_thre[5]     =max_thre*0.4;
	SEN_thre[6]     =max_thre*0.4;
	SEN_thre[9]     =max_thre*0.4;
	SEN_thre[10]    =max_thre*0.4;
	
	SEN_thre[goal]  =max_thre*K_goal;//max_thre*0.3;//TargetGoal*K_goal;//max_thre*0.3;//90;
	SEN_thre[corner]=TargetCorner*K_corner;//max_thre*0.3;//60;

	return 0;
}
	
void ch_mode(int mode)
{
	reset();
	switch(mode){
		case 1: search_max();	break;
		case 2: trace();		break;
		case 3: trace2();		break;
		case 4: line_check();		break;
		case 5: out_senc_drive_mot();		break;
		case 6: out_senc();		break;
		case 7: out_curve();	break;
	}
}

void main()
{
	int mode_count=1;
	
	set();
	init_goal_frag();
	init_thre();
	PORT9.DR.BIT.B3=0;
	mot_STB(STOP);
	Battery_Check();
	ENC_R=3300;
	
	
	while(1)
	{
		if(ENC_R > 1600*2 && ENC_R < 1600*3){
			LED_manager(1);
			mode_count=1;
		}
		if(ENC_R > 1600*3 && ENC_R < 1600*4){
			LED_manager(2);
			mode_count=2;
		}
		if(ENC_R > 1600*4 && ENC_R < 1600*5){
			LED_manager(3);
			mode_count=3;
		}
		if(ENC_R > 1600*5 && ENC_R < 1600*6){
			LED_manager(4);
			mode_count=4;
		}
		if(ENC_R > 1600*6 && ENC_R < 1600*7){
			LED_manager(5);
			mode_count=5;
		}
		if(ENC_R > 1600*7 && ENC_R < 1600*8){
			LED_manager(6);
			mode_count=6;
		}
		if(ENC_R > 1600*8 && ENC_R < 1600*9){
			LED_manager(7);
			mode_count=7;
		}
		
		if(ENC_R > 1600*9)
			ENC_R=(1600*2)+1;
			
		if(PORTE.PORT.BIT.B0 == 0){
			TIMER_WAIT(500);
			ch_mode(mode_count);
			reset();
		}
		
	}
}

