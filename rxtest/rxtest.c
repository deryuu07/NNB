
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

#define ENC_R (unsigned short)MTU1.TCNT
#define ENC_L (unsigned short)MTU2.TCNT

#define max_thre 100.0

/***************カウンタ***************/
unsigned long TimeCount=0;
unsigned long TimeCount2=0;
unsigned long TimeCount3=0;
unsigned short TimeNow=0;
unsigned short StartTimeCount=0;
unsigned short CrossTimeCount=0;
unsigned short stop_count=0;
unsigned short curve_count=0;
unsigned short mot_count=0;
		 short circl_count=1;
		 short err_count=0;
		 short curve_time=0;
		 
/***************床センサ***************/
short SEN_ON[11];
short SEN_OFF[11];
short SEN[11];
double cari_SEN[11];
double sum_SEN[11];
double MAX_SEN[11];
double MIN_SEN[11];
double ave_SEN[11];
short SEN_min[11];
short SEN_max[11];
double SEN_thre[11];
short SEN_minmax[11];
short curve_err=0;
short ave_count=0;
short ERR=0;
short sub_ERR=0; 
short PRE_ERR=0;
short SUM_ERR=0;
short ERR_MAX=200;
short ERR_MIN=-200;
short SDP_CHECK=0;

/***************ジャイロ***************/
double GYRO_ERR=0.0;
double pre_gyro=0.0;
double now_gyro=0.0;
double filter_gyro=0.0;
double pre_filter=255.0;
double diff_gyro=0.0;
double deg_gyro=0.0;
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
unsigned long pre_ENC=0;
unsigned long now_ENC=0;
unsigned long ENC_sub=0;

unsigned long ENC_dis[200];
unsigned long ENC_curve[200];
		 short ST_check[200];
/***************PIDゲイン**************/
double K_P=1.0;
double K_P_sub=1.0;
double first_K_P=0.0;
double K_D=1.0;
double K_I=1.0;
double K_G=1.0;
double P=0.1;
double I=0.001;
double speed=130.0;
double first_speed=0.0;
double max_speed=0.0;

short mot_out_L[500];
short mot_out_R[500];
short C_MAX[500];
short C_MIN[500];
double V_err=0.0;
double sum_V=0.0;
double MOT_V_PI=0.0;
double MOT_PID=0.0;
double V_R=0.0;
double V_L=0.0;
double target_V=0.0;

/***************フラグ*****************/
int goal_frag1=0;
int goal_frag2=0;
int first_check=0;
int cross_frag=0;
int stop_frag=0;
int start0=0;
int sci_frag=0;
int min_frag=0;
int max_frag=0;
int curve_frag=0;
int u_count=0;
int gyro_first=0;
int dis_count=0;

int mot_frag=0;
int curve_one=0;
int speed_check_frag=0;
short SEN_frag=0;
short ST_frag=0;
short curve_frag2=0;
short corner_frag=0;
short cari_frag=0;
short Dispermit_goal=0;

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
void goal_check(unsigned long ENC_NOW);
void TIMER_CALL();
void TIMER_WAIT(unsigned short ms);
void mot_drive_R(double mot_in_p, double mot_in_p_sub, double mot_in_d, double mot_in_i, double mot_in_g);
void mot_drive_L(double mot_in_p, double mot_in_p_sub, double mot_in_d, double mot_in_i, double mot_in_g);
void Battery_Check();
void countdown();
void trace();
void out_senc();
void ch_mode(int mode);
void check_curve();
void mot_brake();

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
	
	filter_gyro=((9.0*pre_filter) + (gyro))/10.0;
	//filter_gyro=gyro;
	pre_filter=filter_gyro;
	
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
		if(SEN[i]<0)
			SEN[i]=0;
	}
}

int calc_senc()		//キャリブレーション処理，平均化，ジャイロ値処理
{
	int i;
	double gyro=0.0;
	double max=0.0;
	
	get_senc();
	for(i=1; i<11; i++){
		cari_SEN[i] = 100.0*(double)(SEN[i]-SEN_min[i])/(double)SEN_minmax[i];	//最高値が100前後になるはず
		sum_SEN[i] += cari_SEN[i];
		
		if(MAX_SEN[i] < cari_SEN[i])
			MAX_SEN[i] = cari_SEN[i];
		
		if(MIN_SEN[i] > cari_SEN[i])
			MIN_SEN[i] = cari_SEN[i];
	}
	ave_count++;
	if(ave_count==5){
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
//		GYRO_ERR=target_gyro-gyro2;

		sum_gyro += gyro2;
		if(MAX_gyro < gyro2)
			MAX_gyro = gyro2;
		
		if(MIN_gyro > gyro2)
			MIN_gyro = gyro2;
		
		gyro_count++;
		if(gyro_count==5){
			pre_gyro=now_gyro;
			ave_gyro=(sum_gyro-MAX_gyro-MIN_gyro)/3.0;
			now_gyro=ave_gyro;
			
			GYRO_ERR=target_gyro-ave_gyro;
			diff_gyro=now_gyro-pre_gyro;
			deg_gyro+=diff_gyro;
			
			sum_gyro=0.0;
			MIN_gyro=MAX_gyro;
			MAX_gyro=0.0;
			gyro_count=0;
		}
		
		PRE_ERR=ERR;					//D
		ERR=ave_SEN[9]-ave_SEN[10];		//P		
		SUM_ERR+=ERR;					
		
		max=ave_SEN[1];
		for(i=2; i<7; i++){
			if(max < ave_SEN[i])
				max=ave_SEN[i];
		}
		if(max==ave_SEN[3] || max==ave_SEN[4]){	//○○● ○○○ or ○○○ ●○○
			LED1=OFF;
			sub_ERR=0.1*(ave_SEN[3]-ave_SEN[4]);
		}
		else if(max==ave_SEN[2] || max==ave_SEN[5]){
			LED1=OFF;
			sub_ERR=5.0*(ave_SEN[2]-ave_SEN[5]);
		}
		else if(max==ave_SEN[1] || max==ave_SEN[6]){
			LED1=ON;
			sub_ERR=10.0*(ave_SEN[1]-ave_SEN[6]);
		}
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
				ENC_curve[dis_count]=1;
				curve_one=1;
			}
			else{							//直線時
				ENC_curve[dis_count]=2;
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
		SEN_max[i]=1;
		SEN_min[i]=10;
	}
	cari_frag=1;
	while(ENC_R < 14990){
		MOT_ctr_R1=1;
		MOT_ctr_R2=0;
		MOT_ctr_L1=0;
		MOT_ctr_L2=1;
		MTU0.TGRC=100 - (0.2*(ENC_R-ENC_L));
		MTU0.TGRA=100 + (0.2*(ENC_R-ENC_L));

		//get_senc();
		for(i=1; i<11; i++){
			if(SEN_max[i] < SEN[i])
				SEN_max[i]=SEN[i];
			if(SEN[i] > 0 && SEN_min[i] > SEN[i])
				SEN_min[i]=SEN[i];
		}	
	}
	cari_frag=0;
	mot_brake();
	ENC_R=0;
	ENC_L=0;
	TIMER_WAIT(1000);
	cari_frag=1;
	while(ENC_R < 14000){
		MOT_ctr_R1=1;
		MOT_ctr_R2=0;
		MOT_ctr_L1=1;
		MOT_ctr_L2=0;
		MTU0.TGRC=100;
		MTU0.TGRA=100;
	}
	cari_frag=0;
	mot_brake();
	MTU0.TGRC=0;
	MTU0.TGRA=0;
	TIMER_WAIT(1000);
	ENC_R=100;
	ENC_L=100;
	
	MOT_ctr_R1=1;
	MOT_ctr_R2=0;
	MOT_ctr_L1=0;
	MOT_ctr_L2=1;
	cari_frag=1;
	
	while(ENC_R < 15090){
		MTU0.TGRC=100 - (0.2*(ENC_R-ENC_L));
		MTU0.TGRA=100 + (0.2*(ENC_R-ENC_L));

		//get_senc();
		for(i=1; i<11; i++){
			if(SEN_max[i] < SEN[i])
				SEN_max[i]=SEN[i];
			if(SEN[i] > 0 && SEN_min[i] > SEN[i])
				SEN_min[i]=SEN[i];
		}	
	}
	cari_frag=0;
	mot_brake();
	ENC_R=0;
	ENC_L=0;
	while(ENC_R < 14000){
		MOT_ctr_R1=1;
		MOT_ctr_R2=0;
		MOT_ctr_L1=1;
		MOT_ctr_L2=0;
		MTU0.TGRC=100;
		MTU0.TGRA=100;
	}
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
	first_check=0;
	StartTimeCount=0;
	CrossTimeCount=0;
	
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
		mot_drive_R((double)ERR, (double)sub_ERR, (double)PRE_ERR, (double)SUM_ERR, (double)GYRO_ERR);
		mot_drive_L((double)ERR, (double)sub_ERR, (double)PRE_ERR, (double)SUM_ERR, (double)GYRO_ERR);		
	}

	init_goal_frag();
	mot_brake();
	TIMER_WAIT(1000);
	mot_STB(STOP);
}	

void Accel_manager(short ENC_NOW)
{
	if(ST_check[dis_count]==ST){	//直線判定時のみ加速
		LED1=ON;
		if(ENC_dis[dis_count] > 22635){	//直線が30cm以上あれば加速
			if((ENC_dis[dis_count]-11317) >= ENC_NOW){
				if(speed < max_speed)
					speed+=1;
			}	
			else{
				if(speed > first_speed)
					speed-=10;
				if(speed < first_speed)
					speed=first_speed;
			}
		}
		else{}
	}
	else if(ST_check[dis_count]==curve)
		LED1=OFF;
}

void goal_check(unsigned long  ENC_NOW)
{
	if(first_check==0){
		if(ave_SEN[goal] > SEN_thre[goal]){						//スタート時 白線上
			goal_frag1=1;
			LED3=ON;
		}
	
		if(goal_frag1==1 && (ave_SEN[goal] < (SEN_thre[goal]))){		//スタート時 白線から外れたとき
			LED3=OFF;
			goal_frag2=1;
			first_check=2;
			StartTimeCount=0;
			CrossTimeCount=0;
			ENC_R=0;
			ENC_sub=0;	
		}
	}
	if(first_check==2)
		StartTimeCount++;
		
	if(StartTimeCount==200 && first_check==2){	//何秒間か読み飛ばす
		first_check=1;
	}
		
	if(first_check==1){		
		if(((ave_SEN[2] > SEN_thre[2]) || (ave_SEN[5] > SEN_thre[5])) && goal_frag2==1){
			goal_frag2=0;
		}
		if(goal_frag2==0){
			CrossTimeCount++;
		}
			
		if(CrossTimeCount==100){		//wait処理
			goal_frag2=1;
			CrossTimeCount=0;
		}
	
	/********************コーナー処理*******************************/	
		curve_time++;
		
		if(circl_count==1){
			if(curve_time==50){			//50msごとにST判定
				curve_err+=ERR;
				if(curve_err <= 50 && curve_err >= -50)
					ST_frag++;
				else if(curve_err > 50 || curve_err < -50 )
					curve_frag2++;	
				curve_time=0;
			}
		}
		
		if((ave_SEN[corner] > SEN_thre[corner]) && goal_frag2==1 && curve_frag==0){
			corner_frag=1;
			LED2=ON;
		}
		if((ave_SEN[corner] < (SEN_thre[corner])) && corner_frag==1){	
			if(circl_count==1){		//１周目のみ記録
				curve_err=0;
				if(ST_frag > curve_frag2){
					ST_check[dis_count]=ST;
					ST_frag=0;
					curve_frag2=0;
				}
				else if(ST_frag < curve_frag2){
					ST_check[dis_count]=curve;
					ST_frag=0;
					curve_frag2=0;
				}
				ENC_dis[dis_count]=ENC_NOW;
			}
			
			if(circl_count==2 || circl_count==3){
				if(ST_check[dis_count]==curve)
					LED1=OFF;
				else if(ST_check[dis_count]==ST)
					LED1=ON;
			}
			
			dis_count++;
			LED2=OFF;	
			ENC_R=0;
			ENC_sub=0;			
			curve_frag=1;
			corner_frag=0;
		}
		if(curve_frag==1)	//複数回認識することへの対策
			curve_count++;
		if(curve_frag==1 && curve_count==50){
			curve_frag=0;
			curve_count=0;
		}
		
	/*********************ゴール処理*******************/				
	
		if((ave_SEN[goal] > SEN_thre[goal]) && goal_frag2==1 && Dispermit_goal!=1){
			LED3=OFF;
			stop_frag=1;
		}
	}
	else{}
}

void TIMER_CALL()
{
	TimeCount++;	
	
	if(start0==1){		
		if(ENC_R > 30000 && ENC_R < 60000){
			ENC_sub+=ENC_R;
			ENC_R=0;
		}
		now_ENC=ENC_sub+ENC_R;
		
		calc_senc();	
		TimeCount2++;
		if(circl_count==2 || circl_count==3)
			Accel_manager(now_ENC);
			
		goal_check(now_ENC);
		if(mot_frag==1){
			mot_drive_R((double)ERR, (double)sub_ERR, (double)PRE_ERR, (double)SUM_ERR, (double)GYRO_ERR);
			mot_drive_L((double)ERR, (double)sub_ERR, (double)PRE_ERR, (double)SUM_ERR, (double)GYRO_ERR);	
		}
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

void mot_drive_R(double mot_in_p, double mot_in_p_sub, double mot_in_d, 
				  double mot_in_i, double mot_in_g)
{
	double MOT_IN=0.0;
	
	MOT_IN=speed+(K_P*mot_in_p)+(K_P_sub*mot_in_p_sub)+(K_D*mot_in_d)+(K_I*mot_in_i);
	
	if(circl_count==2 || circl_count==3){
		if(ST_check[dis_count]==ST)
			MOT_IN-=0.05*(MOT_IN-speed);
	}	
	
	if(max_frag==0 || min_frag==0)	//直線時にのみジャイロの補正を適用
		MOT_IN+=K_G*mot_in_g;
	
	if(TimeCount2==20){
		mot_out_R[mot_count]=(short)MOT_IN;	
		C_MAX[mot_count]=(short)mot_in_p;
		C_MIN[mot_count]=(short)mot_in_p_sub;
	}
	
	if(MOT_IN > 0.0){
		MOT_ctr_R1=1;
		MOT_ctr_R2=0;
		MTU0.TGRC=MOT_IN;
	}
	if(MOT_IN < 0.0){
		MOT_IN*=(-1.0);
		MOT_ctr_R1=0;
		MOT_ctr_R2=1;
		MTU0.TGRC=MOT_IN;
	}

}

void mot_drive_L(double mot_in_p, double mot_in_p_sub, double mot_in_d, 
				  double mot_in_i, double mot_in_g)
{
	double MOT_IN=0.0;
	
	MOT_IN=speed-(K_P*mot_in_p)-(K_P_sub*mot_in_p_sub)-(K_D*mot_in_d)-(K_I*mot_in_i);
	
	if(circl_count==2 || circl_count==3){
		if(ST_check[dis_count]==ST)
			MOT_IN-=0.05*(MOT_IN-speed);
	}
	
	if(max_frag==0 || min_frag==0)	//直線時にのみジャイロの補正を適用
		MOT_IN-=K_G*mot_in_g;
	
	if(TimeCount2==20){
		mot_out_L[mot_count]=(short)MOT_IN;
		mot_count++;
		TimeCount2=0;
	}
	
	if(MOT_IN > 0.0){	
		MOT_ctr_L1=0;
		MOT_ctr_L2=1;
		MTU0.TGRA=MOT_IN;
	}
	if(MOT_IN < 0.0){	
		MOT_IN*=(-1.0);
		MOT_ctr_L1=1;
		MOT_ctr_L2=0;
		MTU0.TGRA=MOT_IN;
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
		LED1=ON; LED2=ON; LED3=ON;
	}
	if(battery < 757 && battery >= 737){
		LED1=ON; LED2=ON; LED3=OFF;
	}
	if(battery < 737){								//7.4V未満になったら未満強制停止
		LED1=OFF; LED2=OFF; LED3=ON;
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
	int trace_roop_frag=0;
	int roop_frag=0;
	int i;
						
	while(trace_roop_frag==0){
		if(PORTE.PORT.BIT.B0 == 0){	
			while(roop_frag==0){
				for(i=0; i<501; i++){
					dec_out(mot_out_R[i], 8); outs(" ");
					dec_out(mot_out_L[i], 8); outs(" ");
					dec_out(C_MAX[i], 4); outs(" ");
					dec_out(C_MIN[i], 4); outs(" ");
					dec_out(ST_check[i], 4); outs(" ");
					dec_out(ENC_dis[i], 8); outs("\n");
				}
				trace_roop_frag=1;
				roop_frag=1;
			}
		}
		else{}
	}
}

void speed_check()
{
	int i=0;

	while(1){
		if(PORTE.PORT.BIT.B0 == 0){
			countdown();
			ENC_R=0;
			ENC_sub=0;
			pre_ENC=0;
			now_ENC=0;
			
			ENC_L=0;
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
							dec_out(mot_out_R[i], 6);	outs("\n");
						//	dec_out(mot_out_L[i], 4);	outs("\n");
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
	
	long sa=0;
	long now_enc_r=0;
	long now_enc_l=0;
	long pre_enc_r=0;
	long pre_enc_l=0;
	long sub_r=0;
	long sub_l=0;
	
	ENC_R=0;	//max:65535
	ENC_L=0;
	while(1){
		now_enc_r=ENC_R+sub_r;
		now_enc_l=ENC_L+sub_l;
		
		if(now_enc_r-pre_enc_r < -50000){
			sub_r+=65535;
			now_enc_r=ENC_R+sub_r;
		}
		if(now_enc_l-pre_enc_l < -50000){
			sub_l+=65535;
			now_enc_l=ENC_L+sub_l;
		}
		
		sa=(now_enc_r-now_enc_l)/123;
			
		dec_out(ENC_R, 8); outs(" ");
		dec_out(ENC_L, 8); outs(" ");
		dec_out(now_enc_r, 8); outs(" ");
		dec_out(now_enc_l, 8); outs(" ");
		dec_out(sa, 8); outs(" ");
		outs("\n");
		
		pre_enc_r=now_enc_r;
		pre_enc_l=now_enc_l;
	}
/*	while(mot_go==0){		
		if(PORTE.PORT.BIT.B0 == 0){
			TIMER_WAIT(500);
			mot_STB(START_A);
			MOT_ctr_R1=1;
			MOT_ctr_R2=0;
			MOT_ctr_L1=1;
			MOT_ctr_L2=0;
			MTU0.TGRC=100;
			MTU0.TGRA=100;
			ENC_R=0;
			ENC_L=0;
			
			while(ENC_R < 7000){	
				sa=(ENC_R-ENC_L)/123;
			
				dec_out(ENC_R, 10); outs(" ");
				dec_out(ENC_L, 10); outs(" ");
				dec_out(sa, 10); outs(" ");
			
				outs("\n");
			}
			mot_brake;
			TIMER_WAIT(1000);
			mot_STB(STOP);
			
			mot_go=1;
			LED1=ON;
		}
		else{}
	}	
*/
}

void ch_para()
{
	short para_frag=0;
 	short para_count=0;

 	ENC_R=1600;
	
 	while(para_frag==0){
		if(ENC_R > 1600*2 && ENC_R < 1600*3){
			LED1=ON; LED2=OFF; LED3=OFF;
			para_count=0;
		}
		if(ENC_R > 1600*3 && ENC_R < 1600*4){
			LED1=OFF; LED2=ON; LED3=OFF;
			para_count=1;
		}
		if(ENC_R > 1600*4 && ENC_R < 1600*5){
			LED1=ON; LED2=ON; LED3=OFF;
			para_count=2;
		}
		if(ENC_R > 1600*5 && ENC_R < 1600*6){
			LED1=OFF; LED2=OFF; LED3=ON;
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
		case 0: speed=200.0;
				K_P=1.7;		//1.5
				K_P_sub=0.8;
				K_D=0.00002;		//0.0002
				K_I=0.00003;	//0.00001
				K_G=0.6;
				break;
		
		case 1: speed=250.0;
				K_P=1.8;		//1.5
				K_P_sub=1.0;
				K_D=0.00002;		//0.0002
				K_I=0.00003;	//0.00001
				K_G=1.2;
				break;
				
		case 2: speed=150.0;
				first_speed=speed;
				max_speed=300.0;
				K_P=1.7;		//1.5
				K_P_sub=0.0;
				K_D=0.00002;		//0.0002
				K_I=0.00003;	//0.00001
				K_G=0.6;
				break;
		
		case 3: speed=180.0;
				first_speed=speed;
				max_speed=400.0;
				K_P=1.8;		//1.5
				K_P_sub=0.0;
				K_D=0.00002;		//0.0002
				K_I=0.00003;	//0.00001
				K_G=0.5;
				break;
	}
	reset();
}

void select_P()
{
	int select_frag=0;
	double selecter=0.0;

	while(select_frag==0){
		calc_senc();
		if(PORTE.PORT.BIT.B0 == 0){
			TIMER_WAIT(200);
			LED2=~LED2;
			selecter+=0.5;
		}
		
		if(cari_SEN[1] > 40){
			K_P=selecter;
			first_K_P=K_P;
			select_frag=1;
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
					ENC_sub=0;
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
	int start_frag[3];
	int trace_roop_frag=0;
	int roop_frag=0;
	short para_frag=0;

	circl_count=2;
	LED2=ON;
	TIMER_WAIT(1000);
	LED2=OFF;
	
	while(trace_roop_frag==0){
		if(PORTE.PORT.BIT.B0 == 0){	
			LED1=OFF;
			TIMER_WAIT(500);
			while(roop_frag==0){
				if(para_frag==0){
					ch_para();
					select_P();
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
					ENC_sub=0;
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

void trace()	//一週目
{
	short start_frag=0;
	short trace_roop_frag=0;
	short i=0;
	short permit_goal=0;
	
	LED1=ON;
	TIMER_WAIT(1000);
	LED1=OFF;
	
	for(i=0; i<201; i++)
		ENC_dis[i]=0;
	
	ch_para();
	select_P();
	
	while(trace_roop_frag==0){
		init_thre();			
		calc_senc();
	
		if(cari_SEN[1] > 40){
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
		
		if(cari_SEN[6] > 40 && permit_goal==1){
			start_frag=1;
			LED3=ON;
			TIMER_WAIT(500);
		}
		else if(cari_SEN[6] > 40 && permit_goal==0){	//ゴールさせない
			LED3=ON;
			Dispermit_goal=1;
		}	
	
		if(start_frag==1){
			countdown();
			TIMER_WAIT(1000);

			zero_gyro=get_gyro();
			ENC_R=0;
			ENC_sub=0;
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
		deg=(short)deg_gyro;
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
	int roop_frag=0;
	short i;
	
	while(roop_frag==0){
		if(PORTE.PORT.BIT.B0 == 0){
			TIMER_WAIT(2000);
			search_max();
			roop_frag=1;
			start0=1;
		}
	}
	while(1){
		for(i=1; i<11; i++){
			dec_out(SEN[i], 4);	outs(" ");
		}
		outs("\n");
	}
} 

void line_check()
{
	int sci_frag=0;
	
	while(sci_frag==0){
		if(PORTE.PORT.BIT.B0 == 0){
			TIMER_WAIT(2000);
			search_max();
			start0=1;
			sci_frag=1;
		}
	}	
	while(1){
		if(ave_SEN[2] > 80)
			LED1=ON;
		else if(ave_SEN[2] < 80)
			LED1=OFF;
		
		if(ave_SEN[5] > 80)
			LED3=ON;
		else if(ave_SEN[5] < 80)
			LED3=OFF;
		
		if(ave_SEN[corner] > 70)
			LED2=ON;
		else if(ave_SEN[corner] < 70)
			LED2=OFF;
	}
}

int init_thre(){
	SEN_thre[2]     =max_thre*0.6;
	SEN_thre[5]     =max_thre*0.6;
	SEN_thre[goal]  =max_thre*0.6;//90;
	SEN_thre[corner]=max_thre*0.4;//60;
	
	return 0;
}
	
void ch_mode(int mode)
{
	reset();
	switch(mode){
		case 1: search_max();	break;
		case 2: trace();		break;
		case 3: trace2();		break;
		case 4: trace3();		break;
		case 5: out_gyro();		break;
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
			LED1=ON; LED2=OFF; LED3=OFF;
			mode_count=1;
		}
		if(ENC_R > 1600*3 && ENC_R < 1600*4){
			LED1=OFF; LED2=ON; LED3=OFF;
			mode_count=2;
		}
		if(ENC_R > 1600*4 && ENC_R < 1600*5){
			LED1=ON; LED2=ON; LED3=OFF;
			mode_count=3;
		}
		if(ENC_R > 1600*5 && ENC_R < 1600*6){
			LED1=OFF; LED2=OFF; LED3=ON;
			mode_count=4;
		}
		if(ENC_R > 1600*6 && ENC_R < 1600*7){
			LED1=ON; LED2=OFF; LED3=ON;
			mode_count=5;
		}
		if(ENC_R > 1600*7 && ENC_R < 1600*8){
			LED1=OFF; LED2=ON; LED3=ON;
			mode_count=6;
		}
		if(ENC_R > 1600*8 && ENC_R < 1600*9){
			LED1=ON; LED2=ON; LED3=ON;
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

