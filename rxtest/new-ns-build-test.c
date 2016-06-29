#include <machine.h>
#include "iodefine.h"
#include "main.h"
#include "math.h"

#define gyro_sensitybity 0.07
unsigned short SPI_out=0;
short gyro=0;

#define PARA_SIZE 20
float SpeedPara[PARA_SIZE] = {1000.0, 1200.0, 1500.0, 1700.0, 1900.0,  //0~9直線用，10~19カーブ用
							  2000.0, 2300.0, 2500.0, 2700.0, 3000.0,
							  1050.0, 1100.0, 1200.0, 1300.0, 1400.0, 
 							  1500.0, 1600.0, 1700.0, 1800.0, 1900.0};

void get_gc_sens()
{
	short i=0;
	short SensOff[SensSize];
	
	SensLEDCorner=OFF;
	SensLEDGoal=OFF;
	for(i=0; i<101; i++){}
	
	AD0.ADCSR.BIT.CH=7;				//AD変換するアナログ入力チャンネルを設定  AD0.ADDRI
	AD0.ADCSR.BIT.ADST = 1;			// AD変換開始
	while(AD0.ADCSR.BIT.ADST==1);	//AD変換終了まで待機
	AD0.ADCSR.BIT.ADST  = 0;		// AD停止
	SensOff[8]=AD0.ADDRH>>6;

	AD0.ADCSR.BIT.CH=8;				//AD変換するアナログ入力チャンネルを設定  AD0.ADDRI
	AD0.ADCSR.BIT.ADST = 1;			// AD変換開始
	while(AD0.ADCSR.BIT.ADST==1);	//AD変換終了まで待機
	AD0.ADCSR.BIT.ADST  = 0;		// AD停止
	SensOff[9]=AD0.ADDRI>>6;
	/********************************/

	SensLEDCorner=ON;
	for(i=0; i<201; i++){}
	AD0.ADCSR.BIT.CH=7;				//AD変換するアナログ入力チャンネルを設定  AD0.ADDRI
	AD0.ADCSR.BIT.ADST = 1;			// AD変換開始
	while(AD0.ADCSR.BIT.ADST==1);	//AD変換終了まで待機
	AD0.ADCSR.BIT.ADST  = 0;		// AD停止
	SENSOR.Original[8]=AD0.ADDRH>>6;
	SENSOR.Original[8]-=SensOff[8];
	SensLEDCorner=OFF;
	SensLEDGoal=ON;
	for(i=0; i<101; i++){}

	AD0.ADCSR.BIT.CH=8;				//AD変換するアナログ入力チャンネルを設定  AD0.ADDRI
	AD0.ADCSR.BIT.ADST = 1;			// AD変換開始
	while(AD0.ADCSR.BIT.ADST==1);	//AD変換終了まで待機
	AD0.ADCSR.BIT.ADST  = 0;		// AD停止
	SENSOR.Original[9]=AD0.ADDRI>>6;
	SENSOR.Original[9]-=SensOff[9];
	SensLEDGoal=OFF;
	for(i=0; i<101; i++){}
	
	if(FLAG.Reverse==1){
		i=SENSOR.Original[8];
		SENSOR.Original[8]=SENSOR.Original[9];
		SENSOR.Original[9]=i;
	}
}

void get_sens()
{
	short i=0;
	short SensOff[SensSize];
	SensLED0=OFF;
	SensLED1=OFF;
	SensLED2=OFF;
	SensLED3=OFF;
	SensLED4=OFF;
	for(i=0; i<101; i++){}

	S12AD0.ADCSR.BIT.ADST=0;			//A/D変換停止
	S12AD0.ADANS.BIT.CH=1;				//AN001をA/D変換
	S12AD0.ADCSR.BIT.ADST=1;			//A/D変換開始
	while(S12AD0.ADCSR.BIT.ADST==1);	//A/D変換が終わるまで待つ
	SensOff[0]=S12AD0.ADDR1;			

	S12AD0.ADCSR.BIT.ADST=0;			//A/D変換停止
	S12AD0.ADANS.BIT.CH=2;				//AN002をA/D変換
	S12AD0.ADCSR.BIT.ADST=1;			//A/D変換開始
	while(S12AD0.ADCSR.BIT.ADST==1);	//A/D変換が終わるまで待つ
	SensOff[1]=S12AD0.ADDR2;			

	S12AD0.ADCSR.BIT.ADST=0;			//A/D変換停止
	S12AD0.ADANS.BIT.CH=3;				//AN003をA/D変換
	S12AD0.ADCSR.BIT.ADST=1;			//A/D変換開始
	while(S12AD0.ADCSR.BIT.ADST==1);	//A/D変換が終わるまで待つ
	SensOff[2]=S12AD0.ADDR3;			

	S12AD1.ADCSR.BIT.ADST=0;			//A/D変換停止
	S12AD1.ADANS.BIT.CH=0;				//AN100をA/D変換
	S12AD1.ADCSR.BIT.ADST=1;			//A/D変換開始
	while(S12AD1.ADCSR.BIT.ADST==1);	//A/D変換が終わるまで待つ
	SensOff[3]=S12AD1.ADDR0A;			

	S12AD0.ADCSR.BIT.ADST=0;			//A/D変換停止
	S12AD0.ADANS.BIT.CH=0;				//AN000をA/D変換
	S12AD0.ADCSR.BIT.ADST=1;			//A/D変換開始
	while(S12AD0.ADCSR.BIT.ADST==1);	//A/D変換が終わるまで待つ
	SensOff[4]=S12AD0.ADDR0A;			

	S12AD1.ADCSR.BIT.ADST=0;			//A/D変換停止
	S12AD1.ADANS.BIT.CH=1;				//AN101をA/D変換
	S12AD1.ADCSR.BIT.ADST=1;			//A/D変換開始
	while(S12AD1.ADCSR.BIT.ADST==1);	//A/D変換が終わるまで待つ
	SensOff[5]=S12AD1.ADDR1;			

	S12AD1.ADCSR.BIT.ADST=0;			//A/D変換停止
	S12AD1.ADANS.BIT.CH=2;				//AN102をA/D変換
	S12AD1.ADCSR.BIT.ADST=1;			//A/D変換開始
	while(S12AD1.ADCSR.BIT.ADST==1);	//A/D変換が終わるまで待つ
	SensOff[6]=S12AD1.ADDR2;			

	S12AD1.ADCSR.BIT.ADST=0;			//A/D変換停止
	S12AD1.ADANS.BIT.CH=3;				//AN103をA/D変換
	S12AD1.ADCSR.BIT.ADST=1;			//A/D変換開始
	while(S12AD1.ADCSR.BIT.ADST==1);	//A/D変換が終わるまで待つ
	SensOff[7]=S12AD1.ADDR3;			
	
	/****************************************************************/
	SensLED0=ON;
	for(i=0; i<101; i++){}
	S12AD0.ADCSR.BIT.ADST=0;			//A/D変換停止
	S12AD0.ADANS.BIT.CH=1;				//AN001をA/D変換
	S12AD0.ADCSR.BIT.ADST=1;			//A/D変換開始
	while(S12AD0.ADCSR.BIT.ADST==1);	//A/D変換が終わるまで待つ
	SENSOR.Original[0]=S12AD0.ADDR1-SensOff[0];			
	
	S12AD0.ADCSR.BIT.ADST=0;			//A/D変換停止
	S12AD0.ADANS.BIT.CH=2;				//AN002をA/D変換
	S12AD0.ADCSR.BIT.ADST=1;			//A/D変換開始
	while(S12AD0.ADCSR.BIT.ADST==1);	//A/D変換が終わるまで待つ
	SENSOR.Original[1]=S12AD0.ADDR2-SensOff[1];			
	SensLED0=OFF;
	SensLED1=ON;
	for(i=0; i<101; i++){}

	S12AD0.ADCSR.BIT.ADST=0;			//A/D変換停止
	S12AD0.ADANS.BIT.CH=3;				//AN003をA/D変換
	S12AD0.ADCSR.BIT.ADST=1;			//A/D変換開始
	while(S12AD0.ADCSR.BIT.ADST==1);	//A/D変換が終わるまで待つ
	SENSOR.Original[2]=S12AD0.ADDR3-SensOff[2];			

	S12AD1.ADCSR.BIT.ADST=0;			//A/D変換停止
	S12AD1.ADANS.BIT.CH=0;				//AN100をA/D変換
	S12AD1.ADCSR.BIT.ADST=1;			//A/D変換開始
	while(S12AD1.ADCSR.BIT.ADST==1);	//A/D変換が終わるまで待つ
	SENSOR.Original[3]=S12AD1.ADDR0A-SensOff[3];			
	SensLED1=OFF;
	SensLED2=ON;
	for(i=0; i<101; i++){}

	S12AD0.ADCSR.BIT.ADST=0;			//A/D変換停止
	S12AD0.ADANS.BIT.CH=0;				//AN000をA/D変換
	S12AD0.ADCSR.BIT.ADST=1;			//A/D変換開始
	while(S12AD0.ADCSR.BIT.ADST==1);	//A/D変換が終わるまで待つ
	SENSOR.Original[4]=S12AD0.ADDR0A-SensOff[4];			
	SensLED2=OFF;
	SensLED3=ON;
	for(i=0; i<101; i++){}

	S12AD1.ADCSR.BIT.ADST=0;			//A/D変換停止
	S12AD1.ADANS.BIT.CH=1;				//AN101をA/D変換
	S12AD1.ADCSR.BIT.ADST=1;			//A/D変換開始
	while(S12AD1.ADCSR.BIT.ADST==1);	//A/D変換が終わるまで待つ
	SENSOR.Original[5]=S12AD1.ADDR1-SensOff[5];			

	S12AD1.ADCSR.BIT.ADST=0;			//A/D変換停止
	S12AD1.ADANS.BIT.CH=2;				//AN102をA/D変換
	S12AD1.ADCSR.BIT.ADST=1;			//A/D変換開始
	while(S12AD1.ADCSR.BIT.ADST==1);	//A/D変換が終わるまで待つ
	SENSOR.Original[6]=S12AD1.ADDR2-SensOff[6];			
	SensLED3=OFF;
	SensLED4=ON;
	for(i=0; i<101; i++){}

	S12AD1.ADCSR.BIT.ADST=0;			//A/D変換停止
	S12AD1.ADANS.BIT.CH=3;				//AN103をA/D変換
	S12AD1.ADCSR.BIT.ADST=1;			//A/D変換開始
	while(S12AD1.ADCSR.BIT.ADST==1);	//A/D変換が終わるまで待つ
	SENSOR.Original[7]=S12AD1.ADDR3-SensOff[7];			
	SensLED4=OFF;
	for(i=0; i<101; i++){}
}
	
void set()	//各種機能初期設定
{
	PORT1.DDR.BYTE = 0x01;
	PORT2.DDR.BYTE = 0x1b;  //0001 1010
	PORT7.DDR.BYTE = 0x7f;
	PORT9.DDR.BYTE = 0x2f;
	PORTA.DDR.BYTE = 0x1f;
	PORTB.DDR.BYTE = 0x80;
	
	SYSTEM.SCKCR.BIT.PCK=1;		//モジュールクロックの変更　12M Hz を4逓倍
	SYSTEM.SCKCR.BIT.ICK=0;		//システムクロックの変更　12M Hz を8逓倍
	init_sci1();
	
	SYSTEM.MSTPCRA.BIT.MSTPA9=0;	//MTU3モジュールの使用を許可
	MTU.TSTRA.BIT.CST3=0;			//カウント停止
	ICU.IER[0x10].BIT.IEN1=1;		//割り込み要求許可
	ICU.IPR[0x57].BIT.IPR=1;		//割り込み優先度を2に
	set_psw(0x00010000);			//PSWのIを割り込み許可に設定
	MTU3.TCR.BIT.CCLR=1;			//TGRAのコンペアマッチでTCNTクリア
	MTU3.TCR.BIT.TPSC=3;			//内部クロック：ICLK/64でカウント (1.5MHz)
	MTU3.TIER.BYTE=0x01;			//コンペアマッチAでの割り込みを許可
	MTU3.TGRA=1500;					//コンペアマッチAまでの時間を0.5msに設定
	MTU.TSTRA.BIT.CST3=1;			//カウントスタート

	SYSTEM.MSTPCRA.BIT.MSTPA9=0;	//MTU4モジュールの使用を許可	
	MTU.TSTRA.BIT.CST4=0;			//カウント停止
	ICU.IER[0x10].BIT.IEN6=1;		//割り込み要求許可
	ICU.IPR[0x59].BIT.IPR=2;		//割り込み優先度を１に
	set_psw(0x00010000);			//PSWのIを割り込み許可に設定
	MTU4.TCR.BIT.CCLR=1;			//TGRAのコンペアマッチでTCNTクリア
	MTU4.TCR.BIT.TPSC=3;			//内部クロック：ICLK/64でカウント (1.5MHz)
	MTU4.TIER.BYTE=0x01;			//コンペアマッチAでの割り込みを許可
	MTU4.TGRA=1500;					//割り込みが0.5msで1周するように設定
	MTU.TSTRA.BIT.CST4=1;			//カウントスタート

	SYSTEM.MSTPCRA.BIT.MSTPA9=0;	//MTU6モジュールの使用を許可	
	MTU.TSTRB.BIT.CST6=0;			//カウント停止
	ICU.IER[0x11].BIT.IEN6=1;		//割り込み要求許可
	ICU.IPR[0x5C].BIT.IPR=1;		//割り込み優先度を１に
	set_psw(0x00010000);			//PSWのIを割り込み許可に設定
	MTU6.TCR.BIT.CCLR=1;			//TGRAのコンペアマッチでTCNTクリア
	MTU6.TCR.BIT.TPSC=3;			//内部クロック：ICLK/64でカウント (1.5MHz)
	MTU6.TIER.BYTE=0x01;			//コンペアマッチAでの割り込みを許可
	MTU6.TGRA=1500;					//コンペアマッチAまでの時間を1msに設定
	MTU.TSTRB.BIT.CST6=1;			//カウントスタート
	
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
	MOT_L=0;				//コンペアマッチの値を0に設定
	MOT_R=0;				
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

void init_param()
{
	SENSOR.Center=300;
	ENC.Right=0;
	ENC.Left=0;
	ENC.R_L=0;
	ENC.Local=0;
	ENC.Count=0;
	VEL.Right=0;
	VEL.Left=0.0;
	VEL.R_L=0.0;
	VEL.Desire=0.0;
	VEL.BasicDesire=0.0;
	VEL.Err=0.0;
	VEL.Pre=0.0;
	VEL.PreErr=0.0;
	VEL.SumErr=0.0;
	GAIN.SensP=0.0;
	GAIN.SensD=0.0;
	GAIN.VelP=4.0;
	GAIN.VelI=0.0005;
	GAIN.VelD=0.01;
	K_P_GYRO=0.0;
	K_I_GYRO=0.0;
	K_D_GYRO=0.0;
	LOG.Count=0;
	LOG.Timing=0;
	LOG.LocalTime=0;
	FLAG.Stop=0;
	FLAG.GetSens=0;
	FLAG.Goal=0;
	FLAG.Corner=0;
	FLAG.FirstAccel=0;
	FLAG.Reverse=0;
	GYRO_SUM_ERR=0.0;
	EG_stop_count=0;
	Angle=0.0;
}

void ch_para()
{
	short i=0; 
	
	switch(select_mode()){
		case 0: 
			K_P_GYRO=3.0;
			first_K_G=K_P_GYRO;
			K_D_GYRO=1.0;
			first_K_D=K_D_GYRO;
			K_I_GYRO=0.001;
			first_K_I=K_I_GYRO;
			VEL.BasicDesire=1000.0;
			break;
			
		case 1: 
			K_P_GYRO=3.0;
			first_K_G=K_P_GYRO;
			K_D_GYRO=1.0;
			K_I_GYRO=0.001;
			VEL.BasicDesire=1100.0;
			break;

		case 2: 
			K_P_GYRO=3.0;
			first_K_G=K_P_GYRO;
			K_D_GYRO=1.0;
			K_I_GYRO=0.001;
			VEL.BasicDesire=1200.0;
			break;
			
		case 3: 
			K_P_GYRO=3.0;
			first_K_G=K_P_GYRO;
			K_D_GYRO=1.0;
			K_I_GYRO=0.001;
			VEL.BasicDesire=1250.0;
			break;		
	}
	
	for(i=0; i<COURSE_SIZE; i++){
		VEL.Now[i]=calc_vel(VEL.BasicDesire, accel, COURSE_DATA.Dis[i], COURSE_DATA.Course[i]);
		if(COURSE_DATA.Dis[i+1]==0){
			VEL.Now[i]=9;
		}
		COURSE_DATA.AccelDis[i]=((SpeedPara[VEL.Now[i]]*SpeedPara[VEL.Now[i]]) - (VEL.BasicDesire*VEL.BasicDesire))/(2.0*accel);
	}
	reset();
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
	for(i=0; i<50; i++)
		BUZZ_ON();
	reset();	
	mot_STB(STOP);
}

void reset()
{
	LED1=LED_OFF;
	LED2=LED_OFF;
}

void LED_manager(char mode)
{
	switch(mode){
		case 0:	LED1=LED_OFF; LED2=LED_OFF; break;
		case 1:	LED1=LED_OFF; LED2=LED_ON; 	break;
		case 2:	LED1=LED_ON; LED2=LED_OFF; 	break;
		case 3:	LED1=LED_ON; LED2=LED_ON; 	break;
	}
}

void init_ENC()
{
	ENC_R=15000;
	ENC_L=15000;
}

void mot_STB(short start_stop)
{
	if(start_stop==START_A)
		MOT_STBY=1;
	else{
		MOT_STBY=0;
		MOT_ctr_L1=0;
		MOT_ctr_L2=0;
		MOT_ctr_R1=0;
		MOT_ctr_R2=0;
	}
}

short get_batt()	//バッテリ残量取得
{
	short Battery;
	
	AD0.ADCSR.BIT.CH=6;				//AD変換するアナログ入力チャンネルを設定 バッテリ AD0.ADDRG
	AD0.ADCSR.BIT.ADST = 1;			// AD変換開始
	while(AD0.ADCSR.BIT.ADST==1);	//AD変換終了まで待機
	AD0.ADCSR.BIT.ADST  = 0;		// AD停止
	Battery=AD0.ADDRG>>6;

	return Battery*9765/1000;		//mv
}

void battery_check()
{
	battery=get_batt();
	if(battery >= 7400){	//mv
		LED_manager(3);
	}
	if(battery < 7400 && battery >= 6700){
		LED_manager(2);	
	}
	if(battery < 6700){								//7.4V未満になったら未満強制停止
		LED_manager(1);
		while(1){
			mot_STB(STOP);
		}
	}
	TIMER_WAIT(1500);
	reset();
}

void calc_sens()
{
	short i;
	short line_place=0;
		
	SENSOR.Pre=SENSOR.Err;
	get_sens();
	for(i=0; i<SensSize; i++){
		if(SENSOR.Original[i]<0)
			SENSOR.Original[i]*=(-1);			
		SENSOR.Calibrated[i] = 400*(SENSOR.Original[i]-SENSOR.MinValue[i])/SENSOR.Max_Min[i];	
	}
	
	/***復帰処理***/
	if((SENSOR.Calibrated[0] > thre_max) && (SENSOR.Calibrated[1] < thre_max && SENSOR.Calibrated[2] < thre_max && SENSOR.Calibrated[3] < thre_max)){
		max_frag=1;
	}
	if(SENSOR.Calibrated[1] > thre_max || SENSOR.Calibrated[2] > thre_max || SENSOR.Calibrated[3] > thre_max){
		max_frag=0;
		LED1=LED_OFF;
	}	
	if((SENSOR.Calibrated[1] < thre_max && SENSOR.Calibrated[2] < thre_max && SENSOR.Calibrated[3] < thre_max) && (SENSOR.Calibrated[4] > thre_max)){
		min_frag=1;
	}
	if(SENSOR.Calibrated[1] > thre_max || SENSOR.Calibrated[2] > thre_max || SENSOR.Calibrated[3] > thre_max){
		min_frag=0;
		LED1=LED_OFF;
	}

	/***クロス判定***/
	if(FLAG.Goal==2){
		if(SENSOR.Calibrated[1] > 250 && SENSOR.Calibrated[2] > 250 && SENSOR.Calibrated[3] > 250){
			FLAG.Goal=3;
			ENC.Local=ENC.R_L;
			LED2=LED_ON;
			GAIN.SensP=FGAIN.SensP*0.1;
			K_P_GYRO=first_K_G*0.1;
		}
	}
	if(FLAG.Goal==3){
		if(((ENC.R_L-ENC.Local)*mmpp) > 100){
			LED2=LED_OFF;
			FLAG.Goal=2;	
			GAIN.SensP=FGAIN.SensP;
			K_P_GYRO=first_K_G;
		}
	}	

	/***ライン位置計算***/
//	if(FLAG.Goal==3 || (SENSOR.Calibrated[1] < 900 || SENSOR.Calibrated[3] < 900)){
//		line_place=SENSOR.Calibrated[1]-SENSOR.Calibrated[3];	
//		SENSOR.Err=line_place;
//	}
//	if(FLAG.Goal!=3 || (SENSOR.Calibrated[1] > 900)){
//		line_place=(SENSOR.Calibrated[1]+(SENSOR.Calibrated[0]*1.1))-SENSOR.Calibrated[3];	
//		SENSOR.Err=line_place;
//	}
//	if(FLAG.Goal!=3 || (SENSOR.Calibrated[3] > 900)){
//		line_place=SENSOR.Calibrated[1]-(SENSOR.Calibrated[3]+(SENSOR.Calibrated[4]*1.1));	
//		SENSOR.Err=line_place;
//	}

	line_place=SENSOR.Calibrated[1]+(SENSOR.Calibrated[0]*1.5);	
	line_place-=(SENSOR.Calibrated[3]+(SENSOR.Calibrated[4]*1.5));
	if(SENSOR.Calibrated[1]>SENSOR.Calibrated[3]){
		line_place+=((398-SENSOR.Calibrated[2])*1.1);
	}
	if(SENSOR.Calibrated[1]<=SENSOR.Calibrated[3]){
		line_place-=((398-SENSOR.Calibrated[2])*1.1);
	}
	SENSOR.Err=line_place;
	
	if(SENSOR.Err < 100 && SENSOR.Err > -100){
		K_P_GYRO=first_K_G*0.5;
	}
	else{
		K_P_GYRO=first_K_G;	
	}
	
	if(max_frag==1){
		LED1=LED_ON;
		SENSOR.Err=1300;	
		K_P_GYRO=first_K_G*1.4;
	}
	if(min_frag==1){
		LED1=LED_ON;
		SENSOR.Err= -1300;	
		K_P_GYRO=first_K_G*1.4;
	}
	SENSOR.PreErr=SENSOR.Err-SENSOR.Pre;
}

void mot_drive(float u_r, float u_l)
{
	if(u_r > 0.0){
		MOT_ctr_R1=1;
		MOT_ctr_R2=0;			
		MOT_R=u_r;
	}
	else if(u_r <= 0.0){
		MOT_ctr_R1=0;
		MOT_ctr_R2=1;			
		u_r=u_r*(-1.0);
		MOT_R=u_r;
	}
		
	if(u_l > 0.0){	
		MOT_ctr_L1=0;
		MOT_ctr_L2=1;
		MOT_L=u_l;
	}
	else if(u_l <= 0.0){	
		MOT_ctr_L1=1;
		MOT_ctr_L2=0;
		u_l=u_l*(-1.0);
		MOT_L=u_l;
	}	
}

void init_log()
{
	short i=0;
	
	for(i=0; i<COURSE_SIZE; i++){
		COURSE_DATA.Right[i]=0;
		COURSE_DATA.Left[i]=0;
		COURSE_DATA.Dis[i]=0;
		COURSE_DATA.Course[i]=0;
		COURSE_DATA.AccelDis[i]=0;
		VEL.Now[i]=0;
	}
}

char calc_vel(float FirstVel, float Accel, float NowDis, float NowCurve)
{
	int i=0;
	char VelNo=0;
	float AccelDis=0.0;
	
	for(i=0; i<PARA_SIZE; i++){
		if(fabsf(NowDis) < 300.0){
			VelNo=0;
			break;
		}
		else{
			if(fabsf(NowCurve) > 500.0){
				AccelDis=((SpeedPara[i]*SpeedPara[i]) - (FirstVel*FirstVel))/Accel;
				if(fabsf(NowDis) > AccelDis){
					VelNo=i;
				}
				else{
					break;
				}

				if(i > ((PARA_SIZE/2)-2)){
					break;
				}	
			}
			else{
				AccelDis=((SpeedPara[i+10]*SpeedPara[i+10]) - (FirstVel*FirstVel))/Accel;
				if(fabsf(NowDis) > AccelDis){
					if(fabsf(NowCurve) >= 150.0){
						VelNo=i+10;
					}
					else{
						VelNo=0;
					}
				}
				else{
					break;
				}
					
				if(i+10 > PARA_SIZE-2){
					break;
				}	
			}
		}
	}
	return VelNo;
}

void goal_check2()
{	
	/***スタート処理***/
	if(FLAG.Goal==0 && SENSOR.Calibrated[GOAL] > SENS_THRE){
		FLAG.Goal=1;
		LED2=LED_ON;
		ENC.Local=ENC.R_L;
		ENC.Count=0;
	}
	if(FLAG.Goal==1){
		if(((ENC.R_L-ENC.Local)*mmpp) > 50){
			FLAG.Goal=2;
			ENC.R_L=0;	
			ENC.Right=0;	
			ENC.Left=0;	
		}		
	}
		
	/***ゴール判定***/
	if(FLAG.Goal==2 && SENSOR.Calibrated[GOAL] > SENS_THRE){
		LED2=LED_OFF;
		FLAG.Goal=4;		
	}
	if(FLAG.Goal==4){
		if(VEL.Desire > 0.0){
			VEL.Desire-=(2000.0/1000.0);
		}
		else{
			VEL.Desire=0.0;
			FLAG.Stop=0;
		}
	}
}

void corner_check2()
{	
	if(FLAG.Goal==2 && FLAG.Corner==0){
		if(SENSOR.Calibrated[CORNER] > SENS_THRE){
			FLAG.Corner=1;
			ENC.Count++;		
			ENC.R_L=0;
			ENC.Right=0;	
			ENC.Left=0;	
			LED1=LED_ON;
		}
	}	
	if(FLAG.Corner==1){
		BUZZ_ON5();
		if((ENC.R_L*mmpp) > 40){
			LED1=LED_OFF;
			FLAG.Corner=0;		
		}
	}	
}	
	
void goal_check()
{
	short i=0;
	short curvature=0;
	long rl=0;
	
	/***スタート処理***/
	if(FLAG.Goal==0 && SENSOR.Calibrated[GOAL] > SENS_THRE){
		FLAG.Goal=1;
		LED2=LED_ON;
		ENC.Local=ENC.R_L;
		SENSOR.Center=SENSOR.Calibrated[2];
	}
	if(FLAG.Goal==1){
		if(((ENC.R_L-ENC.Local)*mmpp) > 50){
			FLAG.Goal=2;
			ENC.R_L=0;	
			ENC.Right=0;	
			ENC.Left=0;	
		}		
	}
		
	/***ゴール判定***/
	if(FLAG.Goal==2 && SENSOR.Calibrated[GOAL] > SENS_THRE){
		LED2=LED_OFF;
		FLAG.Goal=4;		
		COURSE_DATA.Right[ENC.Count]=ENC.Right;
		COURSE_DATA.Left[ENC.Count]=ENC.Left;
	}
	if(FLAG.Goal==4){
		if(VEL.Desire > 0.0){
			VEL.Desire-=(2000.0/1000.0);
		}
		else{
			VEL.Desire=0.0;
			for(i=0; i<COURSE_SIZE; i++){
				COURSE_DATA.Dis[i]=0;
				COURSE_DATA.Course[i]=0;
				
				rl=(COURSE_DATA.Right[i]+COURSE_DATA.Left[i])/2;
				COURSE_DATA.Dis[i]=(short)(rl*mmpp);
				curvature=(short)(rl*TRED/(COURSE_DATA.Right[i]-COURSE_DATA.Left[i]));
				COURSE_DATA.Course[i]=curvature;
			}
			FLAG.Stop=0;
		}
	}
}

void corner_check()
{	
	if(FLAG.Goal==2 && FLAG.Corner==0){
		if(SENSOR.Calibrated[CORNER] > SENS_THRE){
			FLAG.Corner=1;
			COURSE_DATA.Right[ENC.Count]=ENC.Right;
			COURSE_DATA.Left[ENC.Count]=ENC.Left;
			COURSE_DATA.Angle[ENC.Count]=(char)Angle;
			ENC.Count++;		
			ENC.R_L=0;
			ENC.Right=0;	
			ENC.Left=0;	
			Angle=0.0;
			LED1=LED_ON;
		}
	}	
	if(FLAG.Corner==1){
		BUZZ_ON5();
		if((ENC.R_L*mmpp) > 70){
			LED1=LED_OFF;
			FLAG.Corner=0;		
		}
	}	
}	
	
void TIMER_CALL()
{
	float u,u_w=0.0;
	short batt=0;
	volatile float MOT_IN_R=0.0;
	volatile float MOT_IN_L=0.0;
	long now_dis=0;
	short buf[4];
	
	MTU4.TSR.BIT.TGFA=0;	//割り込み許可
	TimeCount++;
		
	if(start0==1){								
		ENC.Right+=(ENC_R-15000);
		ENC.Left+=(ENC_L-15000);
		ENC.R_L=(ENC.Right+ENC.Left)/2;		
		VEL.Right=(float)(ENC_R-15000)*mmpp/0.001;	// mm/s
		VEL.Left=(float)(ENC_L-15000)*mmpp/0.001;	// mm/s			
		VEL.R_L=(VEL.Left+VEL.Right)/2.0;
				
		/***モータへ入力***/
		if(FLAG.FirstAccel==0){
			if(VEL.Desire < VEL.BasicDesire){
				VEL.Desire+=(2000.0/1000.0);
			}
			else{
				GAIN.SensP=FGAIN.SensP;			
				VEL.Desire=VEL.BasicDesire;
				FLAG.FirstAccel=1;
			}
		}
		/******/
		get_gc_sens();
		calc_sens();
		Desire_gyro=1.1*(SENSOR.Err);
		switch(FLAG.Circle){
			case 1:
				goal_check();
				corner_check();
				break;
			
			case 2:
				goal_check2();
				corner_check2();

				/***加速処理***/
				now_dis=ENC.R_L*mmpp;
				if(FLAG.Goal>0 && FLAG.Goal!=4){
					if(now_dis <= COURSE_DATA.AccelDis[ENC.Count]){
						if(VEL.Desire < SpeedPara[VEL.Now[ENC.Count]]){
							VEL.Desire+=(accel/1000.0);
						}
						else{
							VEL.Desire=SpeedPara[VEL.Now[ENC.Count]];
						}
					}
					else if((now_dis > COURSE_DATA.AccelDis[ENC.Count]) && (now_dis < (COURSE_DATA.Dis[ENC.Count]-COURSE_DATA.AccelDis[ENC.Count]))){
						if(VEL.Desire < SpeedPara[VEL.Now[ENC.Count]]){
							VEL.Desire+=(accel/1000.0);
						}
						else{
							VEL.Desire=SpeedPara[VEL.Now[ENC.Count]];
						}
					
						if(VEL.Now[ENC.Count] > 0 && VEL.Now[ENC.Count] < 10){
							K_P_GYRO=first_K_G*0.7;
							K_D_GYRO=first_K_D*15.0;
						}
					}
					else if((now_dis >= (COURSE_DATA.Dis[ENC.Count]-COURSE_DATA.AccelDis[ENC.Count]))){
						if(VEL.Desire > VEL.BasicDesire){
							VEL.Desire-=(accel/1000.0);
						}
						else{
							K_D_GYRO=first_K_D;
							VEL.Desire=VEL.BasicDesire;
						}
					}
				}
				break;
		}	
		
		/******/
		VEL.Pre=VEL.Err;
		VEL.Err=VEL.R_L-VEL.Desire;
		VEL.SumErr+=VEL.Err;
		VEL.PreErr=VEL.Err-VEL.Pre;
		u=(GAIN.VelP*VEL.Err)+(GAIN.VelI*VEL.SumErr)+(GAIN.VelD*VEL.PreErr);
		
		/******/
		PRE_GYRO=GYRO_ERR;
		GYRO_ERR=Desire_gyro-gyro;	
		GYRO_SUM_ERR+=GYRO_ERR;
		PRE_GYRO_ERR=GYRO_ERR-PRE_GYRO;
		u_w=(K_P_GYRO*GYRO_ERR)+(K_I_GYRO*GYRO_SUM_ERR)+(K_D_GYRO*PRE_GYRO_ERR);
		
		/******/
		batt=get_batt();
		speed_R=VEL.Desire*1.5;
		speed_L=VEL.Desire*1.5;
		
		AngVelRate=((0.000714*fabsf(Desire_gyro))+1.0);
		if(Desire_gyro >= 0.0){
			speed_R*=AngVelRate;
			speed_L*=(2.0-AngVelRate);
		}
		if(Desire_gyro < 0.0){
			speed_R*=(2.0-AngVelRate);
			speed_L*=AngVelRate;
		}
		MOT_IN_R=(speed_R + u_w - u)*PWM_MAX/batt;	
		MOT_IN_L=(speed_L - u_w - u)*PWM_MAX/batt;	
		mot_drive(MOT_IN_R, MOT_IN_L);
		
		if(FLAG.Circle==3){		
			if(LOG.Timing==10){
				if(LOG.Count<1000){
					buf[3]=(short)Angle;
					buf[2]=(short)VEL.R_L;
					buf[1]=Desire_gyro;
					buf[0]=gyro;
					if(LOG.Count<256){
						Date_flash_write(11,LOG.Count,buf);
					}
					if(LOG.Count>=256 && LOG.Count<256*2){
						Date_flash_write(12,LOG.Count-256,buf);
					}
					if(LOG.Count>=256*2 && LOG.Count<256*3){
						Date_flash_write(13,LOG.Count-(256*2),buf);
					}
					if(LOG.Count>=256*3 && LOG.Count<256*4){
						Date_flash_write(14,LOG.Count-(256*3),buf);
					}
					if(LOG.Count>=256*4){
					}
					
					LOG.Count++;
				}
				LOG.Timing=0;
			}
			LOG.Timing++;
		}
		init_ENC();
	}
}

void TIMER_CALL2()
{
	MTU3.TSR.BIT.TGFA=0;	//割り込み許可
	if(start0==1){								
		gyro=gyro_sensitybity*get_gyro();
		if(FLAG.Goal!=3 && FLAG.Goal > 0){
			Angle+=((float)gyro*0.001);
		}
	}
}

void TIMER_CALL3()
{
	MTU6.TSR.BIT.TGFA=0;	//割り込み許可

	if(start0==1){								
		/*********セーフティ処理**********/
		if(SENSOR.Calibrated[0]<100 && SENSOR.Calibrated[1]<100 && SENSOR.Calibrated[2]<100 && SENSOR.Calibrated[3]<100 && SENSOR.Calibrated[4]<100){
			EG_stop_count++;
		}
		if(EG_stop_count==800){
			EG_stop();
			FLAG.Stop=0;
		}
		if(EG_stop_count<100 &&  (SENSOR.Calibrated[0]>100 || SENSOR.Calibrated[1]>100 || SENSOR.Calibrated[2]>100 || SENSOR.Calibrated[3]>100 || SENSOR.Calibrated[4]>100)){
			EG_stop_count=0;
		}
	}
}

void TIMER_WAIT(unsigned short ms)
{
	unsigned long TimeNow = TimeCount;
	while((TimeCount-TimeNow) <= ms );
}

void out_course()
{
	short i=0;

	for(i=0; i<COURSE_SIZE; i++){
		dec_out(COURSE_DATA.Dis[i], 5); outs(" ");
		dec_out(COURSE_DATA.Course[i], 6); outs(" ");
		VEL.Now[i]=calc_vel(VEL.BasicDesire, accel, COURSE_DATA.Dis[i], COURSE_DATA.Course[i]);
		COURSE_DATA.AccelDis[i]=((SpeedPara[VEL.Now[i]]*SpeedPara[VEL.Now[i]]) - (VEL.BasicDesire*VEL.BasicDesire))/(2.0*accel);
		dec_out((short)VEL.Now[i], 6); outs(" ");
		dec_out((short)COURSE_DATA.AccelDis[i], 6); outs(" ");
		dec_out((short)COURSE_DATA.Angle[i], 6); outs(" ");
		outs("\n");
	}
}

void out_para()
{
	short i=0;
	for(i=0; i<300; i++){
		outs("\n");
	}
}

void para_load(void)
{
	unsigned short *load;
	long tmp=0;
	short i=0;
	
	for(i=0; i<256; i++){
		load=Data_flash_read(11,i);
		outs("0: ");
		tmp = load[0];
		dec_out((short)tmp, 6);
		outs(" ");
		outs("1: ");
		tmp = load[1];
		dec_out((short)tmp, 6);
		outs(" ");
		outs("2: ");
		tmp = load[2];
		dec_out((short)tmp, 6);
		outs(" ");
		outs("3: ");
		tmp = load[3];
		dec_out((short)tmp, 6);
		outs("\n");
	}
	for(i=0; i<256; i++){
		load=Data_flash_read(12,i);
		outs("0: ");
		tmp = load[0];
		dec_out((short)tmp, 6);
		outs(" ");
		outs("1: ");
		tmp = load[1];
		dec_out((short)tmp, 6);
		outs(" ");
		outs("2: ");
		tmp = load[2];
		dec_out((short)tmp, 6);
		outs(" ");
		outs("3: ");
		tmp = load[3];
		dec_out((short)tmp, 6);
		outs("\n");
	}
	for(i=0; i<256; i++){
		load=Data_flash_read(13,i);
		outs("0: ");
		tmp = load[0];
		dec_out((short)tmp, 6);
		outs(" ");
		outs("1: ");
		tmp = load[1];
		dec_out((short)tmp, 6);
		outs(" ");
		outs("2: ");
		tmp = load[2];
		dec_out((short)tmp, 6);
		outs(" ");
		outs("3: ");
		tmp = load[3];
		dec_out((short)tmp, 6);
		outs("\n");
	}
	for(i=0; i<256; i++){
		load=Data_flash_read(14,i);
		tmp = load[0];
		outs("0: ");
		tmp = load[0];
		dec_out((short)tmp, 6);
		outs(" ");
		outs("1: ");
		tmp = load[1];
		dec_out((short)tmp, 6);
		outs(" ");
		outs("2: ");
		tmp = load[2];
		dec_out((short)tmp, 6);
		outs(" ");
		outs("3: ");
		tmp = load[3];
		dec_out((short)tmp, 6);
		outs("\n");
	}
}


void search_max()
{
	int i;
	short loop_frag=0;
	
	while(loop_frag==0){
		reset();
		if(SW == SW_ON){
			LED_manager(1);
			loop_frag=1;
		}
		else{}
	}
	TIMER_WAIT(1000);
	LED_manager(2);
	
	for(i=0; i<SensSize; i++){
		SENSOR.MaxValue[i]=1;
		SENSOR.MinValue[i]=10;
	}
	while(1){
		get_sens();
		get_gc_sens();
		for(i=0; i<SensSize; i++){
			if(SENSOR.MaxValue[i] < SENSOR.Original[i]){
				SENSOR.MaxValue[i]=SENSOR.Original[i];
			}
			if(SENSOR.Original[i] > 0 && SENSOR.MinValue[i] > SENSOR.Original[i]){
				SENSOR.MinValue[i]=SENSOR.Original[i];
			}
		}	
		if(SW == SW_ON){
			LED_manager(0);
			break;
		}
	}
	ENC_R=0;
	ENC_L=0;
	TIMER_WAIT(500);
	LED_manager(3);
	TIMER_WAIT(500);
	LED_manager(0);
	reset();

	for(i=0; i<SensSize; i++)
		SENSOR.Max_Min[i]=SENSOR.MaxValue[i]-SENSOR.MinValue[i];
}

void r_trace2()
{
	init_param();
	FLAG.Reverse=1;
	ch_para();
	FLAG.Circle=2;
	VEL.Desire=0.0;
	init_ENC();
	FLAG.Stop=1;
	LOG.LocalTime=TimeCount;
	fld_erase_2KB(11);
	fld_erase_2KB(12);
	fld_erase_2KB(13);
	fld_erase_2KB(14);
	start0=1;
	mot_STB(START_A);

	while(FLAG.Stop){}
	start0=0;
	mot_STB(0);
	init_ENC();	
	TIMER_WAIT(1000);	
	reset();
}

void r_trace()
{
	init_param();
	init_log();
	FLAG.Reverse=1;
	ch_para();
	init_ENC();
	FLAG.Circle=1;
	VEL.Desire=0.0;
	FLAG.Stop=1;
	LOG.LocalTime=TimeCount;
	fld_erase_2KB(11);
	fld_erase_2KB(12);
	fld_erase_2KB(13);
	fld_erase_2KB(14);
	start0=1;
	mot_STB(START_A);

	while(FLAG.Stop){}
	start0=0;
	mot_STB(0);
	init_ENC();	
	TIMER_WAIT(1000);	
	reset();
}

void trace2()
{
	init_param();
	ch_para();
	FLAG.Circle=2;
	VEL.Desire=0.0;
	init_ENC();
	FLAG.Stop=1;
	LOG.LocalTime=TimeCount;
	fld_erase_2KB(11);
	fld_erase_2KB(12);
	fld_erase_2KB(13);
	fld_erase_2KB(14);
	start0=1;
	mot_STB(START_A);

	while(FLAG.Stop){}
	start0=0;
	mot_STB(0);
	init_ENC();	
	TIMER_WAIT(1000);	
	reset();
}

void trace()
{
	init_param();
	init_log();
	ch_para();
	init_ENC();
	FLAG.Circle=1;
	VEL.Desire=0.0;
	FLAG.Stop=1;
	LOG.LocalTime=TimeCount;
	fld_erase_2KB(11);
	fld_erase_2KB(12);
	fld_erase_2KB(13);
	fld_erase_2KB(14);
	start0=1;
	mot_STB(START_A);

	while(FLAG.Stop){}
	start0=0;
	mot_STB(0);
	init_ENC();	
	TIMER_WAIT(1000);	
	reset();
}

void out_vel()
{
	init_param();
	ch_para();
	VEL.Desire=0.0;
	init_ENC();
	FLAG.Stop=1;
	start0=1;
	mot_STB(START_A);

	while(1){
		dec_out((short)ENC_R, 5); outs(" ");
		dec_out((short)ENC_L, 5); outs(" ");
		dec_out((short)VEL.Right, 5); outs(" ");
		dec_out((short)VEL.Left, 5); outs(" ");
		dec_out((short)VEL.R_L, 5); outs("\n");
	}
}

void out_enc()
{
	short tmp=0;
	short data, pre_data=0;
	ENC_R=1000;
	while(1){
		data=(short)ENC_R;
		dec_out(data-pre_data,5); outs(" ");
		
		tmp=(short)(ENC_R);
		dec_out(tmp,5); outs(" ");
		tmp=(short)(ENC_R*mmpp);
		dec_out(tmp,5); outs(" ");
		tmp=(short)(ENC_L);
		dec_out(tmp,5); outs(" ");
		tmp=(short)(ENC_L*mmpp);
		dec_out(tmp,5); outs("\n");

		pre_data=data;
	}
}

void out_gyro()
{
	short tmp=0;
	
	while(1){
		tmp=gyro_sensitybity*get_gyro();
		dec_out(tmp,4); 
		outs("\n");
	}
		
}

void out_sens()
{
	short i;
	short line_place=0;
		
	SENSOR.Pre=SENSOR.Err;
	while(1){
		get_sens();
		get_gc_sens();
		for(i=0; i<SensSize; i++){
			if(SENSOR.Original[i]<0)
				SENSOR.Original[i]*=(-1);			
			SENSOR.Calibrated[i] = 400*(SENSOR.Original[i]-SENSOR.MinValue[i])/SENSOR.Max_Min[i];	
			dec_out(SENSOR.Calibrated[i],6); outs(" ");
		}
		
		/***復帰処理***/
		if((SENSOR.Calibrated[0] > thre_max) && (SENSOR.Calibrated[1] < thre_max && SENSOR.Calibrated[2] < thre_max && SENSOR.Calibrated[3] < thre_max)){
			max_frag=1;
		}
		if(SENSOR.Calibrated[1] > thre_max || SENSOR.Calibrated[2] > thre_max || SENSOR.Calibrated[3] > thre_max){
			max_frag=0;
			LED1=LED_OFF;
		}	
		if((SENSOR.Calibrated[1] < thre_max && SENSOR.Calibrated[2] < thre_max && SENSOR.Calibrated[3] < thre_max) && (SENSOR.Calibrated[4] > thre_max)){
			min_frag=1;
		}
		if(SENSOR.Calibrated[1] > thre_max || SENSOR.Calibrated[2] > thre_max || SENSOR.Calibrated[3] > thre_max){
			min_frag=0;
			LED1=LED_OFF;
		}
		
		line_place=SENSOR.Calibrated[1]+(SENSOR.Calibrated[0]*1.5);	
		line_place-=(SENSOR.Calibrated[3]+(SENSOR.Calibrated[4]*1.5));
		if(SENSOR.Calibrated[1]>SENSOR.Calibrated[3]){
			line_place+=((398-SENSOR.Calibrated[2])*1.1);
		}
		if(SENSOR.Calibrated[1]<SENSOR.Calibrated[3]){
			line_place-=((398-SENSOR.Calibrated[2])*1.1);
		}
		SENSOR.Err=line_place;		
		
		if(max_frag==1){
			LED1=LED_ON;
			SENSOR.Err=1200;	
		}
		if(min_frag==1){
			LED1=LED_ON;
			SENSOR.Err= -1200;	
		}
			
		dec_out(SENSOR.Err,4); outs("\n");
	}
}

short select_mode()
{
	short para_frag=1;
 	short para_count=1;
 	ENC_L=1601;
	
 	while(para_frag){
		if(ENC_L > 1600 && ENC_L < 1600*2){
			LED_manager(0);
			para_count=0;
		}
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

		if(ENC_L > 1600*5)
			ENC_L=1600+1;
			
		if(SW == SW_ON){
			BUZZ_ON4();
			reset();
			TIMER_WAIT(1000);
			para_frag=0;
		}
	}
	return para_count;
}

void ch_mode(int mode)
{
	reset();
	switch(mode){
		case 0: search_max(); 	break;
		case 1: out_course();	break;
		case 2: trace();		break;
		case 3: trace2();		break;
	}
}

void ch_debug_mode(int mode)
{
	reset();
	switch(mode){
		case 0: search_max(); 	break;
		case 1: para_load();	break;
		case 2: r_trace();		break;//out_sens();		break;
		case 3: out_gyro();		break;
	}
}

void main()
{		
	set();
	BUZZ_ON3();
	battery_check();
	write_data(0x20, 0x0f);  //パワーダウン解除
	write_data(0x23, 0x30);  //2000dpsに
	
	if(SW==SW_OFF){
		while(1){
			ch_mode(select_mode());
		}
	}
	else{
		LED_manager(3);
		BUZZ_ON();
		TIMER_WAIT(1000);
		reset();
		while(1){
			ch_debug_mode(select_mode());
		}
	}		
}

