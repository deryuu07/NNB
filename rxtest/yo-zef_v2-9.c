#include <machine.h>
#include "iodefine.h"

#define ON 1
#define OFF 0 

#define corner 6
#define corner2 7
#define goal 8

#define LED1 PORTD.DR.BIT.B1	//●○○ 
#define LED2 PORTD.DR.BIT.B2	//○●○
#define LED3 PORT9.DR.BIT.B2	//○○●
#define LED4 PORT9.DR.BIT.B3	//○○●

#define V_LED1  PORT8.DR.BIT.B0	//    12345
#define V_LED2  PORT7.DR.BIT.B4	// 67        8
#define V_LED3  PORT7.DR.BIT.B6	//	   9 10
#define V_LED4  PORT8.DR.BIT.B1  
#define V_LED5  PORT7.DR.BIT.B5
#define	V_LED6  PORT1.DR.BIT.B0	//corner1
#define	V_LED7  PORT8.DR.BIT.B2 //corner2
#define	V_LED8  PORT9.DR.BIT.B1 //goal


#define MOT_ctr_L1 PORTA.DR.BIT.B0
#define	MOT_ctr_L2 PORTA.DR.BIT.B1
#define MOT_ctr_R1 PORTA.DR.BIT.B2
#define	MOT_ctr_R2 PORTA.DR.BIT.B3
#define MOT_STBY   PORTA.DR.BIT.B4		

#define START_A 1
#define STOP 0

#define ENC_R MTU1.TCNT
#define ENC_L MTU2.TCNT

short SEN_OFF[9];
short SEN[9];
short SEN_max[9];
short SEN_min[9];
short SEN_minmax[9];
short MAX_SEN[9];
short MIN_SEN[9];
short ave_SEN[9];
short sum_SEN[9];
short cari_SEN[9];

short gyro=0;
short gyro2=0;
short cari_frag=0;
short get_sens_frag=0;
short ave_count=0;
	
unsigned long TimeCount=0;

void set()	//各種機能初期設定
{
	PORTD.DDR.BYTE = 0x06;
	PORTA.DDR.BYTE = 0x1f;
	PORTB.DDR.BYTE = 0x80;
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
    CMT0.CMCOR = 4000;       	//0.5ms周期で割り込み
	ICU.IER[0x03].BIT.IEN4=1;	//割り込み要求許可
	ICU.IPR[0x04].BIT.IPR=10;	//割り込み優先レベル変更
    CMT.CMSTR0.BIT.STR0 = 1; 	//CMTカウントスタート
	
	MSTP_AD0=0;					//10bitAD変換 モジュールストップ状態の解除
  	AD0.ADCSR.BIT.ADST  = 0;	// AD変換停止
  	AD0.ADCR.BIT.MODE   = 0;	// AD変換モード選択 シングルモード
  	AD0.ADCSR.BIT.ADIE  = 0;	// 割込み禁止
  	AD0.ADCR.BIT.CKS    = 0;	// 周辺動作クロック φ/8
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
	LED4=OFF;
}

void mot_STB(int start_stop)
{
	if(start_stop==START_A)
		MOT_STBY=1;
	else{
		MOT_STBY=0;
	}
}

void mot_brake()		//モータをブレーキ状態に
{
	MOT_ctr_L1=1;
	MOT_ctr_L2=1;
	MOT_ctr_R1=1;
	MOT_ctr_R2=1;
}

void get_sens()
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
	SEN[7]=AD0.ADDRH-SEN_OFF[7];
	V_LED7=OFF;		
	for(count=0; count<101; count++){}

	V_LED8=ON;		
	for(count=0; count<201; count++){}	//LEDが点灯しきるまで少し待機
	AD0.ADCSR.BIT.CH=5;				//AD変換するアナログ入力チャンネルを設定  AD0.ADDRI
	AD0.ADCSR.BIT.ADST = 1;			// AD変換開始
	while(AD0.ADCSR.BIT.ADST==1);	//AD変換終了まで待機
	AD0.ADCSR.BIT.ADST  = 0;		// AD停止
	SEN[8]=AD0.ADDRF-SEN_OFF[8];
	V_LED8=OFF;		
	for(count=0; count<101; count++){}
		
	S12AD1.ADCSR.BIT.ADST=0;			//A/D変換停止		
	S12AD1.ADANS.BIT.CH=1;				//AN101をA/D変換
	S12AD1.ADCSR.BIT.ADST=1;			//A/D変換開始
	while(S12AD1.ADCSR.BIT.ADST==1);	//A/D変換が終わるまで待つ
	gyro=S12AD1.ADDR1;					//LEDを消した状態での値を格納
	gyro2=S12AD1.ADDR1>>6;	
}

int calc_senc()
{
	short i;
	
	get_sens();
	for(i=1; i<9; i++){
		cari_SEN[i] = (SEN[i]-SEN_min[i])/SEN_minmax[i];	
		sum_SEN[i] += cari_SEN[i];
		
		if(MAX_SEN[i] < cari_SEN[i])
			MAX_SEN[i] = cari_SEN[i];
		
		if(MIN_SEN[i] > cari_SEN[i])
			MIN_SEN[i] = cari_SEN[i];
	}
	ave_count++;
	if(ave_count==5){		
		for(i=1; i<9; i++){
			ave_SEN[i]=(sum_SEN[i]-MAX_SEN[i]-MIN_SEN[i])/2;
			sum_SEN[i]=0;
			MAX_SEN[i]=0;
			MIN_SEN[i]=0;
		}
		ave_count=0;
	}	
	return 0;
}

void TIMER_CALL()
{
	TimeCount++;	

	if(cari_frag==1)
		get_sens();
	if(get_sens_frag==1)
		calc_senc();	
}

void TIMER_WAIT(unsigned short ms)
{
	unsigned long TimeNow = TimeCount;
	while((TimeCount-TimeNow) <= ms );
}


void sens_save(void)
{
	short buf[4];
//	long tmp;
	short i=0;
	
	fld_erase_2KB(6);
	buf[0]=0;
	for(i=1; i<9; i++){
		buf[1]=SEN_max[i];
		buf[2]=SEN_min[i];
		buf[3]=SEN_minmax[i];
		Date_flash_write(6,i,buf);
	}
/*
	tmp=(long)(test_data5*1000.0);
	buf[0]=(tmp>>16)&0xFFFF;
	buf[1]=tmp&0xFFFF;
	buf[2]=0;
	buf[3]=0;
	Date_flash_write(6,1,buf);	
*/
}

void sens_load(void)
{
	unsigned short *load;
//	long tmp;
	short i=0;
	
	for(i=1; i<9; i++){
		load=Data_flash_read(6,i);
		SEN_max[i]=load[1];
		SEN_min[i]=load[2];
		SEN_minmax[i]=load[3];
	}
/*	
	load=Data_flash_read(6,1);
	tmp=load[0];
	tmp=(tmp<<16)&0xFFFF0000;
	tmp|=(load[1]&0xFFFF);
	test_data6=(float)tmp/1000.0;
*/
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

		for(i=1; i<9; i++){
			if(SEN_max[i] < SEN[i])
				SEN_max[i]=SEN[i];
			if(SEN[i] > 0 && SEN_min[i] > SEN[i])
				SEN_min[i]=SEN[i];
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

		for(i=1; i<9; i++){
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
	MTU0.TGRC=0;
	MTU0.TGRA=0;
	TIMER_WAIT(1000);
	LED3=OFF;
	mot_STB(STOP);
	reset();
	
	for(i=1; i<9; i++)
		SEN_minmax[i]=SEN_max[i]-SEN_min[i];
}

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

short select_mode()
{
	short para_frag=0;
 	short para_count=1;

 	ENC_R=3201;
	
 	while(para_frag==0){
		if(ENC_R > 1600*2 && ENC_R < 1600*3){
			LED_manager(1);
			para_count=1;
		}
		if(ENC_R > 1600*3 && ENC_R < 1600*4){
			LED_manager(2);
			para_count=2;
		}
		if(ENC_R > 1600*4 && ENC_R < 1600*5){
			LED_manager(3);
			para_count=3;
		}
		if(ENC_R > 1600*5 && ENC_R < 1600*6){
			LED_manager(4);
			para_count=4;
		}
		if(ENC_R > 1600*6 && ENC_R < 1600*7){
			LED_manager(5);
			para_count=5;
		}
		if(ENC_R > 1600*7 && ENC_R < 1600*8){
			LED_manager(6);
			para_count=6;
		}
		if(ENC_R > 1600*8)
			ENC_R=(1600*2)+1;
			
		if(PORTE.PORT.BIT.B0 == 0){
			TIMER_WAIT(500);
			reset();
			para_frag=1;
		}
	}

	return para_count;
}

void maxmin_out()
{
	short i=0;
	
	for(i=1; i<9; i++){
		dec_out(SEN_max[i],5); outs(" ");
		dec_out(SEN_min[i],5); outs(" ");
		dec_out(SEN_minmax[i],5); outs("\n"); outs("\n");
	}
}

void sens_out()
{
	short i=0;

	get_sens_frag=1;
	while(1){
		for(i=1; i<9; i++)
			dec_out(ave_SEN[i],4); outs(" ");
		outs("\n");
	}
}
	
void ch_mode(int mode)
{
	reset();
	switch(mode){
		case 1: search_max();	break;
		case 2: sens_save();	break;
		case 3: sens_load();	break;
		case 4: maxmin_out(); 	break;
		case 5: sens_out();		break;
		case 6: 		break;
	}
}

void main()
{			
	set();
	Battery_Check();
	
	while(1){
		ch_mode(select_mode());
	}

}
