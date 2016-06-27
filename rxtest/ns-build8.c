//実装予定
//優先度の高いやつから
//1．コースログの取り方を見直す（現状，距離が正確に計測できていない）
//2．曲率ごとに速度を変える
//3．ジャイロを使用した制御
//4．ショートカット
#include <machine.h>
#include "iodefine.h"
#include "main.h"

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

void BUZZ_ON3()
{	
	short i=0;
	short buzz_hz=200;

//#ifdef BUZZ_permit	
#if 1
	for(i=0; i<200; i++){
		buzz=ON;
		_wait_usec_(buzz_hz);
		buzz=OFF;
		_wait_usec_(buzz_hz);
	}
	buzz_hz=250;
	for(i=0; i<200; i++){
		buzz=ON;
		_wait_usec_(buzz_hz);
		buzz=OFF;
		_wait_usec_(buzz_hz);
	}
	buzz_hz=200;
	for(i=0; i<200; i++){
		buzz=ON;
		_wait_usec_(buzz_hz);
		buzz=OFF;
		_wait_usec_(buzz_hz);
	}
	buzz_hz=150;
	for(i=0; i<200; i++){
		buzz=ON;
		_wait_usec_(buzz_hz);
		buzz=OFF;
		_wait_usec_(buzz_hz);
	}
#endif	
}

void BUZZ_ON4()
{	
	short i=0;
	short buzz_hz=100;

#ifdef BUZZ_permit	
	for(i=0; i<200; i++){
		buzz=ON;
		_wait_usec_(buzz_hz);
		buzz=OFF;
		_wait_usec_(buzz_hz);
	}
	TIMER_WAIT(50);
	for(i=0; i<200; i++){
		buzz=ON;
		_wait_usec_(buzz_hz);
		buzz=OFF;
		_wait_usec_(buzz_hz);
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
	
	SYSTEM.MSTPCRA.BIT.MSTPA9=0;	//MTU3モジュールの使用を許可
	MTU.TSTRA.BIT.CST3=0;			//カウント停止
	ICU.IER[0x10].BIT.IEN1=1;		//割り込み要求許可
	ICU.IPR[0x57].BIT.IPR=1;		//割り込み優先度を2に
	set_psw(0x00010000);			//PSWのIを割り込み許可に設定
	MTU3.TCR.BIT.CCLR=1;			//TGRAのコンペアマッチでTCNTクリア
	MTU3.TCR.BIT.TPSC=3;			//内部クロック：ICLK/64でカウント (1.5MHz)
	MTU3.TIER.BYTE=0x01;			//コンペアマッチAでの割り込みを許可
	MTU3.TGRA=1500;					//コンペアマッチAまでの時間を1msに設定
	MTU.TSTRA.BIT.CST3=1;			//カウントスタート

	SYSTEM.MSTPCRA.BIT.MSTPA9=0;	//MTU4モジュールの使用を許可	
	MTU.TSTRA.BIT.CST4=0;			//カウント停止
	ICU.IER[0x10].BIT.IEN6=1;		//割り込み要求許可
	ICU.IPR[0x59].BIT.IPR=2;		//割り込み優先度を１に
	set_psw(0x00010000);			//PSWのIを割り込み許可に設定
	MTU4.TCR.BIT.CCLR=1;			//TGRAのコンペアマッチでTCNTクリア
	MTU4.TCR.BIT.TPSC=3;			//内部クロック：ICLK/64でカウント (1.5MHz)
	MTU4.TIER.BYTE=0x01;			//コンペアマッチAでの割り込みを許可
	MTU4.TGRA=1500;					//割り込みが1msで1周するように設定
	MTU.TSTRA.BIT.CST4=1;			//カウントスタート

	SYSTEM.MSTPCRA.BIT.MSTPA9=0;	//MTU6モジュールの使用を許可	
	MTU.TSTRB.BIT.CST6=0;			//カウント停止
	ICU.IER[0x11].BIT.IEN6=1;		//割り込み要求許可
	ICU.IPR[0x5C].BIT.IPR=1;		//割り込み優先度を１に
	set_psw(0x00010000);			//PSWのIを割り込み許可に設定
	MTU6.TCR.BIT.CCLR=1;			//TGRAのコンペアマッチでTCNTクリア
	MTU6.TCR.BIT.TPSC=3;			//内部クロック：ICLK/64でカウント (1.5MHz)
	MTU6.TIER.BYTE=0x01;			//コンペアマッチAでの割り込みを許可
	MTU6.TGRA=750;					//コンペアマッチAまでの時間を500usに設定
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
	V_LED2=ON;		
	for(count=0; count<201; count++){}	//LEDが点灯しきるまで少し待機
	S12AD0.ADCSR.BIT.ADST=0;			//A/D変換停止
	S12AD0.ADANS.BIT.CH=1;				//AN001をA/D変換
	S12AD0.ADCSR.BIT.ADST=1;			//A/D変換開始
	while(S12AD0.ADCSR.BIT.ADST==1);	//A/D変換が終わるまで待つ
	SEN[1]=S12AD0.ADDR1-SEN_OFF[1];			
	S12AD0.ADCSR.BIT.ADST=0;			//A/D変換停止
	S12AD0.ADANS.BIT.CH=2;				//AN002をA/D変換
	S12AD0.ADCSR.BIT.ADST=1;			//A/D変換開始
	while(S12AD0.ADCSR.BIT.ADST==1);	//A/D変換が終わるまで待つ
	SEN[2]=S12AD0.ADDR2-SEN_OFF[2];			
	V_LED1=OFF;		
	V_LED2=OFF;		
	V_LED3=ON;		
	V_LED4=ON;		
	for(count=0; count<201; count++){}

	S12AD1.ADCSR.BIT.ADST=0;			//A/D変換停止
	S12AD1.ADANS.BIT.CH=0;				//AN100をA/D変換
	S12AD1.ADCSR.BIT.ADST=1;			//A/D変換開始
	while(S12AD1.ADCSR.BIT.ADST==1);	//A/D変換が終わるまで待つ
	SEN[3]=S12AD1.ADDR0A-SEN_OFF[3];			
	S12AD1.ADCSR.BIT.ADST=0;			//A/D変換停止
	S12AD1.ADANS.BIT.CH=3;				//AN103をA/D変換
	S12AD1.ADCSR.BIT.ADST=1;			//A/D変換開始
	while(S12AD1.ADCSR.BIT.ADST==1);	//A/D変換が終わるまで待つ
	SEN[4]=S12AD1.ADDR3-SEN_OFF[4];			
	V_LED3=OFF;		
	V_LED4=OFF;		
	V_LED5=ON;		
//	V_LED6=ON;		
	V_LED7=ON;		
	for(count=0; count<201; count++){}

	S12AD0.ADCSR.BIT.ADST=0;			//A/D変換停止
	S12AD0.ADANS.BIT.CH=3;				//AN003をA/D変換
	S12AD0.ADCSR.BIT.ADST=1;			//A/D変換開始
	while(S12AD0.ADCSR.BIT.ADST==1);	//A/D変換が終わるまで待つ
	SEN[5]=S12AD0.ADDR3-SEN_OFF[5];			
	AD0.ADCSR.BIT.CH=8;				//AD変換するアナログ入力チャンネルを設定  AD0.ADDRI
	AD0.ADCSR.BIT.ADST = 1;			// AD変換開始
	while(AD0.ADCSR.BIT.ADST==1);	//AD変換終了まで待機
	AD0.ADCSR.BIT.ADST  = 0;		// AD停止
	SEN[6]=AD0.ADDRI-SEN_OFF[6];
	V_LED5=OFF;		
//	V_LED6=OFF;		
//	V_LED7=ON;		
	V_LED8=ON;		
	for(count=0; count<201; count++){}

	AD0.ADCSR.BIT.CH=7;				//AD変換するアナログ入力チャンネルを設定  AD0.ADDRI
	AD0.ADCSR.BIT.ADST = 1;			// AD変換開始
	while(AD0.ADCSR.BIT.ADST==1);	//AD変換終了まで待機
	AD0.ADCSR.BIT.ADST  = 0;		// AD停止
	SEN[7]=AD0.ADDRH-SEN_OFF[7];
	AD0.ADCSR.BIT.CH=5;				//AD変換するアナログ入力チャンネルを設定  AD0.ADDRI
	AD0.ADCSR.BIT.ADST = 1;			// AD変換開始
	while(AD0.ADCSR.BIT.ADST==1);	//AD変換終了まで待機
	AD0.ADCSR.BIT.ADST  = 0;		// AD停止
	SEN[8]=AD0.ADDRF-SEN_OFF[8];
	V_LED6=OFF;		
	V_LED7=OFF;		
	V_LED8=OFF;		
//	V_LED9=ON;		
//	V_LED10=ON;		
	for(count=0; count<201; count++){}
			
//	S12AD0.ADCSR.BIT.ADST=0;			//A/D変換停止
//	S12AD0.ADANS.BIT.CH=0;				//AN000をA/D変換
//	S12AD0.ADCSR.BIT.ADST=1;			//A/D変換開始
//	while(S12AD0.ADCSR.BIT.ADST==1);	//A/D変換が終わるまで待つ
//	SEN[9]=S12AD0.ADDR0A-SEN_OFF[9];			
//	S12AD1.ADCSR.BIT.ADST=0;			//A/D変換停止
//	S12AD1.ADANS.BIT.CH=2;				//AN102をA/D変換
//	S12AD1.ADCSR.BIT.ADST=1;			//A/D変換開始
//	while(S12AD1.ADCSR.BIT.ADST==1);	//A/D変換が終わるまで待つ
//	SEN[10]=S12AD1.ADDR2-SEN_OFF[10];			
//	V_LED9=OFF;		
//	V_LED10=OFF;		
//	for(count=0; count<201; count++){}
}

void get_cg_senc()
{
	short count=0;
	
	V_LED6=OFF;
	V_LED7=OFF;
	V_LED8=OFF;

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

	V_LED7=ON;		
	for(count=0; count<201; count++){}
	AD0.ADCSR.BIT.CH=8;				//AD変換するアナログ入力チャンネルを設定  AD0.ADDRI
	AD0.ADCSR.BIT.ADST = 1;			// AD変換開始
	while(AD0.ADCSR.BIT.ADST==1);	//AD変換終了まで待機
	AD0.ADCSR.BIT.ADST  = 0;		// AD停止
	SEN[6]=AD0.ADDRI-SEN_OFF[6];
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
	for(count=0; count<201; count++){}			
}

void calc_senc()
{
	short i;
		
	get_senc();
	for(i=1; i<6; i++){
		if(SEN[i]<0)
			SEN[i]*=(-1);			
		ave_SEN[i] = 400*(SEN[i]-SEN_min[i])/SEN_minmax[i];	
	}
}

void calc_cg_senc()
{
	short i,j;
		
	get_cg_senc();
	for(i=6; i<9; i++){
		if(SEN[i]<0)
			SEN[i]*=(-1);
			
		cari_SEN[i] = 400*(SEN[i]-SEN_min[i])/SEN_minmax[i];	

 		sum_SEN[i]-=SEN_window[0][i];
		for(j=0; j < window_size-1; j++){
			SEN_window[j][i]=SEN_window[j+1][i];
		}
		SEN_window[window_size-1][i]=cari_SEN[i];
		sum_SEN[i]+=cari_SEN[i];
		ave_SEN[i]=sum_SEN[i]/window_size;
	}
}

void calc_mot_in_senc()
{
		short max;
		short max_number=1;
		short max_number_err=0;
		short i=0;
		
		calc_senc();					
		max=ave_SEN[1];
		PRE_senc=ERR;
		sub_ERR=0.0;
		
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

		switch(max_number){
			case 1: now_dis=((sens_dis[1]*ave_SEN[1])+(sens_dis[2]*ave_SEN[2]))/(ave_SEN[1]+ave_SEN[2]);
					ERR=now_dis;
					K_P=first_K_P*1.2;
					break;

			case 2: if(ave_SEN[1] > ave_SEN[3]){
						now_dis=((sens_dis[1]*ave_SEN[1])+(sens_dis[2]*ave_SEN[2]))/(ave_SEN[1]+ave_SEN[2]);
						ERR=now_dis;
						K_P=first_K_P;
					}
					else{
						now_dis=(float)(ave_SEN[2]-ave_SEN[4]);	
						ERR=now_dis;
						K_P=first_K_P;			
					}
					break;	

			case 3: now_dis=(float)(ave_SEN[2]-ave_SEN[4]);	
					ERR=now_dis;
//					ERR=0.0;
					break;

			case 4: if(ave_SEN[3] > ave_SEN[5]){
						now_dis=(float)(ave_SEN[2]-ave_SEN[4]);	
						ERR=now_dis;
						K_P=first_K_P;
					}
					else{
						now_dis=((sens_dis[5]*ave_SEN[5])+(sens_dis[4]*ave_SEN[4]))/(ave_SEN[5]+ave_SEN[4]);
						ERR=now_dis;
						K_P=first_K_P;
					}
					break;	

			case 5: now_dis=((sens_dis[5]*ave_SEN[5])+(sens_dis[4]*ave_SEN[4]))/(ave_SEN[5]+ave_SEN[4]);
					ERR=now_dis;
					K_P=first_K_P*1.2;
					break;				
		}

		if(max_frag==1 && ave_SEN[1] < thre_max && ave_SEN[2] < thre_max){
			LED2=ON;
			now_dis=480.0;
			ERR=now_dis;
//			sub_ERR=(float)(ave_SEN[9]-ave_SEN[10]);
			K_P=first_K_P*1.2;
		}

		if(min_frag==1 && ave_SEN[4] < thre_max && ave_SEN[5] < thre_max){
			LED2=ON;
			now_dis= (-480.0);					
			ERR=now_dis;
//			sub_ERR=(float)(ave_SEN[9]-ave_SEN[10]);
			K_P=first_K_P*1.2;
		}
		pre_max_number=max_number;
		PRE_ERR=ERR-PRE_senc;
//		sub_ERR=(float)(ave_SEN[9]-ave_SEN[10]);
}

void mot_STB(short start_stop)
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
		MOT_R=150 - (0.1*(ENC_R-ENC_L));
		MOT_L=150 + (0.1*(ENC_R-ENC_L));
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
		MOT_R=150 - (0.001*(ENC_R-ENC_L));
		MOT_L=150 + (0.001*(ENC_R-ENC_L));
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
	MOT_R=0;
	MOT_L=0;
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
	goal_frag3=0;
	curve_count=0;
	ENC_count=0;
	first_check=0;
	StartTimeCount=0;
	CrossTimeCount=0;
	AccelTimeCount=0.0;
	Desire_vel=Desire_vel_nomal;
	Desire_next_speed=Desire_vel_nomal;
	now_vel_R=0.0;
	now_vel_L=0.0;
	SPEED_ERR=0.0;
	SPEED_SUM_ERR=0.0;
	first_speed=0.0;
	max_frag=0;
	min_frag=0;
	now_comp=0;
	corner_frag=0;
	EG_stop_count=0;
	gyro_frag=0;
	ch_frag=0;
	shortcut_frag=0;
	fail_safe_frag=0;
	speed_count=0;
	emergency_frag=0;
	accel_safety=0;
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
	LED3=ON;
	for(i=0; i<10000; i++){}
	init_goal_frag();	
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
		if(abs(course_data[i]) > 1000){
			for(j=i; j<200; j++){
				if(abs(course_data[j+1]) > 1000){
					ENC_dis[i]+=ENC_dis[j+1];
//					gyro_data[i]+=gyro_data[j+1];
					st_count++;
				}
				else{
					if(st_count>1){
//						gyro_data[i]/=st_count;
						for(k=i; k<200; k++){
							ENC_dis[k+1]=ENC_dis[k+st_count];
							course_data[k+1]=course_data[k+st_count];
//							gyro_data[k+1]=gyro_data[k+st_count];
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
	shortcut_frag=1;
	
	for(i=0; i<200; i++){
		if(course_data[i]<0) course_data[i]*=(-1);
		continue_st[i]=0;
	}

	for(i=0; i<200-4; i++){
		if(course_data[i] > 1000 && course_data[i+1] < 1000 && course_data[i+2] < 1000 && course_data[i+3] < 1000 && course_data[i+4] > 1000){	//ST1 C3 ST1の検出
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
	}
	
//	for(i=0; i<200; i++){
//		if(course_data[i]<0) course_data[i]*=(-1);
//	}

	mot_brake();
	init_goal_frag();
	TIMER_WAIT(1000);
	mot_STB(STOP);

}	

void Accel_manager(short counter, short now_R)
{	
	float tmp = ENC_dis[counter]*mmpp;
	
	if(tmp > 2.0*accel_dis[0]){
		now_comp=0;
		if(tmp > 2.0*accel_dis[1]){
			now_comp=1;
			if((tmp > 2.0*accel_dis[2]))
				now_comp=2;
		}
	}
	
}

void calc_next_speed(short next_R)
{
	if(ENC_dis[ENC_count+1]*mmpp > 300.0){
		if(next_R < 150) 					Desire_next_speed=curve_speed_para[R10];
		if(next_R >= 150 && next_R < 200) 	Desire_next_speed=curve_speed_para[R15];
		if(next_R >= 200 && next_R < 300){	Desire_next_speed=curve_speed_para[R20];	LED_manager(10);}
		if(next_R >= 300 && next_R < 400) 	Desire_next_speed=curve_speed_para[R30];
		if(next_R >= 400 && next_R < 500) 	Desire_next_speed=curve_speed_para[R40];
		if(next_R >= 500 && next_R < 600){	Desire_next_speed=curve_speed_para[R50];	LED_manager(10);}
		if(next_R >= 600 && next_R < 700) 	Desire_next_speed=curve_speed_para[R60];
		if(next_R >= 700 && next_R < 800) 	Desire_next_speed=curve_speed_para[R70];
		if(next_R >= 800 && next_R < 900){	Desire_next_speed=curve_speed_para[R80];	LED_manager(10);}
		if(next_R >= 900 && next_R < 1000) 	Desire_next_speed=curve_speed_para[R90];
		if(next_R >= 1000 && next_R < 1500) 	Desire_next_speed=curve_speed_para[R100];
		if(next_R >= 1500 && next_R < 2000) 	Desire_next_speed=curve_speed_para[R150];
		if(next_R >= 1950 && next_R <= 2000) 	Desire_next_speed=curve_speed_para[R200];
	}
	else{
		Desire_next_speed=curve_speed_para[R10];
	}
}

void goal_check(long RIGHT_ENC, long LEFT_ENC)	//カーブ判定
{
	long distance=(RIGHT_ENC+LEFT_ENC)/2;
	float distance2=distance*mmpp;
	long angl_vel=RIGHT_ENC-LEFT_ENC;
		
	if(first_check==0){
		if(now_vel < Desire_vel_nomal){
			Desire_vel+=(2000.0/1000.0);
		}
		
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
			SPEED_SUM_ERR=0.0;
			goal_count=0;
			Desire_vel=Desire_vel_nomal;
			first_speed=now_vel;
			AccelTimeCount=0;
			LED3=ON;
		}
	}
	if(first_check==2){		
		if((ave_SEN[1] > thre_max) && (ave_SEN[5] > thre_max) && goal_frag2==1){  //クロス処理
			goal_frag2=0;
			LED1=ON;
			K_P=first_K_P*0.01;
			K_P_GYRO=first_K_G*0.0;
			pre_dis=distance2;
		}

//		if(goal_frag2==0){
//			CrossTimeCount++;
//		}
//			
//		if(CrossTimeCount==250){	
//			LED1=OFF;
//			goal_frag2=1;
//			CrossTimeCount=0;
//			gyro_frag=1;
//		}
	
		if((distance2-pre_dis > 100.0) && goal_frag2==0){	
			K_P=first_K_P;
			K_P_GYRO=first_K_G;
			LED1=OFF;
			goal_frag2=1;
			CrossTimeCount=0;
			gyro_frag=1;
			pre_dis=0.0;
		}

	/********************コーナー処理*******************************/	
		if(((ave_SEN[corner2]>SEN_thre[corner2]) || (ave_SEN[corner]>SEN_thre[corner])) && corner_frag==0 && goal_frag2==1 && (distance2 > 40.0)){
//		if(((ave_SEN[corner2]>SEN_thre[corner2])) && corner_frag==0 && goal_frag2==1 && (distance2 > 50.0)){			
			if(ave_SEN[corner] >SEN_thre[corner]) 	corner1_frag=1;
			if(ave_SEN[corner2]>SEN_thre[corner2])	corner2_frag=1;
			
			LED4=ON;
			corner_frag=1;
			if(angl_vel == 0)
				angl_vel=1;
			curvature=(RIGHT_ENC+LEFT_ENC)*Tred/(2*angl_vel);	//曲率計算
					
			now_curvature=(short)curvature;					
			ENC_dis[ENC_count]=distance;				
			if(ENC_dis[ENC_count]<0)
				ENC_dis[ENC_count]*=(-1);
			
			course_data[ENC_count]=now_curvature;
			ENC_count++;
		}
						
		if(corner_frag==1){
#ifdef BUZZ_permit	
			buzz=~buzz;
#endif
			if(corner2_frag==1){
				if(ave_SEN[corner2] < SEN_thre[corner2]){	//チャタリング対策
					LED4=OFF;
					corner1_frag=0;
					corner2_frag=0;
					corner_frag=0;
					now_ENC_R=0;
					now_ENC_L=0;
					buzz=OFF;
				}
			}
			if(corner1_frag==1){
				if(ave_SEN[corner] < SEN_thre[corner]){	//チャタリング対策
					LED4=OFF;
					corner1_frag=0;
					corner2_frag=0;
					corner_frag=0;
					now_ENC_R=0;
					now_ENC_L=0;
					buzz=OFF;
				}
			}
		}
					
		/*********************ゴール処理*******************/				
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
			else{
				angl_vel*=(-1);
				curvature=(RIGHT_ENC+LEFT_ENC)*Tred/(2*angl_vel);
			}
			//course_data[ENC_count]=(short)curvature/10;
			course_data[ENC_count]=10000;
			ENC_dis[ENC_count]=distance;
			Total_goal_count=ENC_count;
			stop_frag=1;
			goal_count=0;
		}
	}
	else{}
}

void goal_check2(long RIGHT_ENC, long LEFT_ENC, short err)	//カーブ判定
{
	long distance=(RIGHT_ENC+LEFT_ENC)/2;
	float distance2=distance*mmpp;
	float now_comp_distance=ENC_dis[ENC_count]*mmpp;
	short now_R=abs(course_data[ENC_count]);
	short next_R=abs(course_data[ENC_count+1]);
			
	if(first_check==0){
		if(now_vel < Desire_vel_nomal){
			Desire_vel+=(2000.0/1000.0);
		}

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
			goal_count=0;
			first_speed=now_vel;
			LED4=ON;
		}
	}
	if(first_check==2){		
//		if((distance2-now_comp_distance) > 200.0){	//測定距離と20cmずれたら加速禁止
////			accel_safety=1;
//			LED_manager(8);
//		}
		
		if((ave_SEN[1] > thre_max) && (ave_SEN[5] > thre_max) && goal_frag2==1){  //クロス処理
			goal_frag2=0;
//			K_P=first_K_P*0.1;
			pre_dis=distance2;
		}

		if((distance2-pre_dis > 50.0) && goal_frag2==0){	
			K_P=first_K_P;
			goal_frag2=1;
			CrossTimeCount=0;
			gyro_frag=1;
			pre_dis=0.0;
		}
					
	/********************コーナー処理*******************************/	
		if(accel_safety==0){
			if(goal_frag1==1){	
				if(corner_frag==0){
					if(now_R > 1000){
						K_D=first_K_D*2.5;
						Desire_gyro=0.0;						
						if(continue_st[ENC_count]==1 && shortcut_frag==1){
							LED2=ON;
							Desire_gyro=0.0;
							K_P=0.0;
							K_D=0.0;
							K_P_GYRO=first_K_G;
						}
					
						if((accel_dis[now_comp] > distance2) && (next_R!=0)){
							LED1=ON;
							Desire_vel+=(Accel[now_comp]/1000.0);
							if(Desire_vel > speed_para[now_comp])
								Desire_vel=speed_para[now_comp];
							calc_speed(Desire_vel);	
						}
						if(next_R==0){	//最後の区画はぶっ飛ばせ
							LED_manager(7);
							Desire_vel+=(9800.0/1000.0);
							if(Desire_vel > speed_para[2])
								Desire_vel=speed_para[2];
							calc_speed(Desire_vel);	
						}		

						if(accel_dis[now_comp] < distance2 && distance2 < (now_comp_distance-(accel_dis[now_comp]))){
							LED2=ON;
							Desire_vel=speed_para[now_comp];
							calc_speed(Desire_vel);	
						}  
						if(distance2 > (now_comp_distance-(accel_dis[now_comp]))){
							LED3=ON;
							//K_P=first_K_P*0.9;
							Desire_vel-=(Accel[now_comp]/1000.0);
							if(Desire_vel < Desire_next_speed)
								Desire_vel=Desire_next_speed;
							calc_speed(Desire_vel);	
						}					
					}
					else{						
						Desire_vel+=(Accel[0]/1000.0);
						if(Desire_vel > Desire_next_speed)
							Desire_vel=Desire_next_speed;
					}
				}

			/***曲率が小さい場合はマーカを読まない***/				
//			if((course_data[ENC_count] < 20) && distance2 >= (now_comp_distance/*-1.9*/) && goal_frag2==1){
//				ch_frag=0;
//				distance2=0.0;
//				Desire_vel=Desire_next_speed;
//				calc_speed(Desire_vel);	
//
//				K_P=first_K_P;
//				K_D=first_K_D;
//				K_P_GYRO=first_K_G;
//
//				ENC_count++;
//				first_speed=now_vel;
//				calc_accel_dis(first_speed);
//				Accel_manager(ENC_count, course_data[ENC_count]);
//				now_ENC_R=0;
//				now_ENC_L=0;			
//			}
			
				if(((ave_SEN[corner2]>SEN_thre[corner2]) || (ave_SEN[corner]>SEN_thre[corner])) && distance2 > (now_comp_distance-50.0) && corner_frag==0 && goal_frag2==1){
					if(ave_SEN[corner] >SEN_thre[corner]) 	corner1_frag=1;
					if(ave_SEN[corner2]>SEN_thre[corner2])	corner2_frag=1;
					LED3=ON;
					corner_frag=1;
					ch_frag=0;
					distance2=0.0;
					Desire_vel=Desire_next_speed;
					calc_speed(Desire_vel);	
//					sub_ERR=0.0;

					K_P=first_K_P;
					K_D=first_K_D;
					K_P_GYRO=first_K_G;
					ENC_count++;
					now_R=abs(course_data[ENC_count]);
					next_R=abs(course_data[ENC_count+1]);
					first_speed=now_vel;
					calc_accel_dis(first_speed);
					Accel_manager(ENC_count, now_R);
					calc_next_speed(next_R);
//				Desire_gyro=Desire_vel/course_data[ENC_count];
					now_ENC_R=0;
					now_ENC_L=0;
				}
				if(corner_frag==1){
#ifdef BUZZ_permit	
					buzz=~buzz;
#endif				
					if(corner2_frag==1){
						if(ave_SEN[corner2] < SEN_thre[corner2]){	//チャタリング対策
							LED3=OFF;
							corner1_frag=0;
							corner2_frag=0;
							corner_frag=0;
							now_ENC_R=0;
							now_ENC_L=0;
							buzz=OFF;
						}
					}
					if(corner1_frag==1){
						if(ave_SEN[corner] < SEN_thre[corner]){	//チャタリング対策
							LED3=OFF;
							corner1_frag=0;
							corner2_frag=0;
							corner_frag=0;
							now_ENC_R=0;
							now_ENC_L=0;
							buzz=OFF;
						}
					}
				}		
			}
		}
		
		/*********************ゴール処理*******************/				
		if((ave_SEN[goal] > SEN_thre[goal]) && goal_frag2==1 && course_data[ENC_count+5]==0 ){
			goal_frag2=0;
			stop_frag=1;
			goal_count=0;
		}
		
	}
	else{}
}

void TIMER_CALL()
{
	float u=0.0;
	float u_omega=0.0;
	float u_sen=0.0;
	float MOT_IN_R=0.0;
	float MOT_IN_L=0.0;
	float now_gyro=0.0;
		
	MTU4.TSR.BIT.TGFA=0;	//割り込み許可

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
		now_vel_R=(float)(ENC_R-15000)*mmpp/0.001;	// mm/s
		now_vel_L=(float)(ENC_L-15000)*mmpp/0.001;	// mm/s			
		now_vel=(now_vel_L+now_vel_R)/2.0;
		Desire_gyro=Desire_vel_nomal*now_dis/300.0;	//300.0*125.0
		now_gyro=(now_vel_R-now_vel_L);
		
		/***ゴールチェック***/
		switch(circl_count){
			case 1:	goal_check(now_ENC_R, now_ENC_L);
					break;

			case 2:	goal_check2(now_ENC_R, now_ENC_L, (short)ERR);
					break;
		}
		/***直線速度成分***/
		PRE_SPEED=SPEED_ERR;
		SPEED_ERR=now_vel-Desire_vel;
		SPEED_SUM_ERR+=SPEED_ERR;
		PRE_SPEED_ERR=SPEED_ERR-PRE_SPEED;
		u=(K_P_SPEED*SPEED_ERR)+(K_I_SPEED*SPEED_SUM_ERR)+(K_D_SPEED*PRE_SPEED_ERR);
		
		/***角速度成分***/
		PRE_GYRO=GYRO_ERR;
		GYRO_ERR=Desire_gyro-now_gyro;	
		GYRO_SUM_ERR+=GYRO_ERR;
		PRE_GYRO_ERR=PRE_GYRO-GYRO_ERR;
		u_omega=(K_P_GYRO*GYRO_ERR)+(K_I_GYRO*GYRO_SUM_ERR)+(K_D_GYRO*PRE_GYRO_ERR);
		
		/***ライン成分***/
		u_sen=(K_P*ERR)+(K_D*PRE_ERR);
		
		/***モータへ入力***/	
		MOT_IN_R=speed+u_sen+u_omega-u;	
		MOT_IN_L=speed-u_sen-u_omega-u;	
		if(MOT_IN_R > 0.0){
			MOT_ctr_R1=1;
			MOT_ctr_R2=0;			
			MOT_R=MOT_IN_R;
		}
		else if(MOT_IN_R < 0.0){
			MOT_IN_R*=(-1.0);
			MOT_ctr_R1=0;
			MOT_ctr_R2=1;
			MOT_R=MOT_IN_R;
		}
		if(MOT_IN_L > 0.0){	
			MOT_ctr_L1=0;
			MOT_ctr_L2=1;
			MOT_L=MOT_IN_L;
		}
		else if(MOT_IN_L < 0.0){	
			MOT_IN_L*=(-1.0);
			MOT_ctr_L1=1;
			MOT_ctr_L2=0;
			MOT_L=MOT_IN_L;
		}		
		
		init_ENC();
	}

	if(stop_frag==1){
		stop_count++;
		Desire_vel-=(4000.0/1000.0);
		if(Desire_vel < 0.0)
			Desire_vel=0.0;
	}
}

void TIMER_CALL2()
{
	MTU3.TSR.BIT.TGFA=0;	//割り込み許可
	if(start0==1){								
		calc_mot_in_senc();

		if(emergency_frag==1){
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

		if(goal_frag1==1 && speed_count < 300 && speed_timing==10){
			now_speed[speed_count]=(short)now_vel;
			speed_count++;
			speed_timing=0;
		}
		speed_timing++;
	}
}

void TIMER_CALL3()
{
	MTU6.TSR.BIT.TGFA=0;	//割り込み許可
	if(start0==1){
		calc_cg_senc();
	}
}

void TIMER_WAIT(unsigned short ms)
{
	unsigned long TimeNow = TimeCount;
	while((TimeCount-TimeNow) <= ms );
}

short get_Batt()	//バッテリ残量取得
{
	short Battery;
	
	AD0.ADCSR.BIT.CH=6;				//AD変換するアナログ入力チャンネルを設定 バッテリ AD0.ADDRG
	AD0.ADCSR.BIT.ADST = 1;			// AD変換開始
	while(AD0.ADCSR.BIT.ADST==1);	//AD変換終了まで待機
	AD0.ADCSR.BIT.ADST  = 0;		// AD停止
	Battery=AD0.ADDRG>>6;

	return Battery*9765/1000;		//mv
}

void Battery_Check()
{
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
	dec_out(battery, 6);	outs("\n");

}

void countdown()
{

	BUZZ_ON3();
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

short select_debug_mode()
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
		if(ENC_R > 1600*8 && ENC_R < 1600*9){
			LED_manager(7);
			para_count=7;
		}
		if(ENC_R > 1600*9 && ENC_R < 1600*10){
			LED_manager(8);
			para_count=8;
		}
		if(ENC_R > 1600*10 && ENC_R < 1600*11){
			LED_manager(9);
			para_count=9;
		}
		if(ENC_R > 1600*11 && ENC_R < 1600*12){
			LED_manager(10);
			para_count=10;
		}

		if(ENC_R > 1600*12)
			ENC_R=(1600*2)+1;
			
		if(SW == SW_ON){
			BUZZ_ON4();
			TIMER_WAIT(500);
			reset();
			para_frag=1;
		}
	}
	return para_count;
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
			BUZZ_ON4();
			TIMER_WAIT(500);
			reset();
			para_frag=1;
		}
	}
	return para_count;
}

char select_two()
{
	short para_frag=0;
 	short para_count=1;
 	ENC_L=3201;
	
 	while(para_frag==0){
		if(ENC_L > 1600*2 && ENC_L < 1600*3){
			LED_manager(1);
			para_count=0;
		}
		if(ENC_L > 1600*3 && ENC_L < 1600*4){
			LED_manager(8);
			para_count=1;
		}

		if(ENC_L > 1600*4)
			ENC_L=(1600*2)+1;
			
		if(SW == SW_ON){
			BUZZ_ON4();
			reset();
			para_frag=1;
		}
	}
	return para_count;
}

void calc_accel_dis(float Initial_velocity)
{	
	accel_dis[0]=((speed_para[0]*speed_para[0])-(Initial_velocity*Initial_velocity))/(2.0*Accel[0]);
	accel_dis[1]=((speed_para[1]*speed_para[1])-(Initial_velocity*Initial_velocity))/(2.0*Accel[1]);
	accel_dis[2]=((speed_para[2]*speed_para[2])-(Initial_velocity*Initial_velocity))/(2.0*Accel[2]);	

	if(accel_dis[0] < 0.0) accel_dis[0]*=(-1.0);
	if(accel_dis[1] < 0.0) accel_dis[1]*=(-1.0);
	if(accel_dis[2] < 0.0) accel_dis[2]*=(-1.0);
}

void ch_para()
{
	switch(select_mode()){
		case 1: speed=500.0;	//220
				K_P=1.0;		//13.5
				K_P_sub=0.0;
				K_D=6.0;		//0.0002
				K_I=0.0;	//12
				K_P_GYRO=0.3;
				first_K_G=K_P_GYRO;
				K_P_SPEED=0.001; //0.0005
				K_I_SPEED=0.09;  
				
				Accel[0]=5000.0;
				speed_para[0]=1500.0;
				speed_para[1]=2200.0;
				speed_para[2]=3000.0;
								
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
				K_P_GYRO=0.0;
				first_K_G=K_P_GYRO;
				K_P_SPEED=0.001;
				K_I_SPEED=0.09;  			
				
				Accel[0]=5000.0;
				speed_para[0]=1500.0;
				speed_para[1]=2200.0;
				speed_para[2]=3500.0;
				
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
				K_P_GYRO=0.0;
				first_K_G=K_P_GYRO;
				K_P_SPEED=0.0005;
				K_I_SPEED=0.09;  
				
				Accel[0]=5000.0;
				speed_para[0]=1500.0;
				speed_para[1]=2200.0;
				speed_para[2]=4000.0;
				
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
				K_P_GYRO=0.0;
				first_K_G=K_P_GYRO;
				
				Accel[0]=8000.0;
				speed_para[0]=1500.0;
				speed_para[1]=2200.0;
				speed_para[2]=4500.0;
				
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
	float selecter=1.5; //1.0

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
			K_P_sub=K_P/3.0;
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
	float selecter=6.8;

	while(select_frag==0){
		calc_senc();
		if(SW == SW_ON){
			TIMER_WAIT(200);
			BUZZ_ON();
			LED2=~LED2;
			//selecter+=0.2;
			selecter+=1.0;
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

void select_D2()
{
	int select_frag=0;
	float selecter=4.0;

	while(select_frag==0){
		calc_senc();
		if(SW == SW_ON){
			TIMER_WAIT(200);
			BUZZ_ON();
			LED2=~LED2;
			//selecter+=0.2;
			selecter+=1.0;
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
	float selecter=1.0;

	while(select_frag==0){
		calc_senc();
		if(SW == SW_ON){
			TIMER_WAIT(200);
			BUZZ_ON();
			LED2=~LED2;
			selecter+=0.1;
		}
		
		if(ave_SEN[1] > 180.0){
			K_P_GYRO=selecter;
			first_K_G=K_P_GYRO;
			select_frag=1;
			LED_manager(7);
		}
	}
	TIMER_WAIT(1000);
	reset();
}

void calc_speed(float PWM_SPEED)
{
	speed=PWM_SPEED*1.57414*PWM_MAX/battery;
}

void select_speed()
{
	int select_frag=0;
	float selecter=1000.0;
	float a=1.1;
	
	while(select_frag==0){
		calc_senc();
		if(SW == SW_ON){
			TIMER_WAIT(200);
			LED2=~LED2;
			BUZZ_ON();
			selecter+=100.0;
		}
		
		if(ave_SEN[1] > 180.0){
			Desire_vel_nomal=selecter;
			Desire_vel=Desire_vel_nomal;
			curve_speed_para[R10]=Desire_vel_nomal;
			curve_speed_para[R15]=(Desire_vel_nomal*a);
			curve_speed_para[R20]=(Desire_vel_nomal*a)+100.0;
			curve_speed_para[R30]=(Desire_vel_nomal*a)+200.0;
			curve_speed_para[R40]=(Desire_vel_nomal*a)+300.0;
			curve_speed_para[R50]=(Desire_vel_nomal*a)+400.0;
			curve_speed_para[R60]=(Desire_vel_nomal*a)+500.0;
			curve_speed_para[R70]=(Desire_vel_nomal*a)+600.0;
			curve_speed_para[R80]=(Desire_vel_nomal*a)+600.0;
			curve_speed_para[R90]=(Desire_vel_nomal*a)+600.0;
			curve_speed_para[R100]=2000.0;		//2000mm/sくらい
			curve_speed_para[R150]=2200.0;
			curve_speed_para[R200]=2500.0;	//2500mm/sくらい
			calc_speed(Desire_vel);
			
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

void select_speedgain_p()
{
	int select_frag=0;
	float selecter=K_P_SPEED;

	while(select_frag==0){
		calc_senc();
		if(SW == SW_ON){
			TIMER_WAIT(200);
			BUZZ_ON();
			LED2=~LED2;
			selecter+=0.01;
		}
		
		if(ave_SEN[1] > 180.0){
			K_P_SPEED=selecter;
			select_frag=1;
			LED_manager(7);
		}
	}
	TIMER_WAIT(1000);
	reset();
}

void select_speedgain_i()
{
	int select_frag=0;
	float selecter=K_I_SPEED;

	while(select_frag==0){
		calc_senc();
		if(SW == SW_ON){
			TIMER_WAIT(200);
			BUZZ_ON();
			LED2=~LED2;
			selecter+=0.01;
		}
		
		if(ave_SEN[1] > 180.0){
			K_I_SPEED=selecter;
			select_frag=1;
			LED_manager(7);
		}
	}
	TIMER_WAIT(1000);
	reset();
}

short select_pwm()
{
	int select_frag=0;
	short selecter=1000;
	
	ENC_R=3000;
	while(select_frag==0){
		if(ENC_R > 6000){
			TIMER_WAIT(200);
			BUZZ_ON();
			LED2=~LED2;
			selecter+=500;
			ENC_R=3000;
		}
		
		if(SW == SW_ON){
			select_frag=1;
			LED_manager(7);
		}
	}
	TIMER_WAIT(1000);
	reset();
	
	return selecter;
}

void select_accel()
{
	int select_frag=0;
	float selecter=4000.0;

	while(select_frag==0){
		calc_senc();
		if(SW == SW_ON){
			TIMER_WAIT(200);
			BUZZ_ON();
			LED2=~LED2;
			selecter+=1000.0;
		}
		
		if(ave_SEN[1] > 180.0){
			Accel[2]=selecter;
			Accel[1]=Accel[2];
			Accel[0]=Accel[2];
			select_frag=1;
			LED_manager(7);
		}
	}
	TIMER_WAIT(1000);
	reset();
}

void trace2()	//二週目以降
{
	char para_change_frag=0;
	short trace_roop_frag=0;
	short i,j=0;
	float R_local=0.0;
	float L_local=0.0;
	float u_sen_local=0.0;
		
	circl_count=2;
	LED2=ON;
	TIMER_WAIT(500);
	LED2=OFF;

	ch_para();
	para_change_frag=select_two();	
	if(para_change_frag==0){
		TIMER_WAIT(100);
		select_P();
		select_D2();
		select_G();
		para_save();
		BUZZ_ON3();
	}
	else{
		para_load();
	}
	LED_manager(7);
	TIMER_WAIT(500);
	reset();
    select_accel();
	select_speed();
	
	while(1){
		if(SW == SW_ON){
			TIMER_WAIT(500);
			while(1){
				i++;
				if(i>500){
					LED4=~LED4;
					i=0;
				}
				if(SW==SW_ON)
					break;
			}
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
	
	calc_accel_dis(Desire_vel_nomal);
	Accel_manager(0,abs(course_data[0]));
	init_ENC();
	for(i=0; i<window_size; i++){
		for(j=0; j<10; j++){
			SEN_window[i][j]=0;
			sum_SEN[j]=0;
		}
	}
	
	while(trace_roop_frag==0){
		countdown();
		TIMER_WAIT(1000);
			
		zero_gyro=get_gyro();	
		sum_gyro=0;
		init_ENC();
		ENC_count=0;		
		accel_safety=0;
		mot_STB(START_A);
		emergency_frag=1;
		Desire_vel=0.0;
		
		start0=1;
		mot_frag=1;
		while(stop_frag==0){}
				
		stop_manager();
		trace_roop_frag=1;			
	}
}

void trace()	//一週目
{
	char para_change_frag=0;
	short trace_roop_frag=0;
	short i,j=0;
	
	circl_count=1;	
	LED1=ON;
	TIMER_WAIT(500);
	LED1=OFF;
	ch_para();

	para_change_frag=select_two();	
	if(para_change_frag==0){
		TIMER_WAIT(100);
		select_P();
		select_D2();
		select_G();
		para_save();
		BUZZ_ON3();
	}
	else{
		para_load();
	}
	LED_manager(7);
	TIMER_WAIT(500);
	reset();
    select_accel();
	select_speed();
	select_circle();	
	init_thre();			
	calc_accel_dis(Desire_vel_nomal);
	init_ENC();	

	while(1){
		if(SW == SW_ON){
			TIMER_WAIT(500);
			while(1){
				i++;
				if(i>500){
					LED4=~LED4;
					i=0;
				}
				if(SW==SW_ON)
					break;
			}
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
		for(i=0; i<window_size; i++){
			for(j=0; j<10; j++){
				SEN_window[i][j]=0;
				sum_SEN[j]=0;
			}
		}
		
		zero_gyro=get_gyro();
		sum_gyro=0;
		
		init_ENC();
		ENC_count=0;
		shortcut_area=0;
		mot_STB(START_A);
		emergency_frag=1;
		Desire_vel=0.0;
		
		start0=1;
		mot_frag=1;
		LED1=OFF;
		while(stop_frag==0){}
		
		stop_manager();
		trace_roop_frag=1;
	}
}

void out_sens()
{
	TIMER_WAIT(2000);
	get_sens_frag=1;	
	while(1){
		dec_out(ave_SEN[1], 6);	outs(" ");
		dec_out(ave_SEN[2], 6);	outs(" ");
		dec_out(ave_SEN[3], 6);	outs(" ");
		dec_out(ave_SEN[4], 6);	outs(" ");
		dec_out(ave_SEN[5], 6);	outs(" ");
		dec_out(ave_SEN[6], 6);	outs(" ");	//corner1
		dec_out(ave_SEN[7], 6);	outs(" ");	//corner2
		dec_out(ave_SEN[8], 6);	outs(" ");	//goal
		dec_out(ave_SEN[9], 6);	outs(" ");	//corner2
		dec_out(ave_SEN[10], 6);	outs(" ");	//goal
		
		dec_out((short)now_dis, 6);	outs(" ");
		dec_out(ave_SEN[9]-ave_SEN[10], 6);	outs(" ");
		dec_out((short)now_dis-(ave_SEN[9]-ave_SEN[10]), 6);	outs(" ");
		
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
//		dec_out(gyro_data[i], 8); outs(" ");
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

void out_speed()
{
	short i=0;
	
	TIMER_WAIT(2000);	
	for(i=0; i<300; i++){
		dec_out(now_speed[i], 8);
//		dec_out(main_ERR[i], 4); outs(" ");
//		dec_out(second_ERR[i], 4); outs(" ");
		
		outs("\n");
	}
}

void init_thre(){
	
	SEN_thre[1]     =400;
	SEN_thre[2]     =400;
	SEN_thre[3]     =400;
	SEN_thre[4]     =400;
	SEN_thre[5]     =400;
	
	if(debug_mode_flag==0){
		SEN_thre[goal]  =200;//2700;//max_thre*0.3;//TargetGoal*K_goal;//max_thre*0.3;//90;
		SEN_thre[corner]=200;//160;//1200;//max_thre*0.3;//60;
		SEN_thre[corner2]=200;//160;//1000;//max_thre*0.3;//60;
	}
	else if(debug_mode_flag==1){
		SEN_thre[goal]  =250;//1000;//max_thre*0.3;//TargetGoal*K_goal;//max_thre*0.3;//90;
		SEN_thre[corner]=250;//1200;//max_thre*0.3;//60;
		SEN_thre[corner2]=170;//2700;//max_thre*0.3;//60;
	}	
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
		Date_flash_write(7,i,buf);
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

void para_save(void)
{
	short buf[4];
	long tmp=0;
	short i=0;
	
	fld_erase_2KB(3);

	buf[2]=0;
	buf[3]=0;
	tmp=(long)(K_P*1000.0);
	buf[0] = (tmp>>16)&0xFFFF;
	buf[1] = (tmp)&0xFFFF;
	Date_flash_write(3,0,buf);

	tmp=(long)(K_D*1000.0);
	buf[0] = (tmp>>16)&0xFFFF;
	buf[1] = (tmp)&0xFFFF;
	Date_flash_write(3,1,buf);

	tmp=(long)(K_P_GYRO*1000.0);
	buf[0] = (tmp>>16)&0xFFFF;
	buf[1] = (tmp)&0xFFFF;
	Date_flash_write(3,2,buf);
}

void sens_load(void)
{
	unsigned short *load;
	short i=0;
	
	for(i=1; i<9; i++){
		load=Data_flash_read(7,i);
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

void para_load(void)
{
	unsigned short *load;
	long tmp=0;
	short i=0;
	
	load=Data_flash_read(3,0);
	tmp = load[0];
	tmp = (tmp<<16)&0xFFFF0000;
	tmp |= load[1]&0xFFFF;
	K_P=(float)tmp/1000.0;

	load=Data_flash_read(3,1);
	tmp = load[0];
	tmp = (tmp<<16)&0xFFFF0000;
	tmp |= load[1]&0xFFFF;
	K_D=(float)tmp/1000.0;

	load=Data_flash_read(3,2);
	tmp = load[0];
	tmp = (tmp<<16)&0xFFFF0000;
	tmp |= load[1]&0xFFFF;
	K_P_GYRO=(float)tmp/1000.0;

	first_K_P=K_P;
	first_K_D=K_D;
	first_K_G=K_P_GYRO;
}

void out_enc()
{
	TIMER_WAIT(2000);
	ENC_R=0;
	ENC_L=0;
	
	while(1){
		dec_out(ENC_R, 6);	outs(" ");
		dec_out((long)(ENC_R*mmpp), 6);	outs(" ");
		dec_out(ENC_L, 6);	outs(" ");
		dec_out((long)(ENC_L*mmpp), 6);	outs(" ");
		outs("\n");
	}
}

void out_PWM()
{	
	short pwm_speed=0;
	
	pwm_speed=select_pwm();
	init_ENC();
	mot_STB(START_A);
	//mot_drive(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, (float)(pwm_speed*PWM_MAX/battery));
	now_ENC_R=0;
	now_ENC_L=0;
	start0=1;
	
	while(now_ENC_R < 5600*20){}
	mot_brake();
	init_goal_frag();
	TIMER_WAIT(1000);
	mot_STB(STOP);
}

void V_LED_ON()
{
	while(1){
		V_LED8=ON;
		TIMER_WAIT(50);
		V_LED8=OFF;
		TIMER_WAIT(50);
	}
}	


void ch_debug_mode(int mode)
{
	reset();
	switch(mode){
		case 1: search_max();	break;
		case 2: out_sens();		break;
		case 3: out_enc();		break;
		case 4: out_PWM();      break;
		case 5: out_speed();    break;
		case 6: V_LED_ON();     break;
		case 7: BUZZ_ON3();     break;
		case 8: BUZZ_ON4();     break;
		case 9:  break;
		case 10: break;
		case 11: break;
		case 12: break;
		case 13: break;
		case 14: break;
		case 15: break;
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
		case 7: out_speed();		break;
		case 8: out_course();	break;
		case 9: make_shortcut1(); break;
		case 10: make_shortcut2(); break;
		case 11: para_save();   break;
		case 12: para_load();   break;
		case 13: break;
		case 14: break;
		case 15: break;
	}
}

void main()
{			
	set();
	BUZZ_ON3();
	init_goal_frag();
	init_thre();
	mot_STB(STOP);
	Battery_Check();
	ENC_R=3300;
	
	if(SW==SW_OFF){
		while(1){
			ch_mode(select_mode());
		}
	}
	else{
		LED_manager(15);
		TIMER_WAIT(1000);
		reset();
		while(1){
			ch_debug_mode(select_debug_mode());
		}
	}		
}
