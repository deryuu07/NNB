#include "iodefine.h"
#define WHO_AM_I 0x0f
#define OUT_Z_L  0x2C
#define OUT_Z_H  0x2D
#define CS       PORT2.DR.BIT.B1		
#define SDO      PORT2.PORT.BIT.B2		
#define SDI      PORT2.DR.BIT.B3		
#define SPC      PORT2.DR.BIT.B4		
#define HIGH     1
#define LOW      0
#define _ICLK_		96000

void spi_wait_usec_(int t){
	int i;
	t = t*_ICLK_/5000-4;
	for(i=0;i<t;i++);
}

void spi_pause(short usec){		//MTU5をSPI用時間計測に使用する
	unsigned short to;

	SYSTEM.MSTPCRA.BIT.MSTPA9=0;	//MTU3モジュールの使用を許可
	MTU5.TSTR.BIT.CSTU5=0;			//カウント停止
	MTU5.TCNTU=0;					//カウンタ初期化
	MTU5.TCRU.BIT.TPSC=1;			//内部クロック：ICLK/4でカウント (24MHz)
	MTU5.TSTR.BIT.CSTU5=1;			//カウントスタート
	
	to=(unsigned short)(usec);	//待ち時間計算
	if(to==0)to=1;					//0はだめなのでとりあえずちょっと待つ
	
	while(to>=MTU5.TCNTU);			//じかんになるまで待つ
}

void write_data(unsigned short addr, unsigned short data)
{
	char i,j=0;	
	unsigned char  tmp=0;
	unsigned char SPI_out=0;
	CS=1;
	SPC=1;
	SDI=0;

	/***SPI start***/
	CS=0;
	_wait_usec_(200);	

	/***RW指定***/ 
	SPC=LOW;
	SDI=LOW;			//HIGH:read mode，LOW:write mode
	spi_wait_usec_(200);
	SPC=HIGH;
	spi_wait_usec_(500);

	/***MSB***/ 	
	SPC=LOW;
	SDI=LOW;
	spi_wait_usec_(200);
	SPC=HIGH;
	spi_wait_usec_(500);

	/***アドレス送信***/ 
	j=5;
	for(i=0; i<6; i++){
		SPC=LOW;
		if((addr>>j) & 0x01 == 1){
			SDI=HIGH;
		}
		else{
			SDI=LOW;
		}
		spi_wait_usec_(200);	
		SPC=HIGH;
		spi_wait_usec_(500);
		j--;			
	}
		
	/***書き込み***/ 
	j=7;
	for(i=0; i<8; i++){
		SPC=LOW;
		if((data>>j) & 0x01 == 1){
			SDI=HIGH;
		}
		else{
			SDI=LOW;
		}
		spi_wait_usec_(200);	
		SPC=HIGH;
		spi_wait_usec_(500);
		j--;			
	}
	CS=HIGH;
	SPC=HIGH;
	SDI=LOW;
	
}

unsigned char read_data(unsigned short addr)
{
	char i,j=0;	
	unsigned char  tmp=0;
	unsigned char SPI_out=0;
	CS=1;
	SPC=1;
	SDI=0;

	/***SPI start***/
	CS=0;
	_wait_usec_(10);	

	/***RW指定***/ 
	SPC=LOW;
	SDI=HIGH;
	spi_wait_usec_(1);
	SPC=HIGH;
	spi_wait_usec_(1);

	/***MSB***/ 	
	SPC=LOW;
	SDI=LOW;
	spi_wait_usec_(1);
	SPC=HIGH;
	spi_wait_usec_(1);

	/***アドレス送信***/ //0010 0001
	j=5;
	for(i=0; i<6; i++){
		SPC=LOW;
		if((addr>>j) & 0x01 == 1){
			SDI=HIGH;
		}
		else{
			SDI=LOW;
		}
		spi_wait_usec_(1);	
		SPC=HIGH;
		spi_wait_usec_(1);
		j--;			
	}
		
	/***読出し***/
	SPI_out=0;
	for(j=0; j<8; j++){	
		SPC=LOW;
		_wait_usec_(1);	
		SPC=HIGH;
		tmp=SDO;
		SPI_out=SPI_out|tmp;
		SPI_out=SPI_out<<1;
		spi_wait_usec_(1);	
		tmp=0;
	}
	CS=HIGH;
	SPC=HIGH;
	SDI=LOW;
	
	return SPI_out;
}

signed short get_gyro()
{
	unsigned char out_L, out_H=0;
	signed short gyro_out=0;
	
	out_H=read_data(OUT_Z_H);
	spi_wait_usec_(10);	
	out_L=read_data(OUT_Z_L);
	gyro_out=(signed short)out_H;
	gyro_out=gyro_out<<8;
	gyro_out=gyro_out|(signed short)out_L;
	
	return gyro_out;
}