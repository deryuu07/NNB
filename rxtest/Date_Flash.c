//データフラッシュのアクセス
//RX62T用
//



#include "iodefine.h"

//書き込み時に削除されるのを防止する
#pragma address ID_CODE = 0xFFFFFFA0 
const unsigned long ID_CODE[4] = {0x52000000, 0x00000000, 0x00000000, 0x00000000}; //idコードを0x00...0x00に設定
#pragma address ROM_CODE = 0xFFFFFF9C 
const unsigned long ROM_CODE = 0x00000000;

//エラーマクロ
#define FLD_OK		0
#define FLD_ERROR	1
#define FLD_BLANK	2
#define FLD_NOBLANK	3
#define FLD_TMOUT	4

#define _PCLK_		48			//PCLKは48MHz
#define _ICLK_		96000		//大まかな時間の計算として ループ数=待ち時間(usec)*ICLK(kHz)/5000-4を使う

/*---------------------------大まかな流れ-----------------------------*/
//まず初期化を行う
//次に書き込みたいブロックにクロックを通知する
//書き込みたかったらそこがブランクと分かっているならそのまま書き込める
//ブランクじゃなかったら書き込めないので消去する必要がある
//しかし消去はブロックごとであるため1ブロック,即ち2KByte一気に消すことになる
//この自作関数において引数addrはブロック番号を表す
//データフラッシュが32KByteのRXならば16ブロックあるので番号は0~15まで
//offsetは2KByteのブロックを8Byteごとに分割したときの何番目に書き込むかを示している
//なおこの関数において書き込みと読み出しはどちらもshort型配列配列長4でやり取りする

/*---------------------------プロトタイプ宣言--------------------------*/
//初期化
void fld_init_fcu_ram(void);
//クロック通知
unsigned long fld_init_pclk_notification(unsigned char addr);
//ブランクチェックしてブランクじゃないならデリートする
void Check_Delete(unsigned char addr);
//1ブロックデリートする
unsigned long fld_erase_2KB(unsigned char addr);
//データフラッシュから配列を使って読み出す
unsigned short * Data_flash_read(unsigned char addr,unsigned char offset);
//データフラッシュに書き込む
void Date_flash_write(unsigned char addr,unsigned char offset,unsigned short *data);
//書き込めてるか確認する
unsigned long verify(unsigned short *verify,unsigned char addr);


/*---------------------------以下プログラム----------------------------*/

const unsigned long data_flash_BlockAddresses[16] = { 
        0x00100000,  /* DB00 */ 
        0x00100800,  /* DB01 */ 
        0x00101000,  /* DB02 */ 
        0x00101800,  /* DB03 */ 
        0x00102000,  /* DB04 */ 
        0x00102800,  /* DB05 */ 
        0x00103000,  /* DB06 */ 
        0x00103800,  /* DB07 */ 
        0x00104000,  /* DB08 */ 
        0x00104800,  /* DB09 */ 
        0x00105000,  /* DB10 */ 
        0x00105800,  /* DB11 */ 
        0x00106000,  /* DB12 */ 
        0x00106800,  /* DB13 */ 
        0x00107000,  /* DB14 */ 
        0x00107800}; /* DB15 */    


//FCUリセット関数
static void reset_fcu(void)
{
	volatile unsigned long w;
	
	//ソフトウェアリセット
	FLASH.FRESETR.BIT.FRESET =1;
	
	//35us待つ
	_wait_usec_(35);
	
	
	//ソフトウェアリセット解除
	FLASH.FRESETR.BIT.FRESET =0;
}

//FCURAMへコピーする関数
void fld_init_fcu_ram(void)
{
	int i;
	static const int fcu_ram_size = 8*1024; //8KByte
	volatile unsigned long *fcu_ram = (unsigned long *)0x007F8000;
	const volatile unsigned long *fcu_fw = (unsigned long *)0xFEFFE000;
	
	//リードモードへ
	if(FLASH.FENTRYR.WORD & 0x00ff){
		FLASH.FENTRYR.WORD = 0xAA00;	//キーコードはAAh
	}
	
	//FCURAMエリアアクセス許可
	FLASH.FCURAME.WORD = 0xC401;		//キーコードはC4h
	
	//FCU F/WからFCURAMへコピー
	for(i=0;i<fcu_ram_size/sizeof(unsigned long);i++){
		*fcu_ram++ = *fcu_fw++;
	}
	
}

//周辺クロックをFCUに通知する関数
unsigned long fld_init_pclk_notification(unsigned char addr)
{
	//00100000hはDB00ブロックのトップアドレス
	volatile unsigned char *addr_b = (unsigned char *)data_flash_BlockAddresses[addr];
	volatile unsigned short *addr_w = (unsigned short *)data_flash_BlockAddresses[addr];
	
	//P/Eモードにチェンジ
	if((FLASH.FENTRYR.WORD & 0x00ff) != 0x0080){
		FLASH.FENTRYR.WORD = 0xAA80;	//キーコードはAAh
	}
	
	//周辺クロックを設定
	FLASH.PCKAR.BIT.PCKA = _PCLK_;			//PCLKは48MHz
	
	//周辺クロック通知コマンド発行
	*addr_b = 0xE9;
	*addr_b = 0x03;
	*addr_w = 0x0F0F;
	*addr_w = 0x0F0F;
	*addr_w = 0x0F0F;
	*addr_b = 0xD0;
	
	//コマンド完了を待つ
	if(wait_FRDY(120)==FLD_TMOUT){
		reset_fcu();	
	}
	
	//エラーチェック
	if(FLASH.FSTATR0.BIT.ILGLERR == 1){
		return FLD_ERROR;
	}
	
	return FLD_OK;
}

//ブランクチェック関数
//2KBブランクチェック
unsigned long fld_blank_check_2KB(unsigned char addr)
{
	volatile unsigned char *addr_b = (unsigned char *)data_flash_BlockAddresses[addr];
	
	//ブランクチェックコマンド使用
	FLASH.FMODR.BIT.FRDMD = 1;
	
	//ブランクチェックのサイズを2KByteに指定
	FLASH.DFLBCCNT.BIT.BCSIZE = 1;
	
	//ブランクチェックコマンド使用
	*addr_b = 0x71;
	*addr_b = 0xD0;
	
	//タイムアウト待ち
	if(wait_FRDY(700*1.1)==FLD_TMOUT){
		reset_fcu();	
	}
	
	//エラーチェック
	if(FLASH.FSTATR0.BIT.ILGLERR == 1){
		return FLD_ERROR;
	}
	
	//ブランクチェック結果を返す
	if(FLASH.DFLBCSTAT.BIT.BCST == 0){
		return FLD_BLANK;
	}
	return FLD_NOBLANK;
}
//8Bブランクチェック
unsigned long fld_blank_check_8B(unsigned char addr,unsigned char offset)
{
	volatile unsigned char *addr_b = (unsigned char *)data_flash_BlockAddresses[addr];
	
	//ブランクチェックコマンド使用
	FLASH.FMODR.BIT.FRDMD = 1;
	
	//ブランクチェックのサイズを8Byteに指定
	FLASH.DFLBCCNT.BIT.BCSIZE = 0;
	FLASH.DFLBCCNT.BIT.BCADR = offset;
	
	//ブランクチェックコマンド使用
	*addr_b = 0x71;
	*addr_b = 0xD0;
	
	//タイムアウト待ち
	if(wait_FRDY(30*1.1)==FLD_TMOUT){
		reset_fcu();	
	}
	
	//エラーチェック
	if(FLASH.FSTATR0.BIT.ILGLERR == 1){
		return FLD_ERROR;
	}
	
	//ブランクチェック結果を返す
	if(FLASH.DFLBCSTAT.BIT.BCST == 0){
		return FLD_BLANK;
	}
	return FLD_NOBLANK;
}

//消去関数
unsigned long fld_erase_2KB(unsigned char addr)
{
	volatile unsigned char *addr_b = (unsigned char *)data_flash_BlockAddresses[addr];
	unsigned long ret = FLD_OK;
	
	
	//P/Eモードにチェンジ
	if((FLASH.FENTRYR.WORD & 0x00ff) != 0x0080){
		FLASH.FENTRYR.WORD = 0xAA80;	//キーコードはAAh
	}
	
	//消去プロテクト解除
	FLASH.FWEPROR.BIT.FLWE = 1;
	
	//ブロック単位削除許可
	FLASH.DFLWE0.WORD = 0x1EFF;		//キーコードは1Eh
	FLASH.DFLWE1.WORD = 0xE1FF;		//キーコードはE1h
	
	//ブロック消去コマンド発行
	*addr_b = 0x20;
	*addr_b = 0xD0;
	
	
	//コマンド完了を待つ
	if(wait_FRDY(250*1000*1.1)==FLD_TMOUT){
		reset_fcu();	
	}
	
	
	//エラーチェック
	if((FLASH.FSTATR0.BIT.ILGLERR == 1)||
	   (FLASH.FSTATR0.BIT.ERSERR == 1)){
		ret = FLD_ERROR;
	}
	 
	//プロテクトと消去無効
	FLASH.FWEPROR.BIT.FLWE = 2;
	FLASH.DFLWE0.WORD = 0x1E00;
	FLASH.DFLWE1.WORD = 0xE100;
	
	return ret;
}

//書き込み関数
unsigned long fld_program_8byte(unsigned long addr,unsigned short *ram)
{
	volatile unsigned char *addr_b = (unsigned char *)addr;
	volatile unsigned short *addr_w = (unsigned short *)addr;
	unsigned long i,ret = FLD_OK;
	
	//書き込みプロテクト解除
	FLASH.FWEPROR.BIT.FLWE = 1;
	
	//ブロック単位の書き込み許可
	FLASH.DFLWE0.WORD = 0x1EFF;		//キーコードは1Eh
	FLASH.DFLWE1.WORD = 0xE1FF;		//キーコードはE1h
	
	//プログラムコマンド発行
	*addr_b = 0xE8;
	*addr_b = 0x04;
	for(i=0;i<4;i++){				//8Byteは4Word分のサイズ
		*addr_w = *ram++;
	}
	*addr_b = 0xD0;
	
	//コマンド完了を待つ
	if(wait_FRDY(2*1000*1.1)==FLD_TMOUT){
		reset_fcu();	
	}
	
	//エラーチェック
	if((FLASH.FSTATR0.BIT.ILGLERR == 1)||
	   (FLASH.FSTATR0.BIT.PRGERR == 1)){
		ret = FLD_ERROR;
	}
	
	
	//プロテクトと消去無効
	FLASH.FWEPROR.BIT.FLWE = 2;
	FLASH.DFLWE0.WORD = 0x1E00;
	FLASH.DFLWE1.WORD = 0xE100;
	
	return ret;
}

//読み出し許可関数
unsigned long fld_enable_read(void)
{
	
	//リードモードへ
	if(FLASH.FENTRYR.WORD & 0x00ff){
		FLASH.FENTRYR.WORD = 0xAA00;	//キーコードはAAh
	}
	
	//ブロック単位の書き込み許可
	FLASH.DFLRE0.WORD = 0x2DFF;		//キーコードは2Dh
	FLASH.DFLRE1.WORD = 0xD2FF;		//キーコードはD2h
	
	return FLD_OK;
}




//コマンド完了まで待つ関数
int wait_FRDY(int t)
{
	int cn=0;
	while(FLASH.FSTATR0.BIT.FRDY==0)
	{
		_wait_usec_(1);//1us　wait
		cn++;
		if(cn==t)return FLD_OK;
	}
	return FLD_TMOUT;
}

//適当にusec待つ
_wait_usec_(int t){
	int i;
	t = t*_ICLK_/5000-4;
	for(i=0;i<t;i++);
}

//初期設定 pclk_notificationの引数はブロック番号
void Data_Flash_init(void){
	fld_init_fcu_ram();
	fld_init_pclk_notification(0);
}

//ブランクチェック後消去 引数は消去するブロック番号
void Check_Delete(unsigned char addr)
{
	
	//P/Eモードにチェンジ
	if((FLASH.FENTRYR.WORD & 0x00ff) != 0x0080){
		FLASH.FENTRYR.WORD = 0xAA80;	//キーコードはAAh
	}
	
	//ブランクチェック
	if(fld_blank_check_2KB(addr)==FLD_NOBLANK){
		fld_erase_2KB(addr);
	}
}

//8Byte書き出し 引数はブロック番号,オフセット番号,データ
void Date_flash_write(unsigned char addr,unsigned char offset,unsigned short *data)
{
	fld_program_8byte(data_flash_BlockAddresses[addr]+8*offset,data);
}

//8Byte読み出し 引数はブロック番号,オフセット番号
unsigned short * Data_flash_read(unsigned char addr,unsigned char offset)
{
	unsigned short *read;
	
	fld_enable_read();
	
	read = (unsigned short *)(data_flash_BlockAddresses[addr]+offset*8);

	
	return read;
}


//ベリファイ
unsigned long verify(unsigned short *verify,unsigned char addr)
{
	int i;
	volatile unsigned short *read;
	
	fld_enable_read();
	
	read = (unsigned short *)data_flash_BlockAddresses[addr];
	
	for(i=0;i<sizeof(verify)/sizeof(verify[0]);i++){
		if( *read++ != *verify++){
			return FLD_ERROR;
		}
	}
	
	return FLD_OK;
	
}

