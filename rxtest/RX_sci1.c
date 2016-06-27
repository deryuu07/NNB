#include "iodefine.h"
#include "RX_sci1.h"

void init_sci1(void){
    int _I;

//    STB.CR3.BIT._SCI1 = 0;       /* SCI1 モジュールスタンバイ解除 */
	MSTP_SCI1=0;
	SCI1.SMR.BIT.CM   = 0;     /* 調歩同期式 */
    SCI1.SMR.BIT.CHR  = 0;     /* データ長8bit */
    SCI1.SMR.BIT.PE  = 0;     /* パリティビットなし */
    SCI1.SMR.BIT.STOP = 0;     /* ストップビット長1bit */
    SCI1.SMR.BIT.CKS  = 0;     /* クロックセレクト Pφ 高速通信なのでもっと遅くする必要有り? */

    SCI1.BRR = 39;//77;             /* 分周比　24MHz -> 9600bps */

    for(_I=0;_I<100;_I++);       /* wait */

    //PFC.PACRL2.BIT.PA4MD = 1;    /* PA4をTXD1に */
    //PFC.PACRL1.BIT.PA3MD = 1;    /* PA3をRXD1に */

    SCI1.SCR.BIT.TE = 1;       // 送信許可
    SCI1.SCR.BIT.RE = 1;       // 受信許可
}
/*========== SCI1初期化 ==========*/
/*================================*/


/*==========================*/
/*========== 送信 ==========*/
void out(char _Data){
    while(!SCI1.SSR.BIT.TDRE);  /* 送信できるようになるまで待機 */
    SCI1.TDR = _Data;           /* 送信する文字をセット */
    SCI1.SSR.BIT.TDRE = 0;      /* 送信を通知 */
}
void outs(char *_Str){
    while(*_Str != '\0'){
        if(*_Str == '\n'){
            out('\r');  /* 改行 */
            out('\n');
        }else if(*_Str == '\t'){
			out('\t');
        }else
            out(*_Str); /* 1文字送信 */

        _Str++;         /* 次の文字へ */
    }
}
void dec_out(long _I,int _N){
    if(_I < 0){
        out('-');
        _I = -_I;
    }else
        out(' ');

    /* 各桁の数値計算 */
    if(_N > 8) _I = dec_outs(_I,100000000L);
    if(_N > 7) _I = dec_outs(_I,10000000L);
    if(_N > 6) _I = dec_outs(_I,1000000L);
    if(_N > 5) _I = dec_outs(_I,100000L);
    if(_N > 4) _I = dec_outs(_I,10000L);
    if(_N > 3) _I = dec_outs(_I,1000L);
    if(_N > 2) _I = dec_outs(_I,100L);
    if(_N > 1) _I = dec_outs(_I,10L);
               _I = dec_outs(_I,1L);
}
long dec_outs(long _X, long _Y){
    short	_I;

    _I = 0;        /* iの初期化 */
    /* x(数値)がy(桁)より大きい間繰り返す */
    while(_X >= _Y){
        _X -= _Y;  /*  */
        _I++;      /* その桁での数値を考える */
        if(_I == 10){
            break;
        }
    }
    out( _I + 0x30 );

    return _X;
}
void bit_out(unsigned char _Ch){
    unsigned char _N = 0x80;

    while(_N > 0){
        if(_Ch&_N){
            out('1');
        }else{
            out('0');
        }
        _N /= 2;
    }
}
/*========== 送信 ==========*/
/*==========================*/


/*==========================*/
/*========== 受信 ==========*/
char in(void){
    char _Data;
    while (!(SCI1.	SSR.BYTE & 0x40)); /* 受信できるようになるまで待機 */
    _Data = SCI1.RDR;                /* キャラクタを取得 */
    SCI1.SSR.BIT.RDRF = 0;           /* 受信の通知 */
    out(_Data);
    return _Data;
}
int dec_in(void){
    unsigned char _Data;     /* 一時保存用 */
    int _Num = 0;            /* 返す値 */
//    int _I = 0;              /* 作業用 */
    int _Cnt = 0;            /* 桁数カウント */

    _Data = in();
    while(_Data != '\r'){
        _Num *= 10;          /* 桁上げ */
        _Num += (int)(_Data - '0');
        _Cnt++;
        if(_Cnt > 7) break;
        _Data = in();
    }

    return _Num;
}
/*========== 受信 ==========*/
/*==========================*/

void fdec_out(long _I,int _N){
    if(_I < 0){
        out('-');
        _I = -_I;
    }
    /* 各桁の数値計算 */
    if(_N > 8) _I = dec_outs(_I,100000000L);
    if(_N > 7) _I = dec_outs(_I,10000000L);
    if(_N > 6) _I = dec_outs(_I,1000000L);
    if(_N > 5) _I = dec_outs(_I,100000L);
    if(_N > 4) _I = dec_outs(_I,10000L);
    if(_N > 3) _I = dec_outs(_I,1000L);
    if(_N > 2) _I = dec_outs(_I,100L);
    if(_N > 1) _I = dec_outs(_I,10L);
               _I = dec_outs(_I,1L);
}

void out_float(float f_data)
{
	long  tmp1=(long)(f_data);		//整数部分
	float tmp2=f_data-tmp1;
	long  tmp3=(long)(tmp2*1000000.0);	//小数部分
	
	outs(" ");
	if(abs(tmp1/10000) > 0){	
		fdec_out(tmp1,5);
	}
	else{
		if(abs(tmp1/1000) > 0){	
			fdec_out(tmp1,4);
		}
		else{
			if(abs(tmp1/100) > 0){	
				fdec_out(tmp1,3);
			}
			else{
				if(abs(tmp1/10) > 0){	
					fdec_out(tmp1,2);
				}
				else{
					fdec_out(tmp1,1);
				}
			}
		}
	}
	outs(".");
	if(tmp3 < 0) tmp3*=(-1);
	fdec_out(tmp3,6);
}