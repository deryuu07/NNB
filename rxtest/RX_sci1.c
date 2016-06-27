#include "iodefine.h"
#include "RX_sci1.h"

void init_sci1(void){
    int _I;

//    STB.CR3.BIT._SCI1 = 0;       /* SCI1 ���W���[���X�^���o�C���� */
	MSTP_SCI1=0;
	SCI1.SMR.BIT.CM   = 0;     /* ���������� */
    SCI1.SMR.BIT.CHR  = 0;     /* �f�[�^��8bit */
    SCI1.SMR.BIT.PE  = 0;     /* �p���e�B�r�b�g�Ȃ� */
    SCI1.SMR.BIT.STOP = 0;     /* �X�g�b�v�r�b�g��1bit */
    SCI1.SMR.BIT.CKS  = 0;     /* �N���b�N�Z���N�g P�� �����ʐM�Ȃ̂ł����ƒx������K�v�L��? */

    SCI1.BRR = 39;//77;             /* ������@24MHz -> 9600bps */

    for(_I=0;_I<100;_I++);       /* wait */

    //PFC.PACRL2.BIT.PA4MD = 1;    /* PA4��TXD1�� */
    //PFC.PACRL1.BIT.PA3MD = 1;    /* PA3��RXD1�� */

    SCI1.SCR.BIT.TE = 1;       // ���M����
    SCI1.SCR.BIT.RE = 1;       // ��M����
}
/*========== SCI1������ ==========*/
/*================================*/


/*==========================*/
/*========== ���M ==========*/
void out(char _Data){
    while(!SCI1.SSR.BIT.TDRE);  /* ���M�ł���悤�ɂȂ�܂őҋ@ */
    SCI1.TDR = _Data;           /* ���M���镶�����Z�b�g */
    SCI1.SSR.BIT.TDRE = 0;      /* ���M��ʒm */
}
void outs(char *_Str){
    while(*_Str != '\0'){
        if(*_Str == '\n'){
            out('\r');  /* ���s */
            out('\n');
        }else if(*_Str == '\t'){
			out('\t');
        }else
            out(*_Str); /* 1�������M */

        _Str++;         /* ���̕����� */
    }
}
void dec_out(long _I,int _N){
    if(_I < 0){
        out('-');
        _I = -_I;
    }else
        out(' ');

    /* �e���̐��l�v�Z */
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

    _I = 0;        /* i�̏����� */
    /* x(���l)��y(��)���傫���ԌJ��Ԃ� */
    while(_X >= _Y){
        _X -= _Y;  /*  */
        _I++;      /* ���̌��ł̐��l���l���� */
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
/*========== ���M ==========*/
/*==========================*/


/*==========================*/
/*========== ��M ==========*/
char in(void){
    char _Data;
    while (!(SCI1.	SSR.BYTE & 0x40)); /* ��M�ł���悤�ɂȂ�܂őҋ@ */
    _Data = SCI1.RDR;                /* �L�����N�^���擾 */
    SCI1.SSR.BIT.RDRF = 0;           /* ��M�̒ʒm */
    out(_Data);
    return _Data;
}
int dec_in(void){
    unsigned char _Data;     /* �ꎞ�ۑ��p */
    int _Num = 0;            /* �Ԃ��l */
//    int _I = 0;              /* ��Ɨp */
    int _Cnt = 0;            /* �����J�E���g */

    _Data = in();
    while(_Data != '\r'){
        _Num *= 10;          /* ���グ */
        _Num += (int)(_Data - '0');
        _Cnt++;
        if(_Cnt > 7) break;
        _Data = in();
    }

    return _Num;
}
/*========== ��M ==========*/
/*==========================*/

void fdec_out(long _I,int _N){
    if(_I < 0){
        out('-');
        _I = -_I;
    }
    /* �e���̐��l�v�Z */
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
	long  tmp1=(long)(f_data);		//��������
	float tmp2=f_data-tmp1;
	long  tmp3=(long)(tmp2*1000000.0);	//��������
	
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