#include "iodefine.h"
#include "SCI.h"

#define BACK_SPACE 0x08
#define ENTER 0x0D
#define ESCAPE 0x1B

#define SCI_PCLK		50 000 000	//覚書
#define SCI_BITRATE		230400		//覚書
#define SCI_BRR			39			//ビットレート調整用の定数。(SCI_PCLK)/(64*SCI_BITRATE*2^(2n-1))-1=BRRで求める

#if 0
void init_sci1(void){
    int _I;

//    STB.CR3.BIT._SCI1 = 0;       /* SCI1 モジュールスタンバイ解除 */
	MSTP_SCI1=0;
	SCI1.SMR.BIT.CM   = 0;     /* 調歩同期式 */
    SCI1.SMR.BIT.CHR  = 0;     /* データ長8bit */
    SCI1.SMR.BIT.PE  = 0;     /* パリティビットなし */
    SCI1.SMR.BIT.STOP = 0;     /* ストップビット長1bit */
    SCI1.SMR.BIT.CKS  = 0;     /* クロックセレクト Pφ 高速通信なのでもっと遅くする必要有り? */

    SCI1.BRR = 39;//155;//77;             /* 分周比　24MHz -> 9600bps */

    for(_I=0;_I<100;_I++);       /* wait */

    //PFC.PACRL2.BIT.PA4MD = 1;    /* PA4をTXD1に */
    //PFC.PACRL1.BIT.PA3MD = 1;    /* PA3をRXD1に */

    SCI1.SCR.BIT.TE = 1;       // 送信許可
    SCI1.SCR.BIT.RE = 1;       // 受信許可
}
#endif
void init_sci1(void){				//SCI�����ݒ�
	unsigned short dummy;			//�_�~�[�ƃ��[�v�J�E���^�̕ϐ�

	IEN(SCI1,TXI1) = 0;				//TXI���荞�݋֎~
	IEN(SCI1,RXI1) = 0;				//RXI���荞�݋֎~
	MSTP_SMCI1 = 0;					//SCI1���W���[���X�^���o�C����
	SCI1.SCR.BYTE = 0x00;			//���M,��M�y�ъ��荞�ݗv�����֎~
	SCI1.SCR.BIT.CKE = 0;			//�N���b�N�ݒ�
	SCI1.SMR.BYTE = 0x00;			//b0	0�F
									//b1 �N���b�N�I���r�b�g	0�FPCLK�N���b�N(n=0)
									//b2 �}���`�v���Z�b�T���[�h�r�b�g	0�F�}���`�v���Z�b�T�ʐM�@�\���֎~
									//b3 �X�g�b�v�r�b�g���I��	0�F1�X�g�b�v�r�b�g
									//b4 �p���e�B���[�h�r�b�g	0�F�����p���e�B�ő���M
									//b5 �p���e�B���r�b�g	0�F�p���e�B�r�b�g��t��
									//b6 �L�����N�^���r�b�g	0�F�f�[�^��8�r�b�g�ő���M
									//b7 �R�~���j�P�[�V�������[�h�r�b�g	0�F�������������[�h�œ���
	SCI1.BRR = SCI_BRR;				//�r�b�g���[�g����
	IPR(SCI1,TXI1) = 7;				//���荞�ݗD��x�ݒ�
	IPR(SCI1,RXI1) = 7;				//���荞�ݗD��x�ݒ�
	IR(SCI1,TXI1) = 0;				//���荞�ݗv���t���O�N���A
	IR(SCI1,RXI1) = 0;				//���荞�ݗv���t���O�N���A

	for(dummy = 0; dummy < 50000; dummy++); //������Ƒ҂�

	SCI1.SCR.BIT.TIE = 1;			//TXI���荞�݋���
	SCI1.SCR.BIT.RIE = 1;			//RXI���荞�݋���
	SCI1.SCR.BIT.RE = 1;			//��M�J�n
	SCI1.SCR.BIT.TE = 1;			//���M�J�n
	dummy = SCI1.SSR.BYTE;			//�_�~�[���[�h
	SCI1.SSR.BYTE = 0xC0;			//�t���O�N���A
}

void sci_print(unsigned char data){	//unsigned char型の文字列を送る関数。下の関数で使っている
	while(!IR(SCI1,TXI1));
	SCI1.TDR = data;				//データを転送
	IR(SCI1,TXI1)=0;
}

char sci_data_get(void){
	char data;
	IR(SCI1,RXI1) = 0;
	while(!IR(SCI1,RXI1));
	data = SCI1.RDR;				//データを返す
	return data;
}

void print_str(unsigned char *str){	//文字列を送る関数。引数から文字列を持ってくる
	int i;
	for(i=0;str[i]!='\0';i++){
		sci_print(str[i]);
	}
}

void print_num(long data,short n){
	char min_flag=0;
	short i,j;
	unsigned char str[11]={0};
	long digit=1;
	
	if(data<0){
		min_flag=1;
		data*=-1;
	}

	/*桁数作成*/
	for(i=0;i<12;i++){
		if(data<digit)break;
		digit*=10;
	}
	digit/=10;
	if(data==0){
		i=1;	
	}

	/*表示桁数と比較*/
	if(n<=i){
		n=i;						//最大桁nを更新
		if(min_flag==1){
			sci_print('-');
		}
		if(n<11)
		str[n+1]='\0';				//文字列終端
	}
	else{
		/*iは桁数nは最大文字列*/
		for(j=0;j<n;j++){		//空白作成 0~n-1
			str[j]=' ';
		}
		if(min_flag==1){
			str[n-i-1]='-';
		}
		
		if(n<11)
		str[n+1]='\0';
	}
	for(j=(n-i);j<n;j++){
		str[j]='0'+(data/digit);
		data%=digit;
		digit/=10;
	}
	print_str(str);
}

void print_clear(void){
	sci_print(0x1B);
	print_str("[2J");
	sci_print(0x1B);
	print_str("[0;0H");
}

void back_space(char n){
	char i;
	for(i = 0;i < n; i++){
		sci_print(0x1B);
		print_str("[1D");
		sci_print(' ');
		sci_print(0x1B);
		print_str("[1D");
	}
}

void pointer_move(unsigned char line,unsigned char column){
	sci_print(0x1B);
	sci_print('[');
	print_num(line,0);
	sci_print(';');
	print_num(column,0);
	sci_print('H');
}

void clear_pointer_to_eol(void){
	sci_print(0x1B);
	print_str("[0K");
}

void clear_sol_to_pointer(void){
	sci_print(0x1B);
	print_str("[1K");
}

float scan_fnum(void){
	char data[20]={0};
	char point=0;
	char period_flag=0;
	char minus_flag=0;

	do{
		data[point]=sci_data_get();
		if((data[point]==ENTER)&&(point>0)){
			break;
		}
		if(('0'<=data[point])&&(data[point]<='9')){
			sci_print(data[point]);
			point++;
		}
		else if((data[point]=='-')&&(minus_flag==0)){
			sci_print(data[point]);
			point++;
			minus_flag = 1;
		}
		else if((data[point]=='.')&&(period_flag==0)){
			sci_print(data[point]);
			point++;
			period_flag = 1;
		}
		else if((data[point]==BACK_SPACE)&&(point>0)){
			back_space(1);
			point--;
			data[point]=0;
		}
	}
	while(1);
	data[point]=0;
	return my_atof(data);
}

short scan_num(void){
	char data[20]={0};
	char point=0;
	do{
		data[point]=sci_data_get();
		if((data[point]==ENTER)&&(point>0)){
			break;
		}
		if((('0'<=data[point])&&(data[point]<='9')) || (data[point]=='-')){
			sci_print(data[point]);
			point++;
		}
		else if((data[point]==BACK_SPACE)&&(point>0)){
			back_space(1);
			point--;
			data[point]=0;
		}
	}
	while(1);
	data[point]=0;
	return my_atos(data);
}

char scan_vector(void){
	char state=0;
	char data;
	while(1){
		data=sci_data_get();
		switch(state){
			case 0:
				if(data==ESCAPE)
					state = 1;
				else if(data==ENTER)
					return 4;
				break;
			case 1:
				if(data=='[')
					state = 2;
				else
					state = 0;
				break;
			case 2:
				if(data=='A')
					return 0;
				else if(data=='B')
					return 2;
				else if(data=='C')
					return 1;
				else if(data=='D')
					return 3;
				else
					state = 0;
				break;

		}
	}
}

void scan_enter(void){
	while(sci_data_get()!=ENTER);
}

short my_atos(char s[]){
	short i, n, sign;

	for(i = 0; my_isspace( s[i] ); i++);	//先頭の空白を読み飛ばす
	sign = (s[i] == '-') ? -1 : 1;			//符号を保存する
	if(s[i] == '-' || s[i] == '+')				//符号を飛ばす
		i++;
	for( n = 0; my_isdigit(s[i]); i++)	//s[i]が数字のあいだ、nへ
		n = 10 * n + s[i] - '0';
	return sign * n;								//符号を反映
}

float my_atof(char s[]){
	float value = 0.0, power = 1.0;
	short i, sign;

	for (i = 0; my_isspace(s[i]); i++);
	sign = (s[i] == '-') ? -1 : 1;
	if (s[i] == '+' || s[i] == '-')
		i++;
	for (; my_isdigit(s[i]); i++){
		value = value * 10 + s[i] - '0';
	}
	if (s[i] == '.')
		i++;
	for (; my_isdigit(s[i]); i++) {
		value = value * 10 + s[i] - '0';
		power *= 10.0;
    }
	return sign * value / power;
}

char my_isspace(char c){
	return (c == ' ' || c == '\t' || c == '\n') ;
}

char my_isdigit(char c){
	return ('0' <= c && c <= '9');
}
