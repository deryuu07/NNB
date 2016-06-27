#include "iodefine.h"
#include "SCI.h"

#define BACK_SPACE 0x08
#define ENTER 0x0D
#define ESCAPE 0x1B

#define SCI_PCLK		50 000 000	//è¦šæ›¸
#define SCI_BITRATE		230400		//è¦šæ›¸
#define SCI_BRR			39			//ãƒ“ãƒƒãƒˆãƒ¬ãƒ¼ãƒˆèª¿æ•´ç”¨ã®å®šæ•°ã€‚(SCI_PCLK)/(64*SCI_BITRATE*2^(2n-1))-1=BRRã§æ±‚ã‚ã‚‹

#if 0
void init_sci1(void){
    int _I;

//    STB.CR3.BIT._SCI1 = 0;       /* SCI1 ãƒ¢ã‚¸ãƒ¥ãƒ¼ãƒ«ã‚¹ã‚¿ãƒ³ãƒã‚¤è§£é™¤ */
	MSTP_SCI1=0;
	SCI1.SMR.BIT.CM   = 0;     /* èª¿æ­©åŒæœŸå¼ */
    SCI1.SMR.BIT.CHR  = 0;     /* ãƒ‡ãƒ¼ã‚¿é•·8bit */
    SCI1.SMR.BIT.PE  = 0;     /* ãƒ‘ãƒªãƒ†ã‚£ãƒ“ãƒƒãƒˆãªã— */
    SCI1.SMR.BIT.STOP = 0;     /* ã‚¹ãƒˆãƒƒãƒ—ãƒ“ãƒƒãƒˆé•·1bit */
    SCI1.SMR.BIT.CKS  = 0;     /* ã‚¯ãƒ­ãƒƒã‚¯ã‚»ãƒ¬ã‚¯ãƒˆ PÏ† é«˜é€Ÿé€šä¿¡ãªã®ã§ã‚‚ã£ã¨é…ãã™ã‚‹å¿…è¦æœ‰ã‚Š? */

    SCI1.BRR = 39;//155;//77;             /* åˆ†å‘¨æ¯”ã€€24MHz -> 9600bps */

    for(_I=0;_I<100;_I++);       /* wait */

    //PFC.PACRL2.BIT.PA4MD = 1;    /* PA4ã‚’TXD1ã« */
    //PFC.PACRL1.BIT.PA3MD = 1;    /* PA3ã‚’RXD1ã« */

    SCI1.SCR.BIT.TE = 1;       // é€ä¿¡è¨±å¯
    SCI1.SCR.BIT.RE = 1;       // å—ä¿¡è¨±å¯
}
#endif
void init_sci1(void){				//SCI‰Šúİ’è
	unsigned short dummy;			//ƒ_ƒ~[‚Æƒ‹[ƒvƒJƒEƒ“ƒ^‚Ì•Ï”

	IEN(SCI1,TXI1) = 0;				//TXIŠ„‚è‚İ‹Ö~
	IEN(SCI1,RXI1) = 0;				//RXIŠ„‚è‚İ‹Ö~
	MSTP_SMCI1 = 0;					//SCI1ƒ‚ƒWƒ…[ƒ‹ƒXƒ^ƒ“ƒoƒC‰ğœ
	SCI1.SCR.BYTE = 0x00;			//‘—M,óM‹y‚ÑŠ„‚è‚İ—v‹‚ğ‹Ö~
	SCI1.SCR.BIT.CKE = 0;			//ƒNƒƒbƒNİ’è
	SCI1.SMR.BYTE = 0x00;			//b0	0F
									//b1 ƒNƒƒbƒN‘I‘ğƒrƒbƒg	0FPCLKƒNƒƒbƒN(n=0)
									//b2 ƒ}ƒ‹ƒ`ƒvƒƒZƒbƒTƒ‚[ƒhƒrƒbƒg	0Fƒ}ƒ‹ƒ`ƒvƒƒZƒbƒT’ÊM‹@”\‚ğ‹Ö~
									//b3 ƒXƒgƒbƒvƒrƒbƒg’·‘I‘ğ	0F1ƒXƒgƒbƒvƒrƒbƒg
									//b4 ƒpƒŠƒeƒBƒ‚[ƒhƒrƒbƒg	0F‹ô”ƒpƒŠƒeƒB‚Å‘—óM
									//b5 ƒpƒŠƒeƒB‹–‰Âƒrƒbƒg	0FƒpƒŠƒeƒBƒrƒbƒg‚ğ•t‰Á
									//b6 ƒLƒƒƒ‰ƒNƒ^’·ƒrƒbƒg	0Fƒf[ƒ^’·8ƒrƒbƒg‚Å‘—óM
									//b7 ƒRƒ~ƒ…ƒjƒP[ƒVƒ‡ƒ“ƒ‚[ƒhƒrƒbƒg	0F’²•à“¯Šú®ƒ‚[ƒh‚Å“®ì
	SCI1.BRR = SCI_BRR;				//ƒrƒbƒgƒŒ[ƒg’²®
	IPR(SCI1,TXI1) = 7;				//Š„‚è‚İ—Dæ“xİ’è
	IPR(SCI1,RXI1) = 7;				//Š„‚è‚İ—Dæ“xİ’è
	IR(SCI1,TXI1) = 0;				//Š„‚è‚İ—v‹ƒtƒ‰ƒOƒNƒŠƒA
	IR(SCI1,RXI1) = 0;				//Š„‚è‚İ—v‹ƒtƒ‰ƒOƒNƒŠƒA

	for(dummy = 0; dummy < 50000; dummy++); //‚¿‚å‚Á‚Æ‘Ò‚Â

	SCI1.SCR.BIT.TIE = 1;			//TXIŠ„‚è‚İ‹–‰Â
	SCI1.SCR.BIT.RIE = 1;			//RXIŠ„‚è‚İ‹–‰Â
	SCI1.SCR.BIT.RE = 1;			//óMŠJn
	SCI1.SCR.BIT.TE = 1;			//‘—MŠJn
	dummy = SCI1.SSR.BYTE;			//ƒ_ƒ~[ƒŠ[ƒh
	SCI1.SSR.BYTE = 0xC0;			//ƒtƒ‰ƒOƒNƒŠƒA
}

void sci_print(unsigned char data){	//unsigned charå‹ã®æ–‡å­—åˆ—ã‚’é€ã‚‹é–¢æ•°ã€‚ä¸‹ã®é–¢æ•°ã§ä½¿ã£ã¦ã„ã‚‹
	while(!IR(SCI1,TXI1));
	SCI1.TDR = data;				//ãƒ‡ãƒ¼ã‚¿ã‚’è»¢é€
	IR(SCI1,TXI1)=0;
}

char sci_data_get(void){
	char data;
	IR(SCI1,RXI1) = 0;
	while(!IR(SCI1,RXI1));
	data = SCI1.RDR;				//ãƒ‡ãƒ¼ã‚¿ã‚’è¿”ã™
	return data;
}

void print_str(unsigned char *str){	//æ–‡å­—åˆ—ã‚’é€ã‚‹é–¢æ•°ã€‚å¼•æ•°ã‹ã‚‰æ–‡å­—åˆ—ã‚’æŒã£ã¦ãã‚‹
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

	/*æ¡æ•°ä½œæˆ*/
	for(i=0;i<12;i++){
		if(data<digit)break;
		digit*=10;
	}
	digit/=10;
	if(data==0){
		i=1;	
	}

	/*è¡¨ç¤ºæ¡æ•°ã¨æ¯”è¼ƒ*/
	if(n<=i){
		n=i;						//æœ€å¤§æ¡nã‚’æ›´æ–°
		if(min_flag==1){
			sci_print('-');
		}
		if(n<11)
		str[n+1]='\0';				//æ–‡å­—åˆ—çµ‚ç«¯
	}
	else{
		/*iã¯æ¡æ•°nã¯æœ€å¤§æ–‡å­—åˆ—*/
		for(j=0;j<n;j++){		//ç©ºç™½ä½œæˆ 0~n-1
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

	for(i = 0; my_isspace( s[i] ); i++);	//å…ˆé ­ã®ç©ºç™½ã‚’èª­ã¿é£›ã°ã™
	sign = (s[i] == '-') ? -1 : 1;			//ç¬¦å·ã‚’ä¿å­˜ã™ã‚‹
	if(s[i] == '-' || s[i] == '+')				//ç¬¦å·ã‚’é£›ã°ã™
		i++;
	for( n = 0; my_isdigit(s[i]); i++)	//s[i]ãŒæ•°å­—ã®ã‚ã„ã ã€nã¸
		n = 10 * n + s[i] - '0';
	return sign * n;								//ç¬¦å·ã‚’åæ˜ 
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
