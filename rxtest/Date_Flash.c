//�f�[�^�t���b�V���̃A�N�Z�X
//RX62T�p
//



#include "iodefine.h"

//�������ݎ��ɍ폜�����̂�h�~����
#pragma address ID_CODE = 0xFFFFFFA0 
const unsigned long ID_CODE[4] = {0x52000000, 0x00000000, 0x00000000, 0x00000000}; //id�R�[�h��0x00...0x00�ɐݒ�
#pragma address ROM_CODE = 0xFFFFFF9C 
const unsigned long ROM_CODE = 0x00000000;

//�G���[�}�N��
#define FLD_OK		0
#define FLD_ERROR	1
#define FLD_BLANK	2
#define FLD_NOBLANK	3
#define FLD_TMOUT	4

#define _PCLK_		48			//PCLK��48MHz
#define _ICLK_		96000		//��܂��Ȏ��Ԃ̌v�Z�Ƃ��� ���[�v��=�҂�����(usec)*ICLK(kHz)/5000-4���g��

/*---------------------------��܂��ȗ���-----------------------------*/
//�܂����������s��
//���ɏ������݂����u���b�N�ɃN���b�N��ʒm����
//�������݂��������炻�����u�����N�ƕ������Ă���Ȃ炻�̂܂܏������߂�
//�u�����N����Ȃ������珑�����߂Ȃ��̂ŏ�������K�v������
//�����������̓u���b�N���Ƃł��邽��1�u���b�N,����2KByte��C�ɏ������ƂɂȂ�
//���̎���֐��ɂ����Ĉ���addr�̓u���b�N�ԍ���\��
//�f�[�^�t���b�V����32KByte��RX�Ȃ��16�u���b�N����̂Ŕԍ���0~15�܂�
//offset��2KByte�̃u���b�N��8Byte���Ƃɕ��������Ƃ��̉��Ԗڂɏ������ނ��������Ă���
//�Ȃ����̊֐��ɂ����ď������݂Ɠǂݏo���͂ǂ����short�^�z��z��4�ł���肷��

/*---------------------------�v���g�^�C�v�錾--------------------------*/
//������
void fld_init_fcu_ram(void);
//�N���b�N�ʒm
unsigned long fld_init_pclk_notification(unsigned char addr);
//�u�����N�`�F�b�N���ău�����N����Ȃ��Ȃ�f���[�g����
void Check_Delete(unsigned char addr);
//1�u���b�N�f���[�g����
unsigned long fld_erase_2KB(unsigned char addr);
//�f�[�^�t���b�V������z����g���ēǂݏo��
unsigned short * Data_flash_read(unsigned char addr,unsigned char offset);
//�f�[�^�t���b�V���ɏ�������
void Date_flash_write(unsigned char addr,unsigned char offset,unsigned short *data);
//�������߂Ă邩�m�F����
unsigned long verify(unsigned short *verify,unsigned char addr);


/*---------------------------�ȉ��v���O����----------------------------*/

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


//FCU���Z�b�g�֐�
static void reset_fcu(void)
{
	volatile unsigned long w;
	
	//�\�t�g�E�F�A���Z�b�g
	FLASH.FRESETR.BIT.FRESET =1;
	
	//35us�҂�
	_wait_usec_(35);
	
	
	//�\�t�g�E�F�A���Z�b�g����
	FLASH.FRESETR.BIT.FRESET =0;
}

//FCURAM�փR�s�[����֐�
void fld_init_fcu_ram(void)
{
	int i;
	static const int fcu_ram_size = 8*1024; //8KByte
	volatile unsigned long *fcu_ram = (unsigned long *)0x007F8000;
	const volatile unsigned long *fcu_fw = (unsigned long *)0xFEFFE000;
	
	//���[�h���[�h��
	if(FLASH.FENTRYR.WORD & 0x00ff){
		FLASH.FENTRYR.WORD = 0xAA00;	//�L�[�R�[�h��AAh
	}
	
	//FCURAM�G���A�A�N�Z�X����
	FLASH.FCURAME.WORD = 0xC401;		//�L�[�R�[�h��C4h
	
	//FCU F/W����FCURAM�փR�s�[
	for(i=0;i<fcu_ram_size/sizeof(unsigned long);i++){
		*fcu_ram++ = *fcu_fw++;
	}
	
}

//���ӃN���b�N��FCU�ɒʒm����֐�
unsigned long fld_init_pclk_notification(unsigned char addr)
{
	//00100000h��DB00�u���b�N�̃g�b�v�A�h���X
	volatile unsigned char *addr_b = (unsigned char *)data_flash_BlockAddresses[addr];
	volatile unsigned short *addr_w = (unsigned short *)data_flash_BlockAddresses[addr];
	
	//P/E���[�h�Ƀ`�F���W
	if((FLASH.FENTRYR.WORD & 0x00ff) != 0x0080){
		FLASH.FENTRYR.WORD = 0xAA80;	//�L�[�R�[�h��AAh
	}
	
	//���ӃN���b�N��ݒ�
	FLASH.PCKAR.BIT.PCKA = _PCLK_;			//PCLK��48MHz
	
	//���ӃN���b�N�ʒm�R�}���h���s
	*addr_b = 0xE9;
	*addr_b = 0x03;
	*addr_w = 0x0F0F;
	*addr_w = 0x0F0F;
	*addr_w = 0x0F0F;
	*addr_b = 0xD0;
	
	//�R�}���h������҂�
	if(wait_FRDY(120)==FLD_TMOUT){
		reset_fcu();	
	}
	
	//�G���[�`�F�b�N
	if(FLASH.FSTATR0.BIT.ILGLERR == 1){
		return FLD_ERROR;
	}
	
	return FLD_OK;
}

//�u�����N�`�F�b�N�֐�
//2KB�u�����N�`�F�b�N
unsigned long fld_blank_check_2KB(unsigned char addr)
{
	volatile unsigned char *addr_b = (unsigned char *)data_flash_BlockAddresses[addr];
	
	//�u�����N�`�F�b�N�R�}���h�g�p
	FLASH.FMODR.BIT.FRDMD = 1;
	
	//�u�����N�`�F�b�N�̃T�C�Y��2KByte�Ɏw��
	FLASH.DFLBCCNT.BIT.BCSIZE = 1;
	
	//�u�����N�`�F�b�N�R�}���h�g�p
	*addr_b = 0x71;
	*addr_b = 0xD0;
	
	//�^�C���A�E�g�҂�
	if(wait_FRDY(700*1.1)==FLD_TMOUT){
		reset_fcu();	
	}
	
	//�G���[�`�F�b�N
	if(FLASH.FSTATR0.BIT.ILGLERR == 1){
		return FLD_ERROR;
	}
	
	//�u�����N�`�F�b�N���ʂ�Ԃ�
	if(FLASH.DFLBCSTAT.BIT.BCST == 0){
		return FLD_BLANK;
	}
	return FLD_NOBLANK;
}
//8B�u�����N�`�F�b�N
unsigned long fld_blank_check_8B(unsigned char addr,unsigned char offset)
{
	volatile unsigned char *addr_b = (unsigned char *)data_flash_BlockAddresses[addr];
	
	//�u�����N�`�F�b�N�R�}���h�g�p
	FLASH.FMODR.BIT.FRDMD = 1;
	
	//�u�����N�`�F�b�N�̃T�C�Y��8Byte�Ɏw��
	FLASH.DFLBCCNT.BIT.BCSIZE = 0;
	FLASH.DFLBCCNT.BIT.BCADR = offset;
	
	//�u�����N�`�F�b�N�R�}���h�g�p
	*addr_b = 0x71;
	*addr_b = 0xD0;
	
	//�^�C���A�E�g�҂�
	if(wait_FRDY(30*1.1)==FLD_TMOUT){
		reset_fcu();	
	}
	
	//�G���[�`�F�b�N
	if(FLASH.FSTATR0.BIT.ILGLERR == 1){
		return FLD_ERROR;
	}
	
	//�u�����N�`�F�b�N���ʂ�Ԃ�
	if(FLASH.DFLBCSTAT.BIT.BCST == 0){
		return FLD_BLANK;
	}
	return FLD_NOBLANK;
}

//�����֐�
unsigned long fld_erase_2KB(unsigned char addr)
{
	volatile unsigned char *addr_b = (unsigned char *)data_flash_BlockAddresses[addr];
	unsigned long ret = FLD_OK;
	
	
	//P/E���[�h�Ƀ`�F���W
	if((FLASH.FENTRYR.WORD & 0x00ff) != 0x0080){
		FLASH.FENTRYR.WORD = 0xAA80;	//�L�[�R�[�h��AAh
	}
	
	//�����v���e�N�g����
	FLASH.FWEPROR.BIT.FLWE = 1;
	
	//�u���b�N�P�ʍ폜����
	FLASH.DFLWE0.WORD = 0x1EFF;		//�L�[�R�[�h��1Eh
	FLASH.DFLWE1.WORD = 0xE1FF;		//�L�[�R�[�h��E1h
	
	//�u���b�N�����R�}���h���s
	*addr_b = 0x20;
	*addr_b = 0xD0;
	
	
	//�R�}���h������҂�
	if(wait_FRDY(250*1000*1.1)==FLD_TMOUT){
		reset_fcu();	
	}
	
	
	//�G���[�`�F�b�N
	if((FLASH.FSTATR0.BIT.ILGLERR == 1)||
	   (FLASH.FSTATR0.BIT.ERSERR == 1)){
		ret = FLD_ERROR;
	}
	 
	//�v���e�N�g�Ə�������
	FLASH.FWEPROR.BIT.FLWE = 2;
	FLASH.DFLWE0.WORD = 0x1E00;
	FLASH.DFLWE1.WORD = 0xE100;
	
	return ret;
}

//�������݊֐�
unsigned long fld_program_8byte(unsigned long addr,unsigned short *ram)
{
	volatile unsigned char *addr_b = (unsigned char *)addr;
	volatile unsigned short *addr_w = (unsigned short *)addr;
	unsigned long i,ret = FLD_OK;
	
	//�������݃v���e�N�g����
	FLASH.FWEPROR.BIT.FLWE = 1;
	
	//�u���b�N�P�ʂ̏������݋���
	FLASH.DFLWE0.WORD = 0x1EFF;		//�L�[�R�[�h��1Eh
	FLASH.DFLWE1.WORD = 0xE1FF;		//�L�[�R�[�h��E1h
	
	//�v���O�����R�}���h���s
	*addr_b = 0xE8;
	*addr_b = 0x04;
	for(i=0;i<4;i++){				//8Byte��4Word���̃T�C�Y
		*addr_w = *ram++;
	}
	*addr_b = 0xD0;
	
	//�R�}���h������҂�
	if(wait_FRDY(2*1000*1.1)==FLD_TMOUT){
		reset_fcu();	
	}
	
	//�G���[�`�F�b�N
	if((FLASH.FSTATR0.BIT.ILGLERR == 1)||
	   (FLASH.FSTATR0.BIT.PRGERR == 1)){
		ret = FLD_ERROR;
	}
	
	
	//�v���e�N�g�Ə�������
	FLASH.FWEPROR.BIT.FLWE = 2;
	FLASH.DFLWE0.WORD = 0x1E00;
	FLASH.DFLWE1.WORD = 0xE100;
	
	return ret;
}

//�ǂݏo�����֐�
unsigned long fld_enable_read(void)
{
	
	//���[�h���[�h��
	if(FLASH.FENTRYR.WORD & 0x00ff){
		FLASH.FENTRYR.WORD = 0xAA00;	//�L�[�R�[�h��AAh
	}
	
	//�u���b�N�P�ʂ̏������݋���
	FLASH.DFLRE0.WORD = 0x2DFF;		//�L�[�R�[�h��2Dh
	FLASH.DFLRE1.WORD = 0xD2FF;		//�L�[�R�[�h��D2h
	
	return FLD_OK;
}




//�R�}���h�����܂ő҂֐�
int wait_FRDY(int t)
{
	int cn=0;
	while(FLASH.FSTATR0.BIT.FRDY==0)
	{
		_wait_usec_(1);//1us�@wait
		cn++;
		if(cn==t)return FLD_OK;
	}
	return FLD_TMOUT;
}

//�K����usec�҂�
_wait_usec_(int t){
	int i;
	t = t*_ICLK_/5000-4;
	for(i=0;i<t;i++);
}

//�����ݒ� pclk_notification�̈����̓u���b�N�ԍ�
void Data_Flash_init(void){
	fld_init_fcu_ram();
	fld_init_pclk_notification(0);
}

//�u�����N�`�F�b�N����� �����͏�������u���b�N�ԍ�
void Check_Delete(unsigned char addr)
{
	
	//P/E���[�h�Ƀ`�F���W
	if((FLASH.FENTRYR.WORD & 0x00ff) != 0x0080){
		FLASH.FENTRYR.WORD = 0xAA80;	//�L�[�R�[�h��AAh
	}
	
	//�u�����N�`�F�b�N
	if(fld_blank_check_2KB(addr)==FLD_NOBLANK){
		fld_erase_2KB(addr);
	}
}

//8Byte�����o�� �����̓u���b�N�ԍ�,�I�t�Z�b�g�ԍ�,�f�[�^
void Date_flash_write(unsigned char addr,unsigned char offset,unsigned short *data)
{
	fld_program_8byte(data_flash_BlockAddresses[addr]+8*offset,data);
}

//8Byte�ǂݏo�� �����̓u���b�N�ԍ�,�I�t�Z�b�g�ԍ�
unsigned short * Data_flash_read(unsigned char addr,unsigned char offset)
{
	unsigned short *read;
	
	fld_enable_read();
	
	read = (unsigned short *)(data_flash_BlockAddresses[addr]+offset*8);

	
	return read;
}


//�x���t�@�C
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

