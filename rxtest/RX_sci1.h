/* PC�Ƃ̒ʐM�p */
/* �ŏ��� init_sci1() �֐������s */
/* ���̌�ʐM�\(��M���荞�ݖ������Ȃ̂�,in()�֐������s����Ə������~�܂�܂�) */

/* �֐��̐��� */
/* void init_sci1(void)               :SCI1�̏������֐�.��������s���Ȃ��ƒʐM�o���Ȃ� */
/* void out(char)                     :1����PC�ɑ��M����֐�.���̕\���֐��ő��p */
/* void outs(char *)                  :�����񑗐M�p.���s������Ɖ��s����悤��.�������[�U�͂����1�Ԏg�� */
/* void dec_out(unsinged long i,int n):���l���M�p�֐�.i�ɕ\�����鐔�l,n�ɕ\�����錅�������� */
/* long dec_outs(long,long)           :dec_outs()�֐����Ŏg�p.���[�U�͎g��Ȃ� */
/* void bit_out(unsigned char)        :bit����1,0�����bit���瑗�M����֐� */
/* char in(void)                      :1������M�̊֐�1byte�ŕԂ� */
/* int dec_in(void)                   :���l����M����֐�.0�����͎󂯕t���Ȃ��Ǝv��.�ꉞ7���܂� */

#ifndef __SCI1
#define __SCI1

extern void init_sci1(void);
extern void out(char);
extern void outs(char *);
extern void dec_out(long,int);
extern long dec_outs(long,long);
extern void bit_out(unsigned char);
extern char in(void);
extern int dec_in(void);

#endif