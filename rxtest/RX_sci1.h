/* PCとの通信用 */
/* 最初に init_sci1() 関数を実行 */
/* その後通信可能(受信割り込み未実装なので,in()関数を実行すると処理が止まります) */

/* 関数の説明 */
/* void init_sci1(void)               :SCI1の初期化関数.これを実行しないと通信出来ない */
/* void out(char)                     :1文字PCに送信する関数.他の表示関数で多用 */
/* void outs(char *)                  :文字列送信用.改行を入れると改行するように.多分ユーザはこれを1番使う */
/* void dec_out(unsinged long i,int n):数値送信用関数.iに表示する数値,nに表示する桁数を入れる */
/* long dec_outs(long,long)           :dec_outs()関数内で使用.ユーザは使わない */
/* void bit_out(unsigned char)        :bit毎の1,0を上位bitから送信する関数 */
/* char in(void)                      :1文字受信の関数1byteで返す */
/* int dec_in(void)                   :数値を受信する関数.0未満は受け付けないと思う.一応7桁まで */

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