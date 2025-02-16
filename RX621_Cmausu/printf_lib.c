/************************************************************************/
/* 対象マイコン RX621                                                   */
/* ﾌｧｲﾙ内容     printf,scanf関連処理                                    */
/* バージョン   Ver.1.00                                                */
/* Date         		                                                */
/* Copyright    		    						                     */
/************************************************************************/

/*======================================*/
/* インクルード                         */
/*======================================*/
#include    <stdio.h>
#include 	<stdarg.h>
#include    <machine.h>
#include    "iodefine.h"
#include    "printf_lib.h"              /* printf関連処理               */


/*======================================*/
/* シンボル定義                         */
/*======================================*/
#define     SEND_BUFF_SIZE  64          /* 送信バッファサイズ           */
#define     RECV_BUFF_SIZE  32          /* 受信バッファサイズ           */

/*======================================*/
/* グローバル変数の宣言                 */
/*======================================*/
/* 送信バッファ */
static volatile char    send_buff[SEND_BUFF_SIZE];
static volatile char    *send_w = send_buff;
static volatile char    *send_r = send_buff;
static volatile int     send_count = 0;

/* 受信バッファ */
static int             recvFlag;        /* 受信フラグ                   */
static unsigned char   recvData;        /* 受信データ                   */
static unsigned char   recvError;       /* 受信エラー                   */

/*======================================*/
/* プロトタイプ宣言                     */
/*======================================*/
void setSendBuff( char c);
int getSendBuff( char *c );


/************************************************************************/
/* UART0の初期化、及びprintf関係をUART0に割り当て                       */
/* 引数　 通信速度                                                      */
/* 戻り値 なし                                                          */
/************************************************************************/
void initSCI1( int sp )
{
    volatile int i;

    MSTP(SCI1) = 0 ;                    // モジュールストップ状態の解除

	IOPORT.PFFSCI.BIT.SCI1S=0;  //P30をRxD1-A端子として設定 P27をSCK1-A端子として設定 P26をTxD1-A端子として設定
 	PORT3.DDR.BIT.B0 = 0;       //RxD1を入力にするために必要
	PORT3.ICR.BIT.B0 = 1; 		// RxD1の入力バッファON
	PORT2.DDR.BIT.B6 = 1;       //TxD1を出力にするために必要	
	
    SCI1.SCR.BYTE = 0 ;                 // 内部クロックの選択
	
    // BRR = PCLK * 10^6 / ( 64 * 2^(2n-1) * B) - 1
    // PCLK = 12.288*2, n=PCLKクロックの場合による B=ボーレート[bps]
    // n = PCLK/1:0 PCLK/4:1 PCLK/16:2 PCLK/64:3
    if( sp == SPEED_4800 ) {
        SCI1.SMR.BYTE = 0x01 ;          // PLCK/4, 調歩同期, bit 8, パリティ なし
        SCI1.BRR = 40-1;                // 24576000 / ( 64 * 2 * 4800 ) - 1
    } else if( sp == SPEED_9600 ) {
        SCI1.SMR.BYTE = 0x00 ;          // PLCK, 調歩同期, bit 8, パリティ なし
        SCI1.BRR = 80-1;               //  24576000 / ( 64 * 0.5 * 9600 ) - 1
    } else if( sp == SPEED_19200 ) {
        SCI1.SMR.BYTE = 0x00 ;          // PLCK, 調歩同期, bit 8, パリティ なし
        SCI1.BRR = 40-1;                // 24576000 / ( 64 * 0.5 * 19200 ) - 1
    } else if( sp == SPEED_38400 ) {
        SCI1.SMR.BYTE = 0x00 ;          // PLCK, 調歩同期, bit 8, パリティ なし
        SCI1.BRR = 20-1;                // 24576000 / ( 64 * 0.5 * 38400 ) - 1
    }
    for(i=0; i<4000; i++);              // 1ビット期間(52μs,ICLK=96MHz)待ち
    SCI1.SCR.BYTE = 0xf0;               // 送受信許可,送受信割り込み許可


    IPR(SCI1,    ) = 7 ;                // SCI1 割り込み優先レベル=7
    IEN(SCI1,TXI1) = 1 ;                // SCI1 送信割り込み許可(IEN)
    IEN(SCI1,RXI1) = 1 ;                // SCI1 受信割り込み許可(IEN)
    IEN(SCI1,ERI1) = 1 ;                // SCI1 受信エラー割り込み許可(IEN)

	printf2("start\r\n");//初回のノイズ？対策
}

/************************************************************************/
/*													                     */
/* 引数　 			                                                      */
/* 戻り値 なし                                                          */
/************************************************************************/
void printf2(const char *format, ...){
    char   put_data;
	char send_data[SEND_BUFF_SIZE];
	va_list va;
	int i = 0;
	volatile int cnt = 0;
	 
    va_start(va, format);
	vsprintf(send_data,format, va );
    va_end(va);
	
	for(i = 0; send_data[i] != '\0';i++){
		
		if( send_data[i] == '\n' )  {
        	setSendBuff( '\r' );
    	} else if( send_data[i] == '\b' ) {
        	setSendBuff( '\b' );
        	setSendBuff( ' ' );
    	}
	
    	setSendBuff( send_data[i]);

    	// 何も送信していないなら、1文字目はここで送信する
    	// その後は送信割り込みで送信する
    	if( SCI1.SSR.BIT.TEND == 1 ) {
			for(cnt = 0;cnt < 1000;cnt++);
        	getSendBuff( &put_data );
        	put_sci1( put_data );
    	}
	}

    return;
}

/************************************************************************/
/*													                     */
/* 引数　 			                                                      */
/* 戻り値 なし                                                          */
/************************************************************************/
void scanf2(const char *format, ...){
    char   c;
	va_list va;
	static char    read_data[RECV_BUFF_SIZE];
	int i = 0;
	
    do {
    	/* 受信待ち */
        while( get_sci1( &c ) != 1 );

        switch( c ) {
    	    case '\b':/* バックスペース */
                /* 何もバッファにないならBSは無効 */
                if( 0 < i ) continue;
                /* あるなら一つ戻る */
                i--;
                break;
            case '\r':/* Enterキー */
                read_data[i++] = c = '\n';
                read_data[i++] = '\r';
                break;
            default:
                read_data[i++] = c;
                break;
        }
        /* エコーバック 入力された文字を返す   なぜかこの処理が必要*/
		printf2("%c",c);
    } while( c != '\n' && i < RECV_BUFF_SIZE-2);
  
	read_data[i++] = '\0';
	
	va_start(va, format);
	vsscanf(read_data,format, va );
    va_end(va);

    return;
}



/************************************************************************/
/* １文字受信                                                           */
/* 引数　 受信文字格納アドレス                                          */
/* 戻り値 -1:受信エラー 0:受信なし 1:受信あり 文字は*sに格納            */
/************************************************************************/
int get_sci1( char *s )
{
    volatile int ret = 0;

    if( recvFlag == 1 ){                // 受信データあり？
        recvFlag = 0;
        *s = recvData;
        ret = 1;
        if( recvError ) {               // エラーあり？
            recvError = 0;
            ret = -1;
        }
    }
    return ret;
}

/************************************************************************/
/* １文字出力                                                           */
/* 引数　 送信データ                                                    */
/* 戻り値 0:送信中のため、送信できず 1:送信セット完了                   */
/************************************************************************/
int put_sci1( char r )
{
    if( SCI1.SSR.BIT.TDRE == 1) {       // TDRに書き込みが出来るまで待つ
        SCI1.TDR = r;
        return 1;
    } else {
        /* 送信中(今回のデータは送信せずに終了) */
        return 0;
    }
}

/************************************************************************/
/* 送信バッファに保存                                                   */
/* 引数　 格納文字                                                      */
/* 戻り値 なし                                                          */
/* メモ   バッファがフルの場合、空くまで待ちます                        */
/************************************************************************/
void setSendBuff( char c )
{
    // バッファが空くまで待つ
    while( SEND_BUFF_SIZE == send_count );

    IEN(SCI1,TXI1) = 0;

    *send_w++ = c;
    if( send_w >= send_buff+SEND_BUFF_SIZE ) send_w = send_buff;
    send_count++;

    IEN(SCI1,TXI1) = 1;
}

/************************************************************************/
/* 送信バッファから取得                                                 */
/* 引数　 格納する文字のアドレス                                        */
/* 戻り値 0:データなし 1:データあり                                     */
/************************************************************************/
int getSendBuff( char *c )
{
    volatile int    ret = 0;

    if( send_count ) {                  // データがあるならバッファから出す
        IEN(SCI1,TXI1) = 0;

        *c = *send_r++;
        if( send_r >= send_buff+SEND_BUFF_SIZE ) send_r = send_buff;
        send_count--;
        ret = 1;

        IEN(SCI1,TXI1) = 1;
    }
    return ret;
}

/************************************************************************/
/* SCI1 ERI1 割り込み処理                                               */
/************************************************************************/
#pragma interrupt Excep_SCI1_ERI1(vect=VECT_SCI1_ERI1)
void Excep_SCI1_ERI1(void)
{
    recvError = SCI1.SSR.BYTE & 0x38;   // 受信エラーフラグ読み出し

    SCI1.SSR.BYTE = 0xc0 ;              // 受信エラーフラグクリア
    while( (SCI1.SSR.BYTE & 0x38) );    // エラーフラグの“0”クリア確認
    while( IR(SCI1,ERI1) );             // 受信エラーの割り込みステータスビットが0か確認
}

/************************************************************************/
/* SCI1 RXI1 割り込み処理                                               */
/************************************************************************/
#pragma interrupt Excep_SCI1_RXI1(vect=VECT_SCI1_RXI1)
void Excep_SCI1_RXI1(void)
{
    recvData = SCI1.RDR ;               // 受信データ読み出し
    recvFlag = 1 ;                      // フラグ変数を１にセット
}

/************************************************************************/
/* SCI1 TXI1 割り込み処理                                               */
/************************************************************************/
#pragma interrupt Excep_SCI1_TXI1(vect=VECT_SCI1_TXI1)
void Excep_SCI1_TXI1(void)
{
    char   c;
    int    ret;

    ret = getSendBuff( &c );            // データ取得
    if( ret ) {
        SCI1.TDR = c;                   // データあるなら送信
    }
}


/************************************************************************/
/* printfで呼び出される関数                                             */
/* ユーザーからは呼び出せません                                         */
/* printfが使用できない（*bufの値が異常）ので上のprint関数を自作        */
/************************************************************************/
long write(long fileno, const unsigned char *buf, long count)
{
    return 1;
}

/************************************************************************/
/* scanfで呼び出される関数                                              */
/* ユーザーからは呼び出せません                                         */
/* なぜか使用できません			                                        */
/************************************************************************/
long read(long fileno, unsigned char *buf, long count)
{
    return 1;
}
/************************************************************************/
/* end of file                                                          */
/************************************************************************/

