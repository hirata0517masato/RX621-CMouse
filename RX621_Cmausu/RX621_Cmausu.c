/****************************************************************************************************/
/* プロジェクト名：RX621_SAMPLE     	                　　　　　          			    */
/* モジュール名：  			                     					    */
/* 履    歴    ：										    */
/* 使用マイコン：RX621 R5F56218BDFP (Flash ROM:512KB, SRAM:64KB, DATA Flash 32KB)                   */
/* 作成者      ：hirata0517masato               				                    */
/* バージョン  ：0.00                                                                               */
/* 作成日      ：2022/02/06 									    */
/****************************************************************************************************/                  
#include "iodefine.h"
#include <machine.h>
#include <stdlib.h>
#include "Parameters.h"
#include "wait.h"
#include "Gyro.h"
#include "DataFlash.h"
#include "Encoder.h"
#include "Motor.h"
#include "Queue.h"
#include "IR.h"
//#include "usb.h"

#include "printf_lib.h"   /* printf2 関連処理    コンパイルおよびライブラリジェネレートオプションにてC99対応が必要  */

#define PRINT /* 使用時は有効化すること*/

#ifndef PRINT
	#define printf2(...)  
	//#define scanf2(...) 　
#endif /* PRINT*/

/*======================================*/
/* プロトタイプ宣言                     */
/*======================================*/
void ALL_init(void);
void CLK_init(void);
void IO_init(void);
void AD_init(void);
void CMT_init(void);
void CMT2_init(void);
void MTU0_init(void);
void MTU1_init(void);

void led(char);
void led_up(void);
void led_down(void);
int get_sw(void);

void maze_save(void);
void maze_load(void);
void log_reset(void);
void log_save(void);
void log_load(void);
void maze_update(char,char,char,char);
void maze_search_adachi(short,short);
void maze_search_all(void);
void shortest_path_search_fin(void);
void remake_shortest_path_list_naname(void);
void run_shortest_path_fin(char);
	
void L_rotate(long long);
void R_rotate(long long);
void S_run(long long,int, char,char);
	
/* 定数設定 */
#define true 1
#define false 0

#define LOG_BUF_MAX 32  //DataFlash_write2 内の数値を合わせること
#define LOG_MAX 2048


//グローバル変数
short motor_stop_cnt = 0;
char maze_w[H][W] = {0};	//上位4bit = 壁の確定bit 下位4bit = 壁の情報（未確定含む）
short maze_d[H][W][4] = {0};	//4方向分の重み

short dx[4] = {0,1,0,-1},dy[4] = {-1,0,1,0};

char my_x = Start_x,my_y = Start_y,my_angle = Start_angle;//0:up 1:right 2:down 3:left

int ir_flag = 0; // 0:赤外線OFF 1:赤外線ON

int run_fin_speed_offset = 0;

long long time_limit = -1;

uint8_t log[LOG_MAX] = {0};
uint8_t log_buff[LOG_BUF_MAX] = {0};
char log_start = 0;
int log_cnt = 0;
int log_save_cnt = 0;
int log_block_num = 1; //max 15

/***********************************************************************/
/* メインプログラム                                                    */
/***********************************************************************/
void main(void)
{
/*	unsigned char c,buf[256];
	
 	float l=0,r=0;
	int aa;
	
	uint8_t prog_buff[512] = "hello world RX621";
	uint8_t read_buff[512];
	
*/
	volatile int i = 0;
	int mode = 0;
	ir_flag = 0;//赤外線OFF
	
	ALL_init();//初期化
	
	delay(100);
/*
	while(1){
		led(0);
		ir_flag = 1;//赤外線ON
		//motor(20,20);

		//printf2("%d\t%d\t%d\t%d :",get_IR_base(0),get_IR_base(1),get_IR_base(2),get_IR_base(3));
		
		printf2("%d\t%d\t%d\t%d\n",get_IR(0),get_IR(1),get_IR(2),get_IR(3));
		//printf2("%ld\n",get_encoder_total_L());
		if(get_sw() == 1){
			Encoder_reset();
		}
		delay(100);
	}
*/			
	while(1){
		Encoder_reset();
		
		ir_flag = 0;//赤外線OFF
		
		//モード選択
		while(1){
			led(mode);
			
			mode = get_encoder_total_R() / 50;
			
			if(get_sw() == 1){
				led_up();
				while(get_sw() == 1)nop();
				break;
			}
		}
		
		my_x = Start_x;
		my_y = Start_y;
		my_angle = Start_angle;
		
		if(mode != 5 && mode != 6 && mode != 7){//速度調整モードではない　＆＆　迷路情報のリセットでなければ ＆＆　ログ出力でなければ
			ir_flag = 1;//赤外線ON
			
			led(6);
			//スイッチ入力待ち
			while(get_sw() == 0){
				if(get_IR(0) > 10 && get_IR(3) < 10){
					led(1);
					
				}else if(get_IR(0) < 10 && get_IR(3) > 10){
					led(8);
				}else if(get_IR(0) > 10 && get_IR(3) > 10){
					led(9);
				}else{
					led(6);
				}
			}
			while(get_sw() == 1) nop();
		
			
			//正面センサーに手をかざす
			while((get_IR(0) < 20) && (get_IR(3) < 20)) led(1);
  			while((get_IR(0) > 20) || (get_IR(3) > 20)) led(8);
				
			led_up();
			
			GyroSum_reset();
			Encoder_reset();
		}
		
		switch(mode){
			case 1://探索モード
			
				log_reset();//ログの初期化
				log_start = 1; //ログ記録開始　30msに１回記録
				
				maze_search_adachi(Goal_x,Goal_y);
		
				maze_search_adachi(Start_x,Start_y);
				
				log_start = 0; //ログ記録終了
				
				break;
				
				
			case 2://最短走行モード
				shortest_path_search_fin();
				
				log_reset();//ログの初期化
				log_start = 2; //ログ記録開始 10msに１回記録
				
				run_shortest_path_fin(false);
				
				log_start = 0; //ログ記録終了
				
				led(9);
				delay(500);
				led_up();
				
				my_x = Goal_x;
				my_y = Goal_y;
				my_angle = Goal_angle;
				
				maze_search_adachi(Pickup_x,Pickup_y);//拾いやすいところまで移動する
				
				break;
			case 3://最短走行（斜めあり）モード
				shortest_path_search_fin();
				remake_shortest_path_list_naname();
				
				log_reset();//ログの初期化
				log_start = 2; //ログ記録開始 10msに１回記録
				
				run_shortest_path_fin(true);
				
				log_start = 0; //ログ記録終了
				
				led(9);
				delay(500);
				led_up();
				
				my_x = Goal_x;
				my_y = Goal_y;
				my_angle = Goal_angle;
				
				maze_search_adachi(Pickup_x,Pickup_y);//拾いやすいところまで移動する
				
				break;
				
			case 4://最短経路上の未確定マスをすべて探しに行く
				maze_search_all();
				
				led(9);
				delay(500);
				led_up();
				
				my_x = Goal_x;
				my_y = Goal_y;
				my_angle = Goal_angle;
				
				maze_search_adachi(Pickup_x,Pickup_y);//拾いやすいところまで移動する
				
				break;
				
			case 5://速度調整モード（調整値は保存しない、本番での微調整用）
				Encoder_reset();
		
				led(6);
				delay(1000);
				
				//モード選択
				while(1){
					led(run_fin_speed_offset);
			
					run_fin_speed_offset = get_encoder_total_R() / 50;
			
					if(get_sw() == 1){
						led_up();
						while(get_sw() == 1)nop();
						break;
					}
				}
		
				break;
			case 6://迷路情報リセットモード
				for(int i = 0; i < H;i++)for(int j = 0; j < W; j++)maze_w[i][j] = 0;
				
				//迷路の外周の確定壁を設定
				//0
  				for(int i = 0; i < W;i++)maze_w[0][i] |= 0x11;
  				//1
  				for(int i = 0; i < H;i++)maze_w[i][W-1] |= 0x22;
  				//2
  				for(int i = 0; i < W;i++)maze_w[H-1][i] |= 0x44;
  				//3
  				for(int i = 0; i < H;i++)maze_w[i][0] |= 0x88;
  
				//スタート地点の確定壁を設定
  				maze_w[my_y][my_x] = 0xfd;//13;//15-2;
  				maze_w[my_y+1][my_x] |= 0x11;
				maze_w[my_y][my_x+1] |= 0x80;
				
				break;
				
			case 7://ログ出力モード
				for(int i = 0; i < H;i++){
					for(int j = 0; j < W; j++)printf2("%d\t",maze_w[i][j]&0x0f);
					printf2("\n");
				}
				printf2("\n");
				
				log_load();//リード＆出力
				break;
				
			default:
				led(9);
				delay(200);
				led(0);
				delay(200);
				led(9);
				delay(200);
				led(0);
				delay(200);
				break;
		}
		
		ir_flag = 0;//赤外線OFF
		
		led(9);
		//スイッチ入力待ち
		while(get_sw() == 0) nop();
		while(get_sw() == 1) nop();
		
		maze_save();
		
		led_up();
		led_down();
		
		//スイッチ入力待ち
		while(get_sw() == 0) nop();
		while(get_sw() == 1) nop();
		
		led(9);
		delay(100);
		
	}
}




/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */
/* 関 数 概 要：初期化    	                                                                    */
/* 関 数 詳 細：各種初期化関数のまとめ 　　　　　　　　　　　　　　                                 */
/* 引       数：なし										    */
/* 戻  り   値：なし										    */
/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */ 
void ALL_init(){
	// マイコン機能の初期化 
	CLK_init();  // クロックの初期化
 	IO_init();   // IOの初期化
	WAIT_init(); //wait(CMT1)の初期化
 	AD_init();   // A/Dの初期化
	MTU0_init();  //モーターの初期化
	MTU1_init();  //エンコーダの初期化
	DataFlash_init();//データフラッシュの初期化
	
	led(0);
	ir(0);
	
	maze_load();//迷路データの読み込み
  
	led(6);
	//スイッチ入力待ち
	while(get_sw() == 0) nop();
	led(0);
	while(get_sw() == 1) nop();
	
	Gyro_init();	//ジャイロ、SPIの初期化 　注意：少し時間かかります 処理中はジャイロセンサーを動かさないこと

	CMT_init();  // CMT0の初期化
	CMT2_init();  // CMT2の初期化

	initSCI1(SPEED_9600);
	//USB_init();  //USB CDCの初期化
	
	led(9);
}

/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */
/* 関 数 概 要：クロックの初期化                                                                    */
/* 関 数 詳 細：システムクロック96MHz,周辺モジュールクロック24MHz                                   */
/* 引       数：なし										    */
/* 戻  り   値：なし										    */
/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */ 
void CLK_init(void)
{
   SYSTEM.SCKCR.BIT.ICK = 0;		// システムクロック(ICLK)       EXTAL×8 (96MHz)
   SYSTEM.SCKCR.BIT.PCK = 2;		// 周辺モジュールクロック(PCLK)	EXTAL×2 (24MHz)
     
   //SYSTEM.SUBOSCCR = 1;              // 0：サブクロック発振器動作（0：デフォルトで動作 1:停止）
   //RTC.RCR2.BIT.RTCOE = 1;           // 1：RTCOUTを端子(P32)から出力する
   
}

/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */
/* 関 数 概 要：I/Oの初期化                                                                    　　 */
/* 関 数 詳 細：                                                                                    */
/* 引       数：なし										    */
/* 戻  り   値：なし										    */
/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */ 
void IO_init(void)
{
 	PORT1.DDR.BYTE = 0xff;           // 4:PWM_R1 3:MTIOC0B
	
    PORT2.DDR.BYTE = 0xff;           // 6:TxD1	4:PWM_R2	2:MTIOC3B	1:MTIOC1B	0:PWM_L2

    PORT3.DDR.BYTE = 0x10;           // 4:PWM_L1	0:RxD1
	
    PORT4.DDR.BYTE = 0x00;           // 7:AN7	6:AN6	5:AN5	4:AN4	3:AN3	2:AN2	1:AN1	0:AN0  
	
	PORT5.DDR.BYTE = 0x00;			 //4:MTIOC4B-B
	
	PORTA.DDR.BYTE = 0x0f; 			 //4:SW	3:LED4	2:LED3	1:LED2	0:LED1
	 
    PORTB.DDR.BYTE = 0x00; 	     	 // 5:左エンコーダB		4:左エンコーダA		3:右エンコーダB		2:右エンコーダA
	
	//PORTC.DDR.BYTE = 0xff;           // PCを出力に設定
	//  PORTC.PCR.BYTE   = 0x03;        // PC0,1をプルアップ指定
	
    PORTD.DDR.BYTE = 0xff;           // 3:IR4	2:IR3	1:IR2	0:IR1
	
    PORTE.DDR.BYTE = 0x70; 	     	 // 7:SPI_SDO	6:SPI_SDI	5:SPI_SCL	4:SPI_CS    (6,7のIOはジャイロ目線)
    PORTE.PCR.BYTE = 0x70;           // PE7をプルアップ指定
}



/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */
/* 関 数 概 要：A/Dの初期化                                                                         */
/* 関 数 詳 細：                                                                                    */
/* 引       数：なし										    */
/* 戻  り   値：なし										    */
/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */ 
void AD_init(void)
{
   //A/Dの初期化
   SYSTEM.MSTPCRA.BIT.MSTPA17 = 0; //12bitでA/D変換する
   
   //MSTP(S12AD) = 0;//モジュールストップ状態を解除

   S12AD.ADCSR.BYTE = 0x0c;//動作モード、変換開始要件、クロックの設定
   S12AD.ADANS.WORD = 0x001f;//スキャン変換端子の設定 AN0-7の全て使用する場合は0xff
   S12AD.ADSTRGR.BYTE = 0x0000;//A/D 変換開始要件の設定
	
}



/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */
/* 関 数 概 要：CMT0(コンペアマッチタイマー)の初期化                                                */
/* 関 数 詳 細：1ms割り込み周期                                                                     */
/* 引       数：なし										    */
/* 戻  り   値：なし										    */
/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */ 
void CMT_init(void)
{
    MSTP(CMT0) = 0;                // SYSTEM.MSTPCRA.BIT.MSTPA15 = 0; // CMT0 タイマースタンバイ解除 （0で解除）
    CMT0.CMCR.WORD = 0x0040;       // 4:割り込み許可　0:PCLK/8 1:PCLK/32 2:PCLK/128 3:PCLK/512
    CMT0.CMCOR = 3000-1;           // 1ms Count： PCLK = 24MHz/8=3MHz 3M/1mS=3000 (得たいカウント数-1) 
    IPR(CMT0,CMI0) = 3;
    IEN(CMT0,CMI0) = 1;
	
    set_psw(0x00010000);
	
    CMT.CMSTR0.BIT.STR0 = 1;   	   // CMT0タイマースタート
}

/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */
/* 関 数 概 要：CMT2(コンペアマッチタイマー)の初期化                                                */
/* 関 数 詳 細：0.25ms割り込み周期                                                                     */
/* 引       数：なし										    */
/* 戻  り   値：なし										    */
/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */ 
void CMT2_init(void)
{
    MSTP(CMT2) = 0;                // SYSTEM.MSTPCRA.BIT.MSTPA15 = 0; // CMT0 タイマースタンバイ解除 （0で解除）
    CMT2.CMCR.WORD = 0x0040;       // 4:割り込み許可　0:PCLK/8 1:PCLK/32 2:PCLK/128 3:PCLK/512
    CMT2.CMCOR = 750-1;           // 0.25ms Count： PCLK = 24MHz/8=3MHz 3M/0.25mS=750 (得たいカウント数-1) 
    IPR(CMT2,CMI2) = 3;
    IEN(CMT2,CMI2) = 1;
	
    set_psw(0x00010000);
	
    CMT.CMSTR1.BIT.STR2 = 1;   	   // CMT2タイマースタート
}
	
/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */
/* 関 数 概 要：左モーター(MTU0,1）,右モーター(MTU3,4） タイマーの初期化	   			            */
/* 関 数 詳 細：		                                                    		                */
/* 引       数：なし										    									*/
/* 戻  り   値：なし										    									*/
/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */ 
void MTU0_init(void){
	volatile int C_cycle;
	
	//C_cycle = 24e6 / 1000;
	C_cycle = 1500;
	
	SYSTEM.MSTPCRA.BIT.MSTPA9 = 0;  // MTUユニット０　モジュールストップ解除
	MTUA.TSTR.BYTE &= 0x04 ;  // カウンタの停止  00xx x100　
	
	MTU0.TCR.BIT.CCLR = 0x1; //
	MTU1.TCR.BIT.CCLR = 0x1; //
	MTU3.TCR.BIT.CCLR = 0x1; //
	MTU4.TCR.BIT.CCLR = 0x1; //
	
	MTU0.TMDR.BIT.MD = 0x2;  // PWM MODE1
	MTU1.TMDR.BIT.MD = 0x2;  // PWM MODE1
	MTU3.TMDR.BIT.MD = 0x2;  // PWM MODE1
	MTU4.TMDR.BIT.MD = 0x2;  // PWM MODE1

	MTU0.TIORH.BIT.IOA = 0x6; //
	MTU0.TIORH.BIT.IOB = 0x5; //
	MTU1.TIOR.BIT.IOA = 0x6; //
	MTU1.TIOR.BIT.IOB = 0x5; //
	MTU3.TIORH.BIT.IOA = 0x6; //
	MTU3.TIORH.BIT.IOB = 0x5; //
	MTU4.TIORH.BIT.IOA = 0x6; //
	MTU4.TIORH.BIT.IOB = 0x5; //
		
	MTU0.TGRA = C_cycle;
	MTU1.TGRA = C_cycle;
	MTU3.TGRA = C_cycle;
	MTU4.TGRA = C_cycle;
	
	MTU0.TGRB = 0;
	MTU1.TGRB = 0;
	MTU3.TGRB = 0;
	MTU4.TGRB = 0;
	
	IOPORT.PFCMTU.BIT.MTUS3=0;
	IOPORT.PFCMTU.BIT.MTUS4=0;
	IOPORT.PFCMTU.BIT.MTUS5=1;
	MTUA.TOER.BIT.OE4A = 1;  //
	
	MTUA.TSTR.BYTE = 0xc3;	// 11xx x011　カウンタの開始
	
  	motor(0,0);

}

/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */
/* 関 数 概 要：MTU7(左エンコーダ）,MTU8(右エンコーダ） タイマーの初期化   		            */
/* 関 数 詳 細：		                                                                    */
/* 引       数：なし										    */
/* 戻  り   値：なし										    */
/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */ 
void MTU1_init(void){
	
	SYSTEM.MSTPCRA.BIT.MSTPA8 = 0;  // MTUユニット1　モジュールストップ解除
	MTUB.TSTR.BYTE = 0x00;  // カウンタの停止
	
	IOPORT.PFDMTU.BIT.TCLKS=1;//ピン設定(MTCLKE-B, MTCLKF-B,MTCLKG-B, MTCLKH-Bを選択)

	MTU7.TMDR.BYTE=4;//MTU7を位相計数モードに
	MTU8.TMDR.BYTE=4;//MTU8を位相計数モードに
	
	PORTB.ICR.BIT.B2 = 1;
	PORTB.ICR.BIT.B3 = 1;
	PORTB.ICR.BIT.B4 = 1;
	PORTB.ICR.BIT.B5 = 1;

	MTUB.TSTR.BYTE = 0x06;	// 0000 0110 カウンタの開始
	
	Encoder_reset();
}
		


/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */
/* 関 数 概 要：LED出力														  			            */
/* 関 数 詳 細：												                                   */
/* 引       数： 	3:LED4 2:LED3 1:LED2 0:LED1													    */
/* 戻  り   値：なし										    									*/
/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */ 
void led(char n){
	
	if((n & 0x01) > 0)PORTA.DR.BIT.B0 = 1;
	else PORTA.DR.BIT.B0 = 0;
	
	if((n & 0x02) > 0)PORTA.DR.BIT.B1 = 1;
	else PORTA.DR.BIT.B1 = 0;
	
	if((n & 0x04) > 0)PORTA.DR.BIT.B2 = 1;
	else PORTA.DR.BIT.B2 = 0;
	
	if((n & 0x08) > 0)PORTA.DR.BIT.B3 = 1;
	else PORTA.DR.BIT.B3 = 0;
}

void led_up(){
	for(int i = 1;i <= 8; i*= 2){
		led(i);
		delay(80);
	}
	led(0);
}

void led_down(){
	for(int i = 8;i > 0; i/= 2){
		led(i);
		delay(80);
	}
	led(0);
}


/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */
/* 関 数 概 要：スイッチ入力												  			            */
/* 関 数 詳 細：												                                   */
/* 引       数： なし																			    */
/* 戻  り   値： on:1 off:0										    									*/
/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */ 
int get_sw(){
	return (PORTA.PORT.BIT.B4 == 1)? 0: 1 ;
}


/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */
/* 関 数 概 要：迷路情報の保存                                                                         */
/* 関 数 詳 細：                                                                                    */
/* 引       数：なし										    */
/* 戻  り   値：なし										    */
/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */ 
void maze_save(){
	uint8_t buff[H*W];
	
	for(int i = 0; i < H;i++){
		for(int j = 0; j < W; j++){
			buff[i*W + j] = maze_w[i][j];
		}
	}
	
	DataFlash_write(0,buff,sizeof(buff));
}

/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */
/* 関 数 概 要：迷路情報の復元                                                                        */
/* 関 数 詳 細：                                                                                    */
/* 引       数：なし										    */
/* 戻  り   値：なし										    */
/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */ 
void maze_load(){
	uint8_t buff[H*W];
	
	DataFlash_read(0,buff,sizeof(buff));
	
	for(int i = 0; i < H;i++){
		for(int j = 0; j < W; j++){
			 maze_w[i][j] = buff[i*W + j];
		}
	}
}


/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */
/* 関 数 概 要：ログの初期化                                                                       */
/* 関 数 詳 細：                                                                                    */
/* 引       数：なし										    */
/* 戻  り   値：なし										    */
/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */ 
void log_reset(){
	log_cnt = 0;
	log_save_cnt = 0;
	log_block_num = 1;
	for(int i = 0; i < LOG_MAX;i++)log[i] = 0;
	
	for(int i = 1; i < 16;i++){//メモ:0は迷路情報なので削除しない
		//DataFlash_write(i,log,sizeof(log));
		R_FlashErase(i + 38);// BLOCK_DB0    38 
	}
}
/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */
/* 関 数 概 要：ログの保存                                                                       */
/* 関 数 詳 細：                                                                                    */
/* 引       数：なし										    */
/* 戻  り   値：なし										    */
/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */ 
void log_save(){
	
	DataFlash_write2(log_block_num,log_save_cnt,log_buff,sizeof(log_buff));
}


/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */
/* 関 数 概 要：ログの読み込み                                                                        */
/* 関 数 詳 細：                                                                                    */
/* 引       数：なし										    */
/* 戻  り   値：なし										    */
/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */ 
int log_minus(uint8_t data){//符号がマイナスの値を修正する
	
	int ret = data;
	
	if((data & 0x80) != 0){//マイナスだった
			data ^= 0xff;
			data += 1;
			ret = -((int)data);
			
	}
	return ret;
}

void log_load(){
	int i = 0;
	int block_num = 1;
	
	printf2("X\tY\tS1\tS2\tS3\tS4\tPWM_L\tPWM_R\n\n");
	
	while(block_num < 16){
		DataFlash_read(block_num,log,sizeof(log));
		i = 0;
		
		while(1){
			if( i >= LOG_MAX)break;
			if(log[i+2] == 0 && log[i+3] == 0 && log[i+3] == 0 && log[i+5] == 0)break;//センサー値がすべて０なら終了
			if(log[i+0] > 16 || log[i+1] > 16)break;//X,Y座標が範囲外なら終了
			
			printf2("%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t\n",log[i],log[i+1]
														,log[i+2],log[i+3],log[i+4],log[i+5]
														,log_minus(log[i+6]),log_minus(log[i+7]) );	
			i += 8;
		}
		block_num++;
	}
}

/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */
/* 関 数 概 要：迷路情報の更新											  			            */
/* 関 数 詳 細：												                                   */
/* 引       数： 現在地のXY座標、方角 ,更新する向き（1:前 2:横 それ以外:両方)					    */
/* 戻  り   値： なし										    									*/
/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */ 
void maze_update(char x,char y,char angle, char type){
  if((maze_w[y][x]&0xf0) == 0xf0)return;

  for(short i = -1; i < 3;i++){
	if(type == 1){
		if(i != 0)continue;//前以外は更新しない
	}else if(type == 2){
		if(i == 0)continue;//横以外は更新しない
	}
	
    short ii = (4 + angle+i)%4;
    int nx = x+dx[ii], ny = y+dy[ii];

 //   if((maze_w[y][x] & (1 << (4+ii))) == 0 ){//未確定の場合
      maze_w[y][x] |= 1 << (4+ii);
      
      switch(i){
        case -1://L
			//if((maze_w[y][x] & (1 << (4+ii))) == 0 ){//未確定の場合
          		if(get_IR(IR_L) > 15){
					maze_w[y][x] |= 1 << ii;
		  		}else{
					maze_w[y][x] &= ~(1 << ii);  
		  		}
			//}
         	break;
        case 0://S
			//if((maze_w[y][x] & (1 << (4+ii))) == 0 ){//未確定の場合
          		if(get_IR(IR_FL) > 13 || get_IR(IR_FR) > 13){
					maze_w[y][x] |= 1 << ii; //左前だけ値が高い?
		  		}else{
					maze_w[y][x] &= ~(1 << ii);
		  		}
			// }
         	 break;
        case 1://R
			//if((maze_w[y][x] & (1 << (4+ii))) == 0 ){//未確定の場合
          		if(get_IR(IR_R) > 15){
		  			maze_w[y][x] |= 1 << ii; 
		  		}else{
					maze_w[y][x] &= ~(1 << ii);
		  		}
			//}
          	break;
      }
      if((0 <= nx && nx < W) && (0 <= ny && ny < H)){
        maze_w[ny][nx] |= 1 << (4+(ii+2)%4);
		maze_w[ny][nx] &= ~(1 << ((ii+2)%4));
		
       	if((maze_w[y][x] & (1 << ii)) != 0){	
			maze_w[ny][nx] |= 1 << ((ii+2)%4);
		}
      }
 //  }
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////
void L_rotate(long long a){
  Tmotor(-a);
  my_angle = (4+my_angle-1)%4;
}

void R_rotate(long long a){
  Tmotor(a);
  my_angle = (4+my_angle+1)%4;
}

void L_curve(long long a,char flag){
  ETmotor(-a,rslsl90,flag);
 
  my_angle = (4+my_angle-1)%4;
}

void R_curve(long long a ,char flag ){
  ETmotor(a,rslsr90,flag);
  
  my_angle = (4+my_angle+1)%4;
}

void L_rotate_naname(long long a){
  Tmotor_naname(-a);
  my_angle = (4+my_angle-1)%4;
}

void R_rotate_naname(long long a){
  Tmotor_naname(a);
  my_angle = (4+my_angle+1)%4;
}

void S_run(long long path,int powor, char non_stop,char kabe){
  //GyroSum_reset();
	ESmotor(path,powor,non_stop,kabe);

  	int cnt2 = 0;

  	if(!non_stop && kabe == 1){
   		// GyroSum_reset();
    	if(15 < get_IR(IR_FL) && 15 < get_IR(IR_FR) ){
	 		while(1){
      			if(get_IR(IR_FR) > 58){
        			Smotor(-5,false);

        			cnt2 = 0;
      			}else if(get_IR(IR_FR) < 55){
       	 			Smotor(+5,false);
       				
        			cnt2 = 0;
      			}else {
        			motor(0,0);
        			cnt2++;
      			}
      			if(cnt2 > 2000)break;
			}
    	}
  	}
	
  	motor(0,0);
  //GyroSum_reset();
}

void S_run_kabe(int powor, char flag, int LR){//壁切れまで走行
  int Lflag = 0,Rflag = 0;
  long long enc_base = (get_encoder_total_L() + get_encoder_total_R())/2;
   int cnt2 = 0;
	
  while(1){
	if(LR == 3 || LR == 1){
    	if(Lflag == 0){
      		if((get_IR(IR_L) > 20) || (get_IR(IR_L) > 13 && get_IR(IR_R) > 70)){//反対の壁が近いときは柱の値が小さくなる
				Lflag = 1;
				led(1);
			}
    	}else if(Lflag == 1){
      		if(get_IR(IR_L) < 11){
				led(0);
				break;
			}
    	}
	}

	if(LR == 3 || LR == 2){
    	if(Rflag == 0){
      		if((get_IR(IR_R) > 20) || (get_IR(IR_L) > 70 && get_IR(IR_R) > 13)){//反対の壁が近いときは柱の値が小さくなる
				Rflag = 1;
				led(8);
			}
    	}else if(Rflag == 1){
      		if(get_IR(IR_R) < 11){
				led(0);
				break;
			}
    	}
	}
    
    Smotor(powor,flag);
	
	if(get_IR(IR_FL) > 70 && get_IR(IR_FR) > 70){//前壁が近すぎる場合は
		while(1){//前壁補正
      		if(get_IR(IR_FR) > 58){
        		Smotor(-5,false);
        		cnt2 = 0;
     		}else if(get_IR(IR_FR) < 55){
	      		Smotor(+5,false);
	     
	       		cnt2 = 0;
	    	}else {
	       		motor(0,0);
	       		cnt2++;
	      	}
	      	if(cnt2 > 2000){
				led(0);
				return;
			}
		}
	}
	
	if(abs((get_encoder_total_L() + get_encoder_total_R())/2 -  enc_base) > (s1+h1 )){
		led(9);
		break; //壁切れが来なかったらブレーク
	}
  }
  
  ESmotor(170,powor,true,false);
  led(0);
}

void S_run_kabe2(int powor, char flag, int LR){//壁切れまで走行 直線からの４５ターン
  int Lflag = 0,Rflag = 0;
  long long enc_base = (get_encoder_total_L() + get_encoder_total_R())/2;
	
  while(1){
	if(LR == 3 || LR == 1){
    	if(Lflag == 0){
      		if((get_IR(IR_L) > 20) || (get_IR(IR_L) > 13 && get_IR(IR_R) > 70)){//反対の壁が近いときは柱の値が小さくなる
				Lflag = 1;
				led(1);
			}
    	}else if(Lflag == 1){
      		if(get_IR(IR_L) < 11){
				led(0);
				break;
			}
    	}
	}

	if(LR == 3 || LR == 2){
    	if(Rflag == 0){
      		if((get_IR(IR_R) > 20) || (get_IR(IR_L) > 70 && get_IR(IR_R) > 13)){//反対の壁が近いときは柱の値が小さくなる
				Rflag = 1;
				led(8);
			}
    	}else if(Rflag == 1){
      		if(get_IR(IR_R) < 11){
				led(0);
				break;
			}
    	}
	}
    
    Smotor(powor,flag);
	
	if(abs((get_encoder_total_L() + get_encoder_total_R())/2 -  enc_base) > (s1+h1 )){
		led(9);
		break; //壁切れが来なかったらブレーク
	}
  }
  
  ESmotor(5,powor,true,false);//　直線からの４５ターン 勢いがあるので不要
  led(0);
}

void S_run_kabe_naname(int powor, char flag, int LR){//壁切れまで走行
  int Lflag = 0,Rflag = 0;
  long long enc_base = get_encoder_total_L();
  
  led(6);
  
  while(1){
	if(LR == 3 || LR == 1){
		if(Lflag == 0){
      		if(get_IR(IR_L) > 30){
				led(1);
				Lflag = 1;
			}
    	}else if(Lflag == 1){
      		if(get_IR(IR_L) < 15){
				led(0);
				break;
			}
    	}
	}

	if(LR == 3 || LR == 2){
		if(Rflag == 0){
      		if(get_IR(IR_R) > 30){
				led(8);
				Rflag = 1;
			}
    	}else if(Rflag == 1){
      		if(get_IR(IR_R) < 15){
				led(0);
				break;
			}
    	}
	}
    
    Smotor(powor,flag);
	
	
	
	if(abs(get_encoder_total_L() -  enc_base) > (s45+s45)  ){
		led(9);
		
		break; //壁切れが来なかったらブレーク
	}
  }
 
  
  ESmotor(180,powor,true,false);
  led(0);
}

void S_run_maze_search(int path,int powor){
	Encoder_reset();
	
	int M_pwm_min = 5;
	int M_pwm = M_pwm_min;
	long long enc_base = (get_encoder_total_L() + get_encoder_total_R())/2;
	long long enc_now = 0;
	
	int path_cnt = 0;
	int maza_update_flag = 0;
	
	int cnt2 = 0;
	
	int ir_L_now = 0,ir_R_now = 0;
	int ir_L_flag = 0,ir_R_flag = 0;
	int path_cnt_save_L = -1;//同じマスで壁切れ処理を２回以上しないように覚えておく変数
	int path_cnt_save_R = -1;//同じマスで壁切れ処理を２回以上しないように覚えておく変数
//	int hosei_kyori_L = -1,hosei_kyori_R = -1;//壁切れ時の補正距離　左異なるタイミングで壁切れした際に利用する
	long long enc_kabe_L,enc_kabe_R;
	
//	int kame_hosei = 230;//170
	
	GyroSum_reset();
	
	while(1){
		
		if(enc_now >= (long long)path * s1){//目標距離に到達
		
			//マスの中心まで移動(戻る）
			while(enc_now - ((long long)s1 * path_cnt ) > s1){
				Smotor(-10,true);
				enc_now = (get_encoder_total_L() + get_encoder_total_R())/2 - enc_base;
			}
			
			if(get_IR(IR_FL) > 15 && get_IR(IR_FR) > 15){//前壁があった場合は
				while(1){//前壁補正
      				if(get_IR(IR_FR) > 58){
        				Smotor(-5,false);

	        			cnt2 = 0;
	      			}else if(get_IR(IR_FR) < 55){
	       	 			Smotor(+5,false);
	       
	        			cnt2 = 0;
	      			}else {
	        			motor(0,0);
	        			cnt2++;
	      			}
	      			if(cnt2 > 2000)break;
				}
			}
			
			//現在地の更新
			my_x += dx[my_angle];
			my_y += dy[my_angle];
			
			maze_update(my_x,my_y,my_angle,3);//迷路情報の更新
			
			break;
		}
		
		if( path_cnt < path-1 && get_IR(IR_FL) > 25 && get_IR(IR_FR) > 25){//目標まで１マス以上残ってる　＆＆　前壁が出現
			//マスの中心まで移動
			while(enc_now - ((long long)s1 * path_cnt ) < s1 && get_IR(IR_FR) < 50){
				Smotor(+10,true);
				enc_now = (get_encoder_total_L() + get_encoder_total_R())/2- enc_base;
			}
		
			while(1){//前壁補正
      			if(get_IR(IR_FR) > 58){
        			Smotor(-5,false);

        			cnt2 = 0;
      			}else if(get_IR(IR_FR) < 55){
       	 			Smotor(+5,false);
       
        			cnt2 = 0;
      			}else {
        			motor(0,0);
        			cnt2++;
      			}
      			if(cnt2 > 2000)break;
			}
			
			if(maza_update_flag != 2){//なぜか壁の更新ができていなければ
				if(maza_update_flag == 0)maze_update(my_x + dx[my_angle],my_y + dy[my_angle],my_angle,3);//迷路情報の更新
				else if(maza_update_flag == 1)maze_update(my_x + dx[my_angle],my_y + dy[my_angle],my_angle,1);//迷路情報の更新
			}
			//現在地の更新
			my_x += dx[my_angle];
			my_y += dy[my_angle];
			
			maze_update(my_x,my_y,my_angle,3);//迷路情報の更新
			maza_update_flag = 0;
			
			break;
		}
	
		if( path_cnt < path-1){//目標まで１マス以上残ってる メモ：最後の１マスは前壁補正後に迷路情報を更新するため
			if(enc_now - ((long long)s1 * path_cnt ) > s1){//１マス進んだ
			
				if(maza_update_flag != 2){//なぜか壁の更新ができていなければ
					if(maza_update_flag == 0)maze_update(my_x + dx[my_angle],my_y + dy[my_angle],my_angle,3);//迷路情報の更新
					else if(maza_update_flag == 1)maze_update(my_x + dx[my_angle],my_y + dy[my_angle],my_angle,1);//迷路情報の更新
				}
				//現在地の更新
				my_x += dx[my_angle];
				my_y += dy[my_angle];
			
				path_cnt++;
				maza_update_flag = 0;
				ir_L_flag = 0;
				ir_R_flag = 0;
			}
		}	
		
		if(maza_update_flag == 0){//まだ横壁の更新をしていなければ
			if(enc_now - ((long long)s1 * path_cnt ) > s1 - 250){//マスの中心ではなく少し手前で壁をチェックする メモ：横壁センサーが少し斜め前を向いているため
				maze_update(my_x + dx[my_angle],my_y + dy[my_angle],my_angle,2);//迷路情報の更新
				maza_update_flag = 1;
			}
		}else if(maza_update_flag == 1){//まだ前壁の更新をしていなければ
			if(enc_now - ((long long)s1 * path_cnt ) > s1 - 50){//マスの中心ではなく少し手前で壁をチェックする メモ：横壁センサーが少し斜め前を向いているため
				maze_update(my_x + dx[my_angle],my_y + dy[my_angle],my_angle,1);//迷路情報の更新
				maza_update_flag = 2;
			}
		}
		
		
		if(enc_now < (long long)path * s1 /4){// 進んだ距離 < 目標距離 * 1/4　＝ 加速区間
			M_pwm = M_pwm_min + (enc_now / 6);	
			
		}else if(enc_now > (long long)path * s1 * 3/4){// 進んだ距離 < 目標距離 * 3/4 = //減速区間
			M_pwm = M_pwm_min + ( ((long long)path * s1 - enc_now) / 4);	
			
		}else{
			M_pwm = powor;
		}
			
		if(M_pwm > powor)M_pwm = powor;
		if(M_pwm < M_pwm_min)M_pwm = M_pwm_min;
		
		Smotor(M_pwm,true);
		
		
		//壁切れの距離補正
		ir_L_now = get_IR(IR_L);
		ir_R_now = get_IR(IR_R);
		if(path_cnt_save_L !=  path_cnt){//現在のマスで壁切れ処理を実行していなければ
		
			if(ir_L_flag == 0 && ir_L_now > 20 && ir_R_now < 70){
				ir_L_flag = 1;
				
			}else if(ir_L_flag == 1 && ir_L_now < 11 && ir_R_now < 70){
				if((enc_now % s1) < s1 * 2 / 3){//マスの半分より手前で壁切れした場合
				
					if(path_cnt == path_cnt_save_R){//左より先に右が壁切れ補正していた場合
/*						enc_base -= hosei_kyori_R;//右での補正を無かったことにする
						enc_now = (get_encoder_total_L() + get_encoder_total_R())/2 - enc_base;
						
						hosei_kyori_L = (enc_now % s1) - kame_hosei;
						
						hosei_kyori_L = (hosei_kyori_L + hosei_kyori_R) / 2;//左右の平均値を使用する
*/						
						//壁切れタイミングの違いで角度補正
						enc_kabe_L = (get_encoder_total_L() + get_encoder_total_R())/2;
						if(abs( (enc_kabe_L - enc_kabe_R) ) < 150){
							GyroSum_add( (enc_kabe_L - enc_kabe_R) * 10);
						}
					}else{
//						hosei_kyori_L = (enc_now % s1) - kame_hosei;
						enc_kabe_L = (get_encoder_total_L() + get_encoder_total_R())/2;
					}
					
//					enc_base += hosei_kyori_L;
					
				}
				ir_L_flag = 0;
				path_cnt_save_L = path_cnt;
			}
		}
		
		if(path_cnt_save_R !=  path_cnt){//現在のマスで壁切れ処理を実行していなければ
			
			if(ir_R_flag == 0 && ir_R_now > 20 && ir_L_now < 70){
				ir_R_flag = 1;
				
			}else if(ir_R_flag == 1 && ir_R_now < 11 && ir_L_now < 70){
				if((enc_now % s1) < s1 * 2 / 3){//マスの半分より手前で壁切れした場合
				
					if(path_cnt == path_cnt_save_L){//右より先に左が壁切れ補正していた場合
/*						enc_base -= hosei_kyori_L;//左での補正を無かったことにする
						enc_now = (get_encoder_total_L() + get_encoder_total_R())/2 - enc_base;
						hosei_kyori_R = (enc_now % s1) - kame_hosei;
						
						hosei_kyori_R = (hosei_kyori_L + hosei_kyori_R) / 2;//左右の平均値を使用する
	*/					
						//壁切れタイミングの違いで角度補正
						enc_kabe_R = (get_encoder_total_L() + get_encoder_total_R())/2;
						if(abs( (enc_kabe_L - enc_kabe_R) ) < 150){
							GyroSum_add( (enc_kabe_L - enc_kabe_R) * 10);
						}
					}else{
//						hosei_kyori_R = (enc_now % s1) - kame_hosei;
						enc_kabe_R = (get_encoder_total_L() + get_encoder_total_R())/2;
					}
					
					//enc_base += hosei_kyori_R;
					
				}
				ir_R_flag = 0;
				path_cnt_save_R = path_cnt;
			}
		}
		
		enc_now = (get_encoder_total_L() + get_encoder_total_R())/2 - enc_base;
	}
	
	motor(0,0);
	GyroSum_reset();
	Encoder_reset();
}


////////////////////////////////////////////////////////////////////////////////////////////////////////

/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */
/* 関 数 概 要：最短経路探索											  			            */
/* 関 数 詳 細：												                                   */
/* 引       数： 目的地のXY座標															    */
/* 戻  り   値： なし										    									*/
/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */ 
void shortest_path_search(short target_x,short target_y){
  queue_reset();
  for(int i = 0; i < H;i++){
    for(int j = 0;j < W; j++){
      for(int k = 0; k < 4; k++){
        maze_d[i][j][k] = maze_d_max;
      }
    }
  }
  for(int k = 0; k < 4; k++){
    if((maze_w[target_y][target_x] & (1<<k)) == 0 )maze_d[target_y][target_x][k] = 0;
  }
  enqueue(target_x*100 + target_y);
  
  while(!queue_empty()){
    short x = dequeue(),y;
    y = x%100;
    x /=100;

    for(char i =0;i<4;i++){
      char update_flag = 0;
      short nx = x+dx[i],ny = y+dy[i];
      if((0 <= nx && nx < W) && (0 <= ny && ny < H) && ((maze_w[y][x] & (1<<i)) == 0 ) ){//未確定の場合は壁無しとして考える

        short num = maze_d[y][x][i];
        for(int k = 0; k < 4; k++){
           
          if(i == k){//S
            if(maze_d[ny][nx][k] > num + 1){
              update_flag = true;
              maze_d[ny][nx][k] = num + 1;
            }
          }else if((i+2+4)%4 == k){//B
            if(maze_d[ny][nx][k] > num+1 + r_cost*2){
              update_flag = true;
              maze_d[ny][nx][k] = num+1 + r_cost*2;
            }
          }else{// L or R
            if(maze_d[ny][nx][k] > num+1 + r_cost){
              update_flag = true;
              maze_d[ny][nx][k] = num+1 + r_cost;
            }
          }
        }
        if(update_flag)enqueue(nx*100 + ny);
      }
    }
  }
}

/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */
/* 関 数 概 要：最短経路作成											  			            */
/* 関 数 詳 細：												                                   */
/* 引       数： 目的地のXY座標															    */
/* 戻  り   値： なし										    									*/
/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */ 
void make_shortest_path_list(short target_x,short target_y){
	queue_reset();
  	short s_path = 0;
 	short x = my_x,y = my_y,angle = my_angle;
  	short unknown_flag = 1; // 0:確定壁　1:未確定壁
	
	int mas_cnt = 0;//直線に進むマスの数
	
  	while(x != target_x || y != target_y){
    	short num = maze_d[y][x][(angle+2)%4];
    	short n_num = 0;
    	char s_flag = 0;
    	short nx = x+dx[angle],ny = y+dy[angle];
    	if((0 <= nx && nx < W) && (0 <= ny && ny < H) && ((maze_w[y][x] & (1<<angle)) == 0 ) ){//目の前が迷路内　&& 壁がない 
      		short next = maze_d[ny][nx][(angle+2)%4];
      		if(num == next+1){//目の前のマスがゴールに近い
        		n_num = (angle+2)%4;
        		num = next;
        		s_flag = true;
      		}
    	}

    	if(s_flag == false){//回転する必要がる
      		for(int i = 0;i < 4;i++){
        		if(i == (angle+2)%4){
        		}else{
          			short next = maze_d[y][x][i];
          			if(num > next){
            			n_num = i;
            			num = next;
          			}
        		}
      		}
    	}
    
    	n_num = (n_num+2)%4;// 0 ~ 4　進みたい方角
    	short ni = ((4 + n_num - ((4+angle-1)%4))%4) -1;// -1 ~ 2　マシンから見た方角

    	if((maze_w[y][x] & (1 << (4+n_num))) == 0){//未確定の壁
			//break;
			
			if( unknown_flag == 1){//初めから未確定の直線の場合は有効　一度でも確定のマスを進むと無効
				if(ni != 0)break;//直進方向でなければ打ち切り//////////////////////////////////////////////////////////////////////////////////////////////////////////////
				ni = 10;//未確定の直線
			}else{
				break;//未確定の壁を確認する必要があるので打ち切り
			}
			
		}else{//確定
			if(ni == 0){//直線
				if(mas_cnt == 0){//はじめての直進の１マス目
					ni = 10;//直進方向なら未確定の直線の可能性あり メモ：目の前の壁は確定してるから
				}else{
					
				}
			}
		}
	
    	switch(ni){
      		case -1://L
        		if(s_path > 0){
          			enqueue(10 * unknown_flag );
					//enqueue(0);
          			enqueue(s_path);
          			s_path = 0;
					
					if(unknown_flag == 1)return;//未確定の直線のあとはルートを作成してはいけない
        		}

        		enqueue(-1);
        		enqueue(1);

        		//s_path += 1;
        
        		angle = (4+angle-1)%4;
		       // x += dx[n_num];
		       // y += dy[n_num];
				
				//unknown_flag = 0;
		        break;
		    case 0://S
   
				if(unknown_flag == 1 && s_path > 0 && mas_cnt > 1){
          			enqueue(10);
          			enqueue(s_path);
          			s_path = 0;
					
					return;//未確定の直線のあとはルートを作成してはいけない
        		}
				
		        s_path +=1;
		        x += dx[n_num];
		        y += dy[n_num];
				
				unknown_flag = 0;
				mas_cnt++;
		        break;
				
			case 10://S 未確定の直線
				
		        s_path +=1;
		        x += dx[n_num];
		        y += dy[n_num];
				
				mas_cnt++;
		        break;
				
      		case 1://R
		        if(s_path > 0){
			    	enqueue(10 * unknown_flag);
					//enqueue(0);
			        enqueue(s_path);
			        s_path = 0;
					
					if(unknown_flag == 1)return;//未確定の直線のあとはルートを作成してはいけない
		        }
        
		        enqueue(1);
		        enqueue(1);

		       // s_path += 1;
		         
		        angle = (4+angle+1)%4;
		       // x += dx[n_num];
		       // y += dy[n_num];
				
				//unknown_flag = 0;
		        break;
      		case 2://B
        		if(s_path > 0){
          			enqueue(10 * unknown_flag);
					//enqueue(0);
          			enqueue(s_path);
          			s_path = 0;
					
					if(unknown_flag == 1)return;//未確定の直線のあとはルートを作成してはいけない
        		}
        
        		enqueue(2);
        		enqueue(1);

				//s_path += 1;
				 
        		angle = (4+angle+2)%4;
				//x += dx[n_num];
		        //y += dy[n_num];
				
				//unknown_flag = 0;
        		break;
    	}
  	}
 
  	if(s_path > 0){
    	enqueue(10 * unknown_flag);
		//enqueue(0);
    	enqueue(s_path);
    	s_path = 0;
  	}
}

/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */
/* 関 数 概 要：最短経路を走行									  			            */
/* 関 数 詳 細：												                                   */
/* 引       数： なし														    */
/* 戻  り   値： なし										    									*/
/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */ 
void run_shortest_path(){
  GyroSum_reset();
  Encoder_reset();
  
  short comand ,path_num;
  int time = 50;

  
  while(!queue_empty()){
    comand = dequeue();path_num = dequeue();
	
    switch(comand){
      case -1://L
	  	delay(time);
		
        L_rotate(l90);
		
		delay(time);
        break;
      case 0://S
	  	if(queue_empty()){
			if(path_num == 1){
	  			S_run(s1,20,false,true);
			}else{
				S_run(s1 * (long long)path_num,30,false,true);
			}
			
		}else{
        	if(path_num == 1){
	  			//S_run(s1,18,false,true);
				
				if(queue_next() < 0){//次　左
	          	  S_run_kabe(20,true,1);
				
			  	}else if(queue_next() > 0){//次　右
				  S_run_kabe(20,true,2);
			  	}else{
				  S_run_kabe(20,true,3);
			  	}
				
				S_run(h1,20,false,true);
				
			}else{
				S_run(s1 * ((long long)path_num - 1),30,true,true);
				
				if(queue_next() < 0){//次　左
	          	  S_run_kabe(20,true,1);
				
			  	}else if(queue_next() > 0){//次　右
				  S_run_kabe(20,true,2);
			  	}else{
				  S_run_kabe(20,true,3);
			  	}
				
				S_run(h1,20,false,true);
			}
		}
		
        switch(my_angle){
          case 0:
            my_y -= path_num;
            break;
          case 1:
            my_x += path_num;
            break;
          case 2:
            my_y += path_num;
            break;
          case 3:
            my_x -= path_num;
            break;
        }
		//delay(time);
        break;
		
	  case 10://S 未確定の直線
        if(path_num == 1){
	  		
			S_run_maze_search(path_num,18);
			
		}else{
			S_run_maze_search(path_num,18);
		}
		
        break;
      case 1://R
	  	delay(time);
		
        R_rotate(r90);
		
		delay(time);
        break;
      case 2://B
	  	delay(time);
		
        Tmotor(r180);
        my_angle = (4+my_angle+2)%4;
		
		delay(time);
        break;
    }
  }
}

/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */
/* 関 数 概 要：足立法で探索走行											  			            */
/* 関 数 詳 細：												                                   */
/* 引       数： 目的地のXY座標															    */
/* 戻  り   値： なし										    									*/
/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */ 
void maze_search_adachi(short target_x,short target_y){
	int cnt = 0;
	
	GyroSum_reset();
	Encoder_reset();

	led_down();
	
	while(1){
    	maze_update(my_x,my_y,my_angle,3);
    	/*if((target_x != Goal_x || target_y != Goal_y) && (target_x != Start_x || target_y != Start_y)){//スタート地点、ゴール地点以外が目標地点のとき
      		if((maze_w[target_y][target_x] & 0xf0) == 0xf0)break;    //目標地点の壁がすべて確定したら探索完了  
    	}*/
		
    	if(target_x == my_x && target_y == my_y){//ゴール
			led_up();
			
      		if(target_x == Start_x && target_y == Start_y){
				
				while(1){//スタートの奥まで進む
	      			if(get_IR(IR_FR) > 60){
	        			Smotor(-7,false);

	        			cnt = 0;
	      			}else if(get_IR(IR_FR) < 55){
	       	 			Smotor(+7,false);
	       				
	        			cnt = 0;
	      			}else {
	        			motor(0,0);
	        			cnt++;
	      			}
	      			if(cnt > 2000)break;
				}
        		Tmotor(r180);
        		my_angle = (4+my_angle+2)%4;
      		}
      		break;
    	}
		
    	shortest_path_search(target_x,target_y);
    	make_shortest_path_list(target_x,target_y);
		run_shortest_path();
  	}
}

/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */
/* 関 数 概 要：最短経路作成	未確定マスの手前で止まる					  			            */
/* 関 数 詳 細：												                                   */
/* 引       数： 目的地のXY座標															    */
/* 戻  り   値： なし										    									*/
/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */ 
void make_shortest_path_list_simple(short target_x,short target_y){
	queue_reset();
  	short s_path = 0;
 	short x = my_x,y = my_y,angle = my_angle;
	
	
  	while(x != target_x || y != target_y){
    	short num = maze_d[y][x][(angle+2)%4];
    	short n_num = 0;
    	char s_flag = 0;
    	short nx = x+dx[angle],ny = y+dy[angle];
    	if((0 <= nx && nx < W) && (0 <= ny && ny < H) && ((maze_w[y][x] & (1<<angle)) == 0 ) ){//目の前が迷路内　&& 壁がない 
      		short next = maze_d[ny][nx][(angle+2)%4];
      		if(num == next+1){//目の前のマスがゴールに近い
        		n_num = (angle+2)%4;
        		num = next;
        		s_flag = true;
      		}
    	}

    	if(s_flag == false){//回転する必要がる
      		for(int i = 0;i < 4;i++){
        		if(i == (angle+2)%4){
        		}else{
          			short next = maze_d[y][x][i];
          			if(num > next){
            			n_num = i;
            			num = next;
          			}
        		}
      		}
    	}
    
    	n_num = (n_num+2)%4;// 0 ~ 4　進みたい方角
    	short ni = ((4 + n_num - ((4+angle-1)%4))%4) -1;// -1 ~ 2　マシンから見た方角

    	if((maze_w[y][x] & (1 << (4+n_num))) == 0){//未確定の壁
			break;
		}
	
    	switch(ni){
      		case -1://L
        		if(s_path > 0){
          			enqueue(0);
          			enqueue(s_path);
          			s_path = 0;		
        		}

        		enqueue(-1);
        		enqueue(1);
        
        		angle = (4+angle-1)%4;
		     
		        break;
		    case 0://S
				
		        s_path +=1;
		        x += dx[n_num];
		        y += dy[n_num];
				
		        break;
				
      		case 1://R
		        if(s_path > 0){
			    	enqueue(0);
			        enqueue(s_path);
			        s_path = 0;
		        }
        
		        enqueue(1);
		        enqueue(1);
		         
		        angle = (4+angle+1)%4;
		     
		        break;
      		case 2://B
        		if(s_path > 0){
          			enqueue(0);
          			enqueue(s_path);
          			s_path = 0;
        		}
        
        		enqueue(2);
        		enqueue(1);
				 
        		angle = (4+angle+2)%4;
        		break;
    	}
  	}
 
  	if(s_path > 0){
    	enqueue(0);
    	enqueue(s_path);
    	s_path = 0;
  	}
}
/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */
/* 関 数 概 要：最短経路上の未確定マスを探す											            */
/* 関 数 詳 細：												                                   */
/* 引       数： 																				    */
/* 戻  り   値： なし										    									*/
/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */ 
void maze_search_unknown(short* target_x,short* target_y){
 
 	short x = Start_x,y = Start_y,angle = Start_angle;
	
	
  	while(x != Goal_x || y != Goal_y){
    	short num = maze_d[y][x][(angle+2)%4];
    	short n_num = 0;
    	char s_flag = 0;
    	short nx = x+dx[angle],ny = y+dy[angle];
    	if((0 <= nx && nx < W) && (0 <= ny && ny < H) && ((maze_w[y][x] & (1<<angle)) == 0 ) ){//目の前が迷路内　&& 壁がない 
      		short next = maze_d[ny][nx][(angle+2)%4];
      		if(num == next+1){//目の前のマスがゴールに近い
        		n_num = (angle+2)%4;
        		num = next;
        		s_flag = true;
      		}
    	}

    	if(s_flag == false){//回転する必要がる
      		for(int i = 0;i < 4;i++){
        		if(i == (angle+2)%4){
        		}else{
          			short next = maze_d[y][x][i];
          			if(num > next){
            			n_num = i;
            			num = next;
          			}
        		}
      		}
    	}
    
    	n_num = (n_num+2)%4;// 0 ~ 4　進みたい方角
    	short ni = ((4 + n_num - ((4+angle-1)%4))%4) -1;// -1 ~ 2　マシンから見た方角

    	if((maze_w[y][x] & (1 << (4+n_num))) == 0){//未確定の壁
			*target_x = x;
			*target_y = y;
			return;
			
		}
	
    	switch(ni){
      		case -1://L
        		angle = (4+angle-1)%4;
				
		        break;
		    case 0://S
   
		        x += dx[n_num];
		        y += dy[n_num];
	
		        break;
				
      		case 1://R
		        angle = (4+angle+1)%4;
			
		        break;
      		case 2://B 不要のはず
        	
        		angle = (4+angle+2)%4;
        		break;
    	}
  	}
	
	//未確定マスを通らずにゴールまで経路を確認できた。
	*target_x = Goal_x;
	*target_y = Goal_y;
}
/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */
/* 関 数 概 要：最短経路はすべて確定マスにする  										            */
/* 関 数 詳 細：												                                   */
/* 引       数： 																				    */
/* 戻  り   値： なし										    									*/
/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */ 
void maze_search_all(){
	GyroSum_reset();
	Encoder_reset();

	led_down();
	
	short target_x,target_y;
	
	time_limit = 60000;//60秒
	
	while(time_limit > 0){//制限時間の間走行可能
	
		maze_update(my_x,my_y,my_angle,3);
		
		shortest_path_search(Goal_x,Goal_y);
		maze_search_unknown(&target_x,&target_y);//最短経路上の未確定マスの座標を取得
		
    	shortest_path_search(target_x,target_y);
    	make_shortest_path_list_simple(target_x,target_y);
		run_shortest_path();
  	
		if(my_x == Goal_x && my_y == Goal_y){//最短経路上に未確定マスがなければ終了
			led_down();
			led_up();
			break;
		}
  	}
	
	if(time_limit <= 0){//　制限時間内に探索できなかった　ゴールまで向かう
		maze_search_adachi(Goal_x,Goal_y);
	}
}

/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */
/* 関 数 概 要：最短経路探索（最終版）											  			            */
/* 関 数 詳 細：												                                   */
/* 引       数： なし														    */
/* 戻  り   値： なし										    									*/
/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */ 
void shortest_path_search_fin(){
  queue_reset();
  for(int i = 0; i < H;i++){
    for(int j = 0;j < W; j++){
      for(int k = 0; k < 4; k++){
        maze_d[i][j][k] = maze_d_max;
      }
    }
  }
  for(int k = 0; k < 4; k++){
    if(((maze_w[Goal_y][Goal_x] & (1<<k)) == 0 ) && ((maze_w[Goal_y][Goal_x] & (1<<(4+k))) != 0 )){
      maze_d[Goal_y][Goal_x][k] = 0;
    }
  }
  enqueue(Goal_x*100 + Goal_y);
  
  while(!queue_empty()){
    short x = dequeue(),y;
    y = x%100;
    x /=100;

    for(char i =0;i<4;i++){
      char update_flag = 0;
      short nx = x+dx[i],ny = y+dy[i];
      if((0 <= nx && nx < W) && (0 <= ny && ny < H) && ((maze_w[y][x] & (1<<i)) == 0 )  && ((maze_w[y][x] & (1<<(4+i))) != 0 )  ){//未確定の壁は通過しない

        short num = maze_d[y][x][i];
        for(int k = 0; k < 4; k++){
           
          if(i == k){//S
            if(maze_d[ny][nx][k] > num + 1){
              update_flag = true;
              maze_d[ny][nx][k] = num + 1;
            }
          }else if((i+2+4)%4 == k){//B
            if(maze_d[ny][nx][k] > num+1 + r_cost*2){
              update_flag = true;
              maze_d[ny][nx][k] = num+1 + r_cost*2;
            }
          }else{// L or R
            if(maze_d[ny][nx][k] > num+1 + r_cost){
              update_flag = true;
              maze_d[ny][nx][k] = num+1 + r_cost;
            }
          }
        }
        if(update_flag)enqueue(nx*100 + ny);
      }
    }
  }
 
  //run_list
  queue_reset();
  short h_path = 0;
  my_x = Start_x;my_y = Start_y;my_angle = Start_angle;
 
  while(my_x != Goal_x || my_y != Goal_y){

    short num = maze_d[my_y][my_x][(my_angle+2)%4];
    short n_num = 0;
    char s_flag = 0;
    int nx = my_x+dx[my_angle],ny = my_y+dy[my_angle];
    if((0 <= nx && nx < W) && (0 <= ny && ny < H) && ((maze_w[my_y][my_x] & (1<<my_angle)) == 0 )  && ((maze_w[my_y][my_x] & (1<<(4+my_angle))) != 0 ) ){
      short next = maze_d[ny][nx][(my_angle+2)%4];
      if(num == next+1){
        n_num = (my_angle+2)%4;
        num = next;
        s_flag = true;
      }
    }

    if(s_flag == false){
      for(int i = 0;i < 4;i++){
        if(i == (my_angle+2)%4){
        }else{
          short next = maze_d[my_y][my_x][i];
          if(num > next){
            n_num = i;
            num = next;
          }
        }
      }
    }
    
    n_num = (n_num+2)%4;// 0 ~ 4
    short ni = ((4 + n_num - ((4+my_angle-1)%4))%4) -1;// -1 ~ 2

    switch(ni){
      case -1://L
        if(h_path > 0){
          if(queue_empty())h_path--;
          enqueue(0);
          enqueue(h_path);
          h_path = 0;
        }

        enqueue(-1);
        enqueue(1);
        my_angle = (4+my_angle-1)%4;
        
        my_x += dx[n_num];
        my_y += dy[n_num];
        break;
      case 0://S
   
        h_path +=2;
        my_x += dx[n_num];
        my_y += dy[n_num];
        break;
      case 1://R
        if(h_path > 0){
          if(queue_empty())h_path--;
          enqueue(0);
          enqueue(h_path);
          h_path = 0;
        }
        
        enqueue(1);
        enqueue(1);
     
        my_angle = (4+my_angle+1)%4;

        my_x += dx[n_num];
        my_y += dy[n_num];
        break;
    }
  }
 
  if(h_path > 0){
    if(queue_empty())h_path--;
    enqueue(0);
    enqueue(h_path+1);
    h_path = 0;
   }else{
    enqueue(0);
    enqueue(1);
   }
  
  led_down();
}

/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */
/* 関 数 概 要：最短経路に斜め有効化										  			            */
/* 関 数 詳 細：												                                   */
/* 引       数： なし													    */
/* 戻  り   値： なし										    									*/
/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */ 
void remake_shortest_path_list_naname(){
  enqueue(99);//目印
  enqueue(99);
  int naname_cnt = 0,lr = 0,first_lr = 0;
  
  while(1){
    short mode = dequeue(),num = dequeue();
    if(mode == 99)break;

    switch(mode){
      case -1://L
	       if(naname_cnt == 0){
	          naname_cnt = 1;
	          lr = -1;
			  first_lr = -1;
	        }else if(naname_cnt == 1 && lr == -1 && queue_next() == 0){//Uターン
				enqueue(-1);
	            enqueue(1);
				
				enqueue(-1);
	            enqueue(1);
				
				naname_cnt = 0;
	          	lr = 0;
			}else if(naname_cnt == 1 && lr == -1 && queue_next() == 1){//L L R //１つ目はスラロームにする
				enqueue(-1);
	            enqueue(1);
				
			}else if(naname_cnt == 1 && lr == 1 && queue_next() != 1){//R L * //スラロームにする
				enqueue(1);
	            enqueue(1);
				
				enqueue(-1);
	            enqueue(1);
				
				naname_cnt = 0;
	          	lr = 0;
				
			}else{
	          if(lr == 1){
	            lr = -1;
	            naname_cnt++;
	          }else{//斜め中のVターン
				if(first_lr == -1){
					enqueue(-11);
	         		enqueue(1);
				}else if(first_lr == 1){
					enqueue(11);
	         		enqueue(1);
				}
				first_lr = 0;
				
	            enqueue(10);
	            enqueue(naname_cnt);
	           
			    enqueue(-11);
	            enqueue(1);
				
				if(queue_next() == 0){//L L (S) //スラロームにする
					naname_cnt = 0;
					enqueue(-1);
	           		enqueue(1);
				}else{
	            	naname_cnt = 1;
					lr = -1;
					first_lr = -1;
				}
	          }
	        }
	        break;
       case 0://S
		    if(naname_cnt == 1){//斜めが1マスのときはスラロームにする
				if(lr == 1){
					enqueue(1);
	            	enqueue(1);
				}else{
					enqueue(-1);
	            	enqueue(1);
				}
				naname_cnt = 0;
	            lr = 0;
				
/*			}else if(naname_cnt == 2){//斜めが2マスのときはスラロームにする
				if(first_lr == 1){
					enqueue(1);
	            	enqueue(1);
					
					enqueue(-1);
	            	enqueue(1);
				}else{
					enqueue(-1);
	            	enqueue(1);
					
					enqueue(1);
	            	enqueue(1);
				}
				naname_cnt = 0;
	            lr = 0;
*/				
			}else if(naname_cnt > 2){
			  if(first_lr == 1){
				enqueue(11);
	         	enqueue(1);
			  }else if(first_lr == -1){
				enqueue(-11);
	         	enqueue(1);
			  }
			  first_lr = 0;
			  
	          if(lr == 1){
	            enqueue(10);
	            enqueue(naname_cnt);
				
	            enqueue(11);
	            enqueue(1);
	            naname_cnt = 0;
	            lr = 0;
	          }else{
	            enqueue(10);
	            enqueue(naname_cnt);
				
	            enqueue(-11);
	            enqueue(1);
	            naname_cnt = 0;
	            lr = 0;
	          }
	        }
	        enqueue(mode);
	        enqueue(num);
	        break;
       case 1://R
	        if(naname_cnt == 0){
	          naname_cnt = 1;
	          lr = 1;
			  first_lr = 1;
	        }else if(naname_cnt == 1 && lr == 1 && queue_next() == 0){//Uターン
				enqueue(1);
	            enqueue(1);
				
				enqueue(1);
	            enqueue(1);
				
				naname_cnt = 0;
	          	lr = 0;
			}else if(naname_cnt == 1 && lr == 1 && queue_next() == -1){//R R L //１つ目はスラロームにする
				enqueue(1);
	            enqueue(1);
				
			}else if(naname_cnt == 1 && lr == -1 && queue_next() != -1){//L R * //スラロームにする
				enqueue(-1);
	            enqueue(1);
				
				enqueue(1);
	            enqueue(1);
				
				naname_cnt = 0;
	          	lr = 0;
				
			}else{
	          if(lr == -1){
	            lr = 1;
	            naname_cnt++;
	          }else{//斜め中のVターン
				if(first_lr == 1){
					enqueue(11);
	         		enqueue(1);
				}else if(first_lr == -1){
					enqueue(-11);
	         		enqueue(1);
				}
				first_lr = 0;
				
	            enqueue(10);
	            enqueue(naname_cnt);
				
				enqueue(11);
	            enqueue(1);
				
				if(queue_next() == 0){//R R (S) //スラロームにする
					naname_cnt = 0;
					enqueue(1);
	           		enqueue(1);
				}else{
	            	naname_cnt = 1;
					lr = 1;
					first_lr = 1;
				}
	          }
	        }
	        break;
   	  }
  }
}

/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */
/* 関 数 概 要：最短経路走行（最終版）											  			            */
/* 関 数 詳 細：												                                   */
/* 引       数： 0 斜めなし　１斜めあり														    */
/* 戻  り   値： なし										    									*/
/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */ 
void run_shortest_path_fin(	char naname){
  GyroSum_reset();
  Encoder_reset();
  
  my_x = Start_x;my_y = Start_y;my_angle = 1;
  int comand ,path_num;
  int first_flag = 0; //0:まだ走行してない 1:走行中
  int cnt = 0;
  
  int over_run = -200;//速度上げるとオーバーランぎみなので少し手前で止める

   
  /*
  while(!queue_empty()){//debug
    comand = dequeue();path_num = dequeue();
	printf2("%d %d\n",comand,path_num);
	delay(1);
  }
  while(1);
 */ 
  while(!queue_empty()){
    comand = dequeue();path_num = dequeue();
    switch(comand){
      case -1://L
        L_curve(sl90,true);
  
		if(queue_next() == -1){//ターン
			ESmotor(20,40,true,true);//距離、スピード
			
		}else if(queue_next() == 1){//ターン
			ESmotor(50,40,true,true);//距離、スピード
		}
		
        break;
      case -11://L45
	  	
		if(queue_next() == -11){//Vターン
			 L_rotate_naname(l45 * path_num * 0.85);
			 
		}else if(queue_next() == -1){//45からの90ターン
			L_rotate_naname(l45 * path_num);
			
			ESmotor(30,40,true,true);//距離、スピード
		
		}else{
			 L_rotate_naname(l45 * path_num);
		}
        break;
      case 0://S
        if(queue_empty()){
			
			S_run(h1 * (long long)path_num ,55 + run_fin_speed_offset,false,true);
			
			while(1){//ゴールの奥まで進む
      			if(get_IR(IR_FR) > 60){
        			Smotor(-7,false);

        			cnt = 0;
      			}else if(get_IR(IR_FR) < 55){
       	 			Smotor(+7,false);
       				
        			cnt = 0;
      			}else {
        			motor(0,0);
        			cnt++;
      			}
      			if(cnt > 2000)break;
			}
		}else {
          path_num--;
          if(path_num > 0){
			  if(first_flag == 0)S_run((h1 *(long long) path_num) - over_run ,55 + run_fin_speed_offset,3,true); // memo : non_stop = 3 加速はゆっくり　減速はすくなめ
		  	  else S_run((h1 * (long long)path_num)  - over_run ,55 + run_fin_speed_offset,true,true);
			  
			  if(queue_next() == -11 || queue_next() == 11){//直線後に45ターン
				  if(queue_next() < 0){//次　左
			      	  S_run_kabe2(35 + run_fin_speed_offset,true,1);
						
				  }else if(queue_next() > 0){//次　右
				  	  S_run_kabe2(35 + run_fin_speed_offset,true,2);
				  }else{
					  S_run_kabe2(35 + run_fin_speed_offset,true,3);
				  }
			  }else{
				   if(queue_next() < 0){//次　左
			      	  S_run_kabe(50 + run_fin_speed_offset,true,1);
						
				  }else if(queue_next() > 0){//次　右
				  	  S_run_kabe(50 + run_fin_speed_offset,true,2);
				  }else{
					  S_run_kabe(50 + run_fin_speed_offset,true,3);
				  }
				  
			  }
			  
			  
		  }else{//半マスだけ＝スタート直後に壁切れ　
			  if(queue_next() < 0){//次　左
	          	  S_run_kabe(30 + run_fin_speed_offset,false,1);
				
			  }else if(queue_next() > 0){//次　右
				  S_run_kabe(30 + run_fin_speed_offset,false,2);
			  }else{
				  S_run_kabe(30 + run_fin_speed_offset,false,3);
			  }
		  }
        }

        //my_x = nx;
        //my_y = ny;
        break;
      case 10://Snaname
	  	path_num-=2;
		 
	    if(path_num <= 0){
			//存在しないはず
		}else{
			S_run(s45 * (long long)path_num + 300,50 + run_fin_speed_offset,true,3); // w_flag = 3 斜めの壁補正あり
		}
		
		
		if(queue_next() < 0){//次　左
        	S_run_kabe_naname(45 + run_fin_speed_offset,3,1);
			
		}else if(queue_next() > 0){//次　右
			S_run_kabe_naname(45 + run_fin_speed_offset,3,2);
		}else{
			S_run_kabe_naname(45 + run_fin_speed_offset,3,3);
		}
		
        //my_x = nx;
        //my_y = ny;
        break;
      case 1://R
        R_curve(sr90,true);
		
		if(queue_next() == -1){//ターン
			ESmotor(50,40,true,true);//距離、スピード
			
		}else if(queue_next() == 1){//ターン
			ESmotor(20,40,true,true);//距離、スピード
		}
        break;
      case 11://R45
        
  		if(queue_next() == 11){//Vターン
			R_rotate_naname(r45 * path_num * 0.85);
			
		}else if(queue_next() == 1){//45からの90ターン
			R_rotate_naname(r45 * path_num);
			
			ESmotor(30,40,true,true);//距離、スピード
			
		}else{
			R_rotate_naname(r45 * path_num);
		}
        break;
    }
	first_flag = 1;
  }

  led_up();
}

/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */
/* 関 数 概 要：CMT0割り込みモジュール                                                              */
/* 関 数 詳 細：                                                                                    */
/* 引       数：なし										    */
/* 戻  り   値：なし										    */
/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */
#pragma interrupt (Excep_CMT0_CMI0(vect=28))
void Excep_CMT0_CMI0(void)
{	
	static int task = 0;
	static char log_flag = 0;
	
	task ++;                         // タスクの更新						
  	if (log_start != 2 && task >= 30) task = 0;       
	if (log_start == 2 && task >= 10) task = 0;       
	
	Gyro_update();
		
	if(ir_flag == 1){//赤外線有効時=走行時にジャイロによる安全停止チェックを行う
		if(abs(Gyro()) > 300){
    		motor_stop_cnt++;
    		if(motor_stop_cnt > 20)motor_stop();
  		}else motor_stop_cnt = 0;
	}
	
	if(time_limit > 0){
		time_limit--;	
	}
	
	switch(task) {                         			
	case 0:
	case 10:
	case 20:
		encoder_update();
        break;
   
   	case 1:
   		if(log_start != 0){
			log_flag = 1;
				
			
			if((LOG_BUF_MAX * log_save_cnt) + log_cnt +8 <= LOG_MAX){
				log_buff[log_cnt++] = my_x;
				log_buff[log_cnt++] = my_y;
				
				log_buff[log_cnt++] = (char)get_IR(IR_FL);
				log_buff[log_cnt++] = (char)get_IR(IR_L);
				log_buff[log_cnt++] = (char)get_IR(IR_R);
				log_buff[log_cnt++] = (char)get_IR(IR_FR);
				
				log_buff[log_cnt++] = get_pwm_buff_L();
				log_buff[log_cnt++] = get_pwm_buff_R();
				
				if(log_cnt >= LOG_BUF_MAX-2){//たまったら保存する
					log_save();
					log_save_cnt++;
					log_cnt = 0;
				}	
				
			}else{
				if(log_block_num < 15){
					log_block_num++;
					
					log_cnt = 0;
					log_save_cnt = 0;
	
				}else{
					log_start = 0;//サイズオーバーなので記録終了
				}
			}
		}else{
			if(log_flag == 1){//ログの保存終了
				while(log_cnt < LOG_BUF_MAX){
					log_buff[log_cnt++] = 0;
				}
				log_save();
				log_save_cnt++;
				log_cnt = 0;
			}
		}
        break;
		
	default:
		break;
   	}
}

/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */
/* 関 数 概 要：CMT2割り込みモジュール                                                              */
/* 関 数 詳 細：                                                                                    */
/* 引       数：なし										    */
/* 戻  り   値：なし										    */
/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */
#pragma interrupt (Excep_CMT2_CMI2(vect=30))
void Excep_CMT2_CMI2(){
	
	if(ir_flag == 1){
		ir_update();
	}else{
		ir(0);
	}
}




 



