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

//#define PRINT /* 使用時は有効化すること*/

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
void MTU0_init(void);
void MTU1_init(void);

void led(char);
void led_up(void);
void led_down(void);
int get_sw(void);

void maze_save(void);
void maze_load(void);
void maze_update(char,char,char);
void maze_search_adachi(short,short);
void shortest_path_search_fin(void);
void remake_shortest_path_list_naname(void);
void run_shortest_path_fin(char);
	
void L_rotate(long long);
void R_rotate(long long);
void S_run(long long,int, char,char);
	
/* 定数設定 */
#define true 1
#define false 0


//グローバル変数
short motor_stop_cnt = 0;
char maze_w[H][W] = {0};	//上位4bit = 壁の確定bit 下位4bit = 壁の情報（未確定含む）
short maze_d[H][W][4] = {0};	//4方向分の重み

short dx[4] = {0,1,0,-1},dy[4] = {-1,0,1,0};

char my_x = Start_x,my_y = Start_y,my_angle = 1;//0:up 1:right 2:down 3:left

int ir_flag = 0; // 0:赤外線OFF 1:赤外線ON
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
	
	ALL_init();//初期化
	
	delay(100);
	/*
	while(1){
		motor(20,0);
			
		//printf2("%d\t%d\t%d\t%d\n",get_IR(0),get_IR(1),get_IR(2),get_IR(3));
		printf2("%ld\n",get_encoder_total_L());
		delay(1000);
	}*/
		
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
		my_angle = 1;
		 
		ir_flag = 1;//赤外線ON
		
		if(mode != 5){//迷路情報のリセットでなければ
			led(6);
			//スイッチ入力待ち
			while(get_sw() == 0) nop();
			while(get_sw() == 1) nop();
		
			//正面センサーに手をかざす
			while((get_IR(0) < 20) || (get_IR(3) < 20)) led(1);
  			while((get_IR(0) > 20) || (get_IR(3) > 20)) led(8);
				
			led_up();
			
			GyroSum_reset();
			Encoder_reset();
		}
		
		switch(mode){
			case 1://探索モード
				
				maze_search_adachi(Goal_x,Goal_y);
		
				maze_search_adachi(Start_x,Start_y);
				
				maze_search_adachi(Goal_x,Goal_y);
				
				break;
				
			case 2://最短走行モード
				shortest_path_search_fin();
				
				run_shortest_path_fin(false);
				
				break;
			case 3://最短走行（斜めあり）モード
				shortest_path_search_fin();
				remake_shortest_path_list_naname();
				
				run_shortest_path_fin(true);
				
				break;
				
				
				
				
			case 5://迷路情報リセットモード
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
  
	//スイッチ入力待ち
	while(get_sw() == 0) nop();
	while(get_sw() == 1) nop();
	
	Gyro_init();	//ジャイロ、SPIの初期化 　注意：少し時間かかります 処理中はジャイロセンサーを動かさないこと

	CMT_init();  // CMT0の初期化

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
/* 関 数 概 要：左モーター(MTU0,1）,右モーター(MTU3,4） タイマーの初期化	   			            */
/* 関 数 詳 細：		                                                    		                */
/* 引       数：なし										    									*/
/* 戻  り   値：なし										    									*/
/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */ 
void MTU0_init(void){
	volatile int C_cycle,duty;
	
	C_cycle = 24e6 / 1000;
	
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
		delay(100);
	}
	led(0);
}

void led_down(){
	for(int i = 8;i > 0; i/= 2){
		led(i);
		delay(100);
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
	
	DataFlash_write(1,buff,sizeof(buff));
}

/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */
/* 関 数 概 要：迷路情報の復元                                                                        */
/* 関 数 詳 細：                                                                                    */
/* 引       数：なし										    */
/* 戻  り   値：なし										    */
/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */ 
void maze_load(){
	uint8_t buff[H*W];
	
	DataFlash_read(1,buff,sizeof(buff));
	
	for(int i = 0; i < H;i++){
		for(int j = 0; j < W; j++){
			 maze_w[i][j] = buff[i*W + j];
		}
	}
}

/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */
/* 関 数 概 要：迷路情報の更新											  			            */
/* 関 数 詳 細：												                                   */
/* 引       数： 現在地のXY座標、方角															    */
/* 戻  り   値： なし										    									*/
/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */ 
void maze_update(char x,char y,char angle){
  if((maze_w[y][x]&0xf0) == 0xf0)return;

  for(short i = -1; i < 3;i++){
    short ii = (4 + angle+i)%4;
    int nx = x+dx[ii], ny = y+dy[ii];

    if((maze_w[y][x] & (1 << (4+ii))) == 0 ){
      maze_w[y][x] |= 1 << (4+ii);
      
      switch(i){
        case -1://L
          if(get_IR(IR_L) > 20)maze_w[y][x] |= 1 << ii;
          break;
        case 0://S
          if(get_IR(IR_FL) > 20)maze_w[y][x] |= 1 << ii;
          break;
        case 1://R
          if(get_IR(IR_R) > 20)maze_w[y][x] |= 1 << ii; 
          break;
      }
      if((0 <= nx && nx < W) && (0 <= ny && ny < H)){
        maze_w[ny][nx] |= 1 << (4+(ii+2)%4);
        if((maze_w[y][x] & (1 << ii)) != 0)maze_w[ny][nx] |= 1 << ((ii+2)%4);
      }
    }
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
  ETmotor(-a,rsls90,flag);
 
  my_angle = (4+my_angle-1)%4;
}

void R_curve(long long a ,char flag ){
  ETmotor(a,rsls90,flag);
  
  my_angle = (4+my_angle+1)%4;
}

void S_run(long long path,int powor, char non_stop,char kabe){
  //GyroSum_reset();
	ESmotor(path,powor,non_stop,kabe);

  	int cnt2 = 0;

  	if(!non_stop && kabe){
   		// GyroSum_reset();
    	if(13 < get_IR(IR_FL) && 13 < get_IR(IR_FR) ){
	 		while(1){
      			if(get_IR(IR_FL) > 90){
        			Smotor(-10,true);

        			cnt2 = 0;
      			}else if(get_IR(IR_FL) < 70){
       	 			Smotor(+10,true);
       				
        			cnt2 = 0;
      			}else {
        			motor(0,0);
        			cnt2++;
      			}
      			if(cnt2 > 100)break;
			}
    	}
  	}
	
  	motor(0,0);
  //GyroSum_reset();
}

void S_run_kabe(int powor, char flag){//壁切れまで走行
  int Lflag = 0,Rflag = 0;
 
  while(1){
    if(Lflag == 0){
      if(get_IR(IR_L) > 20)Lflag = 1;
    }else if(Lflag == 1){
      if(get_IR(IR_L) < 10)break;
    }

    if(Rflag == 0){
      if(get_IR(IR_R) > 20)Rflag = 1;
    }else if(Rflag == 1){
      if(get_IR(IR_R) < 10)break;
    }
    
    Smotor(powor,flag);
  }
 
  ESmotor(160,powor,true,false);
}

void S_run_maze_search(int path,int powor){
	int M_pwm_min = 5;
	int M_pwm = M_pwm_min;
	long long enc_base = get_encoder_total_L();
	long long enc_now = 0;
	
	int path_cnt = 0;
	
	int cnt2 = 0;
	
	GyroSum_reset();
	
	while(1){
		Smotor(M_pwm,true);
		
		
		if(enc_now > (long long)path * s1){//目標距離に到達
		
			//マスの中心まで移動(戻る）
			while(enc_now - ((long long)s1 * path_cnt ) > s1){
				Smotor(-10,true);
				enc_now = get_encoder_total_L() - enc_base;
			}
			
			if(get_IR(IR_FL) > 20 && get_IR(IR_FR) > 20){//前壁があった場合は
				while(1){//前壁補正
      				if(get_IR(IR_FL) > 90){
        				Smotor(-10,true);

	        			cnt2 = 0;
	      			}else if(get_IR(IR_FL) < 70){
	       	 			Smotor(+10,true);
	       
	        			cnt2 = 0;
	      			}else {
	        			motor(0,0);
	        			cnt2++;
	      			}
	      			if(cnt2 > 100)break;
				}
			}
			
			//現在地の更新
			my_x += dx[my_angle];
			my_y += dy[my_angle];
			
			maze_update(my_x,my_y,my_angle);//迷路情報の更新
			
			break;
		}
		
		if( path_cnt < path-1 && get_IR(IR_FL) > 15 && get_IR(IR_FR) > 15){//目標まで１マス以上残ってる　＆＆　前壁が出現
			//マスの中心まで移動
			while(enc_now - ((long long)s1 * path_cnt ) < s1){
				Smotor(+15,true);
				enc_now = get_encoder_total_L() - enc_base;
			}
		
			while(1){//前壁補正
      			if(get_IR(IR_FL) > 90){
        			Smotor(-10,true);

        			cnt2 = 0;
      			}else if(get_IR(IR_FL) < 70){
       	 			Smotor(+10,true);
       
        			cnt2 = 0;
      			}else {
        			motor(0,0);
        			cnt2++;
      			}
      			if(cnt2 > 100)break;
			}
			
			//現在地の更新
			my_x += dx[my_angle];
			my_y += dy[my_angle];
			
			maze_update(my_x,my_y,my_angle);//迷路情報の更新
			
			break;
		}
	
		if(enc_now - ((long long)s1 * path_cnt ) > s1){//１マス進んだ
			//現在地の更新
			my_x += dx[my_angle];
			my_y += dy[my_angle];
			
			maze_update(my_x,my_y,my_angle);//迷路情報の更新
			
			path_cnt++;
		}
		
		if(enc_now < (long long)path * s1 /4){// 進んだ距離 < 目標距離 * 1/4　＝ 加速区間
			M_pwm = M_pwm_min + (enc_now / 4);	
			
		}else if(enc_now > (long long)path * s1 * 3/4){// 進んだ距離 < 目標距離 * 3/4 = //減速区間
			M_pwm = M_pwm_min + ( ((long long)path * s1 - enc_now) / 4);	
			
		}else{
			M_pwm = powor;
		}
			
		if(M_pwm > powor)M_pwm = powor;
		if(M_pwm < M_pwm_min)M_pwm = M_pwm_min;
		
		enc_now = get_encoder_total_L() - enc_base;
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
      if((0 <= nx && nx < W) && (0 <= ny && ny < H) && ((maze_w[y][x] & (1<<i)) == 0 ) ){

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
  	short unknown_flag = 0; // 0:確定壁　1:未確定壁
	
  	while(x != target_x || y != target_y){
    	short num = maze_d[y][x][(angle+2)%4];
    	short n_num = 0;
    	char s_flag = 0;
    	short nx = x+dx[angle],ny = y+dy[angle];
    	if((0 <= nx && nx < W) && (0 <= ny && ny < H) && ((maze_w[y][x] & (1<<angle)) == 0 ) ){
      		short next = maze_d[ny][nx][(angle+2)%4];
      		if(num == next+1){
        		n_num = (angle+2)%4;
        		num = next;
        		s_flag = true;
      		}
    	}

    	if(s_flag == false){
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
    
    	n_num = (n_num+2)%4;// 0 ~ 4
    	short ni = ((4 + n_num - ((4+angle-1)%4))%4) -1;// -1 ~ 2

    	if((maze_w[y][x] & (1 << (4+n_num))) == 0){//未確定の壁
		
			break;
			
			//if(ni != 0)break;//直進方向でなければ打ち切り//////////////////////////////////////////////////////////////////////////////////////////////////////////////
			//ni = 10;//未確定の直線
		}
	
    	switch(ni){
      		case -1://L
        		if(s_path > 0){
          			enqueue(10 * unknown_flag );
					//enqueue(0);
          			enqueue(s_path);
          			s_path = 0;
        		}

        		enqueue(-1);
        		enqueue(1);

        		s_path += 1;
        
        		angle = (4+angle-1)%4;
		        x += dx[n_num];
		        y += dy[n_num];
				
				unknown_flag = 0;
		        break;
		    case 0://S
   
				if(unknown_flag == 1 && s_path > 0){
          			enqueue(10);
          			enqueue(s_path);
          			s_path = 0;
        		}
				
		        s_path +=1;
		        x += dx[n_num];
		        y += dy[n_num];
				
				unknown_flag = 0;
		        break;
				
			case 10://S 未確定の直線
   				
				if(unknown_flag == 0 && s_path > 0){
          			enqueue(0);
          			enqueue(s_path);
          			s_path = 0;
        		}
				
		        s_path +=1;
		        x += dx[n_num];
		        y += dy[n_num];
				
				unknown_flag = 1;
		        break;
				
      		case 1://R
		        if(s_path > 0){
			    	enqueue(10 * unknown_flag);
					//enqueue(0);
			        enqueue(s_path);
			        s_path = 0;
		        }
        
		        enqueue(1);
		        enqueue(1);

		        s_path += 1;
		         
		        angle = (4+angle+1)%4;
		        x += dx[n_num];
		        y += dy[n_num];
				
				unknown_flag = 0;
		        break;
      		case 2://B
        		if(s_path > 0){
          			enqueue(10 * unknown_flag);
					//enqueue(0);
          			enqueue(s_path);
          			s_path = 0;
        		}
        
        		enqueue(2);
        		enqueue(1);

				s_path += 1;
				 
        		angle = (4+angle+2)%4;
				x += dx[n_num];
		        y += dy[n_num];
				
				unknown_flag = 0;
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
        if(path_num == 1){
	  		S_run(s1 * (long long)path_num,17,false,true);
			
		}else{
			S_run(s1 * (long long)path_num,22,false,true);
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
		
	/*  case 10://S 未確定の直線
        if(path_num == 1){
	  		S_run(s1 * (long long)path_num,20,false,true);
			
			my_x += dx[my_angle];
			my_y += dy[my_angle];
		}else{
			S_run_maze_search(path_num,20);
		}
		
        break;*/
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
	GyroSum_reset();
	Encoder_reset();

	led_down();
	
	while(1){
    	maze_update(my_x,my_y,my_angle);
    	if((target_x != Goal_x || target_y != Goal_y) && (target_x != Start_x || target_y != Start_y)){//スタート地点、ゴール地点以外が目標地点のとき
      		if((maze_w[target_y][target_x] & 0xf0) == 0xf0)break;    //目標地点の壁がすべて確定したら探索完了  
    	}
		
    	if(target_x == my_x && target_y == my_y){//ゴール
			led_up();
			
      		if(target_x == Start_x && target_y == Start_y){
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
      if((0 <= nx && nx < W) && (0 <= ny && ny < H) && ((maze_w[y][x] & (1<<i)) == 0 )  && ((maze_w[y][x] & (1<<(4+i))) != 0 )  ){

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
  
  for(int i = 1;i <= 8; i*= 2){
	led(i);
	delay(100);
  }
  led(0);
 
  //run_list
  queue_reset();
  short h_path = 0;
  my_x = Start_x;my_y = Start_y;my_angle = 1;
 
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
  enqueue(99);
  enqueue(99);
  int naname_cnt = 0,lr = 0;
  
  while(1){
    short mode = dequeue(),num = dequeue();
    if(mode == 99)break;

    switch(mode){
      case -1://L
        if(naname_cnt == 0){
          enqueue(-11);
          enqueue(1);
          naname_cnt = 1;
          lr = -1;
        }else{
          if(lr == 1){
            lr = -1;
            naname_cnt++;
          }else{
            enqueue(10);
            enqueue(naname_cnt);
            naname_cnt = 1;
            enqueue(-1);
            enqueue(1);
            lr = -1;
          }
        }
        break;
       case 0://S
        if(naname_cnt > 0){
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
          enqueue(11);
          enqueue(1);
          naname_cnt = 1;
          lr = 1;
        }else{
          if(lr == -1){
            lr = 1;
            naname_cnt++;
          }else{
            enqueue(10);
            enqueue(naname_cnt);
            naname_cnt = 1;
            enqueue(1);
            enqueue(1);
            lr = 1;
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
  char non_stop = 0;
  int comand ,path_num;
  int first_flag = 0; //0:まだ走行してない 1:走行中
  
  while(!queue_empty()){
    comand = dequeue();path_num = dequeue();
    switch(comand){
      case -1://L
        if(naname)L_rotate(l90);
        else L_curve(sl90,non_stop);
        non_stop = 1;
        break;
      case -11://L45
        L_rotate(l45);
        non_stop = 1;
        break;
      case 0://S
        if(queue_empty())S_run(h1 * (long long)path_num + (h1/2),32,false,true);
        else {
          path_num--;
          if(path_num > 0){
			  if(first_flag == 0)S_run(h1 *(long long) path_num ,32,3,true); // memo : non_stop = 3 加速はゆっくり　減速はすくなめ
		  	  else S_run(h1 * (long long)path_num ,32,true,true);
		  }
          S_run_kabe(27,true);
        }

         non_stop = 0;
        //my_x = nx;
        //my_y = ny;
        break;
      case 10://Snaname
        S_run(s45 * (long long)path_num ,30,false,false);

         non_stop = 0;
        //my_x = nx;
        //my_y = ny;
        break;
      case 1://R
        if(naname)R_rotate(r90);
        else R_curve(sr90,non_stop);
        non_stop = 1;
        break;
      case 11://R45
        R_rotate(r45);
        non_stop = 1;
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
	task ++;                         // タスクの更新						
  	if (task == 10) task = 0;            
	
	if(ir_flag == 1){
		ir_update();
	}else{
		ir(0);
	}
	
	Gyro_update();
		
	if(abs(Gyro()) > 250){
    	motor_stop_cnt++;
    	if(motor_stop_cnt > 20)motor_stop();
  	}else motor_stop_cnt = 0;
	
	switch(task) {                         			
	case 0:
		encoder_update();
        break;
   
   	case 1:
   		
        break;
		
	default:
		break;
   	}
}





 



