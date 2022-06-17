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
int get_sw(void);

void maze_update(char,char,char);
void maze_search_adachi(short,short);
void shortest_path_search_fin(void);
void remake_shortest_path_list_naname(void);
void run_shortest_path_fin(char);
	
void S_run(int,int, char,char);
	
/* 定数設定 */
#define true 1
#define false 0

#define maze_d_max	9999
#define H	16
#define W	16
#define Start_x  0
#define Start_y  0
#define Goal_x  0
#define Goal_y  3


#define r45  (11000)
#define l45  (11000)
#define r90  (22000)
#define l90  (22000)
#define r180  (-44000)

//memo : 1mm = 4
#define s1 (699)
#define s45 (12 * 40)
#define h1 (90 * 4)

#define rsls90 (195 * 4)
#define sr90  (22000)
#define sl90  (22000)

#define r_cost 4

//グローバル変数
short motor_stop_cnt = 0;
char maze_w[H][W] = {0};	//上位4bit = 壁の確定bit 下位4bit = 壁の情報（未確定含む）
short maze_d[H][W][4] = {0};	//4方向分の重み

char dx[4] = {0,1,0,-1},dy[4] = {-1,0,1,0};

char my_x = Start_x,my_y = Start_y,my_angle = 1;//0:up 1:right 2:down 3:left

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
	
	ALL_init();//初期化
	
	delay(3);
	
	while(1){
		
		//while(1)printf2("%d\t%d\t%d\t%d\n",get_IR(0),get_IR(1),get_IR(2),get_IR(3));
		
		delay(2000);
		for(i = 1;i <= 8; i*= 2){
			led(i);
			delay(200);
		}
		led(0);
		delay(1000);
		S_run(s1,15,false,false);
		delay(1000);
		while(1);
		
		while(1){
			//motor(10,-10);
			
			printf2("%d\t",get_encoder_total_L());
			printf2("%d\t",get_encoder_total_R());
			
			printf2("%d\t",(int)(MTU7.TCNT));
			printf2("%d\n",(int)(MTU8.TCNT));
			
			if(get_sw() == 1)Encoder_reset();
			delay(1000);
		}
		
		//正面センサーに手をかざす
		while((get_IR(0) < 30) || (get_IR(3) < 30)) nop();
  		while((get_IR(0) > 30) || (get_IR(3) > 30)) nop();
  		delay(1000);
		
		for(i = 1;i <= 8; i*= 2){
			led(i);
			delay(200);
		}
		led(0);
		
		
		
		while(1){
			Tmotor(r90);
			delay(1000);
		}
		
		//maze_search_adachi(Goal_x,Goal_y);
		 
		//maze_search_adachi(Start_x,Start_y);

		//shortest_path_search_fin();
		//remake_shortest_path_list_naname();
		
		for(i = 1;i <= 8; i*= 2){
			led(i);
			delay(200);
		}
		led(0);
		
		//run_shortest_path_fin(false);//not naname
		//run_shortest_path_fin(true);//naname
		
		//スイッチ入力待ち
		while(get_sw() == 0) nop();
		while(get_sw() == 1) nop();
	}
	
	/*
	while(1){
		printf2("input: ");
		scanf2("%d",&aa);
		printf2("%d\n",aa);
	}*/
	
	/*
	DataFlash_read(1,read_buff,sizeof(read_buff));
	
	 // Verify 
    for (i = 0; i < sizeof(read_buff); i++){
    	if (read_buff[i] != prog_buff[i]){
       		printf2("NG1\n");
			break;
        }
    }
	
	DataFlash_write(1,prog_buff,sizeof(prog_buff));
	
	DataFlash_read(1,read_buff,sizeof(read_buff));
	
    // Verify 
    for (i = 0; i < sizeof(read_buff); i++){
    	if (read_buff[i] != prog_buff[i]){
        	printf2("NG_2\n");
            break;
        }
    }
	*/
	
 
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
  
	//スイッチ入力待ち
	while(get_sw() == 0) nop();
	while(get_sw() == 1) nop();
	
	Gyro_init();	//ジャイロ、SPIの初期化 　注意：少し時間かかります 処理中はジャイロセンサーを動かさないこと

	CMT_init();  // CMT0の初期化

	initSCI1(SPEED_9600);
	//USB_init();  //USB CDCの初期化
	
	led(1);
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
/* 関 数 概 要：迷路情報の更新											  			            */
/* 関 数 詳 細：												                                   */
/* 引       数： 現在地のXY座標、方角															    */
/* 戻  り   値： なし										    									*/
/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */ 
void maze_update(char x,char y,char angle){
  if((maze_w[y][x]&0xf0) == 0xf0)return;

  for(short i = -1; i < 3;i++){
    short ii = (4 + angle+i)%4;
    char nx = x+dx[ii], ny = y+dy[ii];

    if((maze_w[y][x] & (1 << (4+ii))) == 0 ){
      maze_w[y][x] |= 1 << (4+ii);
      
      switch(i){
        case -1://L
          if(get_IR(IR_L) > 30)maze_w[y][x] |= 1 << ii;
          break;
        case 0://S
          if(get_IR(IR_FL) > 30)maze_w[y][x] |= 1 << ii;
          break;
        case 1://R
          if(get_IR(IR_R) > 30)maze_w[y][x] |= 1 << ii; 
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

void S_run(int path,int powor, char non_stop,char kabe){
  //GyroSum_reset();
  ESmotor(path,powor,non_stop,kabe);
  int cnt2 = 0;

  if(!non_stop && kabe){
   // GyroSum_reset();
    //while(35 < get_IR(IR_FL)){
	 while(1){
      if(get_IR(IR_FL) > 35){
        Smotor(-10,true);

        cnt2 = 0;
      }else if(get_IR(IR_FL) < 30){
        Smotor(+10,true);
       
        cnt2 = 0;
      }else {
        motor(0,0);
        cnt2++;
      }
      if(cnt2 > 100)break;
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
 
  ESmotor(50,powor,true,false);
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
    int x = dequeue(),y;
    y = x%100;
    x /=100;

    for(char i =0;i<4;i++){
      char update_flag = 0;
      int nx = x+dx[i],ny = y+dy[i];
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
  char x = my_x,y = my_y,angle = my_angle;
  
  while(x != target_x || y != target_y){
    short num = maze_d[y][x][(angle+2)%4];
    char n_num = 0;
    char s_flag = 0;
    int nx = x+dx[angle],ny = y+dy[angle];
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
    char ni = ((4 + n_num - ((4+angle-1)%4))%4) -1;// -1 ~ 2

    if((maze_w[y][x] & (1 << (4+n_num))) == 0)break;//unknown
    switch(ni){
      case -1://L
        if(s_path > 0){
          enqueue(0);
          enqueue(s_path);
          s_path = 0;
        }

        enqueue(-1);
        enqueue(1);

        s_path += 1;
        
        angle = (4+angle-1)%4;
        x += dx[n_num];
        y += dy[n_num];
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

        s_path += 1;
         
        angle = (4+angle+1)%4;
        x += dx[n_num];
        y += dy[n_num];
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
/* 関 数 概 要：最短経路を走行									  			            */
/* 関 数 詳 細：												                                   */
/* 引       数： なし														    */
/* 戻  り   値： なし										    									*/
/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */ 
void run_shortest_path(){
  GyroSum_reset();
  Encoder_reset();
  
  int comand ,path_num;
  
  while(!queue_empty()){
    comand = dequeue();path_num = dequeue();
    switch(comand){
      case -1://L
        L_rotate(l90);
        break;
      case 0://S
        if(path_num == 1)S_run(s1 * path_num,25,false,true);
        else S_run(s1 * path_num,35,false,true);

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
        break;
      case 1://R
        R_rotate(r90);
        break;
      case 2://B
        Tmotor(r180);
        my_angle = (4+my_angle+2)%4;
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

  while(1){
    maze_update(my_x,my_y,my_angle);
    if((target_x != Goal_x || target_y != Goal_y) && (target_x != Start_x || target_y != Start_y)){
      if((maze_w[target_y][target_x] & 0xf0) == 0xf0)break;      
    }
    if(target_x == my_x && target_y == my_y){
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
    int x = dequeue(),y;
    y = x%100;
    x /=100;

    for(char i =0;i<4;i++){
      char update_flag = 0;
      int nx = x+dx[i],ny = y+dy[i];
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
	delay(200);
  }
  led(0);
 
  //run_list
  queue_reset();
  short h_path = 0;
  my_x = Start_x;my_y = Start_y;my_angle = 1;
 
  while(my_x != Goal_x || my_y != Goal_y){

    short num = maze_d[my_y][my_x][(my_angle+2)%4];
    char n_num = 0;
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
    char ni = ((4 + n_num - ((4+my_angle-1)%4))%4) -1;// -1 ~ 2

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
  
  for(int i = 1;i <= 8; i*= 2){
	led(i);
	delay(200);
  }
  led(0);
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
        if(queue_empty())S_run(h1 * path_num + (h1/4),80,false,true);
        else {
          path_num--;
          if(path_num > 0)S_run(h1 * path_num ,80,true,true);
          S_run_kabe(50,true);
        }

         non_stop = 0;
        //my_x = nx;
        //my_y = ny;
        break;
      case 10://Snaname
        S_run(s45 * path_num ,60,false,false);

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
  }

  for(int i = 1;i <= 8; i*= 2){
	led(i);
	delay(200);
  }
  led(0);
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
	
	Gyro_update();
	
	ir_update();
	
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





 



