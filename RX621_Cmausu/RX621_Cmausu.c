/****************************************************************************************************/
/* プロジェクト名：RX621_SAMPLE     	                　　　　　          			    */
/* モジュール名：  			                     					    */
/* 履    歴    ：										    */
/* 使用マイコン：RX621 R5F56218BDFP (Flash ROM:512KB, SRAM:64KB, DATA Flash 32KB)                   */
/* 作成者      ：hirata0517masato               				                    */
/* バージョン  ：1.00     3rd用                                                                          */
/* 作成日      ：2023/06/11 									    */
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

short get_r_cost(void);
short get_r45_cost(void);

void maze_save(void);
void maze_load(void);
void log_reset(void);
void log_save(void);
void log_load(void);
void search_pickup(int*,int*);
void maze_update(char,char,char,char);
void maze_search_adachi(short,short);
void maze_search_all(void);
char shortest_path_search_check(void);
void shortest_path_search_fin(void);
void shortest_path_search_perfect(void);
void remake_shortest_path_list_naname(void);
void remake_shortest_path_list_naname2(void); //2マスでも斜めにする
void path_compression(void);
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

short maze_d_perfect[H][W] = {0};

short r_cost_offset = 0;

short r45_cost_offset = 0;

short dx[4] = {0,1,0,-1},dy[4] = {-1,0,1,0};

char my_x = Start_x,my_y = Start_y,my_angle = Start_angle;//0:up 1:right 2:down 3:left

int ir_flag = 0; // 0:赤外線OFF 1:赤外線ON

int Gy_flag = 0; // 0:ジャイロOFF 1:ジャイロON

int run_fin_speed_offset = 0;

long long time_limit = -1;

int pickup_x = 1;
int pickup_y = 1;

uint8_t log[LOG_MAX] = {0};
uint8_t log_buff[LOG_BUF_MAX] = {0};
char log_start = 0;
int log_cnt = 0;
int log_save_cnt = 0;
int log_block_num = 1; //max 15
short status_log = 99;

long t_1ms = 0;

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
    int first_flag = 0;
    ir_flag = 0;//赤外線OFF
	
    ALL_init();//初期化
	
    delay(100);
	
    /*
      while(1){
      motor(i,i);
      delay(10);
      printf2("%d : %d \n",i,get_encoder_L());
      delay(100);
      i++;
	  
      if(i > 100){
      motor(0,0);
      while(1);
      }
      }*/
 
/*      
      while(1){
	      led(0);
	      //ir(0x10);
	      ir_flag = 1;//赤外線ON
	      //motor(10,10);

	      printf2("%d\t%d\t%d\t%d\t%d\t%d\t%d :",get_IR_base(5),get_IR_base(4),get_IR_base(3),get_IR_base(2),get_IR_base(1),get_IR_base(0),get_IR_base(6));
			
	      printf2("\t%d\t%d\t%d\t%d\t%d\t%d\t%d  : ",get_IR(5),get_IR(4),get_IR(3),get_IR(2),get_IR(1),get_IR(0),get_IR(6));
		
	      delay(10);
	      printf2("%ld ",get_encoder_total_L() );
	      printf2("%ld \n",get_encoder_total_R() );
			
	      //printf2("%ld\n",GyroSum_get());
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
			
	    mode = (get_encoder_total_R() / 60) + (get_encoder_total_L() / 60)*8 ;
			
	    if(get_sw() == 1){
		led_up();
		while(get_sw() == 1)nop();
		break;
	    }
	}
		
	my_x = Start_x;
	my_y = Start_y;
	my_angle = Start_angle;
		
	if(mode < 8){//各種調整モードでなければ
	
	   
	    if(mode != 1 && mode != 4 && shortest_path_search_check() == 1){//最短走行時に最短経路が見つからない時
		led(16); 
		delay(500);
		led(0); 
		delay(500);
		led(16); 
		delay(500);
		led(0); 
		delay(500);
		
		continue;
	    }
	    
	    ir_flag = 1;//赤外線ON
			
	    led(6);
	    delay(500);
			
	    //スイッチ入力待ち
	    while(get_sw() == 0){
				
		if(get_IR(IR_L) - get_IR(IR_R) > 20){//左寄り
		    led(8);
					
		}else if(get_IR(IR_L) - get_IR(IR_R) < -20){//右寄り
		    led(1);
					
		}else{//ほぼ中央
		    led(6);
		}
				
	    }
	    while(get_sw() == 1) nop();
		
			
	    //右前センサーに手をかざす
	    while(get_IR(IR_FR) < 40) led(8);
	    while(get_IR(IR_FR) > 40) led(1);
			
			
	    if( Gy_flag == 0){
		led_up();
				
		Gyro_init();	//ジャイロ、SPIの初期化 　注意：少し時間かかります 処理中はジャイロセンサーを動かさないこと
		Gy_flag = 1;
	    }
			
	    led_up();
			
	    GyroSum_reset();
	    Encoder_reset();
	}
		
	switch(mode){
	case 1://探索モード
			
	    if(( maze_w[0][1] & 0x20) == 0){//迷路が初期化された直後 スタート直後のマスの壁が確定していなければ初期化直後と判定する
		led_down();
		led_up();
					
		first_flag = 1;
	    }
	    log_reset();//ログの初期化
	    log_start = 1; //ログ記録開始　30msに１回記録
				
	    Set_motor_pid_mode(0);//低速
	    maze_search_adachi(Goal_x,Goal_y);
		
	    if(first_flag == 1){
		first_flag = 0;
		maze_save();//片道でも迷路を保存する
	    }
				
	    Set_motor_pid_mode(0);//低速
	    maze_search_adachi(Start_x,Start_y);
				
	    log_start = 0; //ログ記録終了
				
	    break;
				
				
	case 2://最短走行モード
	    shortest_path_search_fin();
	   // path_compression();
	    
	    log_reset();//ログの初期化
	    log_start = 2; //ログ記録開始 10msに１回記録
				
	    Set_motor_pid_mode(1);//高速
	    run_shortest_path_fin(false);
				
	    log_start = 0; //ログ記録終了
				
	    led(9);
	    delay(500);
	    led_up();
				
	    my_x = Goal_x;
	    my_y = Goal_y;
	    my_angle = Goal_angle;
	    
#ifdef Pickup_x
	    pickup_x = Pickup_x;
	    pickup_y = Pickup_y;
#else
	    search_pickup(&pickup_x,&pickup_y);
#endif

	    Set_motor_pid_mode(0);//低速
	    maze_search_adachi(pickup_x,pickup_y);//拾いやすいところまで移動する
				
	    break;
	case 3://最短走行（斜めあり）モード
	    shortest_path_search_fin();
	    remake_shortest_path_list_naname();
	    path_compression();
	    
	    log_reset();//ログの初期化
	    log_start = 2; //ログ記録開始 10msに１回記録
				
	    Set_motor_pid_mode(1);//高速
	    run_shortest_path_fin(true);
				
	    log_start = 0; //ログ記録終了
				
	    led(9);
	    delay(500);
	    led_up();
				
	    my_x = Goal_x;
	    my_y = Goal_y;
	    my_angle = Goal_angle;
				
#ifdef Pickup_x
	    pickup_x = Pickup_x;
	    pickup_y = Pickup_y;
#else
	    search_pickup(&pickup_x,&pickup_y);
#endif

	    Set_motor_pid_mode(0);//低速
	    maze_search_adachi(pickup_x,pickup_y);//拾いやすいところまで移動する
				
	    break;
				
	case 4://最短経路上の未確定マスをすべて探しに行く
	    if(( maze_w[0][1] & 0x20) == 0){//迷路が初期化された直後 スタート直後のマスの壁が確定していなければ初期化直後と判定する
		led_down();
		led_up();
				
		first_flag = 1;
	    }
				
	    log_reset();//ログの初期化
	    log_start = 1; //ログ記録開始　30msに１回記録
				
	    Set_motor_pid_mode(0);//低速
	    maze_search_adachi(Goal_x,Goal_y);//はじめは普通に探索走行
				
				
	    if(first_flag == 1){
		first_flag = 0;
		maze_save();//片道でも迷路を保存する
	    }
	    
	    Set_motor_pid_mode(0);//低速
	    maze_search_all();//最短経路上の未確定マスを探しに行く
				
	    log_start = 0; //ログ記録終了
				
	    break;
			
				
	case 5://最短走行（斜めあり）モード ２マスも斜めにするモード
	    shortest_path_search_fin();
	    remake_shortest_path_list_naname2(); //２マスも斜めにするモード
	    path_compression();//大曲など
	    
	    log_reset();//ログの初期化
	    log_start = 2; //ログ記録開始 10msに１回記録
				
	    Set_motor_pid_mode(1);//高速
	    run_shortest_path_fin(true);
				
	    log_start = 0; //ログ記録終了
				
	    led(9);
	    delay(500);
	    led_up();
				
	    my_x = Goal_x;
	    my_y = Goal_y;
	    my_angle = Goal_angle;
			
#ifdef Pickup_x
	    pickup_x = Pickup_x;
	    pickup_y = Pickup_y;
#else
	    search_pickup(&pickup_x,&pickup_y);
#endif

	    Set_motor_pid_mode(0);//低速
	    maze_search_adachi(pickup_x,pickup_y);//拾いやすいところまで移動する
				
	    break;
	
	case 6://最短走行 斜めも考慮した経路選択モード
	    shortest_path_search_perfect();
	    
	    remake_shortest_path_list_naname2(); //２マスも斜めにするモード
	    path_compression();//大曲など
	    
	    log_reset();//ログの初期化
	    log_start = 2; //ログ記録開始 10msに１回記録
				
	    Set_motor_pid_mode(1);//高速
	    run_shortest_path_fin(true);
				
	    log_start = 0; //ログ記録終了
				
	    led(9);
	    delay(500);
	    led_up();
				
	    my_x = Goal_x;
	    my_y = Goal_y;
	    my_angle = Goal_angle;
			
#ifdef Pickup_x
	    pickup_x = Pickup_x;
	    pickup_y = Pickup_y;
#else
	    search_pickup(&pickup_x,&pickup_y);
#endif

	    Set_motor_pid_mode(0);//低速
	    maze_search_adachi(pickup_x,pickup_y);//拾いやすいところまで移動する
				
	    break;
	    
	//8以降は走行以外の調整モード
			
	case 8://速度調整モード（調整値は保存しない、本番での微調整用）
	    Encoder_reset();
		
	    led(6);
	    delay(500);
				
	    //モード選択
	    while(1){
		led(run_fin_speed_offset);
			
		run_fin_speed_offset = get_encoder_total_R() / 60;
			
		if(get_sw() == 1){
		    led_up();
		    while(get_sw() == 1)nop();
		    break;
		}
	    }
		
	    break;
			
	case 9://90度回転の重み調整（調整値は保存しない、本番での微調整用）
	    Encoder_reset();
		
	    led(9);
	    delay(500);
				
	    //モード選択
	    while(1){
		led(get_r_cost());
			
		r_cost_offset = get_encoder_total_R() / 60;
					
		if(get_sw() == 1){
		    led_up();
		    while(get_sw() == 1)nop();
		    break;
		}
	    }
		
	    break;
	
	case 10://45度回転の重み調整（調整値は保存しない、本番での微調整用）
	    Encoder_reset();
		
	    led(9);
	    delay(500);
				
	    //モード選択
	    while(1){
		led(get_r45_cost());
			
		r45_cost_offset = get_encoder_total_R() / 60;
					
		if(get_sw() == 1){
		    led_up();
		    while(get_sw() == 1)nop();
		    break;
		}
	    }
		
	    break;
	    
	case 11://迷路情報リセットモード
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
				
	case 12://ログ出力モード
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
	
    //Gyro_init();	//ジャイロ、SPIの初期化 　注意：少し時間かかります 処理中はジャイロセンサーを動かさないこと

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
	
    PORTD.DDR.BYTE = 0xff;           // 6:IR7 5:IR6 4:IR5 3:IR4	2:IR3	1:IR2	0:IR1
	
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

    S12AD.ADANS.WORD = 0x007f;//スキャン変換端子の設定 AN0-7の全て使用する場合は0xff
    //高速化のためAD変換前に必要なポートのみに更新している
    
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
/* 関 数 詳 細：0.05ms割り込み周期                                                                     */
/* 引       数：なし										    */
/* 戻  り   値：なし										    */
/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */ 
void CMT2_init(void)
{
    MSTP(CMT2) = 0;                // SYSTEM.MSTPCRA.BIT.MSTPA15 = 0; // CMT0 タイマースタンバイ解除 （0で解除）
    CMT2.CMCR.WORD = 0x0040;       // 4:割り込み許可　0:PCLK/8 1:PCLK/32 2:PCLK/128 3:PCLK/512
    CMT2.CMCOR = 150-1;           // 0.25ms Count： PCLK = 24MHz/8=3MHz 3M/0.05mS=750 (得たいカウント数-1) 
    IPR(CMT2,CMI2) = 15;		//割り込み優先度
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
/* 関 数 概 要：90度回転の重みを取得										  			            */
/* 関 数 詳 細：												                                   */
/* 引       数： 													    */
/* 戻  り   値：なし										    									*/
/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */ 
short get_r_cost(void){
    return max(1,r_cost + r_cost_offset);//１より小さくしない
}

/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */
/* 関 数 概 要：45度回転の重みを取得										  			            */
/* 関 数 詳 細：												                                   */
/* 引       数： 													    */
/* 戻  り   値：なし										    									*/
/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */ 
short get_r45_cost(void){
    return max(1,r45_cost + r45_cost_offset);//１より小さくしない
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
	
    if(shortest_path_search_check() == 1){//最短経路が見つからない時
		led(16); 
		delay(500);
		led(0); 
		delay(500);
		led(16); 
		delay(500);
		led(0); 
		delay(500);
    }else{
	    for(int i = 0; i < H;i++){
		for(int j = 0; j < W; j++){
		    buff[i*W + j] = maze_w[i][j];
		}
	    }
		
	    DataFlash_write(0,buff,sizeof(buff));
    }
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

    int cnt = 0;
    
    printf2("status\tX\tY\tA\t  S5\tS4\tS3\tS2\tS1\tS0\tS6\t   PWM_L PWM_R\tENC_L ENC_R\n\n");
	
    while(block_num < 16){
	DataFlash_read(block_num,log,sizeof(log));
	i = 0;
		
	while(1){
	    if( i >= LOG_MAX)break;
			
	    //センサー値がすべて０なら終了
	    if((log[i+1]== 0) && (log[i+2]== 0) && (log[i+3]== 0) && (log[i+4]== 0) && (log[i+5]== 0))break;
			
	    if((log[i]>>4) > 16 || (log[i]&0x000F) > 16)break;//X,Y座標が範囲外なら終了
				
	    printf2("%3d : %2d %2d %2d\t",log_minus(log[i+13]),log[i]>>4,log[i]&0x000F,log[i+12] );	
														
	    cnt++;
	    if(cnt > 16){
		    cnt = 0;
		    delay(1);//printfを高速、連続で使用すると動作が不安定
	    }
	    
	    printf2(" : \t%3d\t%3d\t%3d\t%3d\t%3d\t%3d\t%3d\t", (((int)log[i+1]) << 2),(((int)log[i+2]) << 2),(((int)log[i+3]) << 2),(((int)log[i+4]) << 2),(((int)log[i+5]) << 2),(((int)log[i+6]) << 2),(((int)log[i+7]) << 2));	
														
	    cnt++;
	    if(cnt > 16){
		    cnt = 0;
		    delay(1);//printfを高速、連続で使用すると動作が不安定
	    }
	    
	    printf2(" : \t%3d\t%3d\t : %3d\t%3d\n",log_minus(log[i+8]),log_minus(log[i+9]),log_minus(log[i+10]) <<1,log_minus(log[i+11]) <<1  );	
														
	    cnt++;
	    if(cnt > 16){
		    cnt = 0;
		    delay(1);//printfを高速、連続で使用すると動作が不安定
	    }
	    
	    i += 16;
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
    int cnt = 0;
	
    if((maze_w[y][x]&0xf0) == 0xf0)return;

    for(short i = -1; i < 3;i++){
	if(type == 1){
	    if(i != 0)continue;//前以外は更新しない
	}else if(type == 2){
	    if(i == 0)continue;//横以外は更新しない
	}
	
	short ii = (4 + angle+i)%4;
	int nx = x+dx[ii], ny = y+dy[ii];

      
	switch(i){
        case -1://L
	    //if((maze_w[y][x] & (1 << (4+ii))) == 0 ){//未確定の場合
				
	    if(get_IR(IR_L) > 40){
		if(((maze_w[y][x] & (1 << (4+ii))) != 0) && ( (maze_w[y][x] | (1 << ii)) == 0 ) ){//確定の場合 && 過去の記録が今回と値が異なる
							
		    //if(type == 3){
		    maze_w[y][x] &= ~(1 << (4+ii));//確定情報を消す
		    L_rotate(l90);//左回転
		    delay(20);
		    maze_update(x,y,my_angle, 1);//前壁のみチェック
		    R_rotate(r90);//右回転
		    delay(20);
		    //}
		}else{
		    maze_w[y][x] |= 1 << ii;
		}
	    }else{
		if(((maze_w[y][x] & (1 << (4+ii))) != 0) && ( (maze_w[y][x] | (1 << ii)) != 0 ) ){//確定の場合 && 過去の記録が今回と値が異なる
							
		    //if(type == 3){
		    maze_w[y][x] &= ~(1 << (4+ii));//確定情報を消す
		    L_rotate(l90);//左回転
		    delay(20);
		    maze_update(x,y,my_angle, 1);//前壁のみチェック
		    R_rotate(r90);//右回転
		    delay(20);
		    //}	
		}else{
		    maze_w[y][x] &= ~(1 << ii); 
		}  
	    }
				
	    maze_w[y][x] |= 1 << (4+ii);
	    //}
	    break;
        case 0://S
	    //if((maze_w[y][x] & (1 << (4+ii))) == 0 ){//未確定の場合
				
				 
	    if(get_IR(IR_F) > 40 ){
		if(((maze_w[y][x] & (1 << (4+ii))) != 0) && ( (maze_w[y][x] | (1 << ii)) == 0 ) ){//確定の場合 && 過去の記録が今回と値が異なる
							
		    //if(type == 3){
		    maze_w[y][x] &= ~(1 << (4+ii));//確定情報を消す
		    L_rotate(l90);//左回転
		    delay(20);
		    maze_update(x,y,my_angle, 2);//横壁のみチェック
		    R_rotate(r90);//右回転
		    delay(20);
		    //}
		}else{
		    maze_w[y][x] |= 1 << ii; 
		}
			
		cnt = 0;
		t_1ms = 0;
		while(t_1ms < F_max_time){
		    if(get_IR(IR_FL) > F_max || get_IR(IR_F) > F_max || get_IR(IR_FR) > F_max){
			Smotor(-F_pow,false);

			cnt = 0;
		    }else if(get_IR(IR_F) < F_min){
			Smotor(+F_pow,true);
		       				
			cnt = 0;
		    }else {
			motor(0,0);
			cnt++;
		    }
		    if(cnt > F_cnt)break;
		}
		motor(0,0);
		
	    }else{
		if(((maze_w[y][x] & (1 << (4+ii))) != 0) && ( (maze_w[y][x] | (1 << ii)) != 0 ) ){//確定の場合 && 過去の記録が今回と値が異なる
						
		    //if(type == 3){
		    maze_w[y][x] &= ~(1 << (4+ii));//確定情報を消す
		    L_rotate(l90);//左回転
		    delay(20);
		    maze_update(x,y,my_angle, 2);//横壁のみチェック
		    R_rotate(r90);//右回転
		    delay(20);
		    //}
		}else{
		    maze_w[y][x] &= ~(1 << ii);
		}
	    }
	    maze_w[y][x] |= 1 << (4+ii);
	    //}
	    break;
        case 1://R
	    //if((maze_w[y][x] & (1 << (4+ii))) == 0 ){//未確定の場合
				 
	    if(get_IR(IR_R) > 40){
		if(((maze_w[y][x] & (1 << (4+ii))) != 0) && ( (maze_w[y][x] | (1 << ii)) == 0 ) ){//確定の場合 && 過去の記録が今回と値が異なる
							
		    //if(type == 3){
		    maze_w[y][x] &= ~(1 << (4+ii));//確定情報を消す
		    R_rotate(r90);//右回転
		    delay(20);
		    maze_update(x,y,my_angle, 1);//前壁のみチェック
		    L_rotate(l90);//左回転
		    delay(20);
		    //}	
		}else{
		    maze_w[y][x] |= 1 << ii; 
		}
	    }else{
		if(((maze_w[y][x] & (1 << (4+ii))) != 0) && ( (maze_w[y][x] | (1 << ii)) != 0 ) ){//確定の場合 && 過去の記録が今回と値が異なる
		    //if(type == 3){
		    maze_w[y][x] &= ~(1 << (4+ii));//確定情報を消す
		    R_rotate(r90);//右回転
		    delay(20);
		    maze_update(x,y,my_angle, 1);//前壁のみチェック
		    L_rotate(l90);//左回転
		    delay(20);
		    //}
		}else{
		    maze_w[y][x] &= ~(1 << ii);
		}
	    }
				
	    maze_w[y][x] |= 1 << (4+ii);
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

void L_curve_afterNaname(long long a,char flag){
    ETmotor_afterNaname(-a,rslsl90,flag);
 
    my_angle = (4+my_angle-1)%4;
}

void L_curveBIG(long long a,char flag){
    ETmotorBIG(-a,rslsl90_BIG,flag);
 
    my_angle = (4+my_angle-1)%4;
}

void L_curveU(long long a,char flag){
    ETmotorU(-a,usll180,flag);
 
    my_angle = (4+my_angle-2)%4;
}

void R_curve(long long a ,char flag ){
    ETmotor(a,rslsr90,flag);
  
    my_angle = (4+my_angle+1)%4;
}

void R_curve_afterNaname(long long a ,char flag ){
    ETmotor_afterNaname(a,rslsr90,flag);
  
    my_angle = (4+my_angle+1)%4;
}

void R_curveBIG(long long a ,char flag ){
    ETmotorBIG(a,rslsr90_BIG,flag);
  
    my_angle = (4+my_angle+1)%4;
}

void R_curveU(long long a ,char flag ){
    ETmotorU(a,uslr180,flag);
  
    my_angle = (4+my_angle+2)%4;
}

void L_rotate_naname(long long a, char inout){
    Tmotor_naname(-a,inout);
    my_angle = (4+my_angle-1)%4;
}

void R_rotate_naname(long long a,char inout){
    Tmotor_naname(a,inout);
    my_angle = (4+my_angle+1)%4;
}

void S_run(long long path,int powor, char non_stop,char kabe){
    //GyroSum_reset();
    ESmotor(path,powor,non_stop,kabe);
	
    int cnt2 = 0;

    if((non_stop == 0 || non_stop == 4) && (kabe == 1 || kabe == 4)){
	// GyroSum_reset();
	if(50 < get_IR(IR_F) ){//前壁補正
	    motor(0,0);
			
	    if(150 < get_IR(IR_F) ){//前壁　激突対策
		ESmotor(-15,powor,true,false);//ちょっと下がる
	    }
			
	    t_1ms = 0;
	    while(t_1ms < F_max_time){
		if(get_IR(IR_FL) > F_max || get_IR(IR_F) > F_max || get_IR(IR_FR) > F_max){
		    Smotor(-F_pow,false);

		    cnt2 = 0;
		}else if(get_IR(IR_F) < F_min){
		    Smotor(+F_pow,true);
       				
		    cnt2 = 0;
		}else {
		    motor(0,0);
		    cnt2++;
		}
		if(cnt2 > F_cnt)break;
	    }
	    motor(0,0);
	}
    }
	
    motor(0,0);
    //GyroSum_reset();
}

void S_run_kabe(int powor, char flag, int LR){//壁切れまで走行
    int Lflag = 0,Rflag = 0;
    //long long enc_base = (get_encoder_total_L() + get_encoder_total_R())/2;
    long long enc_base_L = get_encoder_total_L();
    long long enc_base_R = get_encoder_total_R();
    
    //   int cnt2 = 0;
	
    led(6);
    while(1){
	if(LR == 3 || LR == 1){//両方 || Lだけ
	    if(Lflag == 0){
		if(get_IR(IR_L) > 25){
		    Lflag = 1;
		    led(8);
		}
	    }else if(Lflag == 1){
		if(get_IR(IR_L) < 10){
		    led(0);
		    break;
		}
	    }
	}

	if(LR == 3 || LR == 2){//両方 || Rだけ
	    if(Rflag == 0){
		if(get_IR(IR_R) > 25){
		    Rflag = 1;
		    led(1);
		}
	    }else if(Rflag == 1){
		if(get_IR(IR_R) < 10){
		    led(0);
		    break;
		}
	    }
	}
    
    	Smotor(powor,flag);
	
	
	  if(Lflag == 0 && Rflag == 0){
		//if(abs((get_encoder_total_L() + get_encoder_total_R())/2 -  enc_base) > (s1 + h1/2) ){
		if(min(get_encoder_total_L()  - enc_base_L , get_encoder_total_R() - enc_base_R)  > (s1 + h1/2) ){
			led(9);
			break; //壁切れが来なかったらブレーク
		}
	  }
	
    }
  
    ESmotor(40,powor,true,false);//1cmくらい？
    led(0);
}

void S_run_kabe2(int powor, char flag, int LR){//壁切れまで走行 直線からの４５ターン
    int Lflag = 0,Rflag = 0;
    //long long enc_base = (get_encoder_total_L() + get_encoder_total_R())/2;
    long long enc_base_L = get_encoder_total_L();
    long long enc_base_R = get_encoder_total_R();
    
    led(6);
    while(1){
	if(LR == 3 || LR == 1){//両方 || Lだけ
	    if(Lflag == 0){
		if(get_IR(IR_L) > 25){
		    Lflag = 1;
		    led(8);
		}
	    }else if(Lflag == 1){
		if(get_IR(IR_L) < 10){
		    led(0);
		    break;
		}
	    }
	}

	if(LR == 3 || LR == 2){//両方 || Rだけ
	    if(Rflag == 0){
		if(get_IR(IR_R) > 25){
		    Rflag = 1;
		    led(1);
		}
	    }else if(Rflag == 1){
		if(get_IR(IR_R) < 10){
		    led(0);
		    break;
		}
	    }
	}
    
    	Smotor(powor,flag);
	
	if(Lflag == 0 && Rflag == 0){
		//if(abs((get_encoder_total_L() + get_encoder_total_R())/2 -  enc_base) > (s1+ h1 )){
		if(min(get_encoder_total_L()  - enc_base_L , get_encoder_total_R() - enc_base_R)  > (s1+ h1 ) ){
			led(9);
			break; //壁切れが来なかったらブレーク
		}
	}
   }
  
    ESmotor(40,powor,true,false);//　直線からの４５ターン 勢いがあるので不要
    led(0);
}

void S_run_kabe_BIG(int powor, char flag, int LR, int pathnum){//壁切れまで走行 斜めセンサー用
    int Lflag = 0,Rflag = 0;
    int Lflag2 = 0,Rflag2 = 0;
    //long long enc_base = (get_encoder_total_L() + get_encoder_total_R())/2;
    long long enc_base_L = get_encoder_total_L();
    long long enc_base_R = get_encoder_total_R();
    
    //   int cnt2 = 0;
	
    led(6);
    while(1){
	if(LR == 3 || LR == 1){//両方 || Lだけ
	    if(Lflag == 0){
		if(get_IR(IR_LT) > 28){// || (get_IR(IR_LT) > 14 && get_IR(IR_R) > 140)){
		    Lflag = 1;
		    led(8);
		}
		
		
		if(Lflag2 == 0){
			if(get_IR(IR_LT) > 16){
			    Lflag2 = 1;
			    led(4);
			}
		}else if(Lflag2 == 1){
			if(get_IR(IR_LT) < 4){
			    Lflag2 = 2;
			    led(0);
			    break;
			}
		}
	
	    }else if(Lflag == 1){
		/* if(get_IR(IR_R) > 140){
			if(get_IR(IR_LT) < 8){
			    led(0);
			    break;
			} 
		 }else{*/
			if(get_IR(IR_LT) < 15){
			    led(0);
			    break;
			}
		 //}
	    }
	    
	    
	}

	if(LR == 3 || LR == 2){//両方 || Rだけ
	    if(Rflag == 0){
		if(get_IR(IR_RT) > 28 ){// || (get_IR(IR_RT) > 15 && get_IR(IR_L) > 140)){
		    Rflag = 1;
		    led(1);
		}
		
		
		if(Rflag2 == 0){
			if(get_IR(IR_RT) > 16){
			    Rflag2 = 1;
			    led(2);
			}
		}else if(Rflag2 == 1){
			if(get_IR(IR_RT) < 4){
			    Rflag2 = 2;
			    led(0);
			    break;
			}
		}
		

	    }else if(Rflag == 1){
		/* if(get_IR(IR_L) > 140){
			if(get_IR(IR_RT) < 8){
			    led(0);
			    break;
			} 
		 }else{*/
			if(get_IR(IR_RT) < 15
			){
			    led(0);
			    break;
			}
		// }
	    }
	}
    
    	Smotor(powor,flag);
	
	
	  if(Lflag == 0 && Rflag == 0){
		//if(abs((get_encoder_total_L() + get_encoder_total_R())/2 -  enc_base) > (s1 + h1/2) ){
		
		if(pathnum <= 0){
			if(min(get_encoder_total_L()  - enc_base_L , get_encoder_total_R() - enc_base_R)  > (h1/4) ){
				led(9);
				break; //壁切れが来なかったらブレーク
			}
		}else{
			if(min(get_encoder_total_L()  - enc_base_L , get_encoder_total_R() - enc_base_R)  > (s1 + h1/2) ){
				led(9);
				break; //壁切れが来なかったらブレーク
			}
		}
	  }
	
    }
/*  
    if(get_IR(IR_L) > 20 && get_IR(IR_R) > 20){//左右に壁がある
    	if(abs(get_IR(IR_R) - get_IR(IR_L)) > 50 ){//左右の差が大きい
	     ESmotor(100,powor,true,false);//壁切れ位置のずれが大きいので補正する
    	}
    }
*/

    if( Lflag2 == 2 ||  Rflag2 == 2){
	  ESmotor(100,powor,true,false); 
    }
    led(0);
}

void S_run_kabe_naname(int powor, char flag, int LR){//壁切れまで走行
    int Lflag = 0,Rflag = 0;
    //long long enc_base = (get_encoder_total_L() + get_encoder_total_R())/2;
    long long enc_base_L = get_encoder_total_L();
    long long enc_base_R = get_encoder_total_R();
    int LMax = 0, RMax = 0;
  
    led(6);
  
    while(1){
	LMax = max(LMax,get_IR(IR_L));
	RMax = max(RMax,get_IR(IR_R));
	
	if(LR == 3 || LR == 1){//両方 || Lだけ
	    if(Lflag == 0){
		if(get_IR(IR_L) > 60){
		    led(8);
		    Lflag = 1;
		}
	    }else if(Lflag == 1){
			
		if(LMax -10 > get_IR(IR_L)){
		    led(0);
		    Lflag = 2;
		    break;
		}
		
	    }
	}

	if(LR == 3 || LR == 2){//両方 || Rだけ
	    if(Rflag == 0){
		if(get_IR(IR_R) > 60){
		    led(1);
		    Rflag = 1;
		}
	    }else if(Rflag == 1){
		if(RMax -10 > get_IR(IR_R)){
		    led(0);
		    Rflag = 2;
		    break;
		}
	
	    }
	}
    
    	Smotor(powor,flag);
	
	
	
	//if(abs((get_encoder_total_L() + get_encoder_total_R())/2 -  enc_base) > (s45 + s45/2 )  ){
	if(min(get_encoder_total_L()  - enc_base_L , get_encoder_total_R() - enc_base_R)  > (s45 + s45/2 )  ){
		led(9);
		break; //壁切れが来なかったらブレーク
	}
	
    }
 
    if(Lflag == 2 && get_IR(IR_L) > 70){//壁切れが遅かった場合
	ESmotor(250,powor,true,false);///222
	
    }else if(Rflag == 2 && get_IR(IR_R) > 70){//壁切れが遅かった場合
	ESmotor(250,powor,true,false);///222
	
    }else{
  	ESmotor(250,powor,true,false);///222
    }
    led(0);
}

void S_run_kabe_naname2(int powor, char flag, int LR, int v2_flag){//壁切れまで走行
    int Lflag = 0,Rflag = 0;
    //long long enc_base = (get_encoder_total_L() + get_encoder_total_R())/2;
    long long enc_base_L = get_encoder_total_L();
    long long enc_base_R = get_encoder_total_R();
    int LMax = 0, RMax = 0;

    led(6);
  
    while(1){

	LMax = max(LMax,get_IR(IR_LT));
	RMax = max(RMax,get_IR(IR_RT));
	
	if(LR == 3 || LR == 1){//両方 || Lだけ
	    if(Lflag == 0){
		if(get_IR(IR_LT) > 40){
		    led(8);
		    Lflag = 1;
		}
	    }else if(Lflag == 1){
			
		if(LMax -20 > get_IR(IR_LT)){
		    led(0);
		    Lflag = 2;
		    break;
		}
		
	    }
	}

	if(LR == 3 || LR == 2){//両方 || Rだけ
	    if(Rflag == 0){
		if(get_IR(IR_RT) > 40){
		    led(1);
		    Rflag = 1;
		}
	    }else if(Rflag == 1){
		if(RMax -20 > get_IR(IR_RT)){
		    led(0);
		    Rflag = 2;
		    break;
		}
	    }
	}
	
    	Smotor(powor,flag);
	
	
	if(v2_flag == 1 && Lflag == 0 && Rflag == 0){//２マスVターン直後は壁切れができない可能性が高いので距離を短めに設定する
		//if(abs((get_encoder_total_L() + get_encoder_total_R())/2 -  enc_base) > (s45_V2)  ){
		if(min(get_encoder_total_L()  - enc_base_L , get_encoder_total_R() - enc_base_R)  > (s45_V2)  ){
			led(9);
			break; //壁切れが来なかったらブレーク
		}
	}else if(v2_flag == 2 && Lflag == 0 && Rflag == 0){//２マス斜め(Vターンではない）直後は壁切れができない可能性が高いので距離を短めに設定する
		//if(abs((get_encoder_total_L() + get_encoder_total_R())/2 -  enc_base) > (s45_V2)  ){
		if(min(get_encoder_total_L()  - enc_base_L , get_encoder_total_R() - enc_base_R)  > (s45_V2not)  ){
			led(9);
			break; //壁切れが来なかったらブレーク
		}
	}else if(v2_flag == 4 && Lflag == 0 && Rflag == 0){//２マスVターン出るとき直後は壁切れができない可能性が高いので距離を短めに設定する
		
		if(min(get_encoder_total_L()  - enc_base_L , get_encoder_total_R() - enc_base_R)  > (s45_V2_out)  ){
			led(9);
			break; //壁切れが来なかったらブレーク
		}
	}else if(v2_flag == 5 && Lflag == 0 && Rflag == 0){//２マスVターン出るとき直後は壁切れができない可能性が高いので距離を短めに設定する
		
		if(min(get_encoder_total_L()  - enc_base_L , get_encoder_total_R() - enc_base_R)  > (s45_V2_out_LR)  ){
			led(9);
			break; //壁切れが来なかったらブレーク
		}
	}else{
		//if(abs((get_encoder_total_L() + get_encoder_total_R())/2 -  enc_base) > (s45 + s45/2 )  ){
		if(min(get_encoder_total_L()  - enc_base_L , get_encoder_total_R() - enc_base_R)  > (s45 + s45/2 )  ){
			led(9);
			break; //壁切れが来なかったらブレーク
		}
	}
	
    }
 
    if(v2_flag == 1){//２マスVターン開始
	ESmotor(140,powor,true,false); 
	
    }else if(v2_flag == 2){//２マス Vターンではない
    	ESmotor(330,powor,true,false);
	
    }else if(v2_flag == 3){//２マス以上のVターン開始
    	ESmotor(120,powor,true,false);
	
    }else if(v2_flag == 4){//２マスVターンでるとき
    	ESmotor(250,powor,true,false);
	
    }else if(v2_flag == 5){//２マスVターンでたあとカーブ
    	ESmotor(190,powor,true,false);
	
    }else{
    	ESmotor(250,powor,true,false);
    }
    led(0);
}

void S_run_maze_search(int path,int powor, int powor_up , int ir_up){
    Encoder_reset();
	
    int M_pwm_min = 6;
    int M_pwm = M_pwm_min;
    //long long enc_base = (get_encoder_total_L() + get_encoder_total_R())/2;
    long long enc_base_L = get_encoder_total_L();
    long long enc_base_R = get_encoder_total_R();
    long long enc_now = 0;
	
    int path_cnt = 0;
    int maza_update_flag = 0;
	
    int cnt2 = 0;
	
    int ir_L_now = 0,ir_R_now = 0;
    int ir_L_flag = 0,ir_R_flag = 0;
    int path_cnt_save_L = -1;//同じマスで壁切れ処理を２回以上しないように覚えておく変数
    int path_cnt_save_R = -1;//同じマスで壁切れ処理を２回以上しないように覚えておく変数
//    int hosei_kyori_L = -1,hosei_kyori_R = -1;//壁切れ時の補正距離　左右異なるタイミングで壁切れした際に利用する
    long long enc_kabe_L,enc_kabe_R;
    int led_num = 0;
//    int kame_hosei = 530;
	
    GyroSum_reset();
	
    while(1){
		
	if(enc_now >= (long long)path * s1){//目標距離に到達
		
	    //マスの中心まで移動(戻る）
	    while(enc_now - ((long long)s1 * path_cnt ) > s1){
		Smotor(-10,true);
		enc_now = min(get_encoder_total_L()  - enc_base_L , get_encoder_total_R() - enc_base_R);
	    }
			
	    if(get_IR(IR_F) > 30 ){//前壁があった場合は
	    	t_1ms = 0;
		while(t_1ms < F_max_time){//前壁補正
		    if(get_IR(IR_FL) > F_max || get_IR(IR_F) > F_max || get_IR(IR_FR) > F_max){
			Smotor(-F_pow,false);

			cnt2 = 0;
		    }else if(get_IR(IR_F) < F_min){
			Smotor(+F_pow,true);
	       
			cnt2 = 0;
		    }else {
			motor(0,0);
			cnt2++;
		    }
		    if(cnt2 > F_cnt)break;
		}
		motor(0,0);
	    }
			
	    //現在地の更新
	    my_x += dx[my_angle];
	    my_y += dy[my_angle];
			
	    maze_update(my_x,my_y,my_angle,3);//迷路情報の更新
			
	    break;
	}
		
	if( path_cnt < path-1 && get_IR(IR_F) > 100){//目標まで１マス以上残ってる　＆＆　前壁が出現
		
	    //マスの中心まで移動
	  /*  while(enc_now - ((long long)s1 * path_cnt ) < s1 && get_IR(IR_F) < F_min){
		Smotor(+10,true);
		enc_now = min(get_encoder_total_L()  - enc_base_L , get_encoder_total_R() - enc_base_R);
				
		if(get_IR(IR_F) > F_min){
		    motor(0,0);
		    break;
		}
	    }
	*/	
	    t_1ms = 0;
	    while(t_1ms < F_max_time){//前壁補正
		if(get_IR(IR_FL) > F_max || get_IR(IR_F) > F_max || get_IR(IR_FR) > F_max){
		    Smotor(-F_pow,false);

		    cnt2 = 0;
		}else if(get_IR(IR_F) < F_min){
		    Smotor(+F_pow,true);
       
		    cnt2 = 0;
		}else {
		    motor(0,0);
		    cnt2++;
		}
		if(cnt2 > F_cnt)break;
	    }
	    motor(0,0);	
			
	    //現在地の更新
	    my_x += dx[my_angle];
	    my_y += dy[my_angle];
			
	    maze_update(my_x,my_y,my_angle,3);//迷路情報の更新
	    maza_update_flag = 0;
			
	    break;
	}
	
	if( path_cnt < path-1){//目標まで１マス以上残ってる メモ：最後の１マスは前壁補正後に迷路情報を更新するため
	    if(enc_now - ((long long)s1 * path_cnt ) > s1){//１マス進んだ
			
		led_num = 0;
		led(led_num);
				
		//if(maza_update_flag != 2){//なぜか壁の更新ができていなければ
		    maze_update(my_x + dx[my_angle],my_y + dy[my_angle],my_angle,3);//迷路情報の更新
		//}
		//現在地の更新
		my_x += dx[my_angle];
		my_y += dy[my_angle];
			
		path_cnt++;
		maza_update_flag = 0;
		ir_L_flag = 0;
		ir_R_flag = 0;
	    }
	}	

	/*
	if(maza_update_flag == 0){//まだ横壁の更新をしていなければ
	    if(enc_now - ((long long)s1 * path_cnt ) > s1 - 100){//マスの中心ではなく少し手前で壁をチェックする メモ：横壁センサーが少し斜め前を向いているため
			
		maze_update(my_x + dx[my_angle],my_y + dy[my_angle],my_angle,2);//迷路情報の更新
		maza_update_flag = 1;
	    }
	}
		
	if(maza_update_flag == 1){//まだ前壁の更新をしていなければ
	    if(enc_now - ((long long)s1 * path_cnt ) > s1 - 10){//マスの中心ではなく少し手前で壁をチェックする 
			
		maze_update(my_x + dx[my_angle],my_y + dy[my_angle],my_angle,1);//迷路情報の更新
		maza_update_flag = 2;
	    }
	}
*/		
		
	if(enc_now < (long long)path * s1 /4){// 進んだ距離 < 目標距離 * 1/4　＝ 加速区間
	    M_pwm = M_pwm_min + (enc_now / 8);	
			
	}else if(enc_now > (long long)path * s1 * 3/4){// 進んだ距離 < 目標距離 * 3/4 = //減速区間
	    M_pwm = M_pwm_min + ( ((long long)path * s1 - enc_now) / 8);	
			
	}else{
	    if((get_IR(IR_F) < ir_up) && (path_cnt < path-1)){//前壁が確実になければ速度上げる && 目標まで１マス以上残ってる
	    	M_pwm = powor_up;
	    }else{
		M_pwm = powor;   
	    }
	}

	if((get_IR(IR_F) < ir_up) && (path_cnt < path-1)){//前壁が確実になければ速度上げる && 目標まで1マス以上残ってる
	    if(M_pwm > powor_up)M_pwm = powor_up;
	}else{
	    if(M_pwm > powor)M_pwm = powor;
	}
	if(M_pwm < M_pwm_min)M_pwm = M_pwm_min;
		
	if( path_cnt < path-1){//目標まで１マス以上残ってる
	    Smotor(M_pwm,4);// w_flag = 4 串の壁補正あり
	}else{
	    Smotor(M_pwm,true);
	}
		
	//Smotor(M_pwm,true);
		
		
	//壁切れの距離補正
	ir_L_now = get_IR(IR_L);
	ir_R_now = get_IR(IR_R);
	if(path_cnt_save_L !=  path_cnt){//現在のマスで壁切れ処理を実行していなければ
		
	    if(ir_L_flag == 0 && ir_L_now > 30 && ir_R_now < 160){//左壁がある　&& 右壁に近すぎない
		ir_L_flag = 1;
				
		led_num |= 8;
		led(led_num);
	    }else if(ir_L_flag == 1 && ir_L_now < 10 && ir_R_now < 160){//左壁がない　&& 右壁に近すぎない
		if((enc_now % s1) < s1 / 2){//マスの半分より手前で壁切れした場合
				
		    led_num &= ~8;
		    led(led_num);
		    if(path_cnt == path_cnt_save_R){//左より先に右が壁切れ補正していた場合
			
/*			if(Get_motor_pid_mode() == 0){//探索モード
			    enc_base_L -= hosei_kyori_R;//右での補正を無かったことにする
			    enc_base_R -= hosei_kyori_R;//右での補正を無かったことにする
			    enc_now = min(get_encoder_total_L()  - enc_base_L , get_encoder_total_R() - enc_base_R);
						
			    hosei_kyori_L = (enc_now % s1) - kame_hosei;
						
			    hosei_kyori_L = (hosei_kyori_L + hosei_kyori_R) / 2;//左右の平均値を使用する
			}
*/								
			//壁切れタイミングの違いで角度補正
			enc_kabe_L = min(get_encoder_total_L()  - enc_base_L , get_encoder_total_R() - enc_base_R);
			if(abs( (enc_kabe_L - enc_kabe_R) ) < 500){
			    GyroSum_add( (enc_kabe_L - enc_kabe_R) * 10);
			}
		    }else{
/*			if(Get_motor_pid_mode() == 0){//探索モード
			    hosei_kyori_L = (enc_now % s1) - kame_hosei;
			}
*/
			enc_kabe_L = min(get_encoder_total_L()  - enc_base_L , get_encoder_total_R() - enc_base_R);
		    }
					
/*		    if(Get_motor_pid_mode() == 0){//探索モード
			enc_base_L += hosei_kyori_L;
			enc_base_R += hosei_kyori_L;
		    }
*/					
		}
		ir_L_flag = 0;
		path_cnt_save_L = path_cnt;
	    }
	}
		
	if(path_cnt_save_R !=  path_cnt){//現在のマスで壁切れ処理を実行していなければ
			
	    if(ir_R_flag == 0 && ir_R_now > 30 && ir_L_now < 160){//右壁がある　&& 左壁に近すぎない
				
		ir_R_flag = 1;
				
		led_num |= 1;
		led(led_num);
				
	    }else if(ir_R_flag == 1 && ir_R_now < 10 && ir_L_now < 160){//右壁がない　&& 左壁に近すぎない
		if((enc_now % s1) < s1 / 2){//マスの半分より手前で壁切れした場合
				
		    led_num &= ~1;
		    led(led_num);
					
		    if(path_cnt == path_cnt_save_L){//右より先に左が壁切れ補正していた場合
/*			if(Get_motor_pid_mode() == 0){//探索モード
			    enc_base_L -= hosei_kyori_L;//左での補正を無かったことにする
			    enc_base_R -= hosei_kyori_L;//左での補正を無かったことにする
			    enc_now = min(get_encoder_total_L()  - enc_base_L , get_encoder_total_R() - enc_base_R);
			    hosei_kyori_R = (enc_now % s1) - kame_hosei;
						
			    hosei_kyori_R = (hosei_kyori_L + hosei_kyori_R) / 2;//左右の平均値を使用する
			}
*/
			//壁切れタイミングの違いで角度補正
			enc_kabe_R = min(get_encoder_total_L()  - enc_base_L , get_encoder_total_R() - enc_base_R);
			if(abs( (enc_kabe_L - enc_kabe_R) ) < 500){
			    GyroSum_add( (enc_kabe_L - enc_kabe_R) * 10);
			}
		    }else{
/*			if(Get_motor_pid_mode() == 0){//探索モード
			    hosei_kyori_R = (enc_now % s1) - kame_hosei;
			}
*/
			enc_kabe_R = min(get_encoder_total_L()  - enc_base_L , get_encoder_total_R() - enc_base_R);
		    }
					
/*		    if(Get_motor_pid_mode() == 0){//探索モード
			enc_base_L += hosei_kyori_R;
			enc_base_R += hosei_kyori_R;
		    }
*/					
		}
		ir_R_flag = 0;
		path_cnt_save_R = path_cnt;
	    }
	}
		
	//enc_now = (get_encoder_total_L() + get_encoder_total_R())/2 - enc_base;
	enc_now = min(get_encoder_total_L()  - enc_base_L , get_encoder_total_R() - enc_base_R);
    }
	
    led(0);
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
			if(maze_d[ny][nx][k] > num+1 + get_r_cost()*2){
			    update_flag = true;
			    maze_d[ny][nx][k] = num+1 + get_r_cost()*2;
			}
		    }else{// L or R
			if(maze_d[ny][nx][k] > num+1 + get_r_cost()){
			    update_flag = true;
			    maze_d[ny][nx][k] = num+1 + get_r_cost();
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
    int last = 0;
	
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
          			
		    }else if(num == next){// LとRが同じ重み　斜めを優先したい
			if(last == -1){//前回がLなら今回はR
			    n_num = (angle-1+4)%4;
				 
			}else if(last == 1){//前回がRなら今回はL
			    n_num = (angle+1+4)%4;
				 
			}else{//前回がSなら今回は?
			    //わからんから先に見つかった方にする
			}
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
	    last = -1;
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
				
	    //last = 0; 
	    break;
				
	case 10://S 未確定の直線
				
	    s_path +=1;
	    x += dx[n_num];
	    y += dy[n_num];
				
	    mas_cnt++;
	    //last = 0;
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
	    last = 1;
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
	    //last = 2;
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
  
    int cnt = 0;
	
    short comand ,path_num;
    int time = 1;
    
    int run_speed = 30;
    int run_speed_up = 40;    //未知区間加速
    int run_speed_boost = 50; //既知区間加速
    
    int run_speed_kabe = 30;

  
    while(!queue_empty()){
	comand = dequeue();path_num = dequeue();
	status_log = comand;
	
	switch(comand){
	
	case -1://L
	    delay(time);
		
	    L_rotate(l90);
		
	    delay(time);
	    break;
	case 0://S
	    if(queue_empty()){
			
		if(path_num == 1){
		    S_run(s1,run_speed + run_fin_speed_offset,false,true);
		    //S_run(s1,run_speed + run_fin_speed_offset,false,4);// w_flag = 4 串の壁補正あり
		}else{
		    //S_run(s1 * (long long)path_num,run_speed_boost + run_fin_speed_offset,false,true);
		    S_run(s1 * (long long)path_num,run_speed_boost + run_fin_speed_offset,false,4);// w_flag = 4 串の壁補正あり
		}
			
	    }else{
	        if(path_num == 1){
		  			
		    if(queue_next(1) < 0){//次　左
			S_run_kabe(run_speed_kabe + run_fin_speed_offset,true,1);
			//S_run_kabe(run_speed_kabe + run_fin_speed_offset,4,1);//non_stop = 4
							
		    }else if(queue_next(1) > 0){//次　右
			S_run_kabe(run_speed_kabe + run_fin_speed_offset,true,2);
			//S_run_kabe(run_speed_kabe + run_fin_speed_offset,4,2);//non_stop = 4
		    }else{
			S_run_kabe(run_speed + run_fin_speed_offset,true,3);
			//S_run_kabe(run_speed_kabe + run_fin_speed_offset,4,3);// w_flag = 4 串の壁補正あり
		    }
						
		    S_run(h1_2,run_speed + run_fin_speed_offset,4,true);//non_stop = 4
		    //S_run(h1,run_speed + run_fin_speed_offset,4,4);//non_stop = 4 // w_flag = 4 串の壁補正あり
					
		}else{
		
		 //   Set_motor_pid_mode(1);//高速
		    
		    //S_run(s1 * ((long long)path_num - 1),run_speed_boost + run_fin_speed_offset,3,true);//non_stop = 3
		    S_run(s1 * ((long long)path_num - 1),run_speed_boost + run_fin_speed_offset,3,4);//non_stop = 3 // w_flag = 4 串の壁補正あり
					
		    if(queue_next(1) < 0){//次　左
			S_run_kabe(run_speed_kabe + run_fin_speed_offset,true,1);
			//S_run_kabe(run_speed_kabe + run_fin_speed_offset,4,1);// w_flag = 4 串の壁補正あり
						
		    }else if(queue_next(1) > 0){//次　右
			S_run_kabe(run_speed_kabe + run_fin_speed_offset,true,2);
			//S_run_kabe(run_speed_kabe + run_fin_speed_offset,4,2);// w_flag = 4 串の壁補正あり
		    }else{
			S_run_kabe(run_speed_kabe + run_fin_speed_offset,true,3);
			//S_run_kabe(run_speed_kabe + run_fin_speed_offset,4,3);// w_flag = 4 串の壁補正あり
		    }
				
		    S_run(h1_2,run_speed + run_fin_speed_offset,4,true);//non_stop = 4
		    //S_run(h1,run_speed + run_fin_speed_offset,4,4);//non_stop = 4 // w_flag = 4 串の壁補正あり
		    
		//    Set_motor_pid_mode(0);//低速
		}
	    }
		
	    if(get_IR(IR_F) > 40){
		cnt= 0;
		t_1ms = 0;
		while(t_1ms < F_max_time){//前壁補正
		    if(get_IR(IR_FL) > F_max || get_IR(IR_F) > F_max || get_IR(IR_FR) > F_max){
			Smotor(-F_pow,false);

			cnt = 0;
		    }else if(get_IR(IR_F) < F_min){
			Smotor(+F_pow,true);
		       				
			cnt = 0;
		    }else {
			motor(0,0);
			cnt++;
		    }
		    if(cnt > F_cnt)break;
		}
		motor(0,0);
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
	  		
		S_run_maze_search(path_num,run_speed + run_fin_speed_offset,run_speed_up + run_fin_speed_offset,  6);//串のあり、なしは関数内で設定する
			
	    }else{
		S_run_maze_search(path_num,run_speed + run_fin_speed_offset,run_speed_up + run_fin_speed_offset ,  6);
	    }
		
	    if(get_IR(IR_F) > 40){
		cnt= 0;
		t_1ms = 0;
		while(t_1ms < F_max_time){//前壁補正
		    if(get_IR(IR_FL) > F_max || get_IR(IR_F) > F_max || get_IR(IR_FR) > F_max){
			Smotor(-F_pow,false);

			cnt = 0;
		    }else if(get_IR(IR_F) < F_min){
			Smotor(+F_pow,true);
		       				
			cnt = 0;
		    }else {
			motor(0,0);
			cnt++;
		    }
		    if(cnt > F_cnt)break;
		}
		motor(0,0);
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
    status_log = 99;
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
	    motor(0,0);
	    led_up();
			
	    if(target_x == Start_x && target_y == Start_y){
				
		GyroSum_reset();
				
		t_1ms = 0;
		while(t_1ms < F_max_time || get_IR(IR_FL) < F_min){//スタートの奥まで進む 前壁補正
		    if(get_IR(IR_FL) > F_max || get_IR(IR_F) > F_max || get_IR(IR_FR) > F_max){
			Smotor(-F_pow,false);

			cnt = 0;
		    }else if(get_IR(IR_F) < F_min){
			Smotor(+F_pow,true);
	       				
			cnt = 0;
		    }else {
			motor(0,0);
			cnt++;
		    }
		    if(cnt > F_cnt)break;
		}
		motor(0,0);
		Tmotor(r180);
		my_angle = (4+my_angle+2)%4;
	    }
	    break;
	}
		
	shortest_path_search(target_x,target_y);
	make_shortest_path_list(target_x,target_y);
	run_shortest_path();
	motor(0,0);
    }
    motor(0,0);
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
    int last = 0;
	
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
		    }else if(num == next){// LとRが同じ重み　斜めを優先したい
			if(last == -1){//前回がLなら今回はR
			    n_num = (angle-1+4)%4;
							 
			}else if(last == 1){//前回がRなら今回はL
			    n_num = (angle+1+4)%4;
							 
			}else{//前回がSなら今回は?
			    //わからんから先に見つかった方にする
			}
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
		     	
	    last = -1;
	    break;
	case 0://S
				
	    s_path +=1;
	    x += dx[n_num];
	    y += dy[n_num];
				
	    //last = 0;
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
		     
	    last = 1;
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
				
	    //last = 2;
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
void maze_search_unknown(short* target_x,short* target_y, int Goal_Start){
 
    short x = Start_x,y = Start_y,angle = Start_angle;
    int last = 0;
    
    short target_x_buf,target_y_buf,angle_buf = 99;
    short Goal_x_tmp,Goal_y_tmp;
    
    /*
    if(Goal_Start == 0){
	Goal_x_tmp = Goal_x;
    	Goal_y_tmp = Goal_y;
    }else{
	Goal_x_tmp = Start_x;
    	Goal_y_tmp = Start_y; 
	angle = (Goal_angle + 4 +2 )%4;
	x = Goal_x;
	y = Goal_y;
    }
    */
    Goal_x_tmp = Goal_x;
    Goal_y_tmp = Goal_y;
    
   // *target_x = 99;
   // *target_y = 99;
	    
    while(x != Goal_x_tmp || y != Goal_y_tmp){
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
		    }else if(num == next){// LとRが同じ重み　斜めを優先したい
			if(last == -1){//前回がLなら今回はR
			    n_num = (angle-1+4)%4;
							 
			}else if(last == 1){//前回がRなら今回はL
			    n_num = (angle+1+4)%4;
							 
			}else{//前回がSなら今回は?
			    //わからんから先に見つかった方にする
			}
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
	    /*
	    if(Goal_Start == 0){
	  	return;
	    }
	    */
	    /*
	    if(angle_buf == 99 || angle_buf == ni){//連続する直線は遠い座標を目標値とする
	    	angle_buf = ni;
	    	target_x_buf = x;
	    	target_y_buf = y;
	    }else{
		*target_x = target_x_buf;
	    	*target_y = target_x_buf;
	        return;  
	    }
	    */
	    	
	}
	
    	switch(ni){
	case -1://L
	    angle = (4+angle-1)%4;
				
	    last = -1;
	    break;
	case 0://S
   
	    x += dx[n_num];
	    y += dy[n_num];
				
	    //last = 0;
	    break;
				
	case 1://R
	    angle = (4+angle+1)%4;
			
	    last = 1;
	    break;
	case 2://B 不要のはず
        	
	    angle = (4+angle+2)%4;
				
	    //last = 2;
	    break;
    	}
    }
	
    //未確定マスを通らずにゴールまで経路を確認できた。
   // if(*target_x == 99 && *target_y == 99){
    	*target_x = Goal_x;
    	*target_y = Goal_y;
   // }
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
    int Goal_Start = 0;//0:ゴールに近いほうから 1:スタートに近いほうから
    
    time_limit = 60000;//60秒
	
    while(time_limit > 0){//制限時間の間走行可能
	
	maze_update(my_x,my_y,my_angle,3);
	
	shortest_path_search(Goal_x,Goal_y);
	//shortest_path_search(Start_x,Start_y);
	
	maze_search_unknown(&target_x,&target_y, Goal_Start);//最短経路上の未確定マスの座標を取得
	
	//Goal_Start ++;
	//Goal_Start %= 2;
	
	if(target_x == Goal_x && target_y == Goal_y){//最短経路上に未確定マスがなければ終了
	    motor(0,0);
				
	    maze_search_adachi(Start_x,Start_y);
	    led_down();
	    led_up();
	    led_down();
	    led_up();
			
	    motor(0,0);
	    return;
	}
	
    	shortest_path_search(target_x,target_y);
	make_shortest_path_list(target_x,target_y); //未確定マスでも連続する直線なら進む
    	//make_shortest_path_list_simple(target_x,target_y); //未確定マスでとまる
	run_shortest_path();
  	
	motor(0,0);
	/*	if(my_x == Goal_x && my_y == Goal_y){//最短経路上に未確定マスがなければ終了
		led_down();
		led_up();
		break;
		}
	*/
    }
	
    if(time_limit <= 0){//　制限時間内に探索できなかった　ゴールまで向かう
		
	//maze_search_adachi(Goal_x,Goal_y);
	maze_search_adachi(Start_x,Start_y);
			
	shortest_path_search(Goal_x,Goal_y);
	maze_search_unknown(&target_x,&target_y,0);//最短経路上の未確定マスの座標を取得
			
	if(target_x == Goal_x && target_y == Goal_y){//最短経路上に未確定マスがなければ
	    led_down();
	    led_up();
	    led_down();
	    led_up();
	}else{
	    led(15);
	    delay(500);
	    led(0);
	    delay(500);
	    led(15);
	    delay(500);
	}
    }
}

/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */
/* 関 数 概 要：最短経路が存在するか確認する											  			            */
/* 関 数 詳 細：												                                   */
/* 引       数： なし														    */
/* 戻  り   値： なし										    									*/
/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */ 
char shortest_path_search_check(){
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
			if(maze_d[ny][nx][k] > num+1 + get_r_cost()*2){
			    update_flag = true;
			    maze_d[ny][nx][k] = num+1 + get_r_cost()*2;
			}
		    }else{// L or R
			if(maze_d[ny][nx][k] > num+1 + get_r_cost()){
			    update_flag = true;
			    maze_d[ny][nx][k] = num+1 + get_r_cost();
			}
		    }
		}
		if(update_flag)enqueue(nx*100 + ny);
	    }
	}
    }
    
    char ng_flag = 1;
    for(int k = 0; k < 4; k++){
    	if(maze_d[Start_y][Start_x][k] != maze_d_max ){//スタート位置の重みが更新されてなかったら＝最短経路が存在しない
    		ng_flag = 0;
    	}
    }
    if(ng_flag == 1){
	return 1;    
    }
    
    return 0;
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
			if(maze_d[ny][nx][k] > num+1 + get_r_cost()*2){
			    update_flag = true;
			    maze_d[ny][nx][k] = num+1 + get_r_cost()*2;
			}
		    }else{// L or R
			if(maze_d[ny][nx][k] > num+1 + get_r_cost()){
			    update_flag = true;
			    maze_d[ny][nx][k] = num+1 + get_r_cost();
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
 
    int last = 0;
	
    while(my_x != Goal_x || my_y != Goal_y){

	short num = maze_d[my_y][my_x][(my_angle+2)%4];
	short n_num = 0;
	char s_flag = 0;
	int nx = my_x+dx[my_angle],ny = my_y+dy[my_angle];

	/*
	  for(int i = 0;i < 4;i++){
	  if(i == (my_angle+2)%4){//逆走はありえない
		
	  }else if(i == my_angle){//直線はあとから比較する
		
	  }else{// L or R
	  if( ((maze_w[my_y][my_x] & (1<<((i+2)%4))) == 0 )  && ((maze_w[my_y][my_x] & (1<<(4+((i+2)%4)))) != 0 )){//壁が無い　＆　確定している
	  short next = maze_d[my_y][my_x][i];
	  if(num > next){
	  n_num = i;
	  num = next;
	  }
	  }
	  }
	  }
	
	  if((0 <= nx && nx < W) && (0 <= ny && ny < H) && ((maze_w[my_y][my_x] & (1<<((my_angle+2)%4))) == 0 )  && ((maze_w[my_y][my_x] & (1<<(4+((my_angle+2)%4)))) != 0 ) ){
	  if(num > maze_d[my_y][my_x][my_angle]){//直線
	  n_num = my_angle;
	  num = maze_d[my_y][my_x][my_angle];
	  }
	  }
	*/
	
	
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
		if(i == (my_angle+2)%4){//逆走はありえない
		}else{// L or R
		    short next = maze_d[my_y][my_x][i];
		    if(num > next){
			n_num = i;
			num = next;
		    }else if(num == next){// LとRが同じ重み　斜めを優先したい
			if(last == -1){//前回がLなら今回はR
			    n_num = (my_angle-1+4)%4;
				 
			}else if(last == 1){//前回がRなら今回はL
			    n_num = (my_angle+1+4)%4;
				 
			}else{//前回がSなら今回は?
			    //わからんから先に見つかった方にする
			}
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
		
	    last = -1;
	    break;
	case 0://S
   
	    h_path +=2;
	    my_x += dx[n_num];
	    my_y += dy[n_num];
		
	    //last = 0;
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
		
	    last = 1;
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
  
    ////////////////////////////////////
    /*   for(int i = 0; i < H;i++){
	 for(int j = 0;j < W; j++){
	 int temp = maze_d_max;
	 for(int k = 0; k < 4; k++){
	 if(temp > maze_d[i][j][k]){
	 temp = 	maze_d[i][j][k];
	 }
	 }
	 if(temp != maze_d_max)printf2("%d\t",temp);
	 else printf2("%d\t",-1);
	 }
	 printf2("\n");
	 }*/
    //////////////////////////////////

}

/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */
/* 関 数 概 要：最短経路探索（最終版）斜めも考慮して経路選択する					*/
/* 関 数 詳 細：												                                   */
/* 引       数： なし														    */
/* 戻  り   値： なし										    									*/
/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */ 
void shortest_path_search_perfect(){

    int comand ,path_num;

    led(0);
    ////////////// ゴールからの距離を計算する
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
			if(maze_d[ny][nx][k] > num+1 + get_r_cost()*2){
			    update_flag = true;
			    maze_d[ny][nx][k] = num+1 + get_r_cost()*2;
			}
		    }else{// L or R
			if(maze_d[ny][nx][k] > num+1 + get_r_cost()){
			    update_flag = true;
			    maze_d[ny][nx][k] = num+1 + get_r_cost();
			}
		    }
		}
		if(update_flag)enqueue(nx*100 + ny);
	    }
	}
    }
    
   
    //////////////////////////////////////////////////
    for(int i = 0; i < H;i++){
	for(int j = 0;j < W; j++){
	    maze_d_perfect[i][j] = maze_d_max;
	}
    }
    
   
    for(int i = 0; i < H;i++){//全マスからゴールまでの走行経路を算出する
	for(int j = 0;j < W; j++){
	   led(i);
	  // printf2("%d %d ",i,j);
	    
	   if(i == Goal_y && j == Goal_x){
	   	maze_d_perfect[Goal_y][Goal_x] = 0; 
		
	   }else if (maze_d[i][j][0] == maze_d_max){//到達不能マス
	   	maze_d_perfect[i][j] = maze_d_max;
		   
	   }else{//走行経路を算出する
	   	
	   	 //run_list
    		queue_reset();
    		short h_path = 0;
    		my_x = j;my_y = i;my_angle = 0;//スタート位置の設定
		
		for(int k = 1;k < 4;k++){//スタート向きの設定
			if(maze_d[i][j][my_angle] > maze_d[i][j][k]){
				my_angle = k;
			}
		}
		my_angle = (my_angle+2)%4;
		//printf2("%d ",my_angle);
		int last = 0;
	
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
				if(i == (my_angle+2)%4){//逆走はありえない
				}else{// L or R
				    short next = maze_d[my_y][my_x][i];
				    if(num > next){
					n_num = i;
					num = next;
				    }else if(num == next){// LとRが同じ重み　斜めを優先したい
					if(last == -1){//前回がLなら今回はR
					    n_num = (my_angle-1+4)%4;
						 
					}else if(last == 1){//前回がRなら今回はL
					    n_num = (my_angle+1+4)%4;
						 
					}else{//前回がSなら今回は?
					    //わからんから先に見つかった方にする
					}
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
				
			    last = -1;
			    break;
			case 0://S
		   
			    h_path +=2;
			    my_x += dx[n_num];
			    my_y += dy[n_num];
				
			    //last = 0;
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
				
			    last = 1;
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
		    
		    
		 //ここまでで走行経路が算出完了 
		 remake_shortest_path_list_naname2(); //２マスも斜めにするモード
	   	 path_compression();//大曲など  
		 
		 //走行経路から距離に変換
		 maze_d_perfect[i][j] = 0;
		 while(!queue_empty()){
			comand = dequeue();path_num = dequeue();
			//printf2("%d %d\n",comand,path_num);
			//delay(1)
			
			if(comand == 0){//直線
				maze_d_perfect[i][j] += (path_num+1) / 2;
				
			}else if(comand == 10){//斜め直線
				maze_d_perfect[i][j] += path_num;  //斜め１マス
				
			}else if(comand == -11 || comand == -13 || comand == -14 || comand == 11 || comand == 13 || comand == 14){//斜め45
				maze_d_perfect[i][j] += get_r45_cost();  //１マス
				
			}else{//カーブ
				maze_d_perfect[i][j] += path_num * get_r_cost();
			}
			
		} 
	    }
	   
	   // printf2("OK \n");
	}
    }
    /*
     for(int i = 0; i < H;i++){
	for(int j = 0;j < W; j++){
		if(maze_d_perfect[i][j] == maze_d_max)printf2("xxx");
		else printf2("%3d",maze_d_perfect[i][j]);
		delay(1);
	}
	printf2("\n");
     }
     while(1);
     */
    ///////////////////////////////////////////////
    
    //run_list
    queue_reset();
    short h_path = 0;
    my_x = Start_x;my_y = Start_y;my_angle = Start_angle;
 
    int last = 0;
	
    while(my_x != Goal_x || my_y != Goal_y){

	short num = maze_d_perfect[my_y][my_x];
	short n_num = 0;
	
 
	for(int i = 0;i < 4;i++){//ゴールに近いマスを探す
		int nx = my_x+dx[i],ny = my_y+dy[i];
		
		//迷路の範囲内　＆＆　壁が無いことが確定している
		if((0 <= nx && nx < W) && (0 <= ny && ny < H) && ((maze_w[my_y][my_x] & (1<<i)) == 0 )  && ((maze_w[my_y][my_x] & (1<<(4+i))) != 0 ) ){
			
			if(num > maze_d_perfect[ny][nx]){//ゴールに近いマスを見つけた
				num = maze_d_perfect[ny][nx];
				n_num = i;
				
			 }else if(num == maze_d_perfect[ny][nx]){// LとRが同じ重み　斜めを優先したい
			 
				if(last == -1 && (  ((4 + i - ((4+my_angle-1)%4))%4)) == 1   ){//前回がL かつ　今回はR  
					n_num = i;
						 
				}else if(last == 1 && (  ((4 + i - ((4+my_angle-1)%4))%4)) == -1   ){//前回がR　かつ　今回はL
					n_num = i;
						 
				}else{//前回がSなら今回は?
					//わからんから先に見つかった方にする
				}
			}
		}
	}
	
   	//移動する
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
		
	    last = -1;
	    break;
	case 0://S
   
	    h_path +=2;
	    my_x += dx[n_num];
	    my_y += dy[n_num];
		
	    //last = 0;
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
		
	    last = 1;
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
  
    ////////////////////////////////////
    /*   for(int i = 0; i < H;i++){
	 for(int j = 0;j < W; j++){
	 int temp = maze_d_max;
	 for(int k = 0; k < 4; k++){
	 if(temp > maze_d[i][j][k]){
	 temp = 	maze_d[i][j][k];
	 }
	 }
	 if(temp != maze_d_max)printf2("%d\t",temp);
	 else printf2("%d\t",-1);
	 }
	 printf2("\n");
	 }*/
    //////////////////////////////////
}

/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */
/* 関 数 概 要：最短経路に斜め有効化										  			            */
/* 関 数 詳 細：												                                   */
/* 引       数： なし													    */
/* 戻  り   値： なし										    									*/
/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */ 
void remake_shortest_path_list_naname2(){
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
	    }else if(naname_cnt == 1 && lr == -1 && queue_next(1) == 0){//Uターン
		enqueue(-1);
		enqueue(1);
				
		enqueue(-1);
		enqueue(1);
				
		naname_cnt = 0;
		lr = 0;
	    }else if(naname_cnt == 1 && lr == -1 && queue_next(1) == 1){//L L R //１つ目はスラロームにする
		enqueue(-1);
		enqueue(1);
							
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
				
		    if(queue_next(1) == 0){//L L (S) //スラロームにする
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
							
	    }else if(naname_cnt > 1){
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
	    }else if(naname_cnt == 1 && lr == 1 && queue_next(1) == 0){//Uターン
		enqueue(1);
		enqueue(1);
				
		enqueue(1);
		enqueue(1);
				
		naname_cnt = 0;
		lr = 0;
	    }else if(naname_cnt == 1 && lr == 1 && queue_next(1) == -1){//R R L //１つ目はスラロームにする
		enqueue(1);
		enqueue(1);
				
			
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
				
		    if(queue_next(1) == 0){//R R (S) //スラロームにする
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
	    }else if(naname_cnt == 1 && lr == -1 && queue_next(1) == 0){//Uターン
		enqueue(-1);
		enqueue(1);
				
		enqueue(-1);
		enqueue(1);
				
		naname_cnt = 0;
		lr = 0;
	    }else if(naname_cnt == 1 && lr == -1 && queue_next(1) == 1){//L L R //１つ目はスラロームにする
		enqueue(-1);
		enqueue(1);
				
	    }else if(naname_cnt == 1 && lr == 1 && queue_next(1) != 1){//R L * //スラロームにする
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
				
		    if(queue_next(1) == 0){//L L (S) //スラロームにする
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
	    }else if(naname_cnt == 1 && lr == 1 && queue_next(1) == 0){//Uターン
		enqueue(1);
		enqueue(1);
				
		enqueue(1);
		enqueue(1);
				
		naname_cnt = 0;
		lr = 0;
	    }else if(naname_cnt == 1 && lr == 1 && queue_next(1) == -1){//R R L //１つ目はスラロームにする
		enqueue(1);
		enqueue(1);
				
	    }else if(naname_cnt == 1 && lr == -1 && queue_next(1) != -1){//L R * //スラロームにする
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
				
		    if(queue_next(1) == 0){//R R (S) //スラロームにする
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
/* 関 数 概 要：最短経路の圧縮											  			            */
/* 関 数 詳 細：												                                   */
/* 引       数： なし														    */
/* 戻  り   値： なし										    									*/
/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */ 
void path_compression(){
    enqueue(99);//目印
    enqueue(99);
    
    short next_num_add = 0;
    
    while(1){
	short mode = dequeue(),num = dequeue();
	if(mode == 99)break;

	num += next_num_add;
	next_num_add = 0;
	
	switch(mode){
	
		case 0://S
			if(queue_next(1) == -1 && queue_next(3) == 0){//L大曲
				dequeue();
				dequeue();
				
				enqueue(mode);
				enqueue(num -1);
				 
				enqueue(-12);
				enqueue(1);
				
				next_num_add = -1;
			}else if(queue_next(1) == 1 && queue_next(3) == 0){//R大曲
				dequeue();
				dequeue();
				
				enqueue(mode);
				enqueue(num -1);
				 
				enqueue(12);
				enqueue(1);
				
				next_num_add = -1;
			
			}else if(queue_next(1) == 11){//直線からのR45
				dequeue();
				dequeue();
				
				enqueue(mode);
				enqueue(num -1);
				
				enqueue(14);
				enqueue(1);
				
				//next_num_add = -1;
				
			}else if(queue_next(1) == -11){//直線からのL45
				dequeue();
				dequeue();
				
				enqueue(mode);
				enqueue(num -1);
				
				enqueue(-14);
				enqueue(1);
				
				//next_num_add = -1;
				
			}else{//変更なし
				enqueue(mode);
				enqueue(num);
			}
			break;
		
		case 11://R45
			if(queue_next(1) == 0 || queue_next(1) == 1 || queue_next(1) == 11){//斜め終わり
				enqueue(13);
				enqueue(num);	
			}else{
				enqueue(mode);
				enqueue(num);
			}
			break;
		
		case -11://R45
			if(queue_next(1) == 0 || queue_next(1) == -1 || queue_next(1) == -11){//斜め終わり
				enqueue(-13);
				enqueue(num);	
			}else{
				enqueue(mode);
				enqueue(num);
			}
			break;
			
		default:
			enqueue(mode);
			enqueue(num);
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
    int comand_old = 0 ,path_num_old = 0;
    int comand_old2 = 0 ,path_num_old2 = 0;
    int BIG_NG_flag = 0;
    int path_add; //前回が大曲だった時の距離補正
    int v2_flag = 0;
    
    //１マスでは無効
    int over_run = -200;//速度上げるとオーバーランぎみなので少し手前で止める マイナスにすると距離がプラスになる
    int over_run2 = -400; // 直線距離が短い時に使用する

    int run_speed = 90;
    int run_speed_naname = 90;
    
    /*   
	 R_curveU(ur180,true);
	 L_curveU(ul180,true);
	 R_curveU(ur180,true);
	 led(9);
	 ESmotor(30,20,true,true);//距離、スピード
  
	 motor(-10,-10);
	 while(1)motor(0,0);
    */	
    ////////////////////////////////////////////////////////////最短経路の導出確認
    /*  while(!queue_empty()){//debug
	comand = dequeue();path_num = dequeue();
	printf2("%d %d\n",comand,path_num);
	delay(1);
	}
	while(1);*/
    //////////////////////////////////////////////////////////
 
    //enqueue(99);
    //enqueue(99);
     
    while(!queue_empty()){
	comand = dequeue();path_num = dequeue();
	
	//if(comand == 99)break;
	
	if(BIG_NG_flag == 1){//大曲禁止
		BIG_NG_flag = 2;
		if(comand == -12)comand = -1;
		else if(comand ==  12)comand =  1;
		
	}else if(BIG_NG_flag == 2){//前回大曲をやめた＝次の直線距離を半マス追加する
		BIG_NG_flag = 0;
		if(comand == 0){
			path_num += 1;
		}
	}
	
	status_log = comand;
	switch(comand){
	case -1://L
	    /*
	      if(comand_old == 0 && queue_next(1) == -1 && queue_next(3) == 0){//Uターン
		
	      L_curveU(ul180,true);
	      comand = dequeue();
	      path_num = dequeue();
			
	      }else 
	    */
	    if(queue_next(1) == -1){//上のUターンが無効の時に発動する
		L_curve(sl90,true);
		ESmotor(170,30,true,true);//距離、スピード
		
	    }else if(queue_next(1) == 1){//Sターン
		L_curve(sl90,true);
		ESmotor(200,30,true,true);//距離、スピード
		
	    }else if(queue_next(1) == -11 || queue_next(1) == 11){
		L_curve(sl90,true);
		ESmotor(150,25,true,true);//距離、スピード
		
	    }else if(comand_old == -13){//斜め終わり直後のカーブ
		L_curve_afterNaname(sl90,true);
		
	    }else{
	
		L_curve(sl90,true);
			
		/*if(queue_next(1) == 0){
		    ESmotor(150,30,true,true);//距離、スピード
		}*/
	    }
		
	    break;
	    
	case -12://L大曲
	    L_curveBIG(sl90BIG,true);
	    break;
	    
	case -11://L45
	  	
	    L_rotate_naname(l45 * path_num,true);
	    
	    break;
	case -14://直線からのL45 
	    if(queue_next(2) <=  2 && queue_next(3) == 0) {//２マス斜め Zパターン
	    	if(comand_old == 0 && path_num_old <= 1 && comand_old2 == 13){
			L_rotate_naname(l45 * path_num * 1.00 ,2);
		}else{
			L_rotate_naname(l45 * path_num * 1.20 ,2);
		}
	
	    }else if(queue_next(2) <=  2 ){//２マス斜め
		L_rotate_naname(l45 * path_num * 1.10 ,2);
	    
	    }else if(comand_old == 0 && path_num_old <= 1 && comand_old2 == 13){
		 L_rotate_naname(l45 * path_num * 0.90 ,2);
		 
	    }else{
	    	L_rotate_naname(l45 * path_num,2);
	    }
	    break;
	case -13://L45 出るとき
	  	
	    if(queue_next(1) == -11){//Vターン
	    	if(comand_old != 10 || path_num_old <= 2){//２マスVターン
			L_rotate_naname(l45 * path_num * 2.05,false);//0.75
			v2_flag = 1;
		}else{
			L_rotate_naname(l45 * path_num * 2.00,false);//0.75
			GyroSum_reset();
		}
		comand = dequeue();
	      	path_num = dequeue();
		
	    }else if(queue_next(1) == 0){
		L_rotate_naname(l45 * path_num * 0.95,false);

	    }else{
		L_rotate_naname(l45 * path_num * 1.00,false);
	
	    }
	    break;
	
	    
	case 0://S
	    if(queue_empty()){// || queue_next(1) == 99){
			
		//S_run(h1 * (long long)path_num ,run_speed + run_fin_speed_offset,4,true);//non_stop = 4
		
		if(path_num >= 2){
			S_run(h1 * (long long)path_num ,run_speed + run_fin_speed_offset,4,4);//non_stop = 4 // w_flag = 4 串の壁補正あり
		}
		
		motor(0,0);
		Set_motor_pid_mode(0);//低速 マイナスがあるので低速モードに戻す
		GyroSum_reset();
		
		//if(get_IR(IR_FL) > 10 || get_IR(IR_FR) > 10){
		t_1ms = 0;
		while(t_1ms < F_max_time || get_IR(IR_F) < F_min){//ゴールの奥まで進む 前壁補正
		    if(get_IR(IR_FL) > F_max || get_IR(IR_F) > F_max || get_IR(IR_FR) > F_max){
			Smotor(-F_pow,false);

			cnt = 0;
		    }else if(get_IR(IR_F) < F_min){
			Smotor(+F_pow,true);
	       				
			cnt = 0;
		    }else {
			motor(0,0);
			cnt++;
		    }
		    if(cnt > F_cnt)break;
		}
		motor(0,0);
		//}
	    }else {
		
                path_num--;//壁切れ分距離を半マス減らす
		
		if(path_num < 0){//マイナスにならないようにする
			path_num = 0;
		}
          	//if(path_num > 0){
		
		    if(comand_old == 12 || comand_old == -12){//前回が大曲だったら
			  path_add = 150; 
			  
		    }else if(comand_old == 13 || comand_old == -13){//前回が斜め終わりだったら
			  path_add = 70;  
		    }
		    
		    //if(first_flag == 0)S_run((h1 *(long long) path_num) - over_run ,run_speed + run_fin_speed_offset,3,true); // memo : non_stop = 3 加速はゆっくり　減速はすくなめ
		    //else S_run((h1 * (long long)path_num)  - over_run ,run_speed + run_fin_speed_offset,true,true);
			 
		   
		    if(path_num == 1){
			if(first_flag == 0)S_run((h1 *(long long) path_num)+path_add ,run_speed + run_fin_speed_offset,3,4); // memo : non_stop = 3 加速はゆっくり　減速はすくなめ// w_flag = 4 串の壁補正あり
			else  S_run((h1 * (long long)path_num)+path_add ,run_speed + run_fin_speed_offset,true,4);// w_flag = 4 串の壁補正あり
			
		    }else if(path_num > 10){
				  
			if(first_flag == 0)S_run((h1 *(long long) path_num) - over_run+path_add ,run_speed + run_fin_speed_offset,3,4); // memo : non_stop = 3 加速はゆっくり　減速はすくなめ// w_flag = 4 串の壁補正あり
			else  S_run((h1 * (long long)path_num)  - over_run+path_add ,run_speed + run_fin_speed_offset,true,4);// w_flag = 4 串の壁補正あり
		    }else{
			if(first_flag == 0)S_run((h1 *(long long) path_num) - over_run2+path_add ,run_speed + run_fin_speed_offset,3,4); // memo : non_stop = 3 加速はゆっくり　減速はすくなめ// w_flag = 4 串の壁補正あり
			else  S_run((h1 * (long long)path_num)  - over_run2+path_add ,run_speed + run_fin_speed_offset,true,4);// w_flag = 4 串の壁補正あり  
		    }
		    
		    path_add = 0;
		//}	  
		
		
		
		if(get_IR(IR_L) > 200 || get_IR(IR_R) > 200 || ((abs(get_IR(IR_L) - get_IR(IR_R)) > 120) &&  get_IR(IR_L) > 20 && get_IR(IR_R) > 20)){//左右の差が大きい && 左右に壁がある
    			
			if(queue_next(1) == -12 || queue_next(1) == 12){//次は大曲予定
    				BIG_NG_flag = 1;//ずれが大きいので大曲禁止
					
				ESmotor(h1,40,true,true);//距離、スピード
			}
    			
    		}
		
		status_log = 3;//ログに壁切れ開始を記録するため
		/*
		if(queue_next(3) == 0 && queue_next(4) <= 1 && queue_next(5) == 99){//ゴール最後で壁切れが発生しない時（2か所入口があるとき）
			ESmotor(h1,40,true,true);//距離、スピード　壁切れの代わりに半マス直線
			
		}else{*/
			if(queue_next(1) == -11 || queue_next(1) == 11){//直線後に45ターン
				if(queue_next(1) < 0){//次　左
				    if(path_num <= 1){
					 S_run_kabe2(25,true,1); 
				    }else{
					 S_run_kabe2(20,true,1);
				    }
				    
				    //S_run_kabe2(20,4,1);// w_flag = 4 串の壁補正あり
							
				}else if(queue_next(1) > 0){//次　右
				    if(path_num <= 1){
					 S_run_kabe2(25,true,2); 
				    }else{
					 S_run_kabe2(20,true,2);  
				    }
				    
				    //S_run_kabe2(20,4,2);// w_flag = 4 串の壁補正あり
				}
			}else if(queue_next(1) == -14 || queue_next(1) == 14){//直線後に45ターン 斜めセンサー版
				if(queue_next(1) < 0){//次　左
				    if(path_num <= 1){
					 //S_run_kabe_BIG(40,0,1,path_num); //壁補正無し
					 
					 S_run_kabe_BIG(40,4,1,path_num); //w_flag = 4 串の壁補正あり
				    }else{
					 S_run_kabe_BIG(30,4,1,path_num);  //w_flag = 4 串の壁補正あり 
				    }
						
				}else if(queue_next(1) > 0){//次　右
				    if(path_num <= 1){
					    
					 //S_run_kabe_BIG(40,0,2,path_num);  //壁補正無し
					 
					 S_run_kabe_BIG(40,4,2,path_num);  //w_flag = 4 串の壁補正あり
				    }else{
					 S_run_kabe_BIG(30,4,2,path_num);  //w_flag = 4 串の壁補正あり
				    }

				}
			}else if(BIG_NG_flag == 0 && (queue_next(1) == 12 || queue_next(1) == -12)){//直線後に大曲
				if(queue_next(1) < 0){//次　左
				    if(path_num <= 1){
				        S_run_kabe_BIG(40,4,1,path_num); //w_flag = 4 串の壁補正あり
				    }else{
					S_run_kabe_BIG(30,4,1,path_num);  //w_flag = 4 串の壁補正あり
				    }
						
				}else if(queue_next(1) > 0){//次　右
				    if(path_num <= 1){
				        S_run_kabe_BIG(40,4,2,path_num);  //w_flag = 4 串の壁補正あり
				    }else{
					S_run_kabe_BIG(30,4,2,path_num);   //w_flag = 4 串の壁補正あり
				    }      
				}
					  
			}else{
				if(queue_next(1) < 0){//次　左
				    if(path_num <= 1){
					S_run_kabe(40,true,1); 
				    }else{
				    	S_run_kabe(30,true,1); 
				    //S_run_kabe(35,4,1);// w_flag = 4 串の壁補正あり
				    }
							
				}else if(queue_next(1) > 0){//次　右
				    if(path_num <= 1){
					S_run_kabe(40,true,2);   
				    }else{
					S_run_kabe(30,true,2); 
				    }
				    
				    //S_run_kabe(35,4,2);// w_flag = 4 串の壁補正あり
				}	  
			}
		//}
	    }

	    //my_x = nx;
	    //my_y = ny;
	    break;
	case 10://Snaname
	    path_num-=2;//後で元に戻すこと
		 
	    if(path_num <= 0){//２マスだけの斜め
			
		if(run_fin_speed_offset > 0){//速度オフセットがプラスの時は無効化
		    //S_run(s45 /4 ,run_speed_naname,true,0); //壁補正無し
		    S_run(s45 /4 ,run_speed_naname,true,3); // w_flag = 3 斜めの壁補正あり 少しだけ前に移動した方が安全
		}else{
		   // S_run(s45 /4 ,run_speed_naname + run_fin_speed_offset,true,0);//壁補正無し
		    S_run(s45 /4 ,run_speed_naname + run_fin_speed_offset,true,3); // w_flag = 3 斜めの壁補正あり 少しだけ前に移動した方が安全
		}
		
		
		status_log = 3;//ログに壁切れ開始を記録するため
		
		
		
		//距離が短いので少し速度高めに設定する
		if(queue_next(1) == -11){//次　左
		    S_run_kabe_naname(60,3,1);
			
	        }else if(queue_next(1) == 11){//次　右
		    S_run_kabe_naname(60,3,2);
		
		}else if(queue_next(1) == -13){//次　左
		
		    if(v2_flag == 1 && queue_next(3) != -11){//2マスVターンの出るとき
			 v2_flag = 4; 
		
	            }else if(queue_next(3) == -1){//2マス斜め後にカーブ
		   
		    	v2_flag = 5;
		    }else if(queue_next(3) != -11){//2マスVターンではない Zパターン
		   
		    	v2_flag = 2;
		    }else{
		    	v2_flag = 1;  	
		    }	
		    
		    S_run_kabe_naname2(60,3,1,v2_flag);
	        }else if(queue_next(1) == 13){//次　右
		
		    if(v2_flag == 1 && queue_next(3) != 11){//2マスVターンの出るとき
			 v2_flag = 4;  
			 
		    }else if(queue_next(3) == 1){//2マス斜め後にカーブ
		   
		    	v2_flag = 5;
		    }else  if(queue_next(3) != 11){//2マスVターンではない Zパターン
		    	v2_flag = 2;
		    }else{
		    	v2_flag = 1;
		    }
		    
		    S_run_kabe_naname2(60,3,2,v2_flag);
	        }
		
		//ESmotor(100,40,true,false);//距離、スピード
		
	    }else{
		 /*   
		if(run_fin_speed_offset > 0){//速度オフセットがプラスの時は無効化
			S_run(s45 ,run_speed_naname,true,0); //壁補正なし 
		}else{
			S_run(s45 ,run_speed_naname + run_fin_speed_offset,true,0); //壁補正なし 
		}	
		path_num--;//後で元に戻すこと
		*/
		if( path_num >= 0){     
			if(run_fin_speed_offset > 0){//速度オフセットがプラスの時は無効化
			    S_run(s45 * (long long)path_num + s45/2 ,run_speed_naname,true,3); // w_flag = 3 斜めの壁補正あり
			}else{
			    S_run(s45 * (long long)path_num + s45/2 ,run_speed_naname + run_fin_speed_offset,true,3); // w_flag = 3 斜めの壁補正あり
			}
		}
		//path_num++;
		
		status_log = 3;//ログに壁切れ開始を記録するため
		
		if(queue_next(1) == -11){//次　左
		    S_run_kabe_naname(40,3,1);
			
	        }else if(queue_next(1) == 11){//次　右
		    S_run_kabe_naname(40,3,2);
		
		}else if(queue_next(1) == -13){//次　左
		    if(queue_next(3) == -11){//Vターン
			 v2_flag = 3;   
		   
		    }else if(queue_next(3) == -1){//斜め後にカーブ
		   
		    	v2_flag = 5;
			
		    }else if(queue_next(3) == 0){//斜め後に直線
		   
		    	v2_flag = 2;
		    }
		    S_run_kabe_naname2(40,3,1,v2_flag);
			
	        }else if(queue_next(1) == 13){//次　右
		    if(queue_next(3) == 11){//Vターン
			 v2_flag = 3;   
		    
		    }else if(queue_next(3) == -1){//斜め後にカーブ
		   
		    	v2_flag = 5;
		    }else if(queue_next(3) == 0){//斜め後に直線 
		   
		    	v2_flag = 2;
		    }
		    S_run_kabe_naname2(40,3,2,v2_flag);
		    
	        }
	    }
		
	    v2_flag = 0;
	    //my_x = nx;
	    //my_y = ny;
	    
	    path_num +=2;
	     
	    break;
	case 1://R
	    if(queue_next(1) == -1){//Sターン
		R_curve(sr90,true);
		ESmotor(200,30,true,true);//距離、スピード
			
		/*    }else if(comand_old == 0 && queue_next(1) == 1 && queue_next(3) == 0){//Uターン
		
		      R_curveU(ur180,true);
		      comand = dequeue();
		      path_num = dequeue();
		*/		
	    }else if(queue_next(1) == 1){//上のUターンが無効の時に発動する
		R_curve(sr90,true);
		ESmotor(170,30,true,true);//距離、スピード
		
	    }else if(queue_next(1) == -11 || queue_next(1) == 11){
		R_curve(sr90,true);
		ESmotor(150,25,true,true);//距離、スピード
		
	    }else if(comand_old == 13){//斜め終わり直後のカーブ
		R_curve_afterNaname(sr90,true);
		
	    }else{
		R_curve(sr90,true);
			
		/*if(queue_next(1) == 0){
		    ESmotor(150,30,true,true);//距離、スピード
		}*/
	    }
	    break;
	
	case 12://R大曲
	    R_curveBIG(sr90BIG,true);
	    break;
	    
	case 11://R45
        
	    R_rotate_naname(r45 * path_num,true);
	    
	    break;
	case 14://直線からのR45
        
	    if(queue_next(2) <=  2 && queue_next(3) == 0) {//２マス斜め Zパターン
	    	if(comand_old == 0 && path_num_old <= 1 && comand_old2 == -13){
			R_rotate_naname(r45 * path_num * 1.00 ,2);
		}else{
			R_rotate_naname(r45 * path_num * 1.20 ,2);
		}
	    }else if(queue_next(2) <=  2 ){//２マス斜め
		R_rotate_naname(r45 * path_num * 1.10 ,2);
	
	    }else if(comand_old == 0 && path_num_old <= 1 && comand_old2 == -13){
		R_rotate_naname(r45 * path_num * 0.90 ,2);
		
	    }else{
	    	R_rotate_naname(r45 * path_num,2);
	    }
	    break;
	case 13://R45 出る時
        
	    if(queue_next(1) == 11){//Vターン
	    	if(comand_old != 10 || path_num_old <= 2){//２マスVターン
			R_rotate_naname(r45 * path_num * 2.05,false);//0.75
			v2_flag = 1;
		}else{
			R_rotate_naname(r45 * path_num * 2.00,false);//0.75
			GyroSum_reset();
		}
		comand = dequeue();
	      	path_num = dequeue();
		
	    }else if(queue_next(1) == 0){
		R_rotate_naname(r45 * path_num * 0.95,false);
	
	    }else{
		R_rotate_naname(r45 * path_num * 1.00,false);
			
	    }
	    break;
	   
	
	}
	
	first_flag = 1;
	//前前回の値として記憶
	comand_old2 = comand_old;
	path_num_old2 = path_num_old;
	
	//前回の値として記憶
	comand_old = comand;
	path_num_old = path_num;
    }
    status_log = 99;
    motor(0,0);
    led_up();
}


/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */
/* 関 数 概 要：最短後の取り上げやすい位置を探す   		 			            */
/* 関 数 詳 細：												                                   */
/* 引       数： 取り上げやすいXY座標、								    */
/* 戻  り   値： なし										    									*/
/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */ 
void search_pickup(int* pickuup_x,int* pickuup_y){
	
    int x,y;
    int cost = maze_d_max;
	
    //shortest_path_search(Goal_x,Goal_y);
    shortest_path_search_fin();
	
    for(int i = 0; i < H;i++){
	for(int j = 0;j < W; j++){
				
	    if( !( (Not_Pickup_y_min <= i  && i <=  Not_Pickup_y_max) && (Not_Pickup_x_min <= j &&  j <= Not_Pickup_x_max) )){
				
		if(j != Goal_x && i != Goal_y){
		    for(int k = 0; k < 4; k++){
						
			if(maze_d[i][j][k] < cost){
			    cost = 	maze_d[i][j][k];
			    x = j;
			    y = i;
			}
		    }
		}
	    }
	}
    }
	
    *pickuup_x = x;
    *pickuup_y = y;
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

    t_1ms++;
    if(t_1ms > 99999)t_1ms = 99999;
    
    task ++;                         // タスクの更新						
    if (log_start != 2 && task >= 30) task = 0;       
    if (log_start == 2 && task >= 10) task = 0;       
	
    if(Gy_flag == 1){
	Gyro_update();
			
	if(ir_flag == 1){//赤外線有効時=走行時にジャイロによる安全停止チェックを行う
	    if(abs(Gyro()) > 320){
		motor_stop_cnt++;
		if(motor_stop_cnt > 30)motor_stop();
	    }else motor_stop_cnt = 0;
	}
    }
	
    if(motor_stop_get() == 1){//モータ緊急停止状態
	log_start = 0; //ログの記録も停止
    }
	
    if(time_limit > 0){
	time_limit--;	
    }
	
    motor_pid_flag_reset();
	
    switch(task) {                         			
    case 0:
    case 10:
    case 20:
    case 30:
	encoder_update();
        break;
   
    case 1:
//    case 6:////////最短のログを細かく取得したいときに有効化する　基本は無効
	if(log_start != 0){
	    if(log_flag != 1)log_cnt = 0;
	    log_flag = 1;
				
			
	    if((LOG_BUF_MAX * log_save_cnt) + log_cnt +16 <= LOG_MAX){
		log_buff[log_cnt++] =( my_x << 4) +( my_y & 0x0F);
			
		log_buff[log_cnt++] = (char)(get_IR(IR_LT) >>2);
		
		log_buff[log_cnt++] = (char)(get_IR(IR_L) >>2);
		log_buff[log_cnt++] = (char)(get_IR(IR_FL) >>2);
		log_buff[log_cnt++] = (char)(get_IR(IR_F) >>2);
		log_buff[log_cnt++] = (char)(get_IR(IR_FR) >>2);
		log_buff[log_cnt++] = (char)(get_IR(IR_R) >>2);
		
		log_buff[log_cnt++] = (char)(get_IR(IR_RT) >>2);
				
		
		
		log_buff[log_cnt++] = get_pwm_buff_L();
		log_buff[log_cnt++] = get_pwm_buff_R();
		
		log_buff[log_cnt++] = get_encoder_L() >>1;
		log_buff[log_cnt++] = get_encoder_R() >>1;
		
		log_buff[log_cnt++] = my_angle;
		
		log_buff[log_cnt++] = status_log;
		
		log_cnt+=2;//合計16個になるように調整する
				
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




 



