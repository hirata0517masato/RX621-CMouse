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
#include"dijkstra.h"
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

void mae_kabe(void);

void maze_save(void);
void maze_load(void);
void log_reset(void);
void log_save(void);
void log_load(void);
void search_pickup(int*,int*);
void run_pickup(short,short);
char shortest_path_search_pickup(short,short);
void make_shortest_path_list_pickup(short,short);
void maze_update(char,char,char,char);
void maze_update2(char ,char );
void maze_search_adachi(short,short);
void maze_search_all(void);
char shortest_path_search_check(void);
char shortest_path_search_check_full(void);
void shortest_path_search_fin(void);
void shortest_path_search_perfect(void);
void remake_shortest_path_list_naname(void);
void remake_shortest_path_list_naname2(void); //2マスでも斜めにする
void path_compression(void);
void run_shortest_path_fin(char);

char shortest_path_search(short,short);
void shortest_path_search_perfect_unknown(short* ,short* );

void shortest_path_search_dijkstra();
void shortest_path_search_dijkstra_unknown(short* ,short* );

void L_rotate(long long);
void R_rotate(long long);
void S_run(long long,int, char,char);

char Get_Goal_x(void);
char Get_Goal_y(void);
char Get_Goal_angle(void);

int d_to_p(int);

/* 定数設定 */
#define true 1
#define false 0

#define LOG_BUF_MAX 32  //DataFlash_write2 内の数値を合わせること
#define LOG_MAX 2048


//グローバル変数
short motor_stop_cnt = 0;
char maze_w[H][W] = {0};	//上位4bit = 壁の確定bit 下位4bit = 壁の情報（未確定含む）
short maze_d[H][W][4] = {0};	//4方向分の重み

char maze_w_backup[H][W] = {0};	//上位4bit = 壁の確定bit 下位4bit = 壁の情報（未確定含む）
short maze_d_backup[H][W][4] = {0};	//4方向分の重み


#define backup_irq_max	(5) 		//過去５回分のバックアップを保存する
long long backup_irq_time_ms = 5000;	//探索時に指定時間ごとに迷路情報をバックアップする
#define backup_irq_cnt	(3) 		// 指定回数前の情報に復元する

char maze_w_backup_irq[H][W][backup_irq_max] = {0};	//上位4bit = 壁の確定bit 下位4bit = 壁の情報（未確定含む）


char Goal_x_offset = 0;
char Goal_y_offset = 0;
char Goal_angle_offset = 0;

short maze_d_perfect[H][W] = {0};
short maze_d_dijkstra[H][W] = {0};

short r_cost_offset = 0;

short r45_cost_offset = 0;

short dx[4] = {0,1,0,-1},dy[4] = {-1,0,1,0};

char my_x = Start_x,my_y = Start_y,my_angle = Start_angle;//0:up 1:right 2:down 3:left

int ir_flag = 0; // 0:赤外線OFF 1:赤外線ON

int Gy_flag = 0; // 0:ジャイロOFF 1:ジャイロON

int run_fin_speed_offset = 0;

long long time_limit = -1;
long long time_limit_base = 150000;//3分00秒　180000      2分30秒 150000
long long time_limit_offset = 0;

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

int mode = 0;

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
    //int mode = 0;   割り込みでも使用したいのでグローバルに移動　
    int first_flag = 0;
    ir_flag = 0;//赤外線OFF
	
    ALL_init();//初期化
	
    delay(100);
	

 /*  
    ///////////////////////////////
    //int path_hosei_test[31] = {0,
 //			      0,0, 450,500, 550,600, 650,700, 750,750, 750,750, 500,500, 500,500, 
 //			  500,500, 550,600, 600,600, 600,600, 600,600, 600,600, 600,600};//path_numごとに距離補正する
    int path_hosei_test[16] = {0,
 			 	  0,200,400,500,500,400,300,300, 
 			  	300,300,300,300,300,300,300};//path_numごとに距離補正する
			  
    int path_num_test = 9;
    
    delay(1000);
    
    ir_flag = 1;//赤外線ON
    if( Gy_flag == 0){
		led_up();
				
		Gyro_init();	//ジャイロ、SPIの初期化 　注意：少し時間かかります 処理中はジャイロセンサーを動かさないこと
		Gy_flag = 1;
	    }
    delay(100);
    Encoder_reset();
    //Set_motor_pid_mode(1);//高速
    //ESmotor((h1 *(long long) path_num_test) + path_hosei_test[path_num_test],90,0,1);
    
    Set_motor_pid_mode(0);//低速
    ESmotor((s1 *(long long) path_num_test) + path_hosei_test[path_num_test],65,0,1); 
    
 
    ir_flag = 0;
    motor(0,0);
    
    int l,r;
    l = get_encoder_total_L();
    r = get_encoder_total_R();
    
    while(get_sw() == 0);
    printf2("%d , %d\n",l,r);
    
    while(1);
    /////////////////////////////////////////
*/

/*    
      while(1){
      	//motor(i,i);
      	delay(50);
     	//printf2("%d , %d \n",i,get_encoder_L());
	
	int l,r;
	l = get_encoder_total_L();
	r = get_encoder_total_R();
	printf2("%d , %d \n",l,r);
      	delay(50);
      	//i++;
	  
      	if(i > 100){
     		motor(0,0);
      		while(1);
      	}
	
	if(get_sw() == 1){
		Encoder_reset();
	}
      }
*/ 

      
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
		
	if(mode < 8){//走行モードの場合スタート準備
	
	   
	    if(mode > 2 && shortest_path_search_check_full() == 1){//最短走行時に最短経路が見つからない時　エラー終了
	    	for(int k = 0; k < 4;k++){
			led(15); 
			delay(500);
			led(0); 
			delay(500);
		}
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
	    maze_search_adachi(Get_Goal_x(),Get_Goal_y());
		
	    if(first_flag == 1){
		if(shortest_path_search_check_full() == 1){//ゴールしたはずなのに最短経路が見つからない時
		    	for(int k = 0; k < 4;k++){
				led(15); 
				delay(500);
				led(0); 
				delay(500);
			}
			//本当は全削除ではなくてNGっぽいところだけ削除したい
			maze_load();//迷路データの読み込み 未確定壁ありでも最短経路が見つからない場合は確実にNGなのでロードしなおす
		}else{
			first_flag = 0;
			maze_save();//片道でも迷路を保存する
		}
	    }
				
	    Set_motor_pid_mode(0);//低速
	    maze_search_adachi(Start_x,Start_y);
				
	    log_start = 0; //ログ記録終了
				
	    break;
				
	
	case 2://探索モード　最短経路上の未確定マスをすべて探しに行く

	     time_limit = time_limit_base;
	    //printf2("time_limit = %d \n",time_limit);
	    
	    
	    if(( maze_w[0][1] & 0x20) == 0){//迷路が初期化された直後 スタート直後のマスの壁が確定していなければ初期化直後と判定する
		led_down();
		led_up();
				
		first_flag = 1;
	    }
				
	    log_reset();//ログの初期化
	    log_start = 1; //ログ記録開始　30msに１回記録
		
	   
	    Set_motor_pid_mode(0);//低速
	    maze_search_adachi(Get_Goal_x(),Get_Goal_y());//はじめは普通に探索走行
				
				
	    if(first_flag == 1){
		if(shortest_path_search_check_full() == 1){//ゴールしたはずなのに最短経路が見つからない時
		    	for(int k = 0; k < 4;k++){
				led(15); 
				delay(500);
				led(0); 
				delay(500);
			}
			//本当は全削除ではなくてNGっぽいところだけ削除したい
			maze_load();//迷路データの読み込み 未確定壁ありでも最短経路が見つからない場合は確実にNGなのでロードしなおす
		}else{
			first_flag = 0;
			maze_save();//片道でも迷路を保存する
		}
	    }
	    
	    Set_motor_pid_mode(0);//低速
	    maze_search_all();//最短経路上の未確定マスを探しに行く
				
	    log_start = 0; //ログ記録終了
				
	    break;
	    
	case 3://最短走行モード  小曲げ
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
				
	    my_x = Get_Goal_x();
	    my_y = Get_Goal_y();
	    my_angle = Get_Goal_angle();
	    
#ifdef Pickup_x
	    pickup_x = Pickup_x;
	    pickup_y = Pickup_y;
#else
	    search_pickup(&pickup_x,&pickup_y);
#endif

	    Set_motor_pid_mode(0);//低速
	    //maze_search_adachi(pickup_x,pickup_y);//拾いやすいところまで移動する
	    run_pickup(pickup_x,pickup_y);//拾いやすいところまで移動する
	    break;
/*	    
	case 4://最短走行モード  大曲あり
	    shortest_path_search_fin();
	    path_compression();//大曲など
	    
	    log_reset();//ログの初期化
	    log_start = 2; //ログ記録開始 10msに１回記録
				
	    Set_motor_pid_mode(1);//高速
	    run_shortest_path_fin(false);
				
	    log_start = 0; //ログ記録終了
				
	    led(9);
	    delay(500);
	    led_up();
				
	    my_x = Get_Goal_x();
	    my_y = Get_Goal_y();
	    my_angle = Get_Goal_angle();
	    
#ifdef Pickup_x
	    pickup_x = Pickup_x;
	    pickup_y = Pickup_y;
#else
	    search_pickup(&pickup_x,&pickup_y);
#endif

	    Set_motor_pid_mode(0);//低速
	    //maze_search_adachi(pickup_x,pickup_y);//拾いやすいところまで移動する
	    run_pickup(pickup_x,pickup_y);//拾いやすいところまで移動する			
	    break;
*/	    
	case 4://最短走行モード　斜め（２マス以上）大曲あり
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
				
	    my_x = Get_Goal_x();
	    my_y = Get_Goal_y();
	    my_angle = Get_Goal_angle();
				
#ifdef Pickup_x
	    pickup_x = Pickup_x;
	    pickup_y = Pickup_y;
#else
	    search_pickup(&pickup_x,&pickup_y);
#endif

	    Set_motor_pid_mode(0);//低速
	    //maze_search_adachi(pickup_x,pickup_y);//拾いやすいところまで移動する
	    run_pickup(pickup_x,pickup_y);//拾いやすいところまで移動する			
	    break;
							
	case 5://最短走行モード 　斜め　大曲あり
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
				
	    my_x = Get_Goal_x();
	    my_y = Get_Goal_y();
	    my_angle = Get_Goal_angle();
			
#ifdef Pickup_x
	    pickup_x = Pickup_x;
	    pickup_y = Pickup_y;
#else
	    search_pickup(&pickup_x,&pickup_y);
#endif

	    Set_motor_pid_mode(0);//低速
	    //maze_search_adachi(pickup_x,pickup_y);//拾いやすいところまで移動する
	    run_pickup(pickup_x,pickup_y);//拾いやすいところまで移動する		
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
				
	    my_x = Get_Goal_x();
	    my_y = Get_Goal_y();
	    my_angle = Get_Goal_angle();
			
#ifdef Pickup_x
	    pickup_x = Pickup_x;
	    pickup_y = Pickup_y;
#else
	    search_pickup(&pickup_x,&pickup_y);
#endif

	    Set_motor_pid_mode(0);//低速
	    //maze_search_adachi(pickup_x,pickup_y);//拾いやすいところまで移動する
	    run_pickup(pickup_x,pickup_y);//拾いやすいところまで移動する		
	    break;

	case 7://最短走行 ダイクストラ　経路選択モード
	    shortest_path_search_dijkstra();
	    
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
				
	    my_x = Get_Goal_x();
	    my_y = Get_Goal_y();
	    my_angle = Get_Goal_angle();
			
#ifdef Pickup_x
	    pickup_x = Pickup_x;
	    pickup_y = Pickup_y;
#else
	    search_pickup(&pickup_x,&pickup_y);
#endif

	    Set_motor_pid_mode(0);//低速
	    //maze_search_adachi(pickup_x,pickup_y);//拾いやすいところまで移動する
	    run_pickup(pickup_x,pickup_y);//拾いやすいところまで移動する		
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
	
	case 11://走行時間制限の調整（調整値は保存しない、本番での微調整用）
		Encoder_reset();
		
		led(9);
	    	delay(500);
		
		led(0);
		delay(200);
		led(12);
		delay(200);
		led(0);
		delay(200);
		led(12);
		delay(200);
		
	    	//モード選択 分設定
	    	while(1){
			
			led( ((time_limit_base + time_limit_offset)/60000) );//分
			
			
			time_limit_offset = ((get_encoder_total_L() / 60) * 60000);//Lエンコーダ１カウントで１分  Rエンコーダ１カウントで１０秒
	
			
			if(get_sw() == 1){
			    	led_up();
		    		while(get_sw() == 1)nop();
		    		break;
			}
	    	}
		
		time_limit_base += time_limit_offset;
		
		//printf2("time_limit_base = %d  , time_limit_offset = %d \n",time_limit_base,time_limit_offset);
		 
		led(0);
		delay(200);
		led(3);
		delay(200);
		led(0);
		delay(200);
		led(3);
		delay(200);
		//モード選択　秒設定
	    	while(1){
			
			led((((time_limit_base + time_limit_offset)%60000)/10000)  );//10秒
			
			time_limit_offset = ((get_encoder_total_R() / 60) * 10000)%60000;//Lエンコーダ１カウントで１分  Rエンコーダ１カウントで１０秒
	
			
			if(get_sw() == 1){
			    	led_up();
		    		while(get_sw() == 1)nop();
		    		break;
			}
	    	}
		time_limit_base += time_limit_offset;
		
		//printf2("time_limit_base = %d  , time_limit_offset = %d \n",time_limit_base,time_limit_offset);
		break;
	
	case 12://ゴール座標の調整（調整値は保存しない、本番での微調整用）
		Encoder_reset();
		
		led(9);
	    	delay(500);
		
		led(0);
		delay(200);
		led(8);
		delay(200);
		led(0);
		delay(200);
		led(8);
		delay(200);
		
	    	//モード選択 x座標　設定
	    	while(1){
			
			led(Goal_x + Goal_x_offset);
			
			Goal_x_offset = get_encoder_total_R() / 60 ;
		
			if(get_sw() == 1){
			    	led_up();
		    		while(get_sw() == 1)nop();
		    		break;
			}
	    	}
		
		Encoder_reset();
		led(0);
		delay(200);
		led(4);
		delay(200);
		led(0);
		delay(200);
		led(4);
		delay(200);
		
	    	//モード選択 y座標　設定
	    	while(1){
			
			led(Goal_y + Goal_y_offset);
			
			Goal_y_offset = get_encoder_total_R() / 60 ;
		
			if(get_sw() == 1){
			    	led_up();
		    		while(get_sw() == 1)nop();
		    		break;
			}
	    	}
		
		Encoder_reset();
		led(0);
		delay(200);
		led(2);
		delay(200);
		led(0);
		delay(200);
		led(2);
		delay(200);
		
	    	//モード選択 angle座標　設定
	    	while(1){
			
			led(Goal_angle + Goal_angle_offset);
			
			Goal_angle_offset = get_encoder_total_R() / 60 ;
		
			if(get_sw() == 1){
			    	led_up();
		    		while(get_sw() == 1)nop();
		    		break;
			}
	    	}
		Encoder_reset();
		led(0);
		delay(200);
		led(1);
		delay(200);
		led(0);
		delay(200);
		led(1);
		delay(200);
		
		
		break;
		
	case 14://迷路情報リセットモード
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
				
	case 15://ログ出力モード
		
	    shortest_path_search_check_full();
	    
	    for(int i = 0; i < H;i++){
		for(int j = 0; j < W; j++)printf2("%d\t",maze_w[i][j]&0x0f);
		printf2("\n");
	    }
	    printf2("\n");
	
	    
	    for(int i = 0; i < H;i++){
		for(int j = 0; j < W; j++){
			printf2("+");
			if(maze_w[i][j]&0x01)printf2("----");
			else if(maze_w[i][j]&0x10)printf2("    ");
			else printf2("....");
		}
		printf2("+\n");
		
		for(int j = 0; j < W; j++){
			if(maze_w[i][j]&0x08)printf2("|");
			else if(maze_w[i][j]&0x80)printf2(" ");
			else printf2(":");
			
			if(min(maze_d[i][j][3],min(maze_d[i][j][2],min(maze_d[i][j][1],maze_d[i][j][0] ))) == maze_d_max){
				printf2("    ");
			}else{
				printf2("%4d",min(maze_d[i][j][3],min(maze_d[i][j][2],min(maze_d[i][j][1],maze_d[i][j][0] )))  );	
			}
			
		}
		printf2("|\n");
	    }
	    printf2("+----+----+----+----+----+----+----+----+----+----+----+----+----+----+----+----+\n");
	
	    printf2("\n");
	    
	    if(shortest_path_search_check_full() == 0){//最短経路が存在するとき
		    shortest_path_search(Get_Goal_x(),Get_Goal_y());

	    
	    	    for(int i = 0; i < H;i++){
				for(int j = 0; j < W; j++){
					printf2("+");
					if(maze_w[i][j]&0x01)printf2("----");
					else if(maze_w[i][j]&0x10)printf2("    ");
					else printf2("....");
				}
				printf2("+\n");
		
				for(int j = 0; j < W; j++){
					if(maze_w[i][j]&0x08)printf2("|");
					else if(maze_w[i][j]&0x80)printf2(" ");
					else printf2(":");
			
					if(min(maze_d[i][j][3],min(maze_d[i][j][2],min(maze_d[i][j][1],maze_d[i][j][0] ))) == maze_d_max){
						printf2("    ");
					}else{
						printf2("%4d",min(maze_d[i][j][3],min(maze_d[i][j][2],min(maze_d[i][j][1],maze_d[i][j][0] )))  );	
					}
			
				}
				printf2("|\n");
	    	     }
		     printf2("+----+----+----+----+----+----+----+----+----+----+----+----+----+----+----+----+\n");
		
		     printf2("\n");
	    }
	     
	    printf2("\n");
	    
	    if(shortest_path_search_check_full() == 0){//最短経路が存在するとき
		    shortest_path_search_perfect();
		    for(int i = 0; i < H;i++){
			for(int j = 0; j < W; j++){
				printf2("+");
				if(maze_w[i][j]&0x01)printf2("----");
				else if(maze_w[i][j]&0x10)printf2("    ");
				else printf2("....");
			}
			printf2("+\n");
			
			for(int j = 0; j < W; j++){
				if(maze_w[i][j]&0x08)printf2("|");
				else if(maze_w[i][j]&0x80)printf2(" ");
				else printf2(":");
				
				if(maze_d_perfect[i][j] == maze_d_max){
					printf2("    ");
				}else{
					printf2("%4d",maze_d_perfect[i][j] );	
				}
				
			}
			printf2("|\n");
		    }
		    printf2("+----+----+----+----+----+----+----+----+----+----+----+----+----+----+----+----+\n");
		
		    printf2("\n");
	    }
	    
	    printf2("\n");
	    
	    if(shortest_path_search_check_full() == 0){//最短経路が存在するとき
		    
	    	    short dumy1,dumy2;
		    shortest_path_search_perfect_unknown(&dumy1,&dumy2);
		    
		    for(int i = 0; i < H;i++){
			for(int j = 0; j < W; j++){
				printf2("+");
				if(maze_w[i][j]&0x01)printf2("----");
				else if(maze_w[i][j]&0x10)printf2("    ");
				else printf2("....");
			}
			printf2("+\n");
			
			for(int j = 0; j < W; j++){
				if(maze_w[i][j]&0x08)printf2("|");
				else if(maze_w[i][j]&0x80)printf2(" ");
				else printf2(":");
				
				if(maze_d_perfect[i][j] == maze_d_max){
					printf2("    ");
				}else{
					printf2("%4d",maze_d_perfect[i][j] );	
				}
				
			}
			printf2("|\n");
		    }
		    printf2("+----+----+----+----+----+----+----+----+----+----+----+----+----+----+----+----+\n");
		
		    printf2("\n");
	    }
	    
	     printf2("\n");
	    
	    if(shortest_path_search_check_full() == 0){//最短経路が存在するとき
		    
		    shortest_path_search_dijkstra();
		    
		    for(int i = 0; i < H;i++){
			for(int j = 0; j < W; j++){
				printf2("+");
				if(maze_w[i][j]&0x01)printf2("----");
				else if(maze_w[i][j]&0x10)printf2("    ");
				else printf2("....");
			}
			printf2("+\n");
			
			for(int j = 0; j < W; j++){
				if(maze_w[i][j]&0x08)printf2("|");
				else if(maze_w[i][j]&0x80)printf2(" ");
				else printf2(":");
				
				if(maze_d_dijkstra[i][j] == maze_d_max){
					printf2("    ");
				}else{
					printf2("%4d",maze_d_dijkstra[i][j] );	
				}
				
			}
			printf2("|\n");
		    }
		    printf2("+----+----+----+----+----+----+----+----+----+----+----+----+----+----+----+----+\n");
		
		    printf2("\n");
	    }
	    printf2("\n");
	    
	    if(shortest_path_search_check_full() == 0){//最短経路が存在するとき
		    
	    	    short dumy1,dumy2;
		    shortest_path_search_dijkstra_unknown(&dumy1,&dumy2);
		    
		    for(int i = 0; i < H;i++){
			for(int j = 0; j < W; j++){
				printf2("+");
				if(maze_w[i][j]&0x01)printf2("----");
				else if(maze_w[i][j]&0x10)printf2("    ");
				else printf2("....");
			}
			printf2("+\n");
			
			for(int j = 0; j < W; j++){
				if(maze_w[i][j]&0x08)printf2("|");
				else if(maze_w[i][j]&0x80)printf2(" ");
				else printf2(":");
				
				if(maze_d_dijkstra[i][j] == maze_d_max){
					printf2("    ");
				}else{
					printf2("%4d",maze_d_dijkstra[i][j] );	
				}
				
			}
			printf2("|\n");
		    }
		    printf2("+----+----+----+----+----+----+----+----+----+----+----+----+----+----+----+----+\n");
		
		    printf2("\n");
	    }
	    
	    
	    
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
	
	if(shortest_path_search_check() == 1){//最短経路が見つからない時
		for(int k = 0; k < 4;k++){
			led(15); 
			delay(500);
			led(0); 
			delay(500);
		}
		
		//穏当は全削除ではなくてNGっぽいところだけ削除したい
		maze_load();//迷路データの読み込み 未確定壁ありでも最短経路が見つからない場合は確実にNGなのでロードしなおす
    	}else{ 
		maze_save();
		
		led_up();
		led_down();
	}
		
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
    delay(200);//チャタリング防止
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
	delay(50);
    }
    led(0);
}

void led_down(){
    for(int i = 8;i > 0; i/= 2){
	led(i);
	delay(50);
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
	//DataFlash_write2(i,0,log,sizeof(log));
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
	    
	    printf2(" : \t%3d\t%3d\t : \t%3d\t%3d\n",log_minus(log[i+8]),log_minus(log[i+9]),log_minus(log[i+10]) <<1,log_minus(log[i+11]) <<1  );	
														
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
		    if(get_IR(IR_F) > 40 ){
			mae_kabe();//前壁距離補正    
		    }
		    maze_update(x,y,my_angle, 2);//横壁のみチェック
		    R_rotate(r90);//右回転
		    delay(20);
		    //}
		}else{
		    maze_w[y][x] |= 1 << ii; 
		}
			
		mae_kabe();//前壁距離補正
		
		motor(0,0);
		
	    }else{
		if(((maze_w[y][x] & (1 << (4+ii))) != 0) && ( (maze_w[y][x] | (1 << ii)) != 0 ) ){//確定の場合 && 過去の記録が今回と値が異なる
						
		    //if(type == 3){
		    maze_w[y][x] &= ~(1 << (4+ii));//確定情報を消す
		    L_rotate(l90);//左回転
		    delay(20);
		    if(get_IR(IR_F) > 40 ){
			mae_kabe();//前壁距離補正    
		    }
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
    
    maze_update2(x,y);
    maze_update2(x-1,y);
    maze_update2(x-1,y-1);
    maze_update2(x,y-1);
    maze_update2(x,y);
    maze_update2(x-1,y);
    maze_update2(x-1,y-1);
    maze_update2(x,y-1);
}

void maze_update2(char x,char y){

	if(x == 7 && y == 7)return;
	
	char kakutei_cnt = 0;
	char kakutei_flag = 0;
	char mi_kakutei = 0;
	char kakutei_kabe_cnt = 0;
		
	if(x == 15)x--;
	if(y == 15)y--;
	
	if(0 <= x && x < 15 && 0 <= y && y < 15){//外枠ではない
		
		if(maze_w[y][x] & (1 << (4+1)) != 0 ){
			kakutei_flag |= 1;
			kakutei_cnt++;
			
			if(maze_w[y][x] & (1 << (1)) != 0){
				kakutei_kabe_cnt++;
			}
		}else{
			mi_kakutei = 0;
		}
		
		if(maze_w[y][x+1] & (1 << (4+2)) != 0 ){
			kakutei_flag |= 2;
			kakutei_cnt++;
			
			if(maze_w[y][x+1] & (1 << (2)) != 0){
				kakutei_kabe_cnt++;
			}
		}else{
			mi_kakutei = 1;
		}
		
		if(maze_w[y+1][x] & (1 << (4+1)) != 0 ){
			kakutei_flag |= 4;
			kakutei_cnt++;
			
			if(maze_w[y+1][x] & (1 << (1)) != 0){
				kakutei_kabe_cnt++;
			}
		}else{
			mi_kakutei = 2;
		}
		
		if(maze_w[y][x] & (1 << (4+2)) != 0 ){
			kakutei_flag |= 8;
			kakutei_cnt++;
			
			if(maze_w[y][x] & (1 << (2)) != 0){
				kakutei_kabe_cnt++;
			}
		}else{
			mi_kakutei = 3;
		}
		
		if(kakutei_cnt == 3){//残りの1壁も確定にできる
			if(kakutei_kabe_cnt == 0){//３枚　壁がない　＝残りは壁がある
				
				if(mi_kakutei == 0){
					maze_w[y][x] |= 1 << (4+1);
					maze_w[y][x] |= (1 << (1));
					
					maze_w[y][x+1] |= 1 << (4+3);
					maze_w[y][x+1] |= (1 << (3));
					
				}else if(mi_kakutei == 1){
					maze_w[y][x+1] |= 1 << (4+2);
					maze_w[y][x+1] |= (1 << (2));
					
					maze_w[y+1][x+1] |= 1 << (4+0);
					maze_w[y+1][x+1] |= (1 << (0));
					
				}else if(mi_kakutei == 2){
					maze_w[y+1][x] |= 1 << (4+1);
					maze_w[y+1][x] |= (1 << (1));
					
					maze_w[y+1][x+1] |= 1 << (4+3);
					maze_w[y+1][x+1] |= (1 << (3));
					
				}else if(mi_kakutei == 3){
					maze_w[y][x] |= 1 << (4+2);
					maze_w[y][x] |= (1 << (2));
					
					maze_w[y+1][x] |= 1 << (4+0);
					maze_w[y+1][x] |= (1 << (0));
				}
			}
		}
		
		
	}else{//外枠なので　柱につながる壁は３つ
		
	}
	
}

void maze_update3(char x,char y,char angle, char type){
	
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
        case -1://LT
	    //if((maze_w[y][x] & (1 << (4+ii))) == 0 ){//未確定の場合
				
	    if(get_IR(IR_LT) > 20){
		if(((maze_w[y][x] & (1 << (4+ii))) != 0) && ( (maze_w[y][x] | (1 << ii)) == 0 ) ){//確定の場合 && 過去の記録が今回と値が異なる
		
		}else{
		    maze_w[y][x] |= 1 << ii;
		}
	    }else{
		if(((maze_w[y][x] & (1 << (4+ii))) != 0) && ( (maze_w[y][x] | (1 << ii)) != 0 ) ){//確定の場合 && 過去の記録が今回と値が異なる
		
		}else{
		    maze_w[y][x] &= ~(1 << ii); 
		}  
	    }
				
	    maze_w[y][x] |= 1 << (4+ii);
	    //}
	    break;
        case 0://S
	    //if((maze_w[y][x] & (1 << (4+ii))) == 0 ){//未確定の場合
				
				 
	    if(get_IR(IR_F) > 15 ){
		if(((maze_w[y][x] & (1 << (4+ii))) != 0) && ( (maze_w[y][x] | (1 << ii)) == 0 ) ){//確定の場合 && 過去の記録が今回と値が異なる
							
	
		}else{
		    maze_w[y][x] |= 1 << ii; 
		}
		
	    }else{
		if(((maze_w[y][x] & (1 << (4+ii))) != 0) && ( (maze_w[y][x] | (1 << ii)) != 0 ) ){//確定の場合 && 過去の記録が今回と値が異なる
						
		}else{
		    maze_w[y][x] &= ~(1 << ii);
		}
	    }
	    maze_w[y][x] |= 1 << (4+ii);
	    //}
	    break;
        case 1://R
	    //if((maze_w[y][x] & (1 << (4+ii))) == 0 ){//未確定の場合
				 
	    if(get_IR(IR_RT) > 20){
		if(((maze_w[y][x] & (1 << (4+ii))) != 0) && ( (maze_w[y][x] | (1 << ii)) == 0 ) ){//確定の場合 && 過去の記録が今回と値が異なる
							
		}else{
		    maze_w[y][x] |= 1 << ii; 
		}
	    }else{
		if(((maze_w[y][x] & (1 << (4+ii))) != 0) && ( (maze_w[y][x] | (1 << ii)) != 0 ) ){//確定の場合 && 過去の記録が今回と値が異なる
		 
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
    
    maze_update2(x,y);
    maze_update2(x-1,y);
    maze_update2(x-1,y-1);
    maze_update2(x,y-1);
    maze_update2(x,y);
    maze_update2(x-1,y);
    maze_update2(x-1,y-1);
    maze_update2(x,y-1);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////
void mae_kabe(){
	GyroSum_reset();
	
	int cnt = 0;
	
	if(100 < get_IR(IR_F) ){//前壁　激突対策
	//while(F_min -30 < get_IR(IR_F) ){//前壁　激突対策
		ESmotor(-35,F_pow,true,false);//ちょっと下がる
	}
	
	t_1ms = 0;
	while(t_1ms < F_max_time){//前壁補正

		//}else if(get_IR(IR_FL) > F_max || get_IR(IR_F) > F_max || get_IR(IR_FR) > F_max){
		
		if(get_IR(IR_FL) - get_IR(IR_FR) > 30){
			motor(-F_pow, -F_pow/2);
			
			cnt = 0;
			GyroSum_reset();
			
		}else if(get_IR(IR_FL) - get_IR(IR_FR) < -30){
			motor(-F_pow/2, -F_pow);
			
			cnt = 0;
			GyroSum_reset();
			
		}else if(get_IR(IR_F) > F_max){
			Smotor(-F_pow,false);
			
			cnt = 0;
			GyroSum_reset();
		}else if(get_IR(IR_F) < F_min){
			Smotor(+F_pow,true);
		       				
			cnt = 0;
			GyroSum_reset();
		}else {
			motor(0,0);
			cnt++;
		}
		if(cnt > F_cnt)break;
	}
	motor(0,0);
	GyroSum_reset();	
}

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

void L_curve_search(long long a,char flag){
    ETmotor_search(-a,rslsl90_search,flag);
 
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

void R_curve_search(long long a ,char flag ){
    ETmotor_search(a,rslsr90_search,flag);
  
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
    
    if((non_stop == 0 || non_stop == 4) && (kabe == 1 || kabe == 4)){
	// GyroSum_reset();
	if(50 < get_IR(IR_F) ){//前壁補正
	    motor(0,0);
			
	    if(150 < get_IR(IR_F) ){//前壁　激突対策
		ESmotor(-50,F_pow,true,false);//ちょっと下がる
	    }
			
	    mae_kabe();//前壁距離補正
	    motor(0,0);
	}
    }
	
    motor(0,0);
    //GyroSum_reset();
}

void S_run_kabe_rev(int powor, char flag, int LR){//柱が見えるまで走行
    int Lflag = 0,Rflag = 0;
    //long long enc_base = (get_encoder_total_L() + get_encoder_total_R())/2;
    long long enc_base_L = get_encoder_total_L();
    long long enc_base_R = get_encoder_total_R();
    
    //   int cnt2 = 0;
	
    led(6);
    while(1){
	if(LR == 3 || LR == 1){//両方 || Lだけ
	    if(Lflag == 0){	
		if(get_IR(IR_L) < 10){
		    Lflag = 1;
		    led(8);
		}
	    }else if(Lflag == 1){
		if(get_IR(IR_L) > 25){
		    led(0);
		    break;
		}
	    }
	}

	if(LR == 3 || LR == 2){//両方 || Rだけ
	    if(Rflag == 0){
		if(get_IR(IR_R) < 10){
		    Rflag = 1;
		    led(1);
		}
	    }else if(Rflag == 1){
		if(get_IR(IR_R) > 25){
		    led(0);
		    break;
		}
	    }
	}
    
    	Smotor(powor,flag);
	
	
	  if(Lflag == 0 && Rflag == 0){
		//if(abs((get_encoder_total_L() + get_encoder_total_R())/2 -  enc_base) > (s1 + h1/2) ){
		
		if(Get_motor_pid_mode() == 1){//高速モード
			/*
			if(min(get_encoder_total_L()  - enc_base_L , get_encoder_total_R() - enc_base_R)  > (s1+ h1) ){
				led(9);
				break; //壁切れが来なかったらブレーク
			}*/
		}else{
			if(min(get_encoder_total_L()  - enc_base_L , get_encoder_total_R() - enc_base_R)  > (s1) ){
				led(9);
				break; //壁切れが来なかったらブレーク
			}
		}
	  }
	
    }
  
    status_log = 4;//ログに壁切れ後の距離補正を記録するため
    
    ESmotor(60,powor,true,false);//1cmくらい？
    led(0);
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
		if(Get_motor_pid_mode() == 1){//高速モード
			/*
			if(min(get_encoder_total_L()  - enc_base_L , get_encoder_total_R() - enc_base_R)  > (s1+ h1) ){
				led(9);
				break; //壁切れが来なかったらブレーク
			}
			*/
		}else{
			if(min(get_encoder_total_L()  - enc_base_L , get_encoder_total_R() - enc_base_R)  > (s1) ){
				led(9);
				break; //壁切れが来なかったらブレーク
			}
		}
	  }
	
    }
  
    status_log = 4;//ログに壁切れ後の距離補正を記録するため
    
    ESmotor(60,powor,true,false);//1cmくらい？
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
		if(Get_motor_pid_mode() == 1){//高速モード
		
		}else{
			
			if(min(get_encoder_total_L()  - enc_base_L , get_encoder_total_R() - enc_base_R)  > (s1+ h1 ) ){
				led(9);
				break; //壁切れが来なかったらブレーク
			}
		}
	}
   }
    status_log = 4;//ログに壁切れ後の距離補正を記録するため
    
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
			if(get_IR(IR_LT) < 5){
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
			if(get_IR(IR_RT) < 5){
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
			if(get_IR(IR_RT) < 15){
			    led(0);
			    break;
			}
		// }
	    }
	}
    
    	Smotor(powor,flag);
	
	
	  if(Lflag == 0 && Rflag == 0){
		//if(abs((get_encoder_total_L() + get_encoder_total_R())/2 -  enc_base) > (s1 + h1/2) ){
		
		if(Get_motor_pid_mode() == 1){//高速モード
		
		}else{
			if(pathnum <= 0){
				if(min(get_encoder_total_L()  - enc_base_L , get_encoder_total_R() - enc_base_R)  > (h1) ){
					led(9);
					break; //壁切れが来なかったらブレーク
				}
			}else{
				
				if(min(get_encoder_total_L()  - enc_base_L , get_encoder_total_R() - enc_base_R)  > (s1 + h1) ){
					led(9);
					break; //壁切れが来なかったらブレーク
				}
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
/*
    if( Lflag2 == 2 ||  Rflag2 == 2){
	  ESmotor(100,powor,true,false); 
    }
 */
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
 
    status_log = 4;//ログに壁切れ後の距離補正を記録するため
    
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
		if(get_IR(IR_LT) > 35){
		    led(8);
		    Lflag = 1;
		}
	    }else if(Lflag == 1){
			
		if(LMax -15 > get_IR(IR_LT)){
		    led(0);
		    Lflag = 2;
		    break;
		}
		
	    }
	}

	if(LR == 3 || LR == 2){//両方 || Rだけ
	    if(Rflag == 0){
		if(get_IR(IR_RT) > 35){
		    led(1);
		    Rflag = 1;
		}
	    }else if(Rflag == 1){
		if(RMax -15 > get_IR(IR_RT)){
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
	}else if(v2_flag == 20 && Lflag == 0 && Rflag == 0){//２マス斜め(Vターンではない）直後は壁切れができない可能性が高いので距離を短めに設定する
		//if(abs((get_encoder_total_L() + get_encoder_total_R())/2 -  enc_base) > (s45_V2)  ){
		if(min(get_encoder_total_L()  - enc_base_L , get_encoder_total_R() - enc_base_R)  > (s45_V2notFirst)  ){
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
 
    status_log = 4;//ログに壁切れ後の距離補正を記録するため
    
    if(v2_flag == 1){//２マスVターン開始
	ESmotor(90,powor,true,false);  //140
	
    }else if(v2_flag == 2){//２マス Vターンではない
    
    	if(Lflag == 0 && Rflag == 0){
		ESmotor(150,powor,true,false);
	}else{
    		ESmotor(250,powor,true,false);// 200
	}
    }else if(v2_flag == 20){//２マス Vターンではない
    
    	if(Lflag == 0 && Rflag == 0){
		ESmotor(100,powor,true,false);
	}else{
    		ESmotor(200,powor,true,false);// 200
	}
    }else if(v2_flag == 3){//２マス以上のVターン開始
    	ESmotor(120,powor,true,false);
	
    }else if(v2_flag == 4){//２マスVターンでるとき
    	ESmotor(360,powor,true,false);
	
    }else if(v2_flag == 5){//２マスVターンでたあとカーブ
    	ESmotor(190,powor,true,false);

    }else if(v2_flag == 6){//斜めの後　直線
    	ESmotor(390,powor,true,false);

    }else if(v2_flag == 7){//でたあとカーブ
    	ESmotor(140,powor,true,false);
	
    }else{
    	ESmotor(250,powor,true,false);
    }
    led(0);
}

void S_run_maze_search(int path,int powor, int powor_up , int ir_up){
    //Encoder_reset();
	
    int M_pwm_min = 6;
    int M_pwm = M_pwm_min;
    //long long enc_base = (get_encoder_total_L() + get_encoder_total_R())/2;
    long long enc_base_L = get_encoder_total_L();
    long long enc_base_R = get_encoder_total_R();
    long long enc_now = 0;
	
    int path_cnt = 0;
//    int maza_update_flag = 0;
	
    int ir_L_now = 0,ir_R_now = 0;
    int ir_L_flag = 0,ir_R_flag = 0;
    int path_cnt_save_L = -1;//同じマスで壁切れ処理を２回以上しないように覚えておく変数
    int path_cnt_save_R = -1;//同じマスで壁切れ処理を２回以上しないように覚えておく変数
//    int hosei_kyori_L = -1,hosei_kyori_R = -1;//壁切れ時の補正距離　左右異なるタイミングで壁切れした際に利用する
    long long enc_kabe_L,enc_kabe_R;
    int led_num = 0;
//    int kame_hosei = 530;
	
    int ir_L_flag_1masu = 0,ir_R_flag_1masu = 0;
    
    GyroSum_reset();
	
    while(1){
	
	if((long long)path * s1 >= s1 && ((long long)path * s1 - enc_now) < s1){//１マス以上進 && 残り１マス && 探索中　で壁切れした場合は半マス進んで終了
	
		if(ir_L_flag_1masu == 0){
			if(get_IR(IR_L) > 20){
				ir_L_flag_1masu = 1;
			}
		}else if(ir_L_flag_1masu == 1){
			if(get_IR(IR_L) < 10){
				ESmotor(h1_2,  powor ,0,1);  
				//while(1){
					//led()
				//	PORTA.DR.BIT.B0 = 0;
				//	PORTA.DR.BIT.B1 = 0;
				//	PORTA.DR.BIT.B2 = 1;
				//	PORTA.DR.BIT.B3 = 1;
				
				if(get_IR(IR_F) > 40){//前壁が出現
					mae_kabe();//前壁距離補正
	    				motor(0,0);
				}
				 //現在地の更新
	    			my_x += dx[my_angle];
	    			my_y += dy[my_angle];
			
	    			maze_update(my_x,my_y,my_angle,3);//迷路情報の更新
	    
				motor(0,0);
				//}
				return;
			}
		}
		
		if(ir_R_flag_1masu == 0){
			if(get_IR(IR_R) > 20){
				ir_R_flag_1masu = 1;
			}
		}else if(ir_R_flag_1masu == 1){
			if(get_IR(IR_R) < 10){
				ESmotor(h1_2, powor ,0,1); 
				//while(1){
					//led()
				//	PORTA.DR.BIT.B0 = 1;
				//	PORTA.DR.BIT.B1 = 1;
				//	PORTA.DR.BIT.B2 = 0;
				//	PORTA.DR.BIT.B3 = 0;
				
				if(get_IR(IR_F) > 40){//前壁が出現
					mae_kabe();//前壁距離補正
	    				motor(0,0);
				}
				 //現在地の更新
	    			my_x += dx[my_angle];
	    			my_y += dy[my_angle];
			
	    			maze_update(my_x,my_y,my_angle,3);//迷路情報の更新
	    
				motor(0,0);
				
				//}
				return;
			}
		}
	}
	
	if(enc_now >= (long long)path * s1){//目標距離に到達
		
	    //マスの中心まで移動(戻る）
	    while(enc_now - ((long long)s1 * path_cnt ) > s1){
		Smotor(-10,true);
		enc_now = min(get_encoder_total_L()  - enc_base_L , get_encoder_total_R() - enc_base_R);
	    }
			
	    if(get_IR(IR_F) > 30 ){//前壁があった場合は
	    	mae_kabe();//前壁距離補正
	    }
			
	    //現在地の更新
	    my_x += dx[my_angle];
	    my_y += dy[my_angle];
			
	    maze_update(my_x,my_y,my_angle,3);//迷路情報の更新
	
	    break;
	}
		
	if( path_cnt < path-1 && get_IR(IR_F) > 40){//目標まで１マス以上残ってる　＆＆　前壁が出現
		
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
	    mae_kabe();//前壁距離補正
	    motor(0,0);	
			
	    //現在地の更新
	    my_x += dx[my_angle];
	    my_y += dy[my_angle];
			
	    maze_update(my_x,my_y,my_angle,3);//迷路情報の更新
	    //maza_update_flag = 0;
		
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
		//maza_update_flag = 0;
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
	    M_pwm = M_pwm_min + (enc_now / 10);	
			
	}else if(enc_now > (long long)path * s1 * 5/8){// 進んだ距離 > 目標距離 * 3/4 = //減速区間
	    M_pwm = M_pwm_min + ( ((long long)path * s1 - enc_now) / 10);	
			
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
    //Encoder_reset();
}


////////////////////////////////////////////////////////////////////////////////////////////////////////

/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */
/* 関 数 概 要：最短経路探索											  			            */
/* 関 数 詳 細：												                                   */
/* 引       数： 目的地のXY座標															    */
/* 戻  り   値： なし										    									*/
/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */ 
char shortest_path_search(short target_x,short target_y){
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
/* 関 数 概 要：最短経路探索											  			            */
/* 関 数 詳 細：												                                   */
/* 引       数： 目的地のXY座標															    */
/* 戻  り   値： なし										    									*/
/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */ 
char shortest_path_search_pickup(short target_x,short target_y){
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
	    if((0 <= nx && nx < W) && (0 <= ny && ny < H) && ((maze_w[y][x] & (1<<i)) == 0 ) && ((maze_w[y][x] & (1<<(i+4))) != 0 ) ){//確定マスのみ
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
/* 関 数 概 要：最短経路作成											  			            */
/* 関 数 詳 細：												                                   */
/* 引       数： 目的地のXY座標															    */
/* 戻  り   値： なし										    									*/
/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */ 
void make_shortest_path_list_pickup(short target_x,short target_y){
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
    	if((0 <= nx && nx < W) && (0 <= ny && ny < H) && ((maze_w[y][x] & (1<<angle)) == 0 ) && ((maze_w[y][x] & (1<<(angle+4))) != 0 ) ){//目の前が迷路内　&& 壁がない && 確定マス
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
    //Encoder_reset();
	
    short comand ,path_num,path_num_tmp;
 
    int r_flag = 0,r_flag2 = 0;//スラロームに変更するフラグ
//    int nx,ny,tx,ty;
    
    int time = 1;
    
    //既知区間加速 の距離補正
    int path_hosei[16] = {0,
 			 	  0,200,200,300,300,300,300,300, 
 			  	300,300,300,300,300,300,300};//path_numごとに距離補正する
				
    int run_speed = 35;
    int run_speed_up = 45;    //未知区間加速
    int run_speed_boost = 50; //既知区間加速
    
    int run_speed_kabe = 15; //次が停止の時
    int run_speed_kabe_sr = 20; //次がスラロームの時

  
    while(!queue_empty()){
	comand = dequeue();path_num = dequeue();
	status_log = comand;
	
	switch(comand){
	
	case -1://L
	
	    if(r_flag == 1){//スラロームに変更する
		
	    	//スラローム
		L_curve_search(sl90_search,true);
		
		//ESmotor(100 ,run_speed + run_fin_speed_offset,false,true);
		
	    }else{
		    delay(time);
			
		    if(get_IR(IR_R) > MAKE_KABE_tikai || (get_IR(IR_R) > 40 && get_IR(IR_R) < MAKE_KABE_tooi)){//右壁との距離が近い || 右壁との距離が遠い
			R_rotate(r90);
			
			delay(time);
			mae_kabe();//前壁距離補正
			
			delay(time);
			GyroSum_reset();
			L_rotate(l90);
			delay(time);
			GyroSum_reset();
			if(get_IR(IR_F) > 40){
				mae_kabe();//前壁距離補正
				delay(time);
			}
			maze_update(my_x,my_y,my_angle,3);
			L_rotate(l90);
			
		    }else{
			maze_update(my_x,my_y,my_angle,3);
		    	L_rotate(l90);
		    }
		    delay(time);
		    maze_update(my_x,my_y,my_angle,3);
		    
		    GyroSum_reset();
		    
		    r_flag = 0;
		    r_flag2 = 0;
	    }
	    break;
	case 0://S
		
	    path_num_tmp = path_num;
	    
	    if(r_flag == 1){//スラロームの直後
		    r_flag = 2;
		    path_num --;
	    }
	    
	    if(r_flag == 0){
		    if(get_IR(IR_R) > MAKE_KABE_tikai  || (get_IR(IR_R) > 40 && get_IR(IR_R) < MAKE_KABE_tooi)){//右壁との距離が近い || 右壁との距離が遠い
		    	
			R_rotate(r90);
			
			delay(time);
			mae_kabe();//前壁距離補正

			L_rotate(l90);
			delay(time);
			
			GyroSum_reset();
			
		    }else if(get_IR(IR_L) > MAKE_KABE_tikai || (get_IR(IR_L) > 40 && get_IR(IR_L) < MAKE_KABE_tooi)){//左壁との距離が近い || 左壁との距離が遠い
			L_rotate(l90);
		
			delay(time);
			mae_kabe();//前壁距離補正
			
			R_rotate(r90);
			delay(time);
			
			GyroSum_reset();
		    }
	    }
	    
	    if(queue_empty()){
		/*
		switch(my_angle){//一つ手前の座標を取得する
			    case 0:
			    	nx = my_x;
				ny = my_y - (path_num_tmp - 1);
				tx = my_x;
				ty = my_y - (path_num_tmp);
				break;
			    case 1:
			    	nx = my_x + (path_num_tmp - 1);
				ny = my_y;
				tx = my_x + (path_num_tmp );
				ty = my_y;
				break;
			    case 2:
			    	nx = my_x;
				ny = my_y + (path_num_tmp - 1);
				tx = my_x;
				ty = my_y + (path_num_tmp);
				break;
			    case 3:
			    	nx = my_x - (path_num_tmp - 1);
				ny = my_y;
				tx = my_x - (path_num_tmp);
				ty = my_y;
				break;
		} */
			
		if(path_num == 0){
			if(r_flag == 2){//スラロームの直後
				S_run(h1_2 ,run_speed + run_fin_speed_offset,4,true);//加速速め non_stop = 4
				r_flag = 0;
			}else{
			    	//不要
			}
		}else if(path_num == 1){
		/*	
			//一つ手前の壁が無ければ、壁無し→柱　で距離補正できる
			if((tx != Get_Goal_x() || ty != Get_Goal_y())   &&   ((maze_w[ny][nx] & (1 << (((my_angle+4 -1)%4))+4)) != 0) && ((maze_w[ny][nx] & (1 << (((my_angle+4 -1)%4)))) == 0)  ){ //左
				if(r_flag == 2){//スラロームの直後
					S_run(h1_2 ,run_speed + run_fin_speed_offset,4,true);//加速速め non_stop = 4
					r_flag = 0;
				}else{
			    		//不要
				}
				
				status_log = 3;//ログに壁切れ開始を記録するため
				
				S_run_kabe_rev(run_speed_kabe + run_fin_speed_offset,true,1);//左 
				
				status_log = 4;//ログに壁切れ終了を記録するため
				
				S_run(h1_2,run_speed + run_fin_speed_offset,4,true);//non_stop = 4
				
			}else if((tx != Get_Goal_x() || ty != Get_Goal_y())   &&   ((maze_w[ny][nx] & (1 << (((my_angle+4 +1)%4))+4)) != 0) && ((maze_w[ny][nx] & (1 << (((my_angle+4 +1)%4)))) == 0) ){//右
				if(r_flag == 2){//スラロームの直後
					S_run(h1_2,run_speed + run_fin_speed_offset,4,true);//加速速め non_stop = 4
					r_flag = 0;
				}else{
			    		//不要
				}
				
				status_log = 3;//ログに壁切れ開始を記録するため
				
				S_run_kabe_rev(run_speed_kabe + run_fin_speed_offset,true,2);//右
				
				status_log = 4;//ログに壁切れ終了を記録するため
				
				S_run(h1_2,run_speed + run_fin_speed_offset,4,true);//non_stop = 4
			
			//目標地点の横壁が無いことが確定なら　壁切れで距離補正できる
			}else if((tx != Get_Goal_x() || ty != Get_Goal_y())   &&   ((maze_w[ty][tx] & (1 << (((my_angle+4 -1)%4))+4)) != 0) && ((maze_w[ty][tx] & (1 << (((my_angle+4 -1)%4)))) == 0)  ){ //左
				if(r_flag == 2){//スラロームの直後
					S_run(h1_2 ,run_speed + run_fin_speed_offset,4,true);//加速速め non_stop = 4
					r_flag = 0;
				}else{
			    		//不要
				}
				
				status_log = 3;//ログに壁切れ開始を記録するため
				
				S_run_kabe(run_speed_kabe + run_fin_speed_offset,true,1);//左 
				
				status_log = 4;//ログに壁切れ終了を記録するため
				
				S_run(h1_2,run_speed + run_fin_speed_offset,4,true);//non_stop = 4
				
			}else if((tx != Get_Goal_x() || ty != Get_Goal_y())   &&   ((maze_w[ty][tx] & (1 << (((my_angle+4 +1)%4))+4)) != 0) && ((maze_w[ty][tx] & (1 << (((my_angle+4 +1)%4)))) == 0) ){//右
				if(r_flag == 2){//スラロームの直後
					S_run(h1_2,run_speed + run_fin_speed_offset,4,true);//加速速め non_stop = 4
					r_flag = 0;
				}else{
			    		//不要
				}
				
				status_log = 3;//ログに壁切れ開始を記録するため
				
				S_run_kabe(run_speed_kabe + run_fin_speed_offset,true,2);//右
				
				status_log = 4;//ログに壁切れ終了を記録するため
				
				S_run(h1_2,run_speed + run_fin_speed_offset,4,true);//non_stop = 4
				
			}else{*/
				if(r_flag == 2){//スラロームの直後
					S_run(h1_2 + s1,run_speed + run_fin_speed_offset,4,true);//加速速め non_stop = 4
					r_flag = 0;
				}else{
			    		S_run(s1,run_speed + run_fin_speed_offset,false,true);
			    		//S_run(s1,run_speed + run_fin_speed_offset,false,4);// w_flag = 4 串の壁補正あり
				}
			//}
			
		}else{
			/*
			//一つ手前の壁が無ければ、壁無し→柱　で距離補正できる
			if((tx != Get_Goal_x() || ty != Get_Goal_y())  &&  ((maze_w[ny][nx] & (1 << (((my_angle+4 -1)%4))+4)) != 0) && ((maze_w[ny][nx] & (1 << (((my_angle+4 -1)%4)))) == 0)  ){ //左
				if(r_flag == 2){//スラロームの直後
					S_run(h1_2 +  s1 * (long long)(path_num -1),run_speed_boost + run_fin_speed_offset,4,4);// w_flag = 4 串の壁補正あり //加速速め non_stop = 4
					r_flag = 0;
				}else{
			    		S_run(s1 * (long long)(path_num -1),run_speed_boost + run_fin_speed_offset,false,4);// w_flag = 4 串の壁補正あり
				}
				
				status_log = 3;//ログに壁切れ開始を記録するため
				
				S_run_kabe_rev(run_speed_kabe + run_fin_speed_offset,true,1);//左 
				
				status_log = 4;//ログに壁切れ終了を記録するため
				
				S_run(h1_2,run_speed + run_fin_speed_offset,4,true);//non_stop = 4
				
				
				
			}else if((tx != Get_Goal_x() || ty != Get_Goal_y())  &&  ((maze_w[ny][nx] & (1 << (((my_angle+4 +1)%4))+4)) != 0) && ((maze_w[ny][nx] & (1 << (((my_angle+4 +1)%4)))) == 0) ){//右
				if(r_flag == 2){//スラロームの直後
					S_run(h1_2 +  s1 * (long long)(path_num -1),run_speed_boost + run_fin_speed_offset,4,4);// w_flag = 4 串の壁補正あり //加速速め non_stop = 4
					r_flag = 0;
				}else{
			    		S_run(s1 * (long long)(path_num -1),run_speed_boost + run_fin_speed_offset,false,4);// w_flag = 4 串の壁補正あり
				}
				
				status_log = 3;//ログに壁切れ開始を記録するため
				
				S_run_kabe_rev(run_speed_kabe + run_fin_speed_offset,true,2);//右
				
				status_log = 4;//ログに壁切れ終了を記録するため
				
				S_run(h1_2,run_speed + run_fin_speed_offset,4,true);//non_stop = 4
			
			//目標地点の横壁が無いことが確定なら　壁切れで距離補正できる
			}else if((tx != Get_Goal_x() || ty != Get_Goal_y())   &&   ((maze_w[ty][tx] & (1 << (((my_angle+4 -1)%4))+4)) != 0) && ((maze_w[ty][tx] & (1 << (((my_angle+4 -1)%4)))) == 0)  ){ //左
				if(r_flag == 2){//スラロームの直後
					S_run(h1_2 +  s1 * (long long)(path_num -1),run_speed_boost + run_fin_speed_offset,4,4);// w_flag = 4 串の壁補正あり //加速速め non_stop = 4
					r_flag = 0;
				}else{
			    		S_run(s1 * (long long)(path_num -1),run_speed_boost + run_fin_speed_offset,false,4);// w_flag = 4 串の壁補正あり
				}
				
				status_log = 3;//ログに壁切れ開始を記録するため
				
				S_run_kabe(run_speed_kabe + run_fin_speed_offset,true,1);//左 
				
				status_log = 4;//ログに壁切れ終了を記録するため
				
				S_run(h1_2,run_speed + run_fin_speed_offset,4,true);//non_stop = 4
				
				
				
			}else if((tx != Get_Goal_x() || ty != Get_Goal_y())   &&   ((maze_w[ty][tx] & (1 << (((my_angle+4 +1)%4))+4)) != 0) && ((maze_w[ty][tx] & (1 << (((my_angle+4 +1)%4)))) == 0) ){//右
				if(r_flag == 2){//スラロームの直後
					S_run(h1_2 +  s1 * (long long)(path_num -1),run_speed_boost + run_fin_speed_offset,4,4);// w_flag = 4 串の壁補正あり //加速速め non_stop = 4
					r_flag = 0;
				}else{
			    		S_run(s1 * (long long)(path_num -1),run_speed_boost + run_fin_speed_offset,false,4);// w_flag = 4 串の壁補正あり
				}
				
				status_log = 3;//ログに壁切れ開始を記録するため
				
				S_run_kabe(run_speed_kabe + run_fin_speed_offset,true,2);//右
				
				status_log = 4;//ログに壁切れ終了を記録するため
				
				S_run(h1_2,run_speed + run_fin_speed_offset,4,true);//non_stop = 4
				
			}else{*/
				if(r_flag == 2){//スラロームの直後
					S_run(h1_2 +  (s1 * (long long)path_num) + path_hosei[path_num],run_speed_boost + run_fin_speed_offset,4,4);// w_flag = 4 串の壁補正あり //加速速め non_stop = 4
					r_flag = 0;
				}else{
					//S_run((s1 * (long long)path_num) + path_hosei[path_num],run_speed_boost + run_fin_speed_offset,false,true);
			    		S_run((s1 * (long long)path_num) + path_hosei[path_num] ,run_speed_boost + run_fin_speed_offset,false,4);// w_flag = 4 串の壁補正あり
				}	
			//}
		}
			
	    }else{
		   
		if(path_num > 0){
			if(r_flag == 2){//スラロームの直後
				r_flag = 0;
				r_flag2 = 1;
		    	}
		}
		
		if((queue_next(1) == 1 || queue_next(1) == -1) && queue_next(3) == 0){//スラロームに変更
			r_flag = 1;///////////////////////////////////////////////////////////////////////////これを無効化すると探索中にスラロームしなくなる
		}
		
		if(path_num == 0){//連続スラローム
			switch(my_angle){
			    case 0:
				maze_update3(my_x,my_y - path_num_tmp,my_angle,3);//柱の位置から壁を確認する
				break;
			    case 1:
				maze_update3(my_x + path_num_tmp,my_y,my_angle,3);//柱の位置から壁を確認する
				break;
			    case 2:
				maze_update3(my_x,my_y+path_num_tmp,my_angle,3);//柱の位置から壁を確認する
				break;
			    case 3:
				maze_update3(my_x - path_num_tmp,my_y,my_angle,3);//柱の位置から壁を確認する
				break;
			} 
		}else if(path_num == 1){
		    
		    if(r_flag2 == 1){//スラロームの直後
		    	if(r_flag == 1){//次がスラロームの時
				ESmotor(h1_2 ,run_speed + run_fin_speed_offset,true,true);// non_stop = 1
			}else{
		    		ESmotor(h1_2 ,run_speed + run_fin_speed_offset,4,true);//加速速め non_stop = 4
			}
			r_flag2 = 0;
		    }
		    
		    status_log = 3;//ログに壁切れ開始を記録するため
		    
		    if(queue_next(1) < 0){//次　左
		    
		    	if(r_flag == 1){//次がスラロームの時
				S_run_kabe(run_speed_kabe_sr + run_fin_speed_offset,true,1);
			}else{
				S_run_kabe(run_speed_kabe + run_fin_speed_offset,true,1);
				//S_run_kabe(run_speed_kabe + run_fin_speed_offset,4,1);//non_stop = 4
			}
							
		    }else if(queue_next(1) > 0){//次　右
		    
		    	if(r_flag == 1){//次がスラロームの時
				S_run_kabe(run_speed_kabe_sr + run_fin_speed_offset,true,2);
			}else{
				S_run_kabe(run_speed_kabe + run_fin_speed_offset,true,2);
				//S_run_kabe(run_speed_kabe + run_fin_speed_offset,4,2);//non_stop = 4
			}
		    }else{
			    
			S_run_kabe(run_speed + run_fin_speed_offset,true,3);
			//S_run_kabe(run_speed_kabe + run_fin_speed_offset,4,3);// w_flag = 4 串の壁補正あり
		    }
		    
		   status_log = 4;//ログに壁切れ終了を記録するため
		    
		    if(r_flag == 0){
		   	 S_run(h1_2,run_speed + run_fin_speed_offset,4,true);//non_stop = 4
		    	//S_run(h1,run_speed + run_fin_speed_offset,4,4);//non_stop = 4 // w_flag = 4 串の壁補正あり
		    }else{
			switch(my_angle){
			    case 0:
				maze_update3(my_x,my_y - path_num_tmp,my_angle,3);//柱の位置から壁を確認する
				break;
			    case 1:
				maze_update3(my_x + path_num_tmp,my_y,my_angle,3);//柱の位置から壁を確認する
				break;
			    case 2:
				maze_update3(my_x,my_y+path_num_tmp,my_angle,3);//柱の位置から壁を確認する
				break;
			    case 3:
				maze_update3(my_x - path_num_tmp,my_y,my_angle,3);//柱の位置から壁を確認する
				break;
			}    
		    }
					
		}else{
		    
		    
		    //Set_motor_pid_mode(1);//高速
		    
		    if(r_flag2 == 1){//スラロームの直後
		    	if(r_flag == 1){//次がスラロームの時
				S_run(h1_2 + (s1 * ((long long)path_num - 1)) + path_hosei[path_num-1],run_speed_boost + run_fin_speed_offset,true,4);//non_stop = 1 // w_flag = 4 串の壁補正あり
			}else{
		    		S_run(h1_2 + (s1 * ((long long)path_num - 1)) + path_hosei[path_num-1],run_speed_boost + run_fin_speed_offset,4,4);////加速速め non_stop = 4 // w_flag = 4 串の壁補正あり
		    	}
			r_flag2 = 0;
			
		    }else{
		    	//S_run((s1 * ((long long)path_num - 1)) + path_hosei[path_num-1],run_speed_boost + run_fin_speed_offset,3,true);//non_stop = 3
		    	S_run((s1 * ((long long)path_num - 1)) + path_hosei[path_num-1],run_speed_boost + run_fin_speed_offset,3,4);//non_stop = 3 // w_flag = 4 串の壁補正あり
		    }
		    
		   // Set_motor_pid_mode(0);//低速
		    
		    status_log = 3;//ログに壁切れ開始を記録するため
		    
		    if(queue_next(1) < 0){//次　左
		    	if(r_flag == 1){//次がスラロームの時
				S_run_kabe(run_speed_kabe_sr + run_fin_speed_offset,true,1);
			}else{
				S_run_kabe(run_speed_kabe + run_fin_speed_offset,true,1);
				//S_run_kabe(run_speed_kabe + run_fin_speed_offset,4,1);// w_flag = 4 串の壁補正あり
			}
						
		    }else if(queue_next(1) > 0){//次　右
		    	if(r_flag == 1){//次がスラロームの時
				S_run_kabe(run_speed_kabe_sr + run_fin_speed_offset,true,2);
			}else{
				S_run_kabe(run_speed_kabe + run_fin_speed_offset,true,2);
				//S_run_kabe(run_speed_kabe + run_fin_speed_offset,4,2);// w_flag = 4 串の壁補正あり
			}
		    }else{
			if(r_flag == 1){//次がスラロームの時
				S_run_kabe(run_speed_kabe_sr + run_fin_speed_offset,true,3);
			}else{
				S_run_kabe(run_speed_kabe + run_fin_speed_offset,true,3);
				//S_run_kabe(run_speed_kabe + run_fin_speed_offset,4,3);// w_flag = 4 串の壁補正あり
			}
		    }
			
		    status_log = 4;//ログに壁切れ終了を記録するため
		    
		    if(r_flag == 0){
		    	S_run(h1_2,run_speed + run_fin_speed_offset,4,true);//non_stop = 4
		    	//S_run(h1,run_speed + run_fin_speed_offset,4,4);//non_stop = 4 // w_flag = 4 串の壁補正あり
		    }else{
			switch(my_angle){
			    case 0:
				maze_update3(my_x,my_y - path_num_tmp,my_angle,3);//柱の位置から壁を確認する
				break;
			    case 1:
				maze_update3(my_x + path_num_tmp,my_y,my_angle,3);//柱の位置から壁を確認する
				break;
			    case 2:
				maze_update3(my_x,my_y+path_num_tmp,my_angle,3);//柱の位置から壁を確認する
				break;
			    case 3:
				maze_update3(my_x - path_num_tmp,my_y,my_angle,3);//柱の位置から壁を確認する
				break;
			}    
		    }
		    
		//    Set_motor_pid_mode(0);//低速
		}
	    }
		
	    if(r_flag == 0){
		    if(get_IR(IR_R) > MAKE_KABE_tikai  || (get_IR(IR_R) > 40 && get_IR(IR_R) < MAKE_KABE_tooi)){//右壁との距離が近い || 右壁との距離が遠い
			R_rotate(r90);
			
			delay(time);
			mae_kabe();//前壁距離補正

			L_rotate(l90);
			delay(time);
			
		    }else if(get_IR(IR_L) > MAKE_KABE_tikai || (get_IR(IR_L) > 40 && get_IR(IR_L) < MAKE_KABE_tooi)){//左壁との距離が近い || 左壁との距離が遠い
			L_rotate(l90);
		
			delay(time);
			mae_kabe();//前壁距離補正
			
			R_rotate(r90);
			delay(time);
			
		    }
		    
		    if(get_IR(IR_F) > 40){
		    	mae_kabe();//前壁距離補正
		    }
	    }
		
	    switch(my_angle){
	    case 0:
		my_y -= path_num_tmp;
		break;
	    case 1:
		my_x += path_num_tmp;
		break;
	    case 2:
		my_y += path_num_tmp;
		break;
	    case 3:
		my_x -= path_num_tmp;
		break;
	    }
	    //delay(time);
	    break;
		
	case 10://S 未確定の直線
	
	    r_flag = 0;
	    r_flag2 = 0;
	    
	    if(get_IR(IR_R) > MAKE_KABE_tikai  || (get_IR(IR_R) > 40 && get_IR(IR_R) < MAKE_KABE_tooi)){//右壁との距離が近い || 右壁との距離が遠い
		R_rotate(r90);
		
		delay(time);
		mae_kabe();//前壁距離補正

		L_rotate(l90);
		delay(time);
		
	    }else if(get_IR(IR_L) > MAKE_KABE_tikai || (get_IR(IR_L) > 40 && get_IR(IR_L) < MAKE_KABE_tooi)){//左壁との距離が近い || 左壁との距離が遠い
		L_rotate(l90);
	
		delay(time);
		mae_kabe();//前壁距離補正
		
		R_rotate(r90);
		delay(time);
		
	    }
	    
	    if(path_num == 1){
	  		
		S_run_maze_search(path_num,run_speed + run_fin_speed_offset,run_speed_up + run_fin_speed_offset,  6);//串のあり、なしは関数内で設定する
		
	    }else{
		S_run_maze_search(path_num,run_speed + run_fin_speed_offset,run_speed_up + run_fin_speed_offset ,  4);
		
	    }
		
	    if(get_IR(IR_R) > MAKE_KABE_tikai  || (get_IR(IR_R) > 40 && get_IR(IR_R) < MAKE_KABE_tooi)){//右壁との距離が近い || 右壁との距離が遠い
		R_rotate(r90);
		
		delay(time);
		mae_kabe();//前壁距離補正

		L_rotate(l90);
		delay(time);
		
	    }else if(get_IR(IR_L) > MAKE_KABE_tikai || (get_IR(IR_L) > 40 && get_IR(IR_L) < MAKE_KABE_tooi)){//左壁との距離が近い || 左壁との距離が遠い
		L_rotate(l90);
	
		delay(time);
		mae_kabe();//前壁距離補正
		
		R_rotate(r90);
		delay(time);
		
	    }
	    
	    if(get_IR(IR_F) > 40){
	    	mae_kabe();//前壁距離補正
	    }
	
	    maze_update(my_x,my_y,my_angle,3);
	    break;
	case 1://R
	    if(r_flag == 1){//スラロームに変更する
	    
	    	//スラローム
		R_curve_search(sr90_search,true);
		
		//ESmotor(100 ,run_speed + run_fin_speed_offset,false,true);
		
	    }else{
		    
		    delay(time);
			
		    if(get_IR(IR_L) > MAKE_KABE_tikai || (get_IR(IR_L) > 40 && get_IR(IR_L) < MAKE_KABE_tooi)){//左壁との距離が近い || 左壁との距離が遠い
			L_rotate(l90);
			
			delay(time);
			mae_kabe();//前壁距離補正
			
			delay(time);
			GyroSum_reset();
			R_rotate(r90);
			delay(time);
			GyroSum_reset();
			if(get_IR(IR_F) > 40){
				mae_kabe();//前壁距離補正
				delay(time);
			}
			
			maze_update(my_x,my_y,my_angle,3);
			R_rotate(r90);
			
		    }else{
			maze_update(my_x,my_y,my_angle,3);
		    	R_rotate(r90);
		    }
			
		    delay(time);
		    maze_update(my_x,my_y,my_angle,3);
		    GyroSum_reset();
		    
		    r_flag = 0;
		    r_flag2 = 0;
	    }
	    break;
	case 2://B
	
	    r_flag = 0;
	    r_flag2 = 0;
	    
	    delay(time);
	    GyroSum_reset();
	    
	    if(get_IR(IR_R) > MAKE_KABE_tikai  || (get_IR(IR_R) > 40 && get_IR(IR_R) < MAKE_KABE_tooi)){//右壁との距離が近い || 右壁との距離が遠い
		R_rotate(r90);
		
		delay(time);
		mae_kabe();//前壁距離補正

		maze_update(my_x,my_y,my_angle,3);
		R_rotate(r90);
		
	    }else if(get_IR(IR_L) > MAKE_KABE_tikai || (get_IR(IR_L) > 40 && get_IR(IR_L) < MAKE_KABE_tooi)){//左壁との距離が近い || 左壁との距離が遠い
		L_rotate(l90);
	
		delay(time);
		mae_kabe();//前壁距離補正
		
		maze_update(my_x,my_y,my_angle,3);
		L_rotate(l90);
		
	    }else{
		maze_update(my_x,my_y,my_angle,3);
	    	Tmotor(r180);
		my_angle = (4+my_angle+2)%4;
		delay(100);
	    }
		
	    delay(time);
	    maze_update(my_x,my_y,my_angle,3);
	    GyroSum_reset();
	    
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
   
    char path_ng = 0;
    
    GyroSum_reset();
   // Encoder_reset();

    led_down();
	
    while(1){
	maze_update(my_x,my_y,my_angle,3);
	/*if((target_x != Get_Goal_x() || target_y != Get_Goal_y()) && (target_x != Start_x || target_y != Start_y)){//スタート地点、ゴール地点以外が目標地点のとき
	  if((maze_w[target_y][target_x] & 0xf0) == 0xf0)break;    //目標地点の壁がすべて確定したら探索完了  
	  }*/
	
	path_ng = shortest_path_search(Get_Goal_x(),Get_Goal_y());
	
	if(path_ng == 1){//最短経路が存在しない→迷路情報を元に戻す
		for(int i = 0; i < H;i++){
			for(int j = 0; j < W; j++){
				maze_w[i][j] = maze_w_backup[i][j];
				for(int k = 0; k < 4;k++)maze_d[i][j][k] = maze_d_backup[i][j][k];
			}
		
	    	}
	}else{//迷路情報をバックアップする
		for(int i = 0; i < H;i++){
			for(int j = 0; j < W; j++){
				maze_w_backup[i][j] = maze_w[i][j];
				for(int k = 0; k < 4;k++)maze_d_backup[i][j][k] = maze_d[i][j][k];
			}
		
	    	}	
	}
	
	if(target_x == my_x && target_y == my_y){//ゴール
	    motor(0,0);
	    led_up();
			
	    if((target_x == Start_x && target_y == Start_y) || (target_x == Get_Goal_x() && target_y == Get_Goal_y())){
				
		GyroSum_reset();
		
		mae_kabe();//前壁距離補正
		
		Tmotor(r180);
		motor(0,0);
		delay(100);
		
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
/* 関 数 概 要：ピックアップ位置まで走行 探索走行と同等の設定											  			            */
/* 関 数 詳 細：												                                   */
/* 引       数： 目的地のXY座標															    */
/* 戻  り   値： なし										    									*/
/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */ 
void run_pickup(short target_x,short target_y){
   

    GyroSum_reset();
   // Encoder_reset();

    led_down();
		
    
    shortest_path_search_pickup(target_x,target_y);
    
    make_shortest_path_list_pickup(target_x,target_y);
    
    run_shortest_path();

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
/* 関 数 概 要：斜めも考慮して最短経路上の未確定マスを探す	未確定マスも通過する					*/
/* 関 数 詳 細：												                                   */
/* 引       数： なし														    */
/* 戻  り   値： なし										    									*/
/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */ 
void shortest_path_search_perfect_unknown(short* target_x,short* target_y){

    int comand ,path_num;

    char my_x_tmp = my_x,my_y_tmp = my_y,my_angle_tmp = my_angle;//現在位置のバックアップ
    
    led(0);
    ////////////// ゴールからの距離を計算する
 /*   queue_reset();
    for(int i = 0; i < H;i++){
	for(int j = 0;j < W; j++){
	    for(int k = 0; k < 4; k++){
		maze_d[i][j][k] = maze_d_max;
	    }
	}
    }
    for(int k = 0; k < 4; k++){
	if(((maze_w[Get_Goal_y()][Get_Goal_x()] & (1<<k)) == 0 )){
	    maze_d[Get_Goal_y()][Get_Goal_x()][k] = 0;
	}
    }
    enqueue(Get_Goal_x()*100 + Get_Goal_y());
  
    while(!queue_empty()){
	short x = dequeue(),y;
	y = x%100;
	x /=100;

	for(char i =0;i<4;i++){
	    char update_flag = 0;
	    short nx = x+dx[i],ny = y+dy[i];
	    if((0 <= nx && nx < W) && (0 <= ny && ny < H) && ((maze_w[y][x] & (1<<i)) == 0 )  ){

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
 */   
   shortest_path_search(Get_Goal_x(),Get_Goal_y());
   
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
	    
	   if(i == Get_Goal_y() && j == Get_Goal_x()){
	   	maze_d_perfect[Get_Goal_y()][Get_Goal_x()] = 0; 
		
	   }else if (maze_d[i][j][0] >= maze_d_max){//到達不能マス
	   	maze_d_perfect[i][j] = maze_d_max;
		   
	   }else{//走行経路を算出する
	   	
	   	 //run_list		
		maze_d_perfect[i][j] = maze_d_max;
		
		for(int k = 0;k < 4;k++){//スタート向きの設定
			//printf2("x=%d y=%d a=%d\n",j,i,k);
			
			queue_reset();
    			short h_path = 0;
    			my_x = j;my_y = i;my_angle = 0;//スタート位置の設定
			
			/*if(maze_d[i][j][my_angle] > maze_d[i][j][k]){
				my_angle = k;
			}*/
			if((maze_w[my_y][my_x] & (1 << ((k+2)%4))) != 0){//目の前に壁がある場合スタートの向きにはならない
				//printf2("NG \n");
				continue;
			}
			my_angle = k;
			
			my_angle = (my_angle+2)%4;
			//printf2("%d ",my_angle);
			int last = 0;
	
		    	while(my_x != Get_Goal_x() || my_y != Get_Goal_y()){

				short num = maze_d[my_y][my_x][(my_angle+2)%4];
				short n_num = 0;
				char s_flag = 0;
				int nx = my_x+dx[my_angle],ny = my_y+dy[my_angle];
			
			
				if((0 <= nx && nx < W) && (0 <= ny && ny < H) && ((maze_w[my_y][my_x] & (1<<my_angle)) == 0 ) ){
			    		short next = maze_d[ny][nx][(my_angle+2)%4];
			    		if(num == next+1){
						n_num = (my_angle+2)%4;
						num = next;
						s_flag = true;
			    		}
				}

				if(s_flag == false){
			    		for(int ii = 0;ii < 4;ii++){
						if(ii == (my_angle+2)%4){//逆走はありえない
						}else{// L or R
				    			short next = maze_d[my_y][my_x][ii];
				    			if(num > next){
								n_num = ii;
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
		  // printf2("1-OK %d %d num=%d n_num=%d\n",my_x,my_y,num,n_num);
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
					
					case 2://B  最短走行で逆走はありえない
						
						my_angle = (4+my_angle+2)%4;//メモ　無限ループ回避のため、仕方なく実装
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
		    
		     //printf2("2-OK ");
			 //ここまでで走行経路が算出完了 
			 remake_shortest_path_list_naname2(); //２マスも斜めにするモード
		   	 path_compression();//大曲など  
			 
			 //走行経路から距離に変換
			 short maze_d_perfect_tmp = 0;
			 
			 while(!queue_empty()){
				comand = dequeue();path_num = dequeue();
				//printf2("%d %d\n",comand,path_num);
				//delay(1)
				
				if(comand == 0){//直線
					maze_d_perfect_tmp += max(1,(path_num+1) / 2) * 2; //メモ　斜め1マス＝１としたときの直線の重み
					
				}else if(comand == 10){//斜め直線
					maze_d_perfect_tmp += max(1,path_num);  //斜め１マス
					
				}else if(comand == -11 || comand == -13 || comand == -14 || comand == 11 || comand == 13 || comand == 14){//斜め45
					maze_d_perfect_tmp += get_r45_cost();  //１マス
					
				}else{//カーブ
					maze_d_perfect_tmp += path_num * get_r_cost();
				}
				
			} 
			 
			 //printf2("\td=%d\n",maze_d_perfect_tmp);
			 
			//距離情報の更新確認
			maze_d_perfect[i][j] = min(maze_d_perfect_tmp, maze_d_perfect[i][j]);
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
    char maze_flag[H][W] = {0};
     
    my_x = Start_x;my_y = Start_y;my_angle = Start_angle;
 
    int last = 0;
	
    while(my_x != Get_Goal_x() || my_y != Get_Goal_y()){

	short num = maze_d_max -100; //maze_d_perfect[my_y][my_x];  周囲のマスが現在地より小さいとは言えないため最大値-100に変更
	short n_num = 0;
	char first_flag = 0;
	
 	maze_flag[my_y][my_x] = 1;//一度到達したマスには戻らないようにする
	
	for(int i = 0;i < 4;i++){//ゴールに近いマスを探す
		int nx = my_x+dx[i],ny = my_y+dy[i];
		
		//迷路の範囲内　＆＆　壁が無いことが確定している
		if((0 <= nx && nx < W) && (0 <= ny && ny < H) && ((maze_w[my_y][my_x] & (1<<i)) == 0 )){//  && ((maze_w[my_y][my_x] & (1<<(4+i))) != 0 ) ){
			
			if(maze_flag[ny][nx] != 1){//まだ到達してなければ
				if(first_flag == 0 || num > maze_d_perfect[ny][nx]){//初めのマスは無条件で移動す候補にする || ゴールに近いマスを見つけた
					num = maze_d_perfect[ny][nx];
					n_num = i;
					
					first_flag = 1;
					
				 }else if(num == maze_d_perfect[ny][nx]){// LとRが同じ重み　斜めを優先したい
				 
				 	if( maze_d[ny][nx][i] < maze_d[my_y+dy[n_num]][my_x+dx[n_num]][n_num] ){//斜めを考慮しない重みの小さいほうを優先する
						n_num = i;
						
					}else if( maze_d[ny][nx][i] == maze_d[my_y+dy[n_num]][my_x+dx[n_num]][n_num]) {
						if(last == -1 && (i - my_angle + 4)%4 == 1   ){//前回がL かつ　今回はR  
							n_num = i;
								 
						}else if(last == 1 && (i - my_angle + 4)%4 == -1   ){//前回がR　かつ　今回はL
							n_num = i;
								 
						}else{//前回がSなら今回は?
							//わからんから先に見つかった方にする
						}
					}

					first_flag = 1;
				}
			}
		}
	}
	
	if((maze_w[my_y][my_x] & (1<<(4+n_num))) == 0){//移動する向きが未確定の壁だった場合
		*target_x = my_x + dx[n_num];
		*target_y = my_y + dy[n_num];
		
		//現在位置をバックアップから復元
		my_x = my_x_tmp;
		my_y = my_y_tmp;
   		my_angle = my_angle_tmp;
		return;
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
	 
	 case 2://B
	 	if(h_path > 0){
			if(queue_empty())h_path--;
			enqueue(0);
			enqueue(h_path);
			h_path = 0;
		}
		
	 	enqueue(2);
	    	enqueue(1);
	 	my_angle = (4+my_angle+2)%4;
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
    
   //未確定マスを通らずにゴールまで経路を確認できた。
    *target_x = Get_Goal_x();
    *target_y = Get_Goal_y(); 
    
    //現在位置をバックアップから復元
    my_x = my_x_tmp;
    my_y = my_y_tmp;
    my_angle = my_angle_tmp;
  
}

/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */
/* 関 数 概 要：					*/
/* 関 数 詳 細：												                                   */
/* 引       数： なし														    */
/* 戻  り   値： なし										    									*/
/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */ 
int get_node_num(int x,int y,int a){
/*
  ノード番号の考え方
  
  １マスにつき3つのノードを作成する　中央、上、左
  
   入力
   	x:0~15
	y:0~15
	a:0~4 (中央、上、左、右、下）
	
		右の場合
			x+1 の 左　に置き換える
		下の場合
			y+1 の 上に置き換える
	
	ノード番号= ((y * 16) + x ) + (a * 256)
   
 */
 	//aの変換 (上、右、下、左、中央) →(中央、上、左、右、下）
	switch(a){
		case 0:
			a = 1;
			break;
		case 1:
			a = 3;
			break;
		case 2:
			a = 4;
			break;
		case 3:
			a = 2;
			break;
		case 4:
			a = 0;
			break;
	}
	
 	if(a == 3){//右の場合
		if(x < 15){ //一番したのマスの場合でなければ
			//x+1 の 左　に置き換える
			x++;
			a = 2;
		}else{
			return (3 * 256)+ y ;
		}
	}else if(a == 4){//下の場合
		if(y < 15){ //一番したのマスの場合でなければ
			//y+1 の 上に置き換える
			y++;
			a = 1;
		}else{
			return (3 * 256)+ 16 + x ;
		}
	}
	
	return ((y * 16) + x ) + (a * 256);
}
/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */
/* 関 数 概 要：					*/
/* 関 数 詳 細：												                                   */
/* 引       数： なし														    */
/* 戻  り   値： なし										    									*/
/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */ 
void shortest_path_search_dijkstra_unknown(short* target_x,short* target_y){

    char my_x_tmp = my_x,my_y_tmp = my_y,my_angle_tmp = my_angle;//現在位置のバックアップ
    
    led(0);
 
    int node_num_centor;
    
    init_dijkstra();
    
    //壁情報からパス情報を生成
    		//add_edge(int, int, int );
    for(int i = 0; i < H; i++){
	    for(int j = 0; j < W;j++){
		    
		    node_num_centor = get_node_num(j,i,4);
		    
		    for(int k = 0; k < 4;k++){
			    
			   //マスの中央と壁
			   if((maze_w[i][j] & (1 << k)) == 0) {//壁が無ければマスの中央とつながってる  未確定の壁は壁無しとする
				   add_edge(get_node_num(j,i,k), node_num_centor, cost_centor_wall );
				   
				   //printf2("x=%d y=%d a=%d\n",j,i,k);
				   //斜めの壁と壁
				   if((maze_w[i][j] & (1 << ((k+1)%4))) == 0){ //未確定の壁は壁無しとする
					   add_edge(get_node_num(j,i,k), get_node_num(j,i,((k+1)%4))  ,cost_wall_wall );
				   }
				   if(((maze_w[i][j] & (1 << ((k+4-1)%4)))) == 0){ //未確定の壁は壁無しとする
					   add_edge(get_node_num(j,i,k), get_node_num(j,i,((k+4-1)%4))  , cost_wall_wall );
				   }
			   }   
		    }
	    }
    }
    
    //ゴール地点からダイクストラを実行
    run_dijkstra(get_node_num(Get_Goal_x(),Get_Goal_y(),4)); //ゴールのマス中央を設定する

    //距離情報から歩数マップを生成
    		//long get_dist(int);
     for(int i = 0; i < H; i++){
	    for(int j = 0; j < W;j++){
		    maze_d_dijkstra[i][j] = get_dist(get_node_num(j,i,4));//ゴールからマスの中央の距離を設定する
		    
	    }
     }
    
    //歩数マップから走行リストを生成
    //run_list
    queue_reset();
    short h_path = 0;
    char maze_flag[H][W] = {0};
     
    my_x = Start_x;my_y = Start_y;my_angle = Start_angle;
 
    int last = 0;
	
    while(my_x != Get_Goal_x() || my_y != Get_Goal_y()){

	short num = maze_d_max -100; //maze_d_dijkstra[my_y][my_x];  周囲のマスが現在地より小さいとは言えないため最大値-100に変更
	short n_num = 0;
	char first_flag = 0;
	
 	maze_flag[my_y][my_x] = 1;//一度到達したマスには戻らないようにする
	
	for(int i = 0;i < 4;i++){//ゴールに近いマスを探す
		int nx = my_x+dx[i],ny = my_y+dy[i];
		
		//迷路の範囲内　＆＆　壁が無いことが確定している
		if((0 <= nx && nx < W) && (0 <= ny && ny < H) && ((maze_w[my_y][my_x] & (1<<i)) == 0 )){//  && ((maze_w[my_y][my_x] & (1<<(4+i))) != 0 ) ){
			
			if(maze_flag[ny][nx] != 1){//まだ到達してなければ
				if(first_flag == 0 || num > maze_d_dijkstra[ny][nx]){//初めのマスは無条件で移動す候補にする || ゴールに近いマスを見つけた
					num = maze_d_dijkstra[ny][nx];
					n_num = i;
					
					first_flag = 1;
					
				 }else if(num == maze_d_dijkstra[ny][nx]){// LとRが同じ重み　斜めを優先したい
				 
				 	if( maze_d[ny][nx][i] < maze_d[my_y+dy[n_num]][my_x+dx[n_num]][n_num] ){//斜めを考慮しない重みの小さいほうを優先する
						n_num = i;
						
					}else if( maze_d[ny][nx][i] == maze_d[my_y+dy[n_num]][my_x+dx[n_num]][n_num]) {
						if(last == -1 && (i - my_angle + 4)%4 == 1   ){//前回がL かつ　今回はR  
							n_num = i;
								 
						}else if(last == 1 && (i - my_angle + 4)%4 == -1   ){//前回がR　かつ　今回はL
							n_num = i;
								 
						}else{//前回がSなら今回は?
							//わからんから先に見つかった方にする
						}
					}

					first_flag = 1;
				}
			}
		}
	}
	
	if((maze_w[my_y][my_x] & (1<<(4+n_num))) == 0){//移動する向きが未確定の壁だった場合
		*target_x = my_x + dx[n_num];
		*target_y = my_y + dy[n_num];
		
		//現在位置をバックアップから復元
		my_x = my_x_tmp;
		my_y = my_y_tmp;
   		my_angle = my_angle_tmp;
		return;
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
	 
	 case 2://B
	 	if(h_path > 0){
			if(queue_empty())h_path--;
			enqueue(0);
			enqueue(h_path);
			h_path = 0;
		}
		
	 	enqueue(2);
	    	enqueue(1);
	 	my_angle = (4+my_angle+2)%4;
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
    
   //未確定マスを通らずにゴールまで経路を確認できた。
    *target_x = Get_Goal_x();
    *target_y = Get_Goal_y(); 
    
    
    
    //現在位置をバックアップから復元
    my_x = my_x_tmp;
    my_y = my_y_tmp;
    my_angle = my_angle_tmp;
  
}

/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */
/* 関 数 概 要：					*/
/* 関 数 詳 細：												                                   */
/* 引       数： なし														    */
/* 戻  り   値： なし										    									*/
/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */ 
void shortest_path_search_dijkstra(){

    char my_x_tmp = my_x,my_y_tmp = my_y,my_angle_tmp = my_angle;//現在位置のバックアップ
    
    led(0);
 
    int node_num_centor;
    
    init_dijkstra();
    
    //壁情報からパス情報を生成
    		//add_edge(int, int, int );
    for(int i = 0; i < H; i++){
	    for(int j = 0; j < W;j++){
		    
		    node_num_centor = get_node_num(j,i,4);
		    
		    for(int k = 0; k < 4;k++){
			    
			   //マスの中央と壁
			   if(  ((maze_w[i][j] & (1 << (k+4))) != 0) && ((maze_w[i][j] & (1 << k)) == 0)) {//壁が無ければマスの中央とつながってる 確定壁のみ
				   add_edge(get_node_num(j,i,k), node_num_centor, cost_centor_wall );
				   
				   //printf2("x=%d y=%d a=%d\n",j,i,k);
				   //斜めの壁と壁
				   if(((maze_w[i][j] & (1 << (((k+1)%4)+4) )) != 0) && ((maze_w[i][j] & (1 << ((k+1)%4))) == 0)){ //確定壁
					   add_edge(get_node_num(j,i,k), get_node_num(j,i,((k+1)%4))  ,cost_wall_wall );
				   }
				   if((((maze_w[i][j] & (1 << (((k+4-1)%4)+4) ))) != 0) &&  (((maze_w[i][j] & (1 << ((k+4-1)%4)))) == 0)){ //確定壁
					   add_edge(get_node_num(j,i,k), get_node_num(j,i,((k+4-1)%4))  , cost_wall_wall );
				   }
			   }   
		    }
	    }
    }
    
    //ゴール地点からダイクストラを実行
    run_dijkstra(get_node_num(Get_Goal_x(),Get_Goal_y(),4)); //ゴールのマス中央を設定する

    //距離情報から歩数マップを生成
    		//long get_dist(int);
     for(int i = 0; i < H; i++){
	    for(int j = 0; j < W;j++){
		    maze_d_dijkstra[i][j] = get_dist(get_node_num(j,i,4));//ゴールからマスの中央の距離を設定する
		    
	    }
     }
    
    //歩数マップから走行リストを生成
    //run_list
    queue_reset();
    short h_path = 0;
    char maze_flag[H][W] = {0};
    
    my_x = Start_x;my_y = Start_y;my_angle = Start_angle;
 
    int last = 0;
	
    
    while(my_x != Get_Goal_x() || my_y != Get_Goal_y()){
	
	short num = maze_d_max -100; //maze_d_dijkstra[my_y][my_x];  周囲のマスが現在地より小さいとは言えないため最大値-100に変更
	short n_num = 0;
	char first_flag = 0;
	
	maze_flag[my_y][my_x] = 1;//一度到達したマスには戻らないようにする
 
	//printf2("%d  %d\n",my_x,my_y);
	 
	for(int i = 0;i < 4;i++){//ゴールに近いマスを探す
		int nx = my_x+dx[i],ny = my_y+dy[i];
		
		//迷路の範囲内　＆＆　壁が無いことが確定している
		if((0 <= nx && nx < W) && (0 <= ny && ny < H) && ((maze_w[my_y][my_x] & (1<<i)) == 0 )  && ((maze_w[my_y][my_x] & (1<<(4+i))) != 0 ) ){
			
			if(maze_flag[ny][nx] != 1){//まだ到達してなければ
				if(first_flag == 0 || num > maze_d_dijkstra[ny][nx]){//初めのマスは無条件で移動す候補にする || ゴールに近いマスを見つけた
					num = maze_d_dijkstra[ny][nx];
					n_num = i;
					
					first_flag = 1;
					
				 }else if(num == maze_d_dijkstra[ny][nx]){// LとRが同じ重み　斜めを優先したい
				 
				 	//printf2("hoge   %d ,  %d  , %d\n",my_angle , i, (i - my_angle + 4)%4);
					
					if( maze_d[ny][nx][i] < maze_d[my_y+dy[n_num]][my_x+dx[n_num]][n_num] ){//斜めを考慮しない重みの小さいほうを優先する
						n_num = i;
						
					}else if( maze_d[ny][nx][i] == maze_d[my_y+dy[n_num]][my_x+dx[n_num]][n_num]) {
						if(last == -1 && (i - my_angle + 4)%4 == 1   ){//前回がL かつ　今回はR  
							n_num = i;
								 
						}else if(last == 1 && (i - my_angle + 4)%4 == -1   ){//前回がR　かつ　今回はL
							n_num = i;
								 
						}else{//前回がSなら今回は?
							//わからんから先に見つかった方にする
						}
					}
					
					first_flag = 1;
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
  
    //現在位置をバックアップから復元
    my_x = my_x_tmp;
    my_y = my_y_tmp;
    my_angle = my_angle_tmp;
  
}
    
/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */
/* 関 数 概 要：最短経路上の未確定マスを探す											            */
/* 関 数 詳 細：												                                   */
/* 引       数： 																				    */
/* 戻  り   値： なし										    									*/
/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */ 
void maze_search_unknown(short* target_x,short* target_y){
 
    short x = Start_x,y = Start_y,angle = Start_angle;
    int last = 0;
    
    short Goal_x_tmp,Goal_y_tmp;
    

    Goal_x_tmp = Get_Goal_x();
    Goal_y_tmp = Get_Goal_y();
	    
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
    *target_x = Get_Goal_x();
    *target_y = Get_Goal_y();

}

/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */
/* 関 数 概 要：4方向すべての壁が確定していないマスからゴールに近いマスの座標を取得 無ければゴール座標を設定する           */
/* 関 数 詳 細：												                                   */
/* 引       数： 																				    */
/* 戻  り   値： なし										    									*/
/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */ 
void maze_search_unknown_wall(short* target_x,short* target_y){
 
    char target_flag = 0;
    
    short tmp_x = -1,tmp_y = -1,tmp_maze_d;
    
    for(int y = 0; y < H; y++){
	for(int x = 0; x < W; x++){
		
		if((maze_w[y][x] & 0xF0) == 0){//確定壁が１つも無ければ
			
			short tmp_maze_d2 = min(maze_d[y][x][3],min(maze_d[y][x][2],min(maze_d[y][x][1], maze_d[y][x][0])));
			
			
			if(tmp_x == -1 && tmp_maze_d2 < maze_d_max){
				tmp_x = x;
				tmp_y = y;
				tmp_maze_d = tmp_maze_d2;
				
				target_flag = 1;
				
			}else{
				if(tmp_maze_d > tmp_maze_d2 ){
					tmp_x = x;
					tmp_y = y;
					tmp_maze_d = tmp_maze_d2;
				}
			}
		}
			
	}
    }
    
    if(target_flag == 0){//１つも対象のマスが無ければ
	    *target_x = Get_Goal_x();
	    *target_y = Get_Goal_y();
	    
	    return;
    }
    
    *target_x = tmp_x;
    *target_y = tmp_y;

}

/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */
/* 関 数 概 要：最短経路はすべて確定マスにする  										            */
/* 関 数 詳 細：												                                   */
/* 引       数： 																				    */
/* 戻  り   値： なし										    									*/
/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */ 
void maze_search_all(){
    GyroSum_reset();
   // Encoder_reset();

    led_down();
	
    short target_x,target_y;
    //short target_x_tmp,target_y_tmp;
    char path_ng = 0;
    
    static char phese_flag = 0;//0=大まかに探索, 　1=最短経路の未確定マスを探索
    
    
    //time_limit = xxxx;//60秒  別のところで設定するように変更
	
    while(time_limit > 0){//制限時間の間走行可能
	path_ng = 0;
	
	maze_update(my_x,my_y,my_angle,3);
	
	path_ng = shortest_path_search(Get_Goal_x(),Get_Goal_y());
	//shortest_path_search(Start_x,Start_y);
	
	if(path_ng == 1){//最短経路が存在しない→迷路情報を元に戻す
		for(int i = 0; i < H;i++){
			for(int j = 0; j < W; j++){
				maze_w[i][j] = maze_w_backup[i][j];
				for(int k = 0; k < 4;k++)maze_d[i][j][k] = maze_d_backup[i][j][k];
			}
		
	    	}
	}else{//迷路情報をバックアップする
		for(int i = 0; i < H;i++){
			for(int j = 0; j < W; j++){
				maze_w_backup[i][j] = maze_w[i][j];
				for(int k = 0; k < 4;k++)maze_d_backup[i][j][k] = maze_d[i][j][k];
			}
		
	    	}	
	}
	
	//これ以降は確実に最短経路が存在する迷路情報を持っている必要がある
	
	if(phese_flag == 0){
		maze_search_unknown(&target_x,&target_y);//最短経路上の未確定マスの座標を取得	
		
		if(target_x == Get_Goal_x() && target_y == Get_Goal_y()){//最短経路上に未確定マスがなければ
			phese_flag = 1;
		}
		
	}
	
	if(phese_flag == 1){
		shortest_path_search_perfect_unknown(&target_x,&target_y);//斜めも考慮した最短経路上の未確定マスの座標を取得
		
		if(target_x == Get_Goal_x() && target_y == Get_Goal_y()){//最短経路上に未確定マスがなければ
			phese_flag = 2;
		}
	}
	
	if(phese_flag == 2){
		shortest_path_search_dijkstra_unknown(&target_x,&target_y);//ダイクストラ　最短経路上の未確定マスの座標を取得
		
		if(target_x == Get_Goal_x() && target_y == Get_Goal_y()){//最短経路上に未確定マスがなければ
			motor(0,0);
							
			maze_search_adachi(Start_x,Start_y);
			led_down();
			led_up();
			led_down();
			led_up();
						
			motor(0,0);
			return;
		}
	}
	
	
		
		/*
			//確実に最短経路にならないマスも探索することになる　無効化する
			
			if(phese_flag == 0){//大まかに探索
	
				shortest_path_search(my_x,my_y);//ゴールに近いマスではなく、現在位置に近いマスから探索する場合
				
				maze_search_unknown_wall(&target_x_tmp,&target_y_tmp);//4方向すべての壁が確定していないマスからゴールに近いマスの座標を取得
				
				if(target_x_tmp == Get_Goal_x() && target_y_tmp == Get_Goal_y()){//ゴール座標が設定されているときは対象のマスがないとき
					phese_flag = 1;
					
				}else{//目標地点を大まかに探索した結果に置き換える
					target_x = target_x_tmp;
					target_y = target_y_tmp;
				}
				
			}
		*/	
		
	
		
	
    	shortest_path_search(target_x,target_y);
	make_shortest_path_list(target_x,target_y); //未確定マスでも連続する直線なら進む
    	//make_shortest_path_list_simple(target_x,target_y); //未確定マスでとまる
	
	run_shortest_path();
  	
	motor(0,0);

    }
	
    
    
    if(time_limit <= 0){//　制限時間内に探索できなかった　
		
	//maze_search_adachi(Get_Goal_x(),Get_Goal_y());
	//maze_search_adachi(Start_x,Start_y);//スタート地点に戻る
	maze_search_adachi(pickup_x,pickup_y);//拾いやすいところまで移動する
			
	
	//以下は最短経路を確定できたかどうかの確認用
	shortest_path_search(Get_Goal_x(),Get_Goal_y());
	
	maze_search_unknown(&target_x,&target_y);//最短経路上の未確定マスの座標を取得 
	
	
/*	while(1){
		motor(0,0);
		if(get_sw() == 1){
			printf2("%d : %d \n",target_x,target_y);	
		}
	}
*/
	if(target_x == Get_Goal_x() && target_y == Get_Goal_y()){//最短経路上に未確定マスがなければ
		
		shortest_path_search_perfect_unknown(&target_x,&target_y);//斜めも考慮した最短経路上の未確定マスの座標を取得 
	
		if(target_x == Get_Goal_x() && target_y == Get_Goal_y()){//最短経路上に未確定マスがなければ
		
			shortest_path_search_dijkstra_unknown(&target_x,&target_y);//ダイクストラ　最短経路上の未確定マスの座標を取得
			
			if(target_x == Get_Goal_x() && target_y == Get_Goal_y()){//最短経路上に未確定マスがなければ

		    		led_down();
				led_up();
				led_down();
				led_up();
				
			}else{//ダイクストラは未確定マスがある
				led(15);
			        delay(500);
			        led(0);
			        delay(500);
			        led(15);
			        delay(500);
		    
		        	Tmotor(l45 /2);//45度 / 2 回転し、最後まで探索できなかったことをわかるようにする
			}
			
		}else{//斜め考慮は未確定マスがある
			led(15);
		        delay(500);
		        led(0);
		        delay(500);
		        led(15);
		        delay(500);
		    
		        Tmotor(r45 /2);//45度 / 2 回転し、最後まで探索できなかったことをわかるようにする
		}
	}else{
	    led(15);
	    delay(500);
	    led(0);
	    delay(500);
	    led(15);
	    delay(500);
	    
	    Tmotor(r45);//45度回転し、最後まで探索できなかったことをわかるようにする
	}
    }
}

/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */
/* 関 数 概 要：最短経路が存在するか確認する(未確定の壁は通過する）											  			            */
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
	//if(((maze_w[Get_Goal_y()][Get_Goal_x()] & (1<<k)) == 0 ) && ((maze_w[Get_Goal_y()][Get_Goal_x()] & (1<<(4+k))) != 0 )){
	    maze_d[Get_Goal_y()][Get_Goal_x()][k] = 0;
	//}
    }
    enqueue(Get_Goal_x()*100 + Get_Goal_y());
  
    while(!queue_empty()){
	short x = dequeue(),y;
	y = x%100;
	x /=100;

	for(char i =0;i<4;i++){
	    char update_flag = 0;
	    short nx = x+dx[i],ny = y+dy[i];
	    //if((0 <= nx && nx < W) && (0 <= ny && ny < H) && ((maze_w[y][x] & (1<<i)) == 0 )  && ((maze_w[y][x] & (1<<(4+i))) != 0 )  ){//未確定の壁は通過しない
	    if((0 <= nx && nx < W) && (0 <= ny && ny < H) && ((maze_w[y][x] & (1<<i)) == 0 )   ){

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
		if(update_flag){
			enqueue(nx*100 + ny);
		
			if(Start_y == ny && Start_x == nx){
				for(int k = 0; k < 4; k++){
		    			if(maze_d[Start_y][Start_x][k] != maze_d_max ){//スタート位置の重みが更新されてなかったら＝最短経路が存在しない
		    				return 0;
		    			}
	    			}
			}
		}
	    }
	}
    }
    
    /*
    for(int i = 0; i < H;i++){
		for(int j = 0; j < W; j++)printf2("%d\t",maze_w[i][j]&0x0f);
		printf2("\n");
    }
    printf2("\n");

    for(int k = 0; k < 4; k++){
    	printf2("%d \n",maze_d[Get_Goal_y()][Get_Goal_x()][k]);
    	
    }
    
    for(int k = 0; k < 4; k++){
    	printf2("%d \n",maze_d[Start_y][Start_x][k]);
    	
    }
    
    */ 
    
    
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
/* 関 数 概 要：最短経路が存在するか確認する(未確定の壁はしない）											  			            */
/* 関 数 詳 細：												                                   */
/* 引       数： なし														    */
/* 戻  り   値： なし										    									*/
/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */ 
char shortest_path_search_check_full(){
    queue_reset();
    for(int i = 0; i < H;i++){
	for(int j = 0;j < W; j++){
	    for(int k = 0; k < 4; k++){
		maze_d[i][j][k] = maze_d_max;
	    }
	}
    }
    for(int k = 0; k < 4; k++){
	if(((maze_w[Get_Goal_y()][Get_Goal_x()] & (1<<k)) == 0 ) && ((maze_w[Get_Goal_y()][Get_Goal_x()] & (1<<(4+k))) != 0 )){
	    maze_d[Get_Goal_y()][Get_Goal_x()][k] = 0;
	}
    }
    enqueue(Get_Goal_x()*100 + Get_Goal_y());
  
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
		if(update_flag){
			enqueue(nx*100 + ny);
		
			if(Start_y == ny && Start_x == nx){
				for(int k = 0; k < 4; k++){
		    			if(maze_d[Start_y][Start_x][k] != maze_d_max ){//スタート位置の重みが更新されてなかったら＝最短経路が存在しない
		    				return 0;
		    			}
	    			}
			}
		}
	    }
	}
    }
    
    /*
    for(int i = 0; i < H;i++){
		for(int j = 0; j < W; j++)printf2("%d\t",maze_w[i][j]&0x0f);
		printf2("\n");
    }
    printf2("\n");

    for(int k = 0; k < 4; k++){
    	printf2("%d \n",maze_d[Get_Goal_y()][Get_Goal_x()][k]);
    	
    }
    
    for(int k = 0; k < 4; k++){
    	printf2("%d \n",maze_d[Start_y][Start_x][k]);
    	
    }
    
    */ 
    
    
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
	if(((maze_w[Get_Goal_y()][Get_Goal_x()] & (1<<k)) == 0 ) && ((maze_w[Get_Goal_y()][Get_Goal_x()] & (1<<(4+k))) != 0 )){
	    maze_d[Get_Goal_y()][Get_Goal_x()][k] = 0;
	}
    }
    enqueue(Get_Goal_x()*100 + Get_Goal_y());
  
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
	
    while(my_x != Get_Goal_x() || my_y != Get_Goal_y()){

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

    char my_x_tmp = my_x,my_y_tmp = my_y,my_angle_tmp = my_angle;//現在位置のバックアップ
    
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
	if(((maze_w[Get_Goal_y()][Get_Goal_x()] & (1<<k)) == 0 ) && ((maze_w[Get_Goal_y()][Get_Goal_x()] & (1<<(4+k))) != 0 )){
	    maze_d[Get_Goal_y()][Get_Goal_x()][k] = 0;
	}
    }
    enqueue(Get_Goal_x()*100 + Get_Goal_y());
  
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
	    
	   if(i == Get_Goal_y() && j == Get_Goal_x()){
	   	maze_d_perfect[Get_Goal_y()][Get_Goal_x()] = 0; 
		
	   }else if (maze_d[i][j][0] == maze_d_max){//到達不能マス
	   	maze_d_perfect[i][j] = maze_d_max;
		   
	   }else{//走行経路を算出する
	   	
	   	 //run_list
		
		maze_d_perfect[i][j] = maze_d_max;
			
		for(int k = 0;k < 4;k++){//スタート向きの設定
			queue_reset();
    			short h_path = 0;
    			my_x = j;my_y = i;my_angle = 0;//スタート位置の設定
		
			/*if(maze_d[i][j][my_angle] > maze_d[i][j][k]){
				my_angle = k;
			}*/
		
			if((maze_w[my_y][my_x] & (1 << ((k+2)%4))) != 0){//目の前に壁がある場合スタートの向きにはならない
				//printf2("NG \n");
				continue;
			}
			my_angle = k;
			
			my_angle = (my_angle+2)%4;
			//printf2("%d ",my_angle);
			int last = 0;
		
			    while(my_x != Get_Goal_x() || my_y != Get_Goal_y()){

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
				    for(int ii = 0;ii < 4;ii++){
					if(ii == (my_angle+2)%4){//逆走はありえない
					}else{// L or R
					    short next = maze_d[my_y][my_x][ii];
					    if(num > next){
						n_num = ii;
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
				    
				case 2://B
					my_angle = (4+my_angle+2)%4; //メモ 最短走行ではありえないが実装する
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
			 short maze_d_perfect_tmp = 0;
			 while(!queue_empty()){
				comand = dequeue();path_num = dequeue();
				//printf2("%d %d\n",comand,path_num);
				//delay(1)
				
				if(comand == 0){//直線
					maze_d_perfect_tmp += max(1,(path_num+1) / 2) * 2;//メモ　斜め１マス＝１としたときの直線の重み
					
				}else if(comand == 10){//斜め直線
					maze_d_perfect_tmp += max(1, path_num);  //斜め１マス
					
				}else if(comand == -11 || comand == -13 || comand == -14 || comand == 11 || comand == 13 || comand == 14){//斜め45
					maze_d_perfect_tmp += get_r45_cost();  //１マス
					
				}else{//カーブ
					maze_d_perfect_tmp += path_num * get_r_cost();
				}
				
			} 
			 
			maze_d_perfect[i][j] = min(maze_d_perfect_tmp,maze_d_perfect[i][j]); 
		}
		
	    }
	   
	   // printf2("OK \n");
	}
    }
    
    
    
    /*
    for(int i = 0; i < H;i++){
	for(int j = 0; j < W; j++){
		printf2("+");
		if(maze_w[i][j]&0x01)printf2("----");
		else if(maze_w[i][j]&0x10)printf2("    ");
		else printf2("....");
	}
	printf2("+\n");
		
	for(int j = 0; j < W; j++){
		if(maze_w[i][j]&0x08)printf2("|");
		else if(maze_w[i][j]&0x80)printf2(" ");
		else printf2(":");
			
		if(maze_d_perfect[i][j] == maze_d_max){
			printf2("    ");
		}else{
			printf2("%4d",maze_d_perfect[i][j] );	
		}
			
	}
	printf2("|\n");
    }
    printf2("+----+----+----+----+----+----+----+----+----+----+----+----+----+----+----+----+\n");
    */    
  
    ///////////////////////////////////////////////
    
    //run_list
    queue_reset();
    short h_path = 0;
    char maze_flag[H][W] = {0};
    
    my_x = Start_x;my_y = Start_y;my_angle = Start_angle;
 
    int last = 0;
	
    
    while(my_x != Get_Goal_x() || my_y != Get_Goal_y()){
	
	short num = maze_d_max -100; //maze_d_perfect[my_y][my_x];  周囲のマスが現在地より小さいとは言えないため最大値-100に変更
	short n_num = 0;
	char first_flag = 0;
	
	maze_flag[my_y][my_x] = 1;//一度到達したマスには戻らないようにする
 
	//printf2("%d  %d\n",my_x,my_y);
	 
	for(int i = 0;i < 4;i++){//ゴールに近いマスを探す
		int nx = my_x+dx[i],ny = my_y+dy[i];
		
		//迷路の範囲内　＆＆　壁が無いことが確定している
		if((0 <= nx && nx < W) && (0 <= ny && ny < H) && ((maze_w[my_y][my_x] & (1<<i)) == 0 )  && ((maze_w[my_y][my_x] & (1<<(4+i))) != 0 ) ){
			
			if(maze_flag[ny][nx] != 1){//まだ到達してなければ
				if(first_flag == 0 || num > maze_d_perfect[ny][nx]){//初めのマスは無条件で移動す候補にする || ゴールに近いマスを見つけた
					num = maze_d_perfect[ny][nx];
					n_num = i;
					
					first_flag = 1;
					
				 }else if(num == maze_d_perfect[ny][nx]){// LとRが同じ重み　斜めを優先したい
				 
				 	//printf2("hoge   %d ,  %d  , %d\n",my_angle , i, (i - my_angle + 4)%4);
					
					if( maze_d[ny][nx][i] < maze_d[my_y+dy[n_num]][my_x+dx[n_num]][n_num] ){//斜めを考慮しない重みの小さいほうを優先する
						n_num = i;
						
					}else if( maze_d[ny][nx][i] == maze_d[my_y+dy[n_num]][my_x+dx[n_num]][n_num]) {
						if(last == -1 && (i - my_angle + 4)%4 == 1   ){//前回がL かつ　今回はR  
							n_num = i;
								 
						}else if(last == 1 && (i - my_angle + 4)%4 == -1   ){//前回がR　かつ　今回はL
							n_num = i;
								 
						}else{//前回がSなら今回は?
							//わからんから先に見つかった方にする
						}
					}
					
					first_flag = 1;
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
  
    //現在位置をバックアップから復元
    my_x = my_x_tmp;
    my_y = my_y_tmp;
    my_angle = my_angle_tmp;
		
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
    char first_naname_z = 0;

    int comand_old = 0 ,path_num_old = 0;
    int comand_old2 = 0 ;
    //int path_num_old2 = 0;
    int BIG_NG_flag = 0;
    int path_add; //前回が大曲だった時の距離補正
    int v2_flag = 0;
    
 
    int path_hosei[31] = {0,
 			      0,0, 350,400, 550,550, 550,500, 550,550, 550,550, 400,400, 400,400, 
 			  400,400, 550,500, 500,500, 500,500, 500,500, 500,500, 500,500};//path_numごとに距離補正する

    int run_speed = 90;
    int run_speed_naname = 50;
    
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
 /*     while(!queue_empty()){//debug
	comand = dequeue();path_num = dequeue();
	printf2("%d %d\n",comand,path_num);
	delay(1);
	}
	while(1);*/
    //////////////////////////////////////////////////////////
 
     
    while(!queue_empty()){
	comand = dequeue();path_num = dequeue();
	
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
	    
	    if(comand_old == 0 && queue_next(1) == -1 && queue_next(3) == 0){//Uターン
		
	     	L_curveU(ul180,true);
	      	comand = dequeue();
	      	path_num = dequeue();
			
	    }else if(queue_next(1) == -1){//上のUターンが無効の時に発動する
		L_curve(sl90,true);
		ESmotor(180,25,true,true);//距離、スピード 170
		
	    }else if(queue_next(1) == 1){//Sターン
		L_curve(sl90,true);
		ESmotor(200,25,true,true);//距離、スピード 200,30
		
	    }else if(queue_next(1) == -11 || queue_next(1) == 11){
		L_curve(sl90,true);
		
		ESmotor(80,20,true,true);//距離、スピード
		
	    }else if(comand_old == -13){//斜め終わり直後のカーブ
		L_curve_afterNaname(sl90,true);
		
	    }else{
	
		L_curve(sl90,true);
			
		/*if(queue_next(1) == 0){
		    ESmotor(150,25,true,true);//距離、スピード
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
	    	L_rotate_naname(l45 * path_num  * 0.90,2);
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
		L_rotate_naname(l45 * path_num * 1.00,false);//0.95

	    }else{
		L_rotate_naname(l45 * path_num * 0.95,false);
	
	    }
	    break;
	
	    
	case 0://S
	    if(queue_empty()){
			
		//S_run(h1 * (long long)path_num ,run_speed + run_fin_speed_offset,4,true);//non_stop = 4
		
		if(path_num >= 2){
			S_run((h1 * (long long)path_num) + path_hosei[path_num] ,run_speed + run_fin_speed_offset,4,4);//non_stop = 4 // w_flag = 4 串の壁補正あり
		}
		
		motor(0,0);
		Set_motor_pid_mode(0);//低速 マイナスがあるので低速モードに戻す
		GyroSum_reset();
		
		mae_kabe();
		
		if(get_IR(IR_L) > get_IR(IR_R)){//ゴール後の横壁との距離補正　メモ：斜めにゴールしてしまう可能性があるので強制的に近い方の壁と距離補正する
			
			L_rotate(l90);
			mae_kabe();
			R_rotate(r90);
			
		}else{
			R_rotate(r90);
			mae_kabe();
			L_rotate(l90);
		}
	
		mae_kabe();
		motor(0,0);

	    }else {
		
                path_num--;//壁切れ分距離を半マス減らす
		
		if(path_num < 0){//マイナスにならないようにする
			path_num = 0;
		}
		
          	if(path_num > 0){
		
		    if(comand_old == 12 || comand_old == -12){//前回が大曲だったら
			  path_add = 250; 
			  
		    }else if(comand_old == 13 || comand_old == -13){//前回が斜め終わりだったら
			  path_add = 70;  
		    }
		    
		    //if(first_flag == 0)S_run((h1 *(long long) path_num) + path_hosei[path_num] ,run_speed + run_fin_speed_offset,3,true); // memo : non_stop = 3 加速はゆっくり　減速はすくなめ
		    //else S_run((h1 * (long long)path_num)  + path_hosei[path_num] ,run_speed + run_fin_speed_offset,true,true);
			 
		   
		    if(first_flag == 0)S_run((h1 *(long long) path_num)+path_add + path_hosei[path_num] ,run_speed + run_fin_speed_offset,3,4); // memo : non_stop = 3 加速はゆっくり　減速はすくなめ// w_flag = 4 串の壁補正あり
		    else  S_run((h1 *(long long) path_num)+path_add + path_hosei[path_num],run_speed + run_fin_speed_offset,true,4);// w_flag = 4 串の壁補正あり
		    
		    path_add = 0;
		    
		    first_flag = 1;
		}	  
		
		
		
		if((get_IR(IR_R) > 20 && get_IR(IR_R) < 70 ) || (get_IR(IR_L) > 20 && get_IR(IR_L) < 70 ) || get_IR(IR_L) > 230 || get_IR(IR_R) > 230){
    			
			if(queue_next(1) == -12 || queue_next(1) == 12){//次は大曲予定
    				BIG_NG_flag = 1;//ずれが大きいので大曲禁止
					
				ESmotor(h1,40,true,true);//距離、スピード
			}
    			
    		}
		
		status_log = 3;//ログに壁切れ開始を記録するため
		/*
		if(queue_next(3) == 0 && queue_next(4) <= 1 && queue_next(5) == -99){//ゴール最後で壁切れが発生しない時（2か所入口があるとき）メモ　大曲とかだと条件が変わるので使わないほうが良い
			ESmotor(h1,40,true,true);//距離、スピード　壁切れの代わりに半マス直線
			
		}else{*/
			if(queue_next(1) == -11 || queue_next(1) == 11){//直線後に45ターン
				if(queue_next(1) < 0){//次　左
				    if(path_num <= 1){
					 S_run_kabe2(20,true,1); 
				    }else{
					 S_run_kabe2(15,true,1);
				    }
				    
				    //S_run_kabe2(15,4,1);// w_flag = 4 串の壁補正あり
							
				}else if(queue_next(1) > 0){//次　右
				    if(path_num <= 1){
					 S_run_kabe2(20,true,2); 
				    }else{
					 S_run_kabe2(15,true,2);  
				    }
				    
				    //S_run_kabe2(15,4,2);// w_flag = 4 串の壁補正あり
				}
			}else if(queue_next(1) == -14 || queue_next(1) == 14){//直線後に45ターン 斜めセンサー版
				if(queue_next(1) < 0){//次　左
				    if(path_num <= 1){
					 //S_run_kabe_BIG(35,0,1,path_num); //壁補正無し
					 
					 S_run_kabe_BIG(45,4,1,path_num); //w_flag = 4 串の壁補正あり
					 
					  
					 if(first_flag == 0){
					 	first_naname_z = 1;
						ESmotor(150,45,true,true);//距離、スピード
						
					 }else{
						//ESmotor(50,35,true,true);//距離、スピード 
					 }
					 
				    }else{
					 S_run_kabe_BIG(25,4,1,path_num);  //w_flag = 4 串の壁補正あり 
					 
					 ESmotor(80,25,true,true);//距離、スピード
				    }
						
				}else if(queue_next(1) > 0){//次　右
				    if(path_num <= 1){
					    
					 //S_run_kabe_BIG(35,0,2,path_num);  //壁補正無し
					 
					 S_run_kabe_BIG(45,4,2,path_num);  //w_flag = 4 串の壁補正あり
					 
					 if(first_flag == 0){
					 	first_naname_z = 1;
						ESmotor(150,45,true,true);//距離、スピード 
					 }else{
						//ESmotor(50,35,true,true);//距離、スピード  
					 }
				    }else{
					 S_run_kabe_BIG(25,4,2,path_num);  //w_flag = 4 串の壁補正あり
					 
					 ESmotor(80,25,true,true);//距離、スピード
				    }

				}
			}else if(BIG_NG_flag == 0 && (queue_next(1) == 12 || queue_next(1) == -12)){//直線後に大曲
				if(queue_next(1) < 0){//次　左
				    if(path_num <= 1){
				        S_run_kabe_BIG(30,4,1,path_num); //w_flag = 4 串の壁補正あり
				    }else{
					S_run_kabe_BIG(25,4,1,path_num);  //w_flag = 4 串の壁補正あり
				    }
						
				}else if(queue_next(1) > 0){//次　右
				    if(path_num <= 1){
				        S_run_kabe_BIG(30,4,2,path_num);  //w_flag = 4 串の壁補正あり
				    }else{
					S_run_kabe_BIG(25,4,2,path_num);   //w_flag = 4 串の壁補正あり
				    }      
				}
					  
			}else{
				if(queue_next(1) < 0){//次　左
				
					if( queue_next(1) == -1 && queue_next(3) == -1 && queue_next(5) == 0){//Uターン
						if(path_num <= 1){
							S_run_kabe(35,true,1); 
					    	}else{
					    		S_run_kabe(25,true,1); 
					    	
					    	}
					}else{
					    	if(path_num <= 1){
							S_run_kabe(30,true,1); 
					    	}else{
					    		S_run_kabe(20,true,1); 
					    		//S_run_kabe(30,4,1);// w_flag = 4 串の壁補正あり
					    	}
					}
				    
				  
							
				}else if(queue_next(1) > 0){//次　右
					if( queue_next(1) == 1 && queue_next(3) == 1 && queue_next(5) == 0){//Uターン
						if(path_num <= 1){
							S_run_kabe(35,true,2);   
					    	}else{
							S_run_kabe(25,true,2); 
					    	}
					    
					}else{
					    	if(path_num <= 1){
							S_run_kabe(30,true,2);   
					    	}else{
							S_run_kabe(20,true,2); 
							//S_run_kabe(30,4,2);// w_flag = 4 串の壁補正あり
					    	}
					}
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
		    S_run_kabe_naname(55,3,1);
			
	        }else if(queue_next(1) == 11){//次　右
		    S_run_kabe_naname(55,3,2);
		
		}else if(queue_next(1) == -13){//次　左
		
		    if(v2_flag == 1 && queue_next(3) != -11){//2マスVターンの出るとき
			 v2_flag = 4; 
		
	            }else if(queue_next(3) == -1){//2マス斜め後にカーブ
		   
		    	v2_flag = 5;
		    }else if(queue_next(3) != -11){//2マスVターンではない Zパターン
		   
		    	v2_flag = 2;
			
			if(first_naname_z == 1)v2_flag = 20;
		    }else{
		    	v2_flag = 1; 	
		    }	
		    
		    S_run_kabe_naname2(55,3,1,v2_flag);
	        }else if(queue_next(1) == 13){//次　右
		
		    if(v2_flag == 1 && queue_next(3) != 11){//2マスVターンの出るとき
			 v2_flag = 4;  
			 
		    }else if(queue_next(3) == 1){//2マス斜め後にカーブ
		   
		    	v2_flag = 5;
		    }else  if(queue_next(3) != 11){//2マスVターンではない Zパターン
		    	v2_flag = 2;
			
			if(first_naname_z == 1)v2_flag = 20;
		    }else{
		    	v2_flag = 1;
		    }
		    
		    S_run_kabe_naname2(55,3,2,v2_flag);
	        }
		
		//ESmotor(100,35,true,false);//距離、スピード
		
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
		    S_run_kabe_naname(35,3,1);
			
	        }else if(queue_next(1) == 11){//次　右
		    S_run_kabe_naname(35,3,2);
		
		}else if(queue_next(1) == -13){//次　左
		    if(queue_next(3) == -11){//Vターン
			 v2_flag = 3;   
		   
		    }else if(queue_next(3) == -1){//斜め後にカーブ
		   
		    	v2_flag = 7;
			
		    }else if(queue_next(3) == 0){//斜め後に直線
		   
		    	v2_flag = 6;
		    }
		    S_run_kabe_naname2(35,3,1,v2_flag);
			
	        }else if(queue_next(1) == 13){//次　右
		    if(queue_next(3) == 11){//Vターン
			 v2_flag = 3;   
		    
		    }else if(queue_next(3) == -1){//斜め後にカーブ
		   
		    	v2_flag = 7;
		    }else if(queue_next(3) == 0){//斜め後に直線 
		   
		    	v2_flag = 6;
		    }
		    S_run_kabe_naname2(35,3,2,v2_flag);
		    
	        }
	    }
		
	    v2_flag = 0;
	    //my_x = nx;
	    //my_y = ny;
	    
	    path_num +=2;
	     
	    first_naname_z = 0;
	    
	    break;
	case 1://R
	    if(queue_next(1) == -1){//Sターン
		R_curve(sr90,true);
		ESmotor(200,25,true,true);//距離、スピード 200,30
			
	    }else if(comand_old == 0 && queue_next(1) == 1 && queue_next(3) == 0){//Uターン
		
		R_curveU(ur180,true);
		comand = dequeue();
		path_num = dequeue();
				
	    }else if(queue_next(1) == 1){//上のUターンが無効の時に発動する
		R_curve(sr90,true);
		ESmotor(180,25,true,true);//距離、スピード 170
		
	    }else if(queue_next(1) == -11 || queue_next(1) == 11){
		R_curve(sr90,true);
		
		ESmotor(80,20,true,true);//距離、スピード
		
	    }else if(comand_old == 13){//斜め終わり直後のカーブ
		R_curve_afterNaname(sr90,true);
		
	    }else{
		R_curve(sr90,true);
			
		/*if(queue_next(1) == 0){
		    ESmotor(150,25,true,true);//距離、スピード
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
	    	R_rotate_naname(r45 * path_num * 0.90,2);
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
		R_rotate_naname(r45 * path_num * 1.00,false); //0.95
	
	    }else{
		R_rotate_naname(r45 * path_num * 0.95,false);
			
	    }
	    break;
	   
	
	}
	
	first_flag = 1;
	//前前回の値として記憶
	comand_old2 = comand_old;
	//path_num_old2 = path_num_old;
	
	//前回の値として記憶
	comand_old = comand;
	path_num_old = path_num;
    }
    status_log = 99;
    motor(0,0);
    led_up();
    
   // while(1);////////////////////////////////////////////////////////////////////////////////////////
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
	
    //shortest_path_search(Get_Goal_x(),Get_Goal_y());
    shortest_path_search_fin();
	
    for(int i = 0; i < H;i++){
	for(int j = 0;j < W; j++){
				
	    if( !( (Not_Pickup_y_min <= i  && i <=  Not_Pickup_y_max) && (Not_Pickup_x_min <= j &&  j <= Not_Pickup_x_max) )){
				
		if(j != Get_Goal_x() && i != Get_Goal_y()){
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
/* 関 数 概 要：距離(0.11mm)をパルス数に変換する   		 			            */
/* 関 数 詳 細：										    */
/* 引       数： 距離（0.11mm)									    */
/* 戻  り   値： パルス数										    */
/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */ 
int d_to_p(int d){
	static double hoge = W_P / (W_D * 3.14);
	int p;
	
	p = (hoge * d) / 10;
	return p;
}

/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */
/* 関 数 概 要：ゴール座標を返却する		   		 			            */
/* 関 数 詳 細：										    */
/* 引       数： なし										    */
/* 戻  り   値： ゴール座標									    */
/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */ 
char Get_Goal_x(void){
	return Goal_x + Goal_x_offset;
}

/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */
/* 関 数 概 要：ゴール座標を返却する		   		 			            */
/* 関 数 詳 細：										    */
/* 引       数： なし										    */
/* 戻  り   値： ゴール座標									    */
/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */ 
char Get_Goal_y(void){
	return Goal_y + Goal_y_offset;
}
/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */
/* 関 数 概 要：ゴール座標を返却する		   		 			            */
/* 関 数 詳 細：										    */
/* 引       数： なし										    */
/* 戻  り   値： ゴール座標									    */
/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */ 
char Get_Goal_angle(void){
	return Goal_angle + Goal_angle_offset;
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
    static long long backup_irq_time = 0;
    static char backup_irq_cnt_now = 0;

    t_1ms++;
    if(t_1ms > 99999)t_1ms = 99999;
    
    task ++;                         // タスクの更新						
    if (log_start != 2 && task >= 40) task = 0;       
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
	
    
    if(ir_flag == 1 && mode < 3){//探索走行中
    	backup_irq_time++;
    	if(backup_irq_time >= backup_irq_time_ms){//hoge秒に１回迷路情報をバックアップする
		backup_irq_time = 0;
		for(int i = 0; i < H;i++){
			for(int j = 0; j < W; j++){
				maze_w_backup_irq[i][j][backup_irq_cnt_now] = maze_w[i][j];
			}
		
	    	}
		
		backup_irq_cnt_now++;
		if(backup_irq_cnt_now >= backup_irq_max)backup_irq_cnt_now = 0;
	}
    }
		
	
	
    
    if(motor_stop_get() == 1){//モータ緊急停止状態
	log_start = 0; //ログの記録も停止
	ir_flag = 0; //赤外線停止
	Gy_flag = 0; //ジャイロ停止
	
	if(mode < 3){//探索中の場合
		
		//現在地の周囲の迷路情報が信用できないのでバックアップから過去の情報に置き換える
		for(int i = 0; i < H;i++){
			for(int j = 0; j < W; j++){
				maze_w[i][j] = maze_w_backup_irq[i][j][(backup_irq_cnt_now+backup_irq_max-backup_irq_cnt )%backup_irq_max];
			}
		
	    	}
			
		motor(0,0);
		led(15);
			
		while(1){//動作を停止する　電源をオフする必要がある
			if(get_sw() == 1){
				
				while(get_sw() == 1)nop();
					
				
				if(shortest_path_search_check() == 1){//最短経路が見つからない時
					
					led(4);
					//迷路情報はあきらめるしかない
				 }else{ 
					maze_save();//バックアップで上書きする
						
					led(9);
				}
	    		}
		}	
	}
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
    case 40:
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




 



