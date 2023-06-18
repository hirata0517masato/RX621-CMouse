#include"Motor.h"
#include "Gyro.h"
#include "Encoder.h"
#include "IR.h"
#include "iodefine.h"
#include <machine.h>
#include <stdlib.h>

#include "Parameters.h"

#define true 1
#define false 0

int LE = 0,RE = 0;
char motor_stop_flag = 0;
int LM_prev = 0,RM_prev = 0;
int safe_cnt = 0;
char buff_pwm_L = 0,buff_pwm_R = 0;


void motor_stop(){
	motor_stop_flag = 1;
}

char motor_stop_get(){
	return motor_stop_flag;
}

void pwm_buff(char L,char R){
	buff_pwm_L = L; //ログ用に保存
	buff_pwm_R = R; //ログ用に保存
}

char get_pwm_buff_L(){
	return 	buff_pwm_L;
}
char get_pwm_buff_R(){
	return 	buff_pwm_R;
}

/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */
/* 関 数 概 要：モーターのPWM設定											  			            */
/* 関 数 詳 細：												                                   */
/* 引       数：左モーターPWM、右モーターPWM	（-100.0から+100.0） 							    */
/* 戻  り   値：なし										    									*/
/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */ 
void pwm(float duty_L, float duty_R){
	
    unsigned int dt_L, dt_R;
	int L_PM = 0,R_PM = 0; // 0:プラス 1:マイナス
	
    if(duty_L >  100.0) duty_L =  100.0; // duty_Lを100以上にしない
    if(duty_L < -100.0) duty_L = -100.0; // duty_Lを-100以下にしない
    if(duty_R >  100.0) duty_R =  100.0;
    if(duty_R < -100.0) duty_R = -100.0;
	
	pwm_buff((char)duty_L,(char)duty_R);//ログ用に保存
	
	if(duty_L < 0.0){
		L_PM = 1;
		duty_L = -duty_L;
	}
	
	/* デューティ比の算出 */
   	dt_L = MTU1.TGRA * (100.0 - duty_L) / 100.0;//  dt_L = 0.9445*50/100 = 0.5
		 
	/* デューティ比のオーバーフロー保護 */
    if(dt_L >= MTU1.TGRA)   dt_L = MTU1.TGRA - 1;  // 
		
	if(L_PM == 0){
		 /* デューティ比の設定 */
   		 MTU0.TGRB = MTU1.TGRA - 1;
   		 MTU1.TGRB = dt_L;
	}else{
		/* デューティ比の設定 */
   		 MTU0.TGRB = dt_L;
   		 MTU1.TGRB = MTU1.TGRA - 1;
	}
	
	if(duty_R < 0.0){
		R_PM = 1;
		duty_R = -duty_R;
	}
	
	/* デューティ比の算出 */
   	dt_R = MTU4.TGRA * (100.0 - duty_R) / 100.0;//  dt_R = 0.9445*50/100 = 0.5
		 
	/* デューティ比のオーバーフロー保護 */
    if(dt_R >= MTU4.TGRA)   dt_R = MTU3.TGRA - 1;  // 
		
	if(R_PM == 0){
		 /* デューティ比の設定 */
   		 MTU4.TGRB = MTU4.TGRA - 1;
   		 MTU3.TGRB = dt_R;
	}else{
		/* デューティ比の設定 */
   		 MTU4.TGRB = dt_R;
   		 MTU3.TGRB = MTU3.TGRA - 1;
	}
}

void motor(int LM,int RM){

  if(LM < -100)LM = -100;
  if(100 < LM)LM = 100;
  if(RM < -100)RM = -100;
  if(100 < RM)RM = 100;

  
  //if(((LM - RM) > 50) || ((RM - LM) > 50))motor_stop();

  if(abs(LM) > 5){
	if(abs(get_encoder_L()) < 5){
		safe_cnt ++;
		if(safe_cnt > 20000)motor_stop();
	}else safe_cnt = 0;
  }else safe_cnt = 0;

  if(abs(RM) > 5){
	if(abs(get_encoder_R()) < 5){
		safe_cnt ++;
		if(safe_cnt > 20000)motor_stop();
	}else safe_cnt = 0;
  }else safe_cnt = 0;
  
  if(motor_stop_flag == 1){
	LM = 0;
	RM = 0;
	
	//LED全灯
	PORTA.DR.BIT.B0 = 1;
	PORTA.DR.BIT.B1 = 1;
	PORTA.DR.BIT.B2 = 1;
	PORTA.DR.BIT.B3 = 1;
  }

  LM_prev = LM;
  RM_prev = RM;
  
  pwm(LM,RM);
}


void Smotor(int M,char w_flag){

	static int cnt1 = 0,cnt2 = 0;
	static int cnt3 = 0,cnt4 = 0;
	static int cnt5 = 0;
	
	if(w_flag > 0){
		if((get_encoder_L() > 15 || get_encoder_R() > 15) && (abs(get_encoder_L() - get_encoder_R()) < 30  )  && (w_flag != 3) ){

			if((get_IR(IR_F) < 280) && abs(GyroSum_get()) < 350){
				
				
				if(get_IR(IR_R) > 125 ){ //右壁近い
					cnt1++;
					if((cnt1 > 10 - min(10,(get_encoder_R()/15)) ) || (get_IR(IR_R) > 240 )){
						cnt1 = 0;
						GyroSum_add(-1);
					}
				}else if((get_IR(IR_R) < 50 && get_IR(IR_L) > 50 && get_IR(IR_L) < 90)){ // 右壁なし　左壁あり　左壁遠い
					cnt1++;
					if((cnt1 > 15 - min(20,(get_encoder_R()/15)) ) || (get_IR(IR_L) < 70 )){
						cnt1 = 0;
						GyroSum_add(-1);
					}
				}else cnt1 = 0;
			
				if(get_IR(IR_L) > 125 ){ //左壁近い
					cnt2++;
					if((cnt2 > 10 - min(10,(get_encoder_L()/15))) || (get_IR(IR_L) > 240)){
						cnt2 = 0;
						GyroSum_add(1);
					}
				}else if((get_IR(IR_L) < 50 && get_IR(IR_R) > 50 && get_IR(IR_R) < 90)){ //左壁なし　右壁あり　右壁遠い
				 	cnt2++;
					if((cnt2 > 15 - min(20,(get_encoder_L()/15))) || (get_IR(IR_R) < 70)){
						cnt2 = 0;
						GyroSum_add(1);
					}
				}else cnt2 = 0;
			}else{
				cnt1 = 0;
				cnt2 = 0;
			}
		}
	
				
		//斜め対策
		if(w_flag == 3){
			if((get_encoder_L() > 5 || get_encoder_R() > 5) && abs(GyroSum_get()) < 450){
				if(get_IR(IR_FL) > 40  && get_IR(IR_F) < 40 && get_IR(IR_FR) < 40 &&  get_IR(IR_R) < 40 ){//左前のみ
					cnt3++;
					if(cnt3 > 0){
						cnt3 = 0;
						if(get_IR(IR_FL) > 20){
							GyroSum_add(2);
						}else{
							GyroSum_add(1);
						}
						//PORTA.DR.BIT.B3 = 1;
					}	
				}else cnt3 = 0;
				
				if(get_IR(IR_L) < 40 && get_IR(IR_FL) < 40 &&  get_IR(IR_F) < 40 && get_IR(IR_FR) > 40){//右前のみ
					cnt4++;
					if(cnt4 > 0){
						cnt4 = 0;
						if(get_IR(IR_FR) > 20){
							GyroSum_add(-2);
						}else{
							GyroSum_add(-1);
						}
						//PORTA.DR.BIT.B0 = 1;
					}	
				}else cnt4 = 0;
			}
		}else if(w_flag == 4){//串対策 壁あり、斜め以外 メモ：探索では使用しない方が良い
			if((get_encoder_L() > 15 || get_encoder_R() > 15) && abs(GyroSum_get()) < 350){
				if( get_IR(IR_L) < 40 &&  get_IR(IR_FL) > 40  &&  get_IR(IR_F) < 40  && get_IR(IR_FR) < 40 &&  get_IR(IR_R) < 40){//左前のみ
					cnt3++;
					if(cnt3 > 0){
						cnt3 = 0;
						GyroSum_add(2);
						//PORTA.DR.BIT.B3 = 1;
					}	
				}else cnt3 = 0;
				
				if( get_IR(IR_L) < 40 &&  get_IR(IR_FL) < 40  &&  get_IR(IR_F) < 40  && get_IR(IR_FR) > 40 &&  get_IR(IR_R) < 40){//右前のみ
					cnt4++;
					if(cnt4 > 0){
						cnt4 = 0;
						GyroSum_add(-2);
						//PORTA.DR.BIT.B0 = 1;
					}	
				}else cnt4 = 0;
			}
		}
		
		
		 //前壁補正　
		if((get_encoder_L() > 5 && get_encoder_R() > 5) && abs(GyroSum_get()) < 350){
			if(get_IR(IR_FL) > 40 && (get_IR(IR_F) > 20) &&  get_IR(IR_FR) > 40){//前壁あり
				long long diff = (long long)((get_IR(IR_FR)) - get_IR(IR_FL));
				if(abs(diff) < 15){
					cnt5++;
					if(cnt5 > 5){
						cnt5 = 0;
						
						if(diff > 1)diff = 1;
						if(diff < -1)diff = -1;
						GyroSum_add(diff);
					}
				}else cnt5 = 0;	
			}else cnt5 = 0;
		}
	
	}
	int powor_max = 80;
	int powor = gyro_powor_L() + (get_encoder_L() + get_encoder_R())/20;
	if(powor > powor_max)powor = powor_max;
	else if(-powor_max > powor)powor = -powor_max; 
	
/*	if( get_encoder_L() < 10){//速度が遅い時はジャイロ弱める	
		powor /= 2;
	}*/
	motor(M + powor ,M - powor);
}



void ESmotor(long long A, int max_M,char non_stop,char w_flag){
	Encoder_reset();
	
	long long enc_base = (get_encoder_total_L() + get_encoder_total_R())/2;
	long long enc_now = 0;
	

	int cnt = 0;
	
	int path_cnt = 0;
	//int ir_L_old = 0,ir_R_old = 0;
	int ir_L_now = 0,ir_R_now = 0;
	int ir_L_flag = 0,ir_R_flag = 0;
	int path_cnt_save_L = -1;//同じマスで壁切れ処理を２回以上しないように覚えておく変数
	int path_cnt_save_R = -1;//同じマスで壁切れ処理を２回以上しないように覚えておく変数
	int hosei_kyori_L = -1,hosei_kyori_R = -1;//壁切れ時の補正距離　左異なるタイミングで壁切れした際に利用する
	int kame_hosei = 0;
	long long enc_kabe_L,enc_kabe_R;
	
	int p = 5,min_M = 5,M = 5;
	
	int non_stop_min_M = 25;
	int min_M_use = 0;
	
	int M_max_safe = 60;//横壁が近すぎる場合は減速
	
	kame_hosei = 200;//170
	
	
	GyroSum_reset();
	
	while(1){
	
		
		if(enc_now < A){//目標地点まで移動中	
			
			if(non_stop == 1){
				min_M_use = non_stop_min_M;
				
			}else if(non_stop == 3){//加速ゆっくり　減速すくなめ
				if(enc_now < A/2){// 進んだ距離 < A/2
					min_M_use = min_M;
				}else{
					min_M_use = non_stop_min_M;
				}
			}else if(non_stop == 4){//加速はやめ　減速しっかり
				if(enc_now < A/2){// 進んだ距離 < A/2
					min_M_use = non_stop_min_M;
				}else{
					min_M_use = min_M;
				}
			}else{
				min_M_use = min_M;
			}
			
			
			if(enc_now > A * 7/8){// 進んだ距離 < 目標距離 * 3/4 = //減速区間
				
				M = min_M_use + ( (A - enc_now) / 6);
			
			}else if(enc_now < 50){//出だしは加速しすぎないように
				M = min_M_use + (enc_now / 8);
				
			}else{
				M = min_M_use + (enc_now / 6);	
			}
			
			
			if(max_M < M)M = max_M;
			
			if(ir_L_now > 250 || ir_R_now > 250){//横壁が近すぎる場合は減速
				 if(M_max_safe < M)M = M_max_safe;
			}

			if(non_stop == 1){
				if(M < non_stop_min_M)M = non_stop_min_M;
				
			}else if(non_stop == 3){//加速ゆっくり　減速すくなめ
				if(enc_now < A/2){// 進んだ距離 < A/2
					if(M < min_M)M = min_M;
				}else{
					if(M < non_stop_min_M)M = non_stop_min_M;
				}
			}else{
				if(M < min_M){
					M = min_M;
				}
			}
			
		}else{//行き過ぎた
			M = (A - enc_now) * p ;	
		}
		 
		Smotor(M,w_flag);
		
		
		if(enc_now - ((long long)s1 * path_cnt ) > s1){//１マス進んだ
			path_cnt++;
			ir_L_flag = 0;
			ir_R_flag = 0;
		}
		
		
		if(w_flag == 1 && A > s1+100){//壁補正あり　ななめでもない && １マス以上進む時
			//壁切れの距離補正
			ir_L_now = get_IR(IR_L);
			ir_R_now = get_IR(IR_R);
			if(path_cnt_save_L !=  path_cnt){//現在のマスで壁切れ処理を実行していなければ
			
				if(ir_L_flag == 0 && ir_L_now > 35 && ir_R_now < 160){//左壁がある　&& 右壁に近すぎない
					ir_L_flag = 1;
				
				}else if(ir_L_flag == 1 && ir_L_now < 20 && ir_R_now < 160){//左壁がない　&& 右壁に近すぎない
				
					if( (non_stop == 0 && (enc_now % s1) < s1 * 2 / 3) || 
						(non_stop == 1 && ((enc_now - h1) % s1) < s1 * 2 / 3)	){
					
						if(path_cnt == path_cnt_save_R){//左より先に右が壁切れ補正していた場合
						
/*
							enc_base -= hosei_kyori_R;//右での補正を無かったことにする
							enc_now = (get_encoder_total_L() + get_encoder_total_R())/2 - enc_base;
							
							if( non_stop == 0){//迷路探索時
								hosei_kyori_L = (enc_now % s1) - kame_hosei;
							}else if(non_stop == 1 && enc_now > h1){
								hosei_kyori_L = ((enc_now - h1) % s1) - kame_hosei;
							}
							hosei_kyori_L = (hosei_kyori_L + hosei_kyori_R) / 2;//左右の平均値を使用する
*/							
							//壁切れタイミングの違いで角度補正
							enc_kabe_L = (get_encoder_total_L() + get_encoder_total_R())/2;
							if(abs( (enc_kabe_L - enc_kabe_R) ) < 150){
								GyroSum_add( (enc_kabe_L - enc_kabe_R) * 10);
							}
						}else{
/*							if( non_stop == 0){//迷路探索時
								hosei_kyori_L = (enc_now % s1) - kame_hosei;
							}else if(non_stop == 1 && enc_now > h1){
								hosei_kyori_L = ((enc_now - h1) % s1) - kame_hosei;
							}
*/							enc_kabe_L = (get_encoder_total_L() + get_encoder_total_R())/2;
						}
						
//						enc_base += hosei_kyori_L;
						
					}
					ir_L_flag = 0;
					path_cnt_save_L = path_cnt;
				}
			}
			
			if(path_cnt_save_R !=  path_cnt){//現在のマスで壁切れ処理を実行していなければ
			
				if(ir_R_flag == 0 && ir_R_now > 35 && ir_L_now < 160){//右壁がある　&& 左壁に近すぎない
					ir_R_flag = 1;
				
				}else if(ir_R_flag == 1 && ir_R_now < 20 && ir_L_now < 160){//右壁がない　&& 左壁に近すぎない
					if( (non_stop == 0 && (enc_now % s1) < s1 * 2 / 3) || 
						(non_stop == 1 && ((enc_now - h1) % s1) < s1 * 2 / 3)	){
					
						if(path_cnt == path_cnt_save_L){//右より先に左が壁切れ補正していた場合
/*							
							enc_base -= hosei_kyori_L;//左での補正を無かったことにする
							enc_now = (get_encoder_total_L() + get_encoder_total_R())/2 - enc_base;
							
							if( non_stop == 0){//迷路探索時
								hosei_kyori_R = (enc_now % s1) - kame_hosei;
							}else if(non_stop == 1 && enc_now > h1){
								hosei_kyori_R = ((enc_now - h1) % s1) - kame_hosei;
							}
							
							hosei_kyori_R = (hosei_kyori_L + hosei_kyori_R) / 2;//左右の平均値を使用する
*/							
							
							//壁切れタイミングの違いで角度補正
							enc_kabe_R = (get_encoder_total_L() + get_encoder_total_R())/2;
							if(abs( (enc_kabe_L - enc_kabe_R) ) < 150){
								GyroSum_add( (enc_kabe_L - enc_kabe_R) * 10);
							}
						}else{
/*							if( non_stop == 0){//迷路探索時
								hosei_kyori_R = (enc_now % s1) - kame_hosei;
							}else if(non_stop == 1 && enc_now > h1){
								hosei_kyori_R = ((enc_now - h1) % s1) - kame_hosei;
							}
*/							enc_kabe_R = (get_encoder_total_L() + get_encoder_total_R())/2;
						}
						
//						enc_base += hosei_kyori_R;
						
					}
					ir_R_flag = 0;
					path_cnt_save_R = path_cnt;
				}
			}
		}
			
		
		
		if(non_stop != 0){
			if(A - enc_now  < 30)break; 
		}else{
			if(abs(enc_now - A) < 30){
				cnt++;	
			}else{
				cnt = 0;
			}
			if(cnt > 2000)break;
		}
		
		enc_now = (get_encoder_total_L() + get_encoder_total_R())/2 - enc_base;
	}

	motor(0,0);
//	GyroSum_reset();
	Encoder_reset();
}


void Tmotor(long long A){
	GyroSum_reset();
	GyroSum_add(A);
	Encoder_reset();
	
	long long L = get_encoder_total_L();
	long long R = get_encoder_total_R();

	int sa = 0;
	int LM = 0, RM = 0,LM_prev = 0, RM_prev = 0;
	int MA = 1,min_M = 4;
	
	int cnt = 0;
	int powor_max = 10;
	int powor;

	while(1){
		if((A > 0 && GyroSum_get() < 0) || (A < 0 && GyroSum_get() > 0)){
			powor = gyro_powor_L() * 1.5; //行き過ぎた
			
		}else{
			powor = gyro_powor_L();
		}
		
		if(powor > powor_max)powor = powor_max;
		else if(-powor_max > powor)powor = -powor_max; 

		sa = abs(L) - abs(R);
		
		if(powor > 0){
			LM = powor - sa;
			RM = -powor - sa;
		}else{
			LM = powor + sa;
			RM = -powor + sa;
		}

		if(LM_prev + MA < LM)LM = LM_prev + MA;
		if(LM_prev - MA > LM)LM = LM_prev - MA;
		
		if(RM_prev + MA < RM)RM = RM_prev + MA;
		if(RM_prev - MA > RM)RM = RM_prev - MA;
		
		if(0 < LM && LM < min_M)LM = min_M;
		if(0 > LM && LM > -min_M)LM = -min_M;

		if(0 < RM && RM < min_M)RM = min_M;
		if(0 > RM && RM > -min_M)RM = -min_M;
		 
		motor(LM ,RM);

		LM_prev = LM;
		RM_prev = RM;
		L = get_encoder_total_L();
		R = get_encoder_total_R();

		if(abs(GyroSum_get()) < 70)cnt += 10;
		else if(abs(GyroSum_get()) < 150)cnt++;
		else cnt = 0;

		if(cnt > 500)break;
	}


	motor(0,0);
	GyroSum_reset();
	Encoder_reset();
}


void ETmotorBIG(long long A, long long E, char non_stop){
	//GyroSum_reset();
	//Encoder_reset();

	int M = 20;//25 20 20
	int M_kabe = 30;//40 35 40
	
	char flag = 0;
	
	//壁切れ
	if(A > 0){//R
		while(get_IR(IR_R) > 40){
			Smotor(M_kabe,true);
			flag = 1;
		}
		if(flag)ESmotor(115,M_kabe,true,false);
	}else{//L
		while(get_IR(IR_L) > 40){
			Smotor(M_kabe,true);
			flag = 1;
		}
		if(flag)ESmotor(115,M_kabe,true,false);
	}

//	GyroSum_reset();
	//Encoder_reset();
	

//	ESmotor(55,M_kabe,true,true);//60

	
/*	if(get_IR(IR_FL) > 13 || get_IR(IR_FR) > 13){//前に壁がある
		PORTA.DR.BIT.B1 = 1;
		while(get_IR(IR_FR) < 20){//理想より前壁が遠い
			Smotor(M_kabe,true);
		}
		PORTA.DR.BIT.B1 = 0;
	}
*/	
	long long L = get_encoder_total_L();
	long long R = get_encoder_total_R();
	long long L_prev = L, R_prev = R;
	long long E_sum = 0;
	
	int powor_max = 25;
	int powor;
	
	int LM = 0, RM = 0;
	if(non_stop){
		LM = M;
		RM = M;
	}
	int LM_prev = LM, RM_prev = RM;
	int MA = 5,min_M = 5;
	

	static int cnt1 = 0;
	
	if(A > 0){//R
		PORTA.DR.BIT.B3 = 1;
	}else{//L
		PORTA.DR.BIT.B0 = 1;
	}
	while(1){
		
		if(A > 0){//R
			if(get_IR(IR_L) > 150){ //左壁近い
				cnt1++;
				if(cnt1 > 5){
					cnt1 = 0;
					GyroSum_add(1);
				}
			}else cnt1 = 0;
		
			GyroSum_add( (A * (((L - L_prev)*100000) / E)) / 100000);
			E_sum += (L - L_prev);
			
		}else{//L
			if(get_IR(IR_R) > 150 ){ //右壁近い
				cnt1++;
				if(cnt1 > 5){
					cnt1 = 0;
					GyroSum_add(-1);
				}
			}else cnt1 = 0;
		
		   	GyroSum_add( (A * (((R - R_prev)*100000) / E)) / 100000);
			E_sum += (R - R_prev);
		}
		
		
		powor = gyro_powor_L();
		
		if(powor > powor_max)powor = powor_max;
		else if(-powor_max > powor)powor = -powor_max; 
		
		
		LM = M + powor;
		RM = M + -powor;
	

		if(LM_prev + MA < LM)LM = LM_prev + MA;
		if(LM_prev - MA > LM)LM = LM_prev - MA;
		
		if(RM_prev + MA < RM)RM = RM_prev + MA;
		if(RM_prev - MA > RM)RM = RM_prev - MA;
		
		if(0 < LM && LM < min_M)LM = min_M;
		if(0 > LM && LM > -min_M)LM = -min_M;

		if(0 < RM && RM < min_M)RM = min_M;
		if(0 > RM && RM > -min_M)RM = -min_M;
		 
		motor(LM ,RM);
		//delay(1);
		
		LM_prev = LM;
		RM_prev = RM;
		L_prev = L;
		R_prev = R;
		L = get_encoder_total_L();
		R = get_encoder_total_R();

		if(E - E_sum < 0)break;
	}

	PORTA.DR.BIT.B0 = 0;
	PORTA.DR.BIT.B3 = 0;
	
//	ESmotor(60,M_kabe,true,true);//60
	
	//motor(0,0);
	//GyroSum_reset();
	//Encoder_reset();
}


void ETmotor(long long A, long long E, char non_stop){
	//GyroSum_reset();
	//Encoder_reset();

	int M = 30;//25 20 20
	int M_kabe = 35;//40 35 40
	
	char flag = 0;
	
	//壁切れ
	if(A > 0){//R
		while(get_IR(IR_R) > 40){
			Smotor(M_kabe,true);
			flag = 1;
		}
		if(flag)ESmotor(115,M_kabe,true,false);
	}else{//L
		while(get_IR(IR_L) > 40){
			Smotor(M_kabe,true);
			flag = 1;
		}
		if(flag)ESmotor(115,M_kabe,true,false);
	}

//	GyroSum_reset();
	//Encoder_reset();
	

	ESmotor(45,M_kabe,true,true);//60

	
/*	if(get_IR(IR_FL) > 13 || get_IR(IR_FR) > 13){//前に壁がある
		PORTA.DR.BIT.B1 = 1;
		while(get_IR(IR_FR) < 20){//理想より前壁が遠い
			Smotor(M_kabe,true);
		}
		PORTA.DR.BIT.B1 = 0;
	}
*/	
	long long L = get_encoder_total_L();
	long long R = get_encoder_total_R();
	long long L_prev = L, R_prev = R;
	long long E_sum = 0;
	
	int powor_max = 40;
	int powor;
	
	int LM = 0, RM = 0;
	if(non_stop){
		LM = M;
		RM = M;
	}
	int LM_prev = LM, RM_prev = RM;
	int MA = 5,min_M = 5;
	

	static int cnt1 = 0;
	
	if(A > 0){//R
		PORTA.DR.BIT.B3 = 1;
	}else{//L
		PORTA.DR.BIT.B0 = 1;
	}
	while(1){
		
		if(A > 0){//R
			if(get_IR(IR_L) > 150){ //左壁近い
				cnt1++;
				if(cnt1 > 5){
					cnt1 = 0;
					GyroSum_add(1);
				}
			}else cnt1 = 0;
		
			GyroSum_add( (A * (((L - L_prev)*100000) / E)) / 100000);
			E_sum += (L - L_prev);
			
		}else{//L
			if(get_IR(IR_R) > 150 ){ //右壁近い
				cnt1++;
				if(cnt1 > 5){
					cnt1 = 0;
					GyroSum_add(-1);
				}
			}else cnt1 = 0;
		
		   	GyroSum_add( (A * (((R - R_prev)*100000) / E)) / 100000);
			E_sum += (R - R_prev);
		}
		
		
		powor = gyro_powor_L();
		
		if(powor > powor_max)powor = powor_max;
		else if(-powor_max > powor)powor = -powor_max; 
		
		
		LM = M + powor;
		RM = M + -powor;
	

		if(LM_prev + MA < LM)LM = LM_prev + MA;
		if(LM_prev - MA > LM)LM = LM_prev - MA;
		
		if(RM_prev + MA < RM)RM = RM_prev + MA;
		if(RM_prev - MA > RM)RM = RM_prev - MA;
		
		if(0 < LM && LM < min_M)LM = min_M;
		if(0 > LM && LM > -min_M)LM = -min_M;

		if(0 < RM && RM < min_M)RM = min_M;
		if(0 > RM && RM > -min_M)RM = -min_M;
		 
		motor(LM ,RM);
		//delay(1);
		
		LM_prev = LM;
		RM_prev = RM;
		L_prev = L;
		R_prev = R;
		L = get_encoder_total_L();
		R = get_encoder_total_R();

		if(E - E_sum < 0)break;
	}

	PORTA.DR.BIT.B0 = 0;
	PORTA.DR.BIT.B3 = 0;
	
	ESmotor(45,M_kabe,true,true);//60
	
	//motor(0,0);
	GyroSum_reset();
	//Encoder_reset();
}

void Tmotor_naname(long long A){
	//GyroSum_reset();
	GyroSum_add(A);
	Encoder_reset();

	static int cnt1 = 0;
		
	int LM = 0, RM = 0,LM_prev = 0, RM_prev = 0;
	int MA = 5,min_M = 15;
	
	int powor_max = 30;
	int powor;

	int M_kabe = 30;
	
	char flag = 0;

	
	//壁切れ
	if(A > 0){//R
		while(get_IR(IR_R) > 40){
			Smotor(M_kabe,true);
			flag = 1;
		}
		if(flag)ESmotor(115,M_kabe,true,false);
	}else{//L
		while(get_IR(IR_L) > 40){
			Smotor(M_kabe,true);
			flag = 1;
		}
		if(flag)ESmotor(115,M_kabe,true,false);
	}
	
	if(A > 0){//R
		PORTA.DR.BIT.B3 = 1;
	}else{//L
		PORTA.DR.BIT.B0 = 1;
	}
	
	Encoder_reset();
	while(1){
		if(A > 0){//R
			if(get_IR(IR_L) > 150 || get_IR(IR_R) > 150 ){ //左壁近い || 右壁が近い
				cnt1++;
				if(cnt1 > 5){
					cnt1 = 0;
					GyroSum_add(1);
				}
			}else cnt1 = 0;
		
		}else{//L
			if(get_IR(IR_L) > 150 || get_IR(IR_R) > 150 ){ //右壁近い
				cnt1++;
				if(cnt1 > 5){
					cnt1 = 0;
					GyroSum_add(-1);
				}
			}else cnt1 = 0;
		}
		
		powor = gyro_powor_L();
		
		if(powor > powor_max)powor = powor_max;
		else if(-powor_max > powor)powor = -powor_max;
		
		
		LM = powor;
		RM = -powor;
		

		if(LM_prev + MA < LM)LM = LM_prev + MA;
		if(LM_prev - MA > LM)LM = LM_prev - MA;
		
		if(RM_prev + MA < RM)RM = RM_prev + MA;
		if(RM_prev - MA > RM)RM = RM_prev - MA;
		
		if(0 < LM && LM < min_M)LM = min_M;
		if(0 > LM && LM > -min_M)LM = -min_M;

		if(0 < RM && RM < min_M)RM = min_M;
		if(0 > RM && RM > -min_M)RM = -min_M;
		 
		if(A > 0){//R
			//motor(LM ,-get_encoder_total_R() * 2);
			motor(LM ,-1);
		}else{//L
			//motor(-get_encoder_total_L() * 2 ,RM);
			motor(-1 ,RM);
		}

		LM_prev = LM;
		RM_prev = RM;
		
		if(A > 0){//R
			if(GyroSum_get() < 0)break;
		}else{//L
			if(GyroSum_get() > 0)break;
		}
	}
	
	PORTA.DR.BIT.B0 = 0;
	PORTA.DR.BIT.B3 = 0;

	motor(0,0);
	//GyroSum_reset();
	Encoder_reset();
}
