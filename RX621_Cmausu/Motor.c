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
char motor_pid_flag = 0;
char motor_pid_mode = 0; //0:低速 1:高速

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

void motor_pid_flag_reset(){
    motor_pid_flag = 0;	
}

void Set_motor_pid_mode(char n){
    motor_pid_mode = n;
}

int Get_motor_pid_mode(){
    return motor_pid_mode;
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
	
    //static float duty_L_old = 0.0, duty_R_old = 0.0;
    //const float duty_a = 0.1;
	
	
    if(duty_L >  100.0) duty_L =  100.0; // duty_Lを100以上にしない
    if(duty_L < -100.0) duty_L = -100.0; // duty_Lを-100以下にしない
    if(duty_R >  100.0) duty_R =  100.0;
    if(duty_R < -100.0) duty_R = -100.0;

    /*
      if(motor_pid_mode == 1){//高速
      //急激な変化を抑制する
		
      //if(duty_L > 0){
      if(duty_L - duty_L_old > duty_a){
      duty_L = duty_L_old + duty_a;
      }else if(duty_L_old - duty_L < -duty_a){
      duty_L = duty_L_old - duty_a;
      }
      //}
      duty_L_old = duty_L;
		
      //if(duty_R > 0){
      if(duty_R - duty_R_old > duty_a){
      duty_R = duty_R_old + duty_a;
      }else if(duty_R_old - duty_R < -duty_a){
      duty_R = duty_R_old - duty_a;
      }
      //}
      duty_R_old = duty_R;
      }
    */
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

    if(abs(LM) > 8){
	if(abs(get_encoder_L()) < 3){
	    safe_cnt ++;
	    if(safe_cnt > 20000)motor_stop();
	}else safe_cnt = 0;
    }else safe_cnt = 0;

    if(abs(RM) > 8){
	if(abs(get_encoder_R()) < 3){
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

    //	static int cnt1 = 0,cnt2 = 0;
    static int cnt3 = 0,cnt4 = 0;
    static int cnt5 = 0;
	
    static int ir_sa = 0,ir_sa_buf = 0;
    static int ir_wall = 130,ir_core = 0,ir_wall2 = 50;
    static float kp = 0,kd = 0;
    
    static int naname_flag_old = 0;
    
    int kusi_flag = 0;
    int naname_flag = 0;
    int mae_flag = 0;
	
    int LM,RM;
	
    if(w_flag > 0){
			
	//斜め対策
	if(w_flag != 3){
		naname_flag_old = 0;
	}
	
	if(w_flag == 3){
	    if((get_encoder_L() > 30 && get_encoder_R() > 30) ){// && abs(GyroSum_get()) < 2000){
		//if(   get_IR(IR_FL) > 35 && get_IR(IR_FR) < 30  ){//左前のみ
		if(   get_IR(IR_FL) > 25 && abs(GyroSum_get()) < 2000){//左前のみ
		    cnt3++;
		    if(cnt3 > 0){
			cnt3 = 0;
			if(get_encoder_L() > 50 && get_encoder_R() > 50){
				if( GyroSum_get() < 0){
					GyroSum_reset();
				}
				
				if(get_IR(IR_FL) > 100){// 左前がかなり近い
				    GyroSum_add(200);
				    naname_flag = 1;
				     
				}else if(get_IR(IR_FL) > 70){// || get_IR(IR_F) > 20){ // 左前がかなり近い　|| 正面にも壁があるとき
				    GyroSum_add(100);
				    naname_flag = 1;
				     
				}else if(get_IR(IR_FL) > 55  ){
				    GyroSum_add(30);
				    naname_flag = 1;
				    
				}else if(get_IR(IR_FL) > 45 ){
				    GyroSum_add(20);
				   naname_flag = 1;
				
				}else if(get_IR(IR_FL) > 30 ){
				    GyroSum_add(10);
				   naname_flag = 1;
				  
				}else{
			            GyroSum_add(5);
				    naname_flag = 1;
				}
			}else{
				if(get_IR(IR_FL) > 100){// 左前がかなり近い
				    GyroSum_add(30);
				    naname_flag = 1;
				    
				}else if(get_IR(IR_FL) > 45  ){
					GyroSum_add(1);
					naname_flag = 1;
				}
			}
			//PORTA.DR.BIT.B3 = 1;
		    }	
		}else{
			cnt3 = 0;
			/*if( GyroSum_get() > 0){
				GyroSum_add(-1);
			}*/
		}
				
		//if(get_IR(IR_FL) < 30 &&  get_IR(IR_FR) > 35  ){//右前のみ
		if(get_IR(IR_FR) > 25 && abs(GyroSum_get()) < 2000 ){//右前のみ
		    cnt4++;
		    if(cnt4 > 0){
			cnt4 = 0;
			if(get_encoder_L() > 50 && get_encoder_R() > 50){
				
				if( GyroSum_get() > 0){
					GyroSum_reset();
				}
				
				if(get_IR(IR_FR) > 100){// 右前がかなり近い
				    GyroSum_add(-200);
				    naname_flag = 1;
				    
				}else if(get_IR(IR_FR) > 70){// || get_IR(IR_F) > 20){ // 右前がかなり近い　|| 正面にも壁があるとき
				    GyroSum_add(-100);
				    naname_flag = 1;
				     
				}else if(get_IR(IR_FR) > 55){
				    GyroSum_add(-30);
				    naname_flag = 1;
				     
				}else if(get_IR(IR_FR) > 45){
				    GyroSum_add(-20);
				    naname_flag = 1;
				
				}else if(get_IR(IR_FR) > 30){
				    GyroSum_add(-10);
				    naname_flag = 1;
				  
				}else{
				    GyroSum_add(-5);
				    naname_flag = 1;
				}
			}else{
				if(get_IR(IR_FR) > 100){// 右前がかなり近い
				    GyroSum_add(-30);
				    naname_flag = 1;
				    
				}else if(get_IR(IR_FR) > 45){
					GyroSum_add(-1);
					naname_flag = 1;
				}
			}
			//PORTA.DR.BIT.B0 = 1;
		    }	
		}else{
			cnt4 = 0;
			/*if( GyroSum_get() < 0){
				GyroSum_add(1);
			}*/
		}
	    }
	
	    if(naname_flag_old == 1 && naname_flag == 0){
		    GyroSum_reset();//壁をよけきれたので補正をリセットする
	    }
	    
	}else if(w_flag == 4){//串対策 壁あり、斜め以外 メモ：探索では使用しない方が良い
			
	    if(motor_pid_mode == 0){//低速
		if((get_encoder_L() > 10 || get_encoder_R() > 10) && abs(GyroSum_get()) < 400){
		    if( get_IR(IR_L) < 110 &&  get_IR(IR_FL) > 25  &&  get_IR(IR_F) < 15  && get_IR(IR_FR) < 10 /*&&  get_IR(IR_R) < 40*/){//左前のみ
			cnt3++;
			if(cnt3 > 0){
			    cnt3 = 0;
				
			    if( GyroSum_get() < 0){
				GyroSum_reset();
			    }
			    
			    if(get_IR(IR_FL) > 30){
				GyroSum_add(20);
				kusi_flag = 1;
			    }else{
				GyroSum_add(5);
				kusi_flag = 1;
			    }
			    //PORTA.DR.BIT.B3 = 1;
			}	
		    }else cnt3 = 0;
					
		    if(/* get_IR(IR_L) < 40 &&*/  get_IR(IR_FL) < 10  &&  get_IR(IR_F) < 15  && get_IR(IR_FR) > 25 &&  get_IR(IR_R) < 110){//右前のみ
			cnt4++;
			if(cnt4 > 0){
			    cnt4 = 0;
			    
			    if( GyroSum_get() > 0){
				GyroSum_reset();
			    }
			    
			    if(get_IR(IR_FR) > 30){
				GyroSum_add(-20);
				kusi_flag = 1;
			    }else{
				GyroSum_add(-5);
				kusi_flag = 1;
			    }
			    //PORTA.DR.BIT.B0 = 1;
			}	
		    }else cnt4 = 0;
		}
	    }else{//高速
		if((get_encoder_L() > 10 || get_encoder_R() > 10) && abs(GyroSum_get()) < 1000){
		    if( get_IR(IR_L) < 110 &&  get_IR(IR_FL) > 15  /*&&  get_IR(IR_F) < 15*/  && get_IR(IR_FR) < 5 /*&&  get_IR(IR_R) < 40*/){//左前のみ
			cnt3++;
			if(cnt3 > 0){
			    cnt3 = 0;
				
			    if( GyroSum_get() < 0){
				GyroSum_reset();
			    }
			    
			    if(get_IR(IR_FL) > 25){
				GyroSum_add(10);
				kusi_flag = 1;
			    }else{
				GyroSum_add(5);
				kusi_flag = 1;
			    }
			    //PORTA.DR.BIT.B3 = 1;
			}	
		    }else cnt3 = 0;
					
		    if(/* get_IR(IR_L) < 40 &&*/  get_IR(IR_FL) < 5 /* &&  get_IR(IR_F) < 15 */ && get_IR(IR_FR) > 15 &&  get_IR(IR_R) < 110){//右前のみ
			cnt4++;
			if(cnt4 > 0){
			    cnt4 = 0;
			    
			    if( GyroSum_get() > 0){
				GyroSum_reset();
			    }
			    
			    if(get_IR(IR_FR) > 25){
				GyroSum_add(-10);
				kusi_flag = 1;
			    }else{
				GyroSum_add(-5);
				kusi_flag = 1;
			    }
			    //PORTA.DR.BIT.B0 = 1;
			}	
		    }else cnt4 = 0;
		}
	    }
	}
		
		
	//前壁補正　
	if(motor_pid_mode == 0){//低速
		if(w_flag != 3 && kusi_flag == 0){ //斜め中ではない かつ　串対策が反応してない
		
		    if((get_encoder_L() >= 0 || get_encoder_R() >= 0) && abs(GyroSum_get()) < 1000){
	            
			if(get_IR(IR_L) < 180 && get_IR(IR_FL) > 40 && (get_IR(IR_F) > 40) &&  get_IR(IR_FR) > 40 && get_IR(IR_R) < 180 ){//前壁あり 横壁が近くない
					
				
			    long long diff = (long long)((get_IR(IR_FR)) - get_IR(IR_FL));
			    if(abs(diff) > 5 && abs(diff) < 50){
				//if(abs(diff) > 0 && abs(diff) < 80){
				cnt5++;
				if(cnt5 > 0){
				    cnt5 = 0;
								
				    if(get_IR(IR_F) > 250){
					if(diff > 5)diff = 5;
					if(diff < -5)diff = -5;
									
				    }else if(get_IR(IR_F) > 200){
					if(diff > 10)diff = 10;
					if(diff < -10)diff = -10;
				    }else{
					if(diff > 20)diff = 20;
					if(diff < -20)diff = -20;
				    }
				    
				    mae_flag = 1;
				    GyroSum_add(diff);
				}
			    }else cnt5 = 0;	
			}else cnt5 = 0;
		    }
		}
	}
	    
	if(motor_pid_flag == 0 && w_flag != 3 && kusi_flag == 0){//1msの割り込み内でフラグはリセットされる && 斜めではない && 串が反応していない
			
	    //ir_wall = 115;//マス中央でのセンサー値
	    if(get_IR(IR_L) > 100 && get_IR(IR_R) > 100 &&  get_encoder_L() > 5 &&  get_encoder_R() > 5){//左右に壁がある &&  動いている
				
		if(abs(get_IR(IR_L) - get_IR(IR_R)) < 50 && abs(GyroSum_get()) < 500 ){//左右の差が少ない && まっすぐ
		    ir_wall = ir_wall*1/10 + ((get_IR(IR_L) + get_IR(IR_R))/2)*9/10 ; //マス中央でのセンサー値を更新
		}
	    }
	    
	    if(get_IR(IR_LT) > 40 && get_IR(IR_RT) > 40 &&  get_encoder_L() > 5 &&  get_encoder_R() > 5){//左右に壁がある &&  動いている
				
		if(abs(get_IR(IR_LT) - get_IR(IR_RT)) < 30 && abs(GyroSum_get()) < 550 ){//左右の差が少ない && まっすぐ
		    ir_wall2 = ir_wall2*1/10 + ((get_IR(IR_LT) + get_IR(IR_RT))/2)*9/10 ; //マス中央でのセンサー値を更新
		}
	    }
			
	    if(motor_pid_mode == 0){//低速
	    	if(get_encoder_L() > 10 && get_encoder_R() > 10){
			ir_core = 2;//1;//左右の差の許容範囲
			
			kp = 0.3;//0.7;
			kd = 0.0;//3.5;
		}else{
			ir_core = 15;//1;//左右の差の許容範囲
			
			kp = 0.4;//0.2;
			kd = 0.0;
		}
				
		
	    }else{//高速
	    	
	    	if(get_encoder_L() > 5 && get_encoder_R() > 5){
			ir_core = 5; // 25  //左右の差の許容範囲
					
			kp = 0.3; //0.3 0.5
			kd = 25.0; //1.5 15.0
		}else{
			ir_core = 15; // 25  //左右の差の許容範囲
					
			kp = 0.4;//0.4
			kd = 20.0;//15.0
		}
	    }
			
			
	    //if((get_encoder_L() > 10 && get_encoder_R() > 10)  && (w_flag != 3) ){
	    if((get_encoder_L() > 2 && get_encoder_R() > 2)  && (w_flag != 3) ){
				
		//左右に壁がある 
		if(get_IR(IR_L) > 40 && get_IR(IR_R) > 40 ){
		    if(abs(get_IR(IR_L) - get_IR(IR_R)) > ir_core){//  左右の差が小さきすぎない
		    
		    	if(get_IR(IR_L) > 200 || get_IR(IR_R) > 200 ){//どちらかの壁が近い
				ir_sa =  (get_IR(IR_L) - get_IR(IR_R)) * 1;
			}else{
				ir_sa =  get_IR(IR_L) - get_IR(IR_R);
			}
						
			motor_pid_flag = 1;
		    }
		
		}else if(motor_pid_mode == 0 && get_IR(IR_LT) > 30 && get_IR(IR_RT) > 30 && get_IR(IR_F) < 50){//斜め45度センサー 低速モードのみ
		    if(abs(get_IR(IR_LT) - get_IR(IR_RT))*2 > ir_core){//  左右の差が小さきすぎない
					
			ir_sa =  (get_IR(IR_LT) - get_IR(IR_RT));// *2;
						
			motor_pid_flag = 1;
		    }
	
		}else if(get_IR(IR_L) > 40 && get_IR(IR_R) < 40){// && abs(get_IR(IR_L) - ir_wall) > ir_core/2){//左だけ壁がある
		    if(motor_pid_mode == 0){//低速
			if(abs(get_IR(IR_L) - ir_wall) > ir_core /2 ) {// 左右の差が小さきすぎない
					
			    ir_sa =  (get_IR(IR_L) - ir_wall);    
			    
			   // if(mae_flag == 1)ir_sa /= 4;//前壁補正が反応していたら
			    
			    motor_pid_flag = 1;
			}
		    }else{//高速
			if(abs(get_IR(IR_L) - ir_wall) > ir_core/2) {// 左右の差が小さきすぎない
					
				if(get_IR(IR_L) > 200 || get_IR(IR_R) > 200 ){//どちらかの壁が近い
					ir_sa =  (get_IR(IR_L) - ir_wall) * 1 ;
					
				}else{
			    		ir_sa =  (get_IR(IR_L) - ir_wall) * 2 / 4 ;//* 2;// * 20 /10;
				}
			    	motor_pid_flag = 1;
			}
		    }
					
		}else if(get_IR(IR_L) < 40 && get_IR(IR_R) > 40){// && abs(ir_wall - get_IR(IR_R)) > ir_core/2 ){//右だけ壁がある
		    if(motor_pid_mode == 0){//低速
			if(abs(ir_wall - get_IR(IR_R)) > ir_core /2 ){//左右の差が小さきすぎない
				
			    ir_sa =  (ir_wall - get_IR(IR_R));
			
			    
			    
			   // if(mae_flag == 1)ir_sa /= 4;//前壁補正が反応していたら
			    motor_pid_flag = 1;
			}
		    }else{//高速
			if(abs(ir_wall - get_IR(IR_R)) > ir_core/2 ){//左右の差が小さきすぎない
				
				if(get_IR(IR_L) > 200 || get_IR(IR_R) > 200 ){//どちらかの壁が近い
					ir_sa =  (ir_wall - get_IR(IR_R)) * 1;
					
				}else{
			    		ir_sa =  (ir_wall - get_IR(IR_R)) * 2 / 4;// * 2;//  * 20 /10;
				}
			    
			    	motor_pid_flag = 1;
			}
		    }
					
		}else if(get_IR(IR_LT) > 40 && get_IR(IR_RT) < 25){//斜め左だけ壁がある
		    if(motor_pid_mode == 0){//低速
			if(abs(get_IR(IR_LT) - ir_wall2) > ir_core/2) {// 左右の差が小さきすぎない
						
			    ir_sa =  (get_IR(IR_LT) - ir_wall2);// / 8;// / 6;
			    
			    if(mae_flag == 1)ir_sa /= 4;//前壁補正が反応していたら
			    motor_pid_flag = 1;
			}
		    }else{//高速
			if(abs(get_IR(IR_LT) - ir_wall2) > ir_core/2) {// 左右の差が小さきすぎない
						
			    ir_sa =  (get_IR(IR_LT) - ir_wall2) / 8;
			    motor_pid_flag = 1;
			}
		    }	
		}else if(get_IR(IR_LT) < 25 && get_IR(IR_RT) > 40){//斜め右だけ壁がある
		    if(motor_pid_mode == 0){//低速
			if(abs(ir_wall2 - get_IR(IR_RT)) > ir_core/2 ){//左右の差が小さきすぎない
					
			    ir_sa =  (ir_wall2 - get_IR(IR_RT)) ;/// 8;// / 6;
			    
			    if(mae_flag == 1)ir_sa /= 4;//前壁補正が反応していたら
			    motor_pid_flag = 1;
			}
		    }else{//高速
			if(abs(ir_wall2 - get_IR(IR_RT)) > ir_core/2 ){//左右の差が小さきすぎない
					
			    ir_sa =  (ir_wall2 - get_IR(IR_RT)) / 8;
			    motor_pid_flag = 1;
			}
		    }	
		}
				
		if(motor_pid_flag == 1){
			
		    ir_sa += (ir_sa > 0)? -ir_core : ir_core;
		    
		    ir_sa = max(min(ir_sa,1000),-1000);
					
		    GyroSum_add(ir_sa * kp - ((ir_sa_buf - ir_sa) * kd) );
					
		    ir_sa_buf = ir_sa;
		}
				
	    }
	}
    }
    int powor_max;//ジャイロのパワー
    int powor = gyro_powor_L();
	
    if(motor_pid_mode == 0){//低速
	powor_max = 30;	
    }else{//高速
	if( w_flag == 3){ //斜め中は壁をよけるために数値を大きくする
	    if(  get_encoder_L() < 30 || get_encoder_R() < 30){//速度が遅い時はジャイロ弱める

	       powor = powor * 1 / 4;
	       powor_max = 20;
	    }else{
	    
	       powor_max = 80;
	    }
	}else{
	    if(kusi_flag == 1){//串対策が反応しているとき
		powor_max = 100;
				
	    }else{
		if( get_encoder_L() < 30 || get_encoder_R() < 30){//速度が遅い時はジャイロ弱める
		
		    if( get_encoder_L() < 5 || get_encoder_R() < 5){
			powor = powor * 1 / 4;
			powor_max = 5;
			
		    }else if( get_encoder_L() < 10 || get_encoder_R() < 10){
			powor = powor * 2 / 4;
			powor_max = 10;
			
		    }else{
			powor = powor * 3 / 4;
			powor_max = 40;
		    }

		}else{
				
			powor_max = 60;
		}
	    }
	}
    }
	
    if(powor > powor_max)powor = powor_max;
    else if(-powor_max > powor)powor = -powor_max; 
	
    /*	if( get_encoder_L() < 10){//速度が遅い時はジャイロ弱める	
	powor /= 2;
	}*/
	
    //if((0 < M &&  F_max < get_IR(IR_F))  || (kusi_flag == 1) || ( naname_flag == 1)){ //(前進　かつ　前壁が近すぎる場合は) || 串対策が反応しているとき || ななめが強めに反応しているとき
    if((0 < M &&  F_max < get_IR(IR_F))  || (kusi_flag == 1) ){ //(前進　かつ　前壁が近すぎる場合は) || 串対策が反応しているとき 
	M /= 2; //速度を下げる
    }
	
    LM = M + powor;
    RM = M - powor;

    if(motor_pid_mode == 1){//高速モード
	if(LM != 0 || RM != 0){// 0,0でなければ //斜め時のマイナスは有効　
	    if(0 < LM && LM < 10){
		RM += 10 - LM;
		LM = 10;
				
	    }
	    if(0 < RM && RM < 10){
		LM += 10 - RM;
		RM = 10;
	    }
			
	    //if(LM < 10)LM = 10;	
	    //if(RM < 10)RM = 10;
	}
    }
	
    motor(LM ,RM);
    
    naname_flag_old = naname_flag;
}
		




void ESmotor(long long A, int max_M,char non_stop,char w_flag){
     //   Encoder_reset();
	
    //long long enc_base = (get_encoder_total_L() + get_encoder_total_R())/2;
    long long enc_base_L = get_encoder_total_L();
    long long enc_base_R = get_encoder_total_R();
    long long enc_now = 0;
	

    int cnt = 0;
	
    int path_cnt = 0;
    //int ir_L_old = 0,ir_R_old = 0;
    int ir_L_now = 0,ir_R_now = 0;
    int ir_L_flag = 0,ir_R_flag = 0;
    int ir_L_flag_1masu = 0,ir_R_flag_1masu = 0;
    int path_cnt_save_L = -1;//同じマスで壁切れ処理を２回以上しないように覚えておく変数
    int path_cnt_save_R = -1;//同じマスで壁切れ処理を２回以上しないように覚えておく変数
//    int hosei_kyori_L = -1,hosei_kyori_R = -1;//壁切れ時の補正距離　左異なるタイミングで壁切れした際に利用する
//    int kame_hosei;
    long long enc_kabe_L,enc_kabe_R;
	
    int p = 5,min_M = 5,M = 5;
    int min_M_use = 0;
	
    int non_stop_min_M = 20;//探索の既知区間用
	
    if(motor_pid_mode == 1){//高速モード
	if(h1 < A){//距離が半マス以上
	    non_stop_min_M = 50;
	}else{
	    non_stop_min_M = 25;
	}
    }
	
	
    int M_max_safe = 60;//横壁が近すぎる場合は減速
	
 //   kame_hosei = 530;
	
	
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
			
			
	    if(enc_now > A * 5/8){// 進んだ距離 < 目標距離 * 3/4 = //減速区間
		
	    	if(motor_pid_mode == 0 || non_stop == 3){//低速 || 加速ゆっくり　減速すくなめ
			if((A - enc_now) < 200){
				M = min_M_use + ( (A - enc_now) / 20);
			
			}else{
				M = min_M_use + ( (A - enc_now -200) / 8);
			}
			
		}else{
			if(A <= s1){//距離が１マス以下の場合、あまり加速してないので減速も少なめ
				M = min_M_use + ( (A - enc_now) / 4);
				
			}else{
				if((A - enc_now) < 200){
					M = min_M_use;
				
				}else{
					M = min_M_use + ( (A - enc_now -200) / 20);
				}
			}
		}
			
	    }else{//加速区間
		if(motor_pid_mode == 0 || non_stop == 3){//低速 || 加速ゆっくり　減速すくなめ
		    if(enc_now < 100){//出だしは加速しすぎないように
			M = min_M_use + (enc_now / 10);
				
		    }else{
			M = min_M_use + (enc_now / 7);
		    }
		    
		    if(motor_pid_mode == 1 && enc_now > s45){//半マス進んだらモード変更
			  non_stop = 1;  
		    }
		}else{//高速
		    if(A <= s1){//距離が１マス以下の場合
		    	 M = min_M_use + ((enc_now) / 2);
		    }else{
			if(enc_now < 200){//出だしは加速しすぎないように
			    M = min_M_use ;
					
			}else{
		            M = min_M_use + ((enc_now-200) / 4);
			}
		    }
		}
	    }
			
			
	    if(max_M < M)M = max_M;
			
	    if(ir_L_now > 220 || ir_R_now > 220){//横壁が近すぎる場合は減速
		//if(ir_L_now > 999 || ir_R_now > 999){//横壁が近すぎる場合は減速
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
			
		if(ir_L_flag == 0 && ir_L_now > 30 && ir_R_now < 160){//左壁がある　&& 右壁に近すぎない
		    ir_L_flag = 1;
				
		}else if(ir_L_flag == 1 && ir_L_now < 10 && ir_R_now < 160){//左壁がない　&& 右壁に近すぎない
				
		    if( (non_stop == 0 && (enc_now % s1) < s1 * 2 / 3) || 
			(non_stop == 1 && ((enc_now - h1) % s1) < s1 * 2 / 3)	){
					
			if(path_cnt == path_cnt_save_R){//左より先に右が壁切れ補正していた場合
/*			    if(Get_motor_pid_mode() == 0){//探索モード

				enc_base_L -= hosei_kyori_R;//右での補正を無かったことにする
				enc_base_R -= hosei_kyori_R;//右での補正を無かったことにする
				enc_now = min(get_encoder_total_L()  - enc_base_L , get_encoder_total_R() - enc_base_R);
							
				if( non_stop == 0){//迷路探索時
				    hosei_kyori_L = (enc_now % s1) - kame_hosei;
				}else if(non_stop == 1 && enc_now > h1){
				    hosei_kyori_L = ((enc_now - h1) % s1) - kame_hosei;
				}
				hosei_kyori_L = (hosei_kyori_L + hosei_kyori_R) / 2;//左右の平均値を使用する
			    }
*/
			    //壁切れタイミングの違いで角度補正
			    enc_kabe_L = min(get_encoder_total_L()  - enc_base_L , get_encoder_total_R() - enc_base_R);
			    if(abs( (enc_kabe_L - enc_kabe_R) ) < 500){
				GyroSum_add( (enc_kabe_L - enc_kabe_R) * 5);
			    }
			}else{
/*			    if(Get_motor_pid_mode() == 0){//探索モード
			    	if( non_stop == 0){//迷路探索時
				    hosei_kyori_L = (enc_now % s1) - kame_hosei;
				}else if(non_stop == 1 && enc_now > h1){
				    hosei_kyori_L = ((enc_now - h1) % s1) - kame_hosei;
				}
			    }
*/
			    enc_kabe_L = min(get_encoder_total_L()  - enc_base_L , get_encoder_total_R() - enc_base_R);
			}
			
/*			if(Get_motor_pid_mode() == 0){//探索モード
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
				
		}else if(ir_R_flag == 1 && ir_R_now < 10 && ir_L_now < 160){//右壁がない　&& 左壁に近すぎない
		    if( (non_stop == 0 && (enc_now % s1) < s1 * 2 / 3) || 
			(non_stop == 1 && ((enc_now - h1) % s1) < s1 * 2 / 3)	){
					
			if(path_cnt == path_cnt_save_L){//右より先に左が壁切れ補正していた場合
							
/*			    if(Get_motor_pid_mode() == 0){//探索モード
			    	enc_base_L -= hosei_kyori_L;//左での補正を無かったことにする
				enc_base_R -= hosei_kyori_L;//左での補正を無かったことにする
				enc_now = min(get_encoder_total_L()  - enc_base_L , get_encoder_total_R() - enc_base_R);
							
				if( non_stop == 0){//迷路探索時
				    hosei_kyori_R = (enc_now % s1) - kame_hosei;
				}else if(non_stop == 1 && enc_now > h1){
				    hosei_kyori_R = ((enc_now - h1) % s1) - kame_hosei;
				}
							
				hosei_kyori_R = min(get_encoder_total_L()  - enc_base_L , get_encoder_total_R() - enc_base_R);
							
			    }
*/
			    //壁切れタイミングの違いで角度補正
			    enc_kabe_R = min(get_encoder_total_L()  - enc_base_L , get_encoder_total_R() - enc_base_R);
			    if(abs( (enc_kabe_L - enc_kabe_R) ) < 500){
				GyroSum_add( (enc_kabe_L - enc_kabe_R) * 5);
			    }
			}else{
/*			    if(Get_motor_pid_mode() == 0){//探索モード
			    	if( non_stop == 0){//迷路探索時
				    hosei_kyori_R = (enc_now % s1) - kame_hosei;
				}else if(non_stop == 1 && enc_now > h1){
				    hosei_kyori_R = ((enc_now - h1) % s1) - kame_hosei;
				}
			    }
*/
			    enc_kabe_R = min(get_encoder_total_L()  - enc_base_L , get_encoder_total_R() - enc_base_R);
			}
						
/*			if(Get_motor_pid_mode() == 0){//探索モード
			    enc_base_L += hosei_kyori_R;
			    enc_base_R += hosei_kyori_R;
			}
*/
		    }
		    
		    ir_R_flag = 0;
		    path_cnt_save_R = path_cnt;
		}
	    }
	}
	
	
	if(A >= s1 && (A - enc_now) < s1 && non_stop == false){//１マス以上進 && 残り１マス && 探索中　で壁切れした場合は半マス進んで終了
	
		if(ir_L_flag_1masu == 0){
			if(get_IR(IR_L) > 20){
				ir_L_flag_1masu = 1;
			}
		}else if(ir_L_flag_1masu == 1){
			if(get_IR(IR_L) < 10){
				ESmotor(h1_2,  25 ,non_stop,w_flag);  
				//while(1){
					//led()
				//	PORTA.DR.BIT.B0 = 0;
				//	PORTA.DR.BIT.B1 = 0;
				//	PORTA.DR.BIT.B2 = 1;
				//	PORTA.DR.BIT.B3 = 1;
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
				ESmotor(h1_2, 25 ,non_stop,w_flag); 
				//while(1){
					//led()
				//	PORTA.DR.BIT.B0 = 1;
				//	PORTA.DR.BIT.B1 = 1;
				//	PORTA.DR.BIT.B2 = 0;
				//	PORTA.DR.BIT.B3 = 0;
				motor(0,0);
				//}
				return;
			}
		}
	}
		
		
	if(motor_pid_mode == 0){//探索中
		if((A - enc_now) < s1 && 180 < get_IR(IR_F)){//残り1マス　＆＆　前壁が近い場合はストップ
		    break; //激突防止
				
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
		
		
	//enc_now = (get_encoder_total_L() + get_encoder_total_R())/2 - enc_base;
	enc_now = min(get_encoder_total_L()  - enc_base_L , get_encoder_total_R() - enc_base_R);
    }

    motor(0,0);
    //	GyroSum_reset();
    //    Encoder_reset();
}


void Tmotor(long long A){
    GyroSum_reset();
    GyroSum_add(A);
    //	Encoder_reset(); 有効化しないこと　壁読み間違えの再チェックが動作しなくなる
	
    long long L = get_encoder_total_L();
    long long R = get_encoder_total_R();
	
    long long L_base = L,R_base = R;

    int sa = 0;
    int LM = 0, RM = 0,LM_prev = 0, RM_prev = 0;
    int MA = 1,min_M = 4;
	
    //	int cnt = 0;
    int powor_max = 20;
    int powor;

    while(1){
	if((A > 0 && GyroSum_get() < 0) || (A < 0 && GyroSum_get() > 0)){
	    powor = gyro_powor_L() * 1.5; //行き過ぎた
			
	}else{
	    powor = gyro_powor_L();
	}
		
	if(powor > powor_max)powor = powor_max;
	else if(-powor_max > powor)powor = -powor_max; 

	sa = abs(L-L_base) - abs(R-R_base);//なくてもいいかも？　滑りが影響するので
		
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

	if(A > 0){//R
	    if(GyroSum_get() < 0)break;
	}else{//L
	    if(GyroSum_get() > 0)break;
	}
	/*
	  if(abs(GyroSum_get()) < 60)cnt += 10;
	  else if(abs(GyroSum_get()) < 160)cnt++;
	  else cnt = 0;

	  if(cnt > 500)break;*/
    }


    motor(0,0);
    GyroSum_reset();
    //	Encoder_reset(); 有効化しないこと　壁読み間違えの再チェックが動作しなくなる
}


void ETmotorU(long long A, long long E, char non_stop){
    GyroSum_reset();
    //Encoder_reset();

    int M_kabe = 30;
    int M 		= 30;
	
    //壁切れ
    if(A > 0){//R
	while(get_IR(IR_R) > 15){
	    Smotor(M_kabe,true);
	}

    }else{//L
	while(get_IR(IR_L) > 15){
	    Smotor(M_kabe,true);
	}
    }

    //	GyroSum_reset();
    //Encoder_reset();
		
	
    long long L = get_encoder_total_L();
    long long R = get_encoder_total_R();
    long long L_prev = L, R_prev = R;
    long long E_sum = 0;
    long long E_add = 0;
    long long E_add_U = 0;
	
    int powor_max = 20;
    int powor;
	
    int LM = 0, RM = 0;
    if(non_stop){
	LM = M;
	RM = M;
    }
    int LM_prev = LM, RM_prev = RM;
    int MA = 10,min_M = 5;
	

    static int cnt1 = 0;
	
    if(A > 0){//R
	GyroSum_add(5);
		
	PORTA.DR.BIT.B3 = 1;
    }else{//L
	GyroSum_add(-5);
		
	PORTA.DR.BIT.B0 = 1;
    }
    while(1){
		
	if(A > 0){//R
	    if(get_IR(IR_L) > 2000){ //左壁近い //どうしても以外は使わない方がよい
		cnt1++;
		if(cnt1 > 5){
		    cnt1 = 0;
		    GyroSum_add(1);
		    E_add += 1;
		}
	    }else cnt1 = 0;
		
	    GyroSum_add( (A * (((L - L_prev)*100000) / E)) / 100000);
	    E_sum += (L - L_prev);
			
	}else{//L
	    if(get_IR(IR_R) > 2000 ){ //右壁近い //どうしても以外は使わない方がよい
		cnt1++;
		if(cnt1 > 5){
		    cnt1 = 0;
		    GyroSum_add(-1);
		    E_add += 1;//マイナスでもプラスを設定
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

	if(A > 0){//R
	    E_add_U = uslr180_fin;
	}else{//L
	    E_add_U = usll180_fin;
	}
		
	if(E + E_add_U - E_add - E_sum < 0)break;
		
    }

    PORTA.DR.BIT.B0 = 0;
    PORTA.DR.BIT.B3 = 0;
	

    //motor(0,0);
    GyroSum_reset();
    //Encoder_reset();

}


void ETmotorBIG(long long A, long long E, char non_stop){
    GyroSum_reset();
    //Encoder_reset();
	
    int M_kabe = 40;
    int M 		= 40;

    
    ESmotor(80,M_kabe,true,true);
    GyroSum_reset();
    
    long long L = get_encoder_total_L();
    long long R = get_encoder_total_R();
    long long L_prev = L, R_prev = R;
    long long E_sum = 0;
    long long E_add = 0;
    long long E_offset = 0;	
	
    int powor_max = 20;
    int powor;
	
    int LM = 0, RM = 0;
    if(non_stop){
	LM = M;
	RM = M;
    }
    int LM_prev = LM, RM_prev = RM;
    int MA = 10,min_M = 5;
	

    static int cnt1 = 0;
	
    if(A > 0){//R
	
	PORTA.DR.BIT.B3 = 1;
    }else{//L
	
	PORTA.DR.BIT.B0 = 1;
    }
    while(1){
		
	if(A > 0){//R
	    if(get_IR(IR_FR) > 5000){ //左壁近い //どうしても以外は使わない方がよい
		cnt1++;
		if(cnt1 > 5){
		    cnt1 = 0;
		    GyroSum_add(-1);
		   // E_add += 1;
		}
	    }else cnt1 = 0;
		
	    GyroSum_add((A * (((L - L_prev)*100000) / E)) / 100000);
	    E_sum += (L - L_prev);
			
	}else{//L
	    if(get_IR(IR_FL) > 5000 ){ //右壁近い //どうしても以外は使わない方がよい
		cnt1++;
		if(cnt1 > 5){
		    cnt1 = 0;
		    GyroSum_add(1);
		   // E_add += 1;//マイナスでもプラスを設定
		}
	    }else cnt1 = 0;
		
	    GyroSum_add((A * (((R - R_prev)*100000) / E)) / 100000);
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

	if(A > 0){//R
	    E_offset = rslsr90_BIG_offset;
	}else{//L
	    E_offset = rslsl90_BIG_offset;
	}
		
	if(E + E_offset - E_add - E_sum < 0)break;
		
    }

    //motor(0,0);
     
    GyroSum_reset();
	
   // ESmotor(140,M,true,true);//距離、速度
    
    PORTA.DR.BIT.B0 = 0;
    PORTA.DR.BIT.B3 = 0;
}




void ETmotor(long long A, long long E, char non_stop){
    GyroSum_reset();
    //Encoder_reset();
	
    int M_kabe = 25;
    int M 		= 25;
    int M_kabe2 = 20;
	
    //char flag = 0;
	
    //壁切れ
    if(A > 0){//R 
	//while(get_IR(IR_R) > 10){
	while((get_IR(IR_R) > 10) || ( get_IR(IR_F) > 10 && get_IR(IR_F) < 20) ){ //前壁補正は斜めになると悪影響がある
	    Smotor(M_kabe,true);
	   // flag = 1;
	}
	//if(flag)ESmotor(45,M_kabe,true,false);
    }else{//L
	//while(get_IR(IR_L) > 10){
	while((get_IR(IR_L) > 10) || ( get_IR(IR_F) > 10 && get_IR(IR_F) < 20) ){
	    Smotor(M_kabe,true);
	    //flag = 1;
	}
	//if(flag)ESmotor(45,M_kabe,true,false);
    }

    GyroSum_reset();
    //Encoder_reset();
	
    if(25 < get_IR(IR_F)){//前壁が近すぎ  //////上の数値と合わせる
    	//ESmotor(45,M_kabe,true,true);//60
    }else{
	ESmotor(35,M_kabe,true,true);//60 45
    }
    
    GyroSum_reset();
	
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
    long long E_add = 0;
    long long E_offset = 0;	
	
    int powor_max = 20;
    int powor;
	
    int LM = 0, RM = 0;
    if(non_stop){
	LM = M;
	RM = M;
    }
    int LM_prev = LM, RM_prev = RM;
    int MA = 10,min_M = 5;
	

    static int cnt1 = 0;
	
    if(A > 0){//R
	GyroSum_add(5);
		
	PORTA.DR.BIT.B3 = 1;
    }else{//L
	GyroSum_add(-5);
		
	PORTA.DR.BIT.B0 = 1;
    }
    while(1){
		
	if(A > 0){//R
	    if(get_IR(IR_L) > 2000){ //左壁近い //どうしても以外は使わない方がよい
		cnt1++;
		if(cnt1 > 5){
		    cnt1 = 0;
		    GyroSum_add(1);
		    E_add += 1;
		}
	    }else cnt1 = 0;
		
	    GyroSum_add((A * (((L - L_prev)*100000) / E)) / 100000);
	    E_sum += (L - L_prev);
			
	}else{//L
	    if(get_IR(IR_R) > 2000 ){ //右壁近い //どうしても以外は使わない方がよい
		cnt1++;
		if(cnt1 > 5){
		    cnt1 = 0;
		    GyroSum_add(-1);
		    E_add += 1;//マイナスでもプラスを設定
		}
	    }else cnt1 = 0;
		
	    GyroSum_add((A * (((R - R_prev)*100000) / E)) / 100000);
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

	if(A > 0){//R
	    E_offset = rslsr90_offset;
	}else{//L
	    E_offset = rslsl90_offset;
	}
		
	if(E + E_offset - E_add - E_sum < 0)break;
		
    }

    //motor(0,0);
    GyroSum_reset();
		
    PORTA.DR.BIT.B0 = 0;
    PORTA.DR.BIT.B3 = 0;
	
    //ESmotor(45,M_kabe2,true,true);//60
    ESmotor(35,M_kabe2,true,false);//60 45
	
    //motor(0,0);
    GyroSum_reset();
    //Encoder_reset();
    /*	
	if(A > 0){//R
	GyroSum_add(-30);
	}else{//L
	GyroSum_add( 30);
	}
    */
}

void ETmotor_afterNaname(long long A, long long E, char non_stop){
    GyroSum_reset();
    //Encoder_reset();
	
    //int M_kabe = 30; //不要　これが無い版のカーブだから
    int M 		= 30;
    int M_kabe2 = 30;
	
    //	char flag = 0;


    GyroSum_reset();
		
    long long L = get_encoder_total_L();
    long long R = get_encoder_total_R();
    long long L_prev = L, R_prev = R;
    long long E_sum = 0;
    long long E_add = 0;
    long long E_offset = 0;	
	
    int powor_max = 20;
    int powor;
	
    int LM = 0, RM = 0;
    if(non_stop){
	LM = M;
	RM = M;
    }
    int LM_prev = LM, RM_prev = RM;
    int MA = 10,min_M = 5;
	

    static int cnt1 = 0;
	
    if(A > 0){//R
	GyroSum_add(5);
		
	PORTA.DR.BIT.B3 = 1;
    }else{//L
	GyroSum_add(-5);
		
	PORTA.DR.BIT.B0 = 1;
    }
    while(1){
		
	if(A > 0){//R
	    if(get_IR(IR_L) > 2000){ //左壁近い //どうしても以外は使わない方がよい
		cnt1++;
		if(cnt1 > 5){
		    cnt1 = 0;
		    GyroSum_add(1);
		    E_add += 1;
		}
	    }else cnt1 = 0;
		
	    GyroSum_add((A * (((L - L_prev)*100000) / E)) / 100000);
	    E_sum += (L - L_prev);
			
	}else{//L
	    if(get_IR(IR_R) > 2000 ){ //右壁近い //どうしても以外は使わない方がよい
		cnt1++;
		if(cnt1 > 5){
		    cnt1 = 0;
		    GyroSum_add(-1);
		    E_add += 1;//マイナスでもプラスを設定
		}
	    }else cnt1 = 0;
		
	    GyroSum_add((A * (((R - R_prev)*100000) / E)) / 100000);
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

	if(A > 0){//R
	    E_offset = rslsr90_offset;
	}else{//L
	    E_offset = rslsl90_offset;
	}
		
	if(E + E_offset - E_add - E_sum < 0)break;
		
    }

   // motor(0,0);
    GyroSum_reset();
		
    PORTA.DR.BIT.B0 = 0;
    PORTA.DR.BIT.B3 = 0;
	
    //ESmotor(45,M_kabe2,true,true);//60
    ESmotor(45,M_kabe2,true,false);//60
	
    //motor(0,0);
    GyroSum_reset();
    //Encoder_reset();
    /*	
	if(A > 0){//R
	GyroSum_add(-30);
	}else{//L
	GyroSum_add( 30);
	}
    */
}


 
void Tmotor_naname_in(long long A){
    //GyroSum_reset();
    //    Encoder_reset();

    //	static int cnt1 = 0;
		
    int LM = 0, RM = 0,LM_prev = 0, RM_prev = 0;
    int MA = 5,min_M = 15;
	
    int powor_max = 30;
    int powor;

    int M_kabe = 30;
	
    //	char flag = 0;

	
   
    if(A > 0){//R
	    //while(get_IR(IR_R) > 15){
	    while((get_IR(IR_R) > 10) || ( get_IR(IR_F) > 15 && get_IR(IR_F) < 23 ) ){ //前壁補正は斜めになると悪影響がある
	  
		Smotor(M_kabe,true);
		//	flag = 1;
	    }
	    //if(flag)ESmotor(115,M_kabe,true,false);
    }else{//L
	    //while(get_IR(IR_L) > 15){
	    while((get_IR(IR_L) > 10) || ( get_IR(IR_F) > 15 && get_IR(IR_F) < 23 ) ){ //前壁補正は斜めになると悪影響がある
		Smotor(M_kabe,true);
		//	flag = 1;
	    }
	    //if(flag)ESmotor(115,M_kabe,true,false);
    }
	    
    while( 50 < get_IR(IR_F) ){//前壁が近すぎる
	Smotor(-M_kabe,0); //バックする
    }
    
	
    if(A > 0){//R
	PORTA.DR.BIT.B3 = 1;
    }else{//L
	PORTA.DR.BIT.B0 = 1;
    }
	
    GyroSum_add(A);
    
    //   Encoder_reset();
    while(1){

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
	        motor(LM ,-4);
	}else{//L
	        //motor(-get_encoder_total_L() * 2 ,RM);
	        motor(-4 ,RM);
	}
	

	LM_prev = LM;
	RM_prev = RM;
		
	if(A > 0){//R
	    if(GyroSum_get() < 0)break;
	}else{//L
	    if(GyroSum_get() > 0)break;
	}
    }
    //motor(0,0);
    GyroSum_reset();
    //motor(0,0);
	
    PORTA.DR.BIT.B0 = 0;
    PORTA.DR.BIT.B3 = 0;

}

void Tmotor_naname_in_BIG(long long A ){
    //GyroSum_reset();
    //    Encoder_reset();

    //	static int cnt1 = 0;
	
    int M_kabe = 30;
    
    int LM = 0, RM = 0,LM_prev = 0, RM_prev = 0;
    int MA = 5,min_M = 20;
	
    int powor_max = 30;
    int powor;

    int M = 5;

	
    if(A > 0){//R
	PORTA.DR.BIT.B3 = 1;
    }else{//L
	PORTA.DR.BIT.B0 = 1;
    }
	
    if(A > 0){//R
	    while(get_IR(IR_RT) > 18){ 
	  
		Smotor(M_kabe,true);
	    }
    }else{
	  while(get_IR(IR_LT) > 18){ 
	  
		Smotor(M_kabe,true);
	    }  
    }
    
    GyroSum_add(A);
    
    //   Encoder_reset();
    while(1){
	
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
	    motor(LM + M ,10 + M);
	}else{//L
	    motor(10 + M ,RM + M);
	}
	
	
	LM_prev = LM;
	RM_prev = RM;
		
	if(A > 0){//R
	    if(GyroSum_get() < 0)break;
	}else{//L
	    if(GyroSum_get() > 0)break;
	}
    }
    
    //motor(0,0);
    GyroSum_reset();
    //motor(0,0);
	
    PORTA.DR.BIT.B0 = 0;
    PORTA.DR.BIT.B3 = 0;
 
}  

void Tmotor_naname_out(long long A ){
    //GyroSum_reset();
    //    Encoder_reset();

    //	static int cnt1 = 0;
		
    int LM = 0, RM = 0,LM_prev = 0, RM_prev = 0;
    int MA = 5,min_M = 20;
	
    int powor_max = 30;
    int powor;

    int M = 5;

	
    if(A > 0){//R
	PORTA.DR.BIT.B3 = 1;
    }else{//L
	PORTA.DR.BIT.B0 = 1;
    }
	
    GyroSum_add(A);
    
    //   Encoder_reset();
    while(1){
	
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
	    motor(LM + M ,10 + M);
	}else{//L
	    motor(10 + M ,RM + M);
	}
	
	
	LM_prev = LM;
	RM_prev = RM;
		
	if(A > 0){//R
	    if(GyroSum_get() < 0)break;
	}else{//L
	    if(GyroSum_get() > 0)break;
	}
    }
    
    //motor(0,0);
    GyroSum_reset();
    //motor(0,0);
	
    PORTA.DR.BIT.B0 = 0;
    PORTA.DR.BIT.B3 = 0;
 
}
void Tmotor_naname(long long A ,char inout){
   
    if(inout == 1){
	Tmotor_naname_in(A);
    }else if(inout == 2){
	Tmotor_naname_in_BIG(A);
    }else{
	Tmotor_naname_out(A);   
    }
}
