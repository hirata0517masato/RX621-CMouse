#include"Motor.h"
#include "Gyro.h"
#include "Encoder.h"
#include "IR.h"

#include "iodefine.h"
#include <machine.h>
#include <stdlib.h>

#define true 1
#define false 0

int LE = 0,RE = 0;
char motor_stop_flag = 0;
int LM_prev = 0,RM_prev = 0;
int safe_cnt = 0;


void motor_stop(){
	motor_stop_flag = 1;
}

char motor_stop_get(){
	return motor_stop_flag;
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
   	dt_R = MTU3.TGRA * (100.0 - duty_R) / 100.0;//  dt_R = 0.9445*50/100 = 0.5
		 
	/* デューティ比のオーバーフロー保護 */
    if(dt_R >= MTU3.TGRA)   dt_R = MTU3.TGRA - 1;  // 
		
	if(R_PM == 0){
		 /* デューティ比の設定 */
   		 MTU3.TGRB = MTU3.TGRA - 1;
   		 MTU4.TGRB = dt_R;
	}else{
		/* デューティ比の設定 */
   		 MTU3.TGRB = dt_R;
   		 MTU4.TGRB = MTU3.TGRA - 1;
	}
}

void motor(int LM,int RM){

  if(LM < -100)LM = -100;
  if(100 < LM)LM = 100;
  if(RM < -100)RM = -100;
  if(100 < RM)RM = 100;

  
  if(((LM - RM) > 400) || ((RM - LM) > 400))motor_stop();

  if(LM > 5){
	if(get_encoder_L() == 0){
		safe_cnt ++;
		if(safe_cnt > 10000)motor_stop();
	}else safe_cnt = 0;
  }

  if(RM > 5){
	if(get_encoder_R() == 0){
		safe_cnt ++;
		if(safe_cnt > 10000)motor_stop();
	}else safe_cnt = 0;
  }
  
  if(motor_stop_flag == 1){
	LM = 0;
	RM = 0;
  }

  LM_prev = LM;
  RM_prev = RM;
  
  pwm(LM,RM);
}


void Smotor(int M,char w_flag){

	if(w_flag){
		if(get_encoder_L() > 0){

			if((get_IR(IR_FL) < 30) && abs(GyroSum_get()) < 400){
				if((get_IR(IR_R) > 35)
			 	|| (get_IR(IR_L) > 20 && get_IR(IR_L) < 25)){
					long long n = (long long)-14 * get_encoder_L();
					if(n < -30)n = -30;
					GyroSum_add(n);
				}
			
				if((get_IR(IR_L) > 35 )
			 	|| (get_IR(IR_R) > 20 && get_IR(IR_R) < 25)){
					long long n = (long long)14 * get_encoder_L();
					if(n > 30)n = 30;
					GyroSum_add(n);
				}
			}
		}
	
		if(get_encoder_L() > 0){	
			if(get_IR(IR_FL) > 20){
				long long diff = (long long)((get_IR(IR_FR)) - get_IR(IR_FL));
				if(abs(diff) < 8){
					long long n = diff * get_encoder_L();
					if(n > 10)n = 10;
					if(n < -10)n = -10;
					GyroSum_add(n);
				}	
			}
		}
	}
	int powor_max = 40;
	int powor = gyro_powor_L();
	if(powor > powor_max)powor = powor_max;
	else if(-powor_max > powor)powor = -powor_max; 
	
	motor(M + powor ,M - powor);
}


void ESmotor(int A, int max_M,char non_stop,char w_flag){
	long long L = get_encoder_total_L();
	long long L_target = L + A;
	long long L_prev = A;
	int cnt = 0;
	
	int p = 5,d = 0,min_M = 10,M = 10;
	int max_FL = 35,Eb = 5,Ebf = 0;
	
	int non_stop_min_M = 20;
	
	while(1){
		if(get_IR(IR_FL) > max_FL && Ebf == 0){
			L_target -= Eb;
			Ebf = 1;
		}
		
		if((L_target - L) >= 0){	

			if(get_encoder_total_L() - (L_target-A) < A*2/3){//up
				M += (get_encoder_total_L() - (L_target-A)) / 3;	
			}else{
				M = (L_target - get_encoder_total_L())/4;//down
			}

			if(max_M < M)M = max_M;

			if(non_stop){
				if(M < non_stop_min_M)M = non_stop_min_M;
			}else{
				if(M < min_M)M = min_M;
			}

		}else{
			M = (L_target - L) * p   + ((L_target - L) - L_prev) * d;
		}
		 
		Smotor(M,w_flag);
		
		
		//delay(1);
		L_prev = (L_target - L);
		L = get_encoder_total_L();
	
		if(non_stop){
			if(L_target - L < 2)break; 
		}else{
			if(abs(L-L_target) < 3)cnt++;
			else cnt = 0;

			if(cnt > 100)break;
		}
	}


	motor(0,0);
//	GyroSum_reset();
	Encoder_reset();
}

void Tmotor(long long A){
	GyroSum_add(A);
	Encoder_reset();
	
	long long L = get_encoder_total_L();
	long long R = get_encoder_total_R();

	int sa = 0;
	int LM = 0, RM = 0,LM_prev = 0, RM_prev = 0;
	int MA = 8,min_M = 8;
	
	int cnt = 0;
	int powor_max = 20;
	int powor;

	while(1){
		powor = gyro_powor_L();
		
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

		if(abs(GyroSum_get()) < 80)cnt += 10;
		else if(abs(GyroSum_get()) < 150)cnt++;
		else cnt = 0;

		if(cnt > 1000)break;
	}


	motor(0,0);
//	GyroSum_reset();
	Encoder_reset();
}


void ETmotor(long long A, long long E, char non_stop){
//	GyroSum_reset();
	Encoder_reset();

	int M = 30;//35
	char flag = 0;
	if(A > 0){//R
		while(get_IR(IR_R) > 25){
			Smotor(M,true);
			flag = 1;
		}
		if(flag)ESmotor(40,M,true,false);
	}else{//L
		while(get_IR(IR_L) > 25){
			Smotor(M,true);
			flag = 1;
		}
		if(flag)ESmotor(40,M,true,false);
	}

//	GyroSum_reset();
	Encoder_reset();
	
	long long L = get_encoder_total_L();
	long long R = get_encoder_total_R();
	long long L_prev = L, R_prev = R;
	long long E_sum = 0;
	
	int powor_max = 30;
	int powor;
	
	int LM = 0, RM = 0;
	if(non_stop){
		LM = M;
		RM = M;
	}
	int LM_prev = LM, RM_prev = RM;
	int MA = 5,min_M = 5;
	
	int i = 0;


	while(1){
		
		i++;
		if(A > 0){//R
			GyroSum_add(A * (((L - L_prev)*1000) / E) / 1000 + 1 + (i&1));
			E_sum += (int)(L - L_prev);
		}else{//L
		   	GyroSum_add(A * (((R - R_prev)*1000) / E) / 1000 - 1 - (i&1));
			E_sum += (int)(R - R_prev);
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


	//motor(0,0);
//	GyroSum_reset();
	Encoder_reset();
}

