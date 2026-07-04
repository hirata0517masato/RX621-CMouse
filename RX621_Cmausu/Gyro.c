#include "MPU9250.h"
#include"Gyro.h"
#include "rspi.h"
#include "wait.h"	
#include"Motor.h"
#include"Parameters.h"

int gyro= 0;        /* ジャイロセンサーの値  左＋　右ー   */
long long gyro_base = 0;
long long gyro_sum = 0;  //左＋　右ー  
long long gyro_sum_global = 0;  //左＋　右ー  

int Gyro_kp_search = 2,Gyro_kd_search = 30;//探索用
int Gyro_kp = 2,Gyro_kd = 45;//最短用 



void GyroSum_reset(){
  gyro_sum = 0;
}

void GyroSumGlobal_reset(){
  gyro_sum_global = 0;
}

void GyroSum_add(long long a){
  gyro_sum += a;
}

long long GyroSum_get(){
  return gyro_sum;
}

long long GyroSum_dig_get(){
	
  long long g_tmp = (GyroSum_get()/g_1)%360;
  if(180 < g_tmp) g_tmp -= 360;
  if(g_tmp < -180) g_tmp += 360;
  
  return g_tmp;
}

long long GyroSumGlobal_get(){
  return gyro_sum_global;
}

long long GyroSumGlobal_dig_get(){
	
  long long g_tmp = (GyroSumGlobal_get()/g_1)%360;
  if(180 < g_tmp) g_tmp -= 360;
  if(g_tmp < -180) g_tmp += 360;
  
  return g_tmp;
}

int Gyro(){
	return gyro;
}

long long Gyro_get(){
	read_gyro();
	return (long long)get_gyro_data(2)/4;
}

void GyroSum_add_auto(){
	/*
	long long g_global = GyroSumGlobal_dig_get();
	long long target = 0;
	
	if( 0 <= g_global && g_global < 22){//0
		target = 0;
	}else if(22 <= g_global && g_global < 67){//45
		target = 45;
	}else if(67 <= g_global && g_global < 112){//90
		target = 90;
	}else if(112 <= g_global && g_global < 157){//135
		target = 135;
	}else if(157 <= g_global){//180
		target = 180;
	}else if( -22 <= g_global && g_global < 0){//0
		target = 0;
	}else if( -67 <= g_global && g_global < -22){//-45
		target = -45;
	}else if(-112 <= g_global && g_global < -67){//-90
		target = -90;
	}else if(-157 <= g_global && g_global < -112){//-135
		target = -135;
	}else if(g_global < -157){//-180
		target = -180;
	}
	
	GyroSum_add((g_global - target) * g_1);
	*/
}

void Gyro_update(){
 
  //long long g = (((long long)gyro * 7) + (((long long)Gyro_get() - gyro_base) * 3)) / 10;
  //gyro = (long long)Gyro_get() - gyro_base;

 /* int g = (Gyro_get() - gyro_base);
  
  if(g < 0)gyro = (g * 105) / 100;
  else gyro = g;
*/


  gyro = (Gyro_get() - gyro_base);
  gyro_sum += (long long)gyro ; 
  gyro_sum_global += (long long)gyro ;
}

int gyro_powor_L(){
  if(Get_motor_pid_mode() == 0) return ((gyro_sum * (long long)Gyro_kp_search))/100  + ((long long)(gyro *Gyro_kd_search)) / 100;
  
  return ((gyro_sum * (long long)Gyro_kp))/100  + ((long long)(gyro *Gyro_kd)) / 100;
}

void Gyro_init(){
	int i;
	
  //pinMode(CS, OUTPUT);
  SPI_init();//SPI.begin();

  delay_ITP(100);
  MPU_init();
  delay_ITP(100);
   
  calib_acc();
  calib_mag();
	
  delay_ITP(300);
  
  gyro_base = 0;
  for(i = 0;i < 16;i++){
    gyro_base += Gyro_get();
    delay_ITP(1);
  }
  gyro_base >>= 4;
  gyro_sum = 0;
  //gyro = gyro_base;
}
