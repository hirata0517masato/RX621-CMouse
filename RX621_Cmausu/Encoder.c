#include"Encoder.h"
#include "iodefine.h"
#include <machine.h>

#define LencTotal	(short)(MTU7.TCNT)	     //A:CN4-15 B:CN4-16
#define RencTotal	(short)(MTU8.TCNT)	     //A:CN4-13 B:CN4-14

volatile long long  L_enc_total_rev = 0;
volatile short  L_enc_base = 0;
volatile int  L_enc = 0;
volatile int  L_plus = 1;
volatile int  L_cnt = 0;

volatile long long  R_enc_total_rev = 0;
volatile short  R_enc_base = 0;
volatile int  R_enc = 0;
volatile int  R_plus = 1;
volatile int  R_cnt = 0;

void Encoder_reset(){
	L_enc = 0;
	R_enc = 0;
	L_enc_base = LencTotal;
	R_enc_base = RencTotal;
	L_enc_total_rev = 0;
	R_enc_total_rev = 0;
	L_plus = 1;
	L_cnt = 0;
	R_plus = 1;
	R_cnt = 0;
}

void encoder_update(){
	long long tmp;
	
	tmp = get_encoder_total_R();
	R_enc =  tmp -  R_enc_total_rev;
	R_enc_total_rev =  tmp;

	tmp = get_encoder_total_L();
	L_enc =  tmp -  L_enc_total_rev;
	L_enc_total_rev =  tmp;
	
}
int get_encoder_L(){
	return L_enc;
}

int get_encoder_R(){
	return R_enc;
}

long long get_encoder_total_R(){
	short tmp = RencTotal - R_enc_base;
	
	if(R_plus == 1 && tmp < -2000){
		R_cnt++;
		R_enc_base = RencTotal;
		tmp = 0;
	}
	
	if(R_plus == 0 && tmp > 2000){
		R_cnt--;
		R_enc_base = RencTotal;
		tmp = 0;
	}
	
	if(RencTotal - R_enc_base >= 0)R_plus = 1;
	else R_plus = 0;
	
	return (long long)tmp + (R_cnt * 32767);
}

long long get_encoder_total_L(){
	short tmp = LencTotal - L_enc_base;
	
	if(L_plus == 1 && tmp < -2000){
		L_cnt++;
		L_enc_base = LencTotal;
		tmp = 0;
	}
	
	if(L_plus == 0 && tmp > 2000){
		L_cnt--;
		L_enc_base = LencTotal;
		tmp = 0;
	}
	
	if(LencTotal - L_enc_base >= 0)L_plus = 1;
	else L_plus = 0;
	
	return (long long)tmp + (L_cnt * 32767);
}

