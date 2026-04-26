#include"Encoder.h"
#include "iodefine.h"
#include <machine.h>
#include <stdlib.h>

#define LencData	(-(short)(MTU7.TCNT))	     //A:CN4-15 B:CN4-16
#define RencData	(-(short)(MTU8.TCNT))	     //A:CN4-13 B:CN4-14

volatile long long  L_enc_total_rev = 0;
volatile long long  L_enc_total = 0;
volatile short  L_enc_base = 0;
volatile int  L_enc = 0;

volatile long long  R_enc_total_rev = 0;
volatile long long  R_enc_total = 0;
volatile short  R_enc_base = 0;
volatile int  R_enc = 0;


void Encoder_reset(){
	L_enc = 0;
	R_enc = 0;
	L_enc_base = LencData;
	R_enc_base = RencData;
	L_enc_total_rev = 0;
	L_enc_total = 0;
	R_enc_total_rev = 0;
	R_enc_total = 0;
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
	
	static char flag = 0;
	static short tmp = 0;
	static short sa = 0;
	
	if(flag == 1){
	    return R_enc_total;
	}
	flag = 1;
	
	tmp = RencData;
	sa = tmp - R_enc_base;
	
	if(abs(sa) > 10000){
		if( tmp >  R_enc_base){
			sa = (32767 - tmp) + ( R_enc_base + 32768);
		}else{
			sa = (32767 - R_enc_base) + ( tmp + 32768);
		}
	}
	
	R_enc_total += sa;
	R_enc_base = tmp;
	
	flag = 0;
	return R_enc_total;
}

long long get_encoder_total_L(){
	
	static char flag = 0;
	static short tmp = 0;
	static short sa = 0;
	
	if(flag == 1){
	    return L_enc_total;
	}
	flag = 1;
	
	tmp = LencData;
	sa = tmp - L_enc_base;
	
	if(abs(sa) > 10000){
		if( tmp >  L_enc_base){
			sa = (32767 - tmp) + ( L_enc_base + 32768);
		}else{
			sa = (32767 - L_enc_base) + ( tmp + 32768);
		}
	}
	
	L_enc_total += sa;
	L_enc_base = tmp;
	
	flag = 0;
	
	return L_enc_total;
}

