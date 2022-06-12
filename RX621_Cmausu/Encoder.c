#include"Encoder.h"
#include "iodefine.h"
#include <machine.h>

#define LencTotal	MTU7.TCNT	     //A:CN4-15 B:CN4-16
#define RencTotal	MTU8.TCNT	     //A:CN4-13 B:CN4-14

volatile long long  L_enc_total_rev = 0;
volatile long long  L_enc_base = 0;
volatile int  L_enc = 0;

volatile long long  R_enc_total_rev = 0;
volatile long long  R_enc_base = 0;
volatile int  R_enc = 0;


void Encoder_reset(){
	L_enc = 0;
	R_enc = 0;
	L_enc_base = LencTotal;
	R_enc_base = RencTotal;
	L_enc_total_rev = 0;
	R_enc_total_rev = 0;
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
	return RencTotal - R_enc_base;
}
long long get_encoder_total_L(){
	return LencTotal - L_enc_base;
}

