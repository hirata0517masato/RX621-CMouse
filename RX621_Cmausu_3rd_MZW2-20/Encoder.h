#ifndef ENCODER_H
#define ENCODER_H


void Encoder_reset(void);
void encoder_update(void);
int get_encoder_R(void);
int get_encoder_L(void);
long long get_encoder_total_R(void);
long long get_encoder_total_L(void);


#endif