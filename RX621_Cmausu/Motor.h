#ifndef MOTOR_H
#define MOTOR_H

//#define	L1_PWM	PORT3.DR.BIT.B4       // CN2-4 : L1モーターPWM出力 
//#define	L2_PWM	PORT2.DR.BIT.B0       // CN2-16: L2モーターPWM出力 

//#define R1_PWM	PORT1.DR.BIT.B4       // CN2-10: R1モーターPWM出力
//#define R2_PWM	PORT2.DR.BIT.B4       // CN2-12: R2モーターPWM出力

void pwm_buff(char L,char R);
char get_pwm_buff_L(void);
char get_pwm_buff_R(void);
void motor_pid_flag_reset(void);
void Set_motor_pid_mode(char);

void motor_stop(void);
void motor(int,int);
void Smotor(int,char);
void ESmotor(long long,int,char,char);
char motor_stop_get(void);
void Tmotor(long long);
void ETmotor(long long , long long,char);
void ETmotorBIG(long long , long long,char);
void ETmotorU(long long , long long,char);
void Tmotor_naname(long long);
#endif