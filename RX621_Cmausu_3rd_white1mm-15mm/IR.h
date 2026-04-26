#ifndef IR_H
#define IR_H

#define IR_L 4
#define IR_FL 3
#define IR_F  2
#define IR_FR 1
#define IR_R 0

#define IR_LT 5
#define IR_RT 6

void AD_update( void );
void ir(char);
void ir_update(void);
int get_IR(int);
int get_IR_base(int);

#endif