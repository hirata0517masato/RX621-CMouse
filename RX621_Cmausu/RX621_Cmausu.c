/****************************************************************************************************/
/* �v���W�F�N�g���FRX621_SAMPLE     	                �@�@�@�@�@          			    */
/* ���W���[�����F  			                     					    */
/* ��    ��    �F										    */
/* �g�p�}�C�R���FRX621 R5F56218BDFP (Flash ROM:512KB, SRAM:64KB, DATA Flash 32KB)                   */
/* �쐬��      �Fhirata0517masato               				                    */
/* �o�[�W����  �F0.00                                                                               */
/* �쐬��      �F2022/02/06 									    */
/****************************************************************************************************/                  
#include "iodefine.h"
#include <machine.h>
#include <stdlib.h>
#include "Parameters.h"
#include "wait.h"
#include "Gyro.h"
#include "DataFlash.h"
#include "Encoder.h"
#include "Motor.h"
#include "Queue.h"
#include "IR.h"
//#include "usb.h"

#include "printf_lib.h"   /* printf2 �֘A����    �R���p�C������у��C�u�����W�F�l���[�g�I�v�V�����ɂ�C99�Ή����K�v  */

//#define PRINT /* �g�p���͗L�������邱��*/

#ifndef PRINT
	#define printf2(...)  
	//#define scanf2(...) �@
#endif /* PRINT*/

/*======================================*/
/* �v���g�^�C�v�錾                     */
/*======================================*/
void ALL_init(void);
void CLK_init(void);
void IO_init(void);
void AD_init(void);
void CMT_init(void);
void MTU0_init(void);
void MTU1_init(void);

void led(char);
void led_up(void);
void led_down(void);
int get_sw(void);

void maze_save(void);
void maze_load(void);
void maze_update(char,char,char);
void maze_search_adachi(short,short);
void shortest_path_search_fin(void);
void remake_shortest_path_list_naname(void);
void run_shortest_path_fin(char);
	
void L_rotate(long long);
void R_rotate(long long);
void S_run(long long,int, char,char);
	
/* �萔�ݒ� */
#define true 1
#define false 0


//�O���[�o���ϐ�
short motor_stop_cnt = 0;
char maze_w[H][W] = {0};	//���4bit = �ǂ̊m��bit ����4bit = �ǂ̏��i���m��܂ށj
short maze_d[H][W][4] = {0};	//4�������̏d��

short dx[4] = {0,1,0,-1},dy[4] = {-1,0,1,0};

char my_x = Start_x,my_y = Start_y,my_angle = 1;//0:up 1:right 2:down 3:left

int ir_flag = 0; // 0:�ԊO��OFF 1:�ԊO��ON
/***********************************************************************/
/* ���C���v���O����                                                    */
/***********************************************************************/
void main(void)
{
/*	unsigned char c,buf[256];
	
 	float l=0,r=0;
	int aa;
	
	uint8_t prog_buff[512] = "hello world RX621";
	uint8_t read_buff[512];
	
*/
	volatile int i = 0;
	int mode = 0;
	
	ALL_init();//������
	
	delay(100);
	/*
	while(1){
		motor(20,0);
			
		//printf2("%d\t%d\t%d\t%d\n",get_IR(0),get_IR(1),get_IR(2),get_IR(3));
		printf2("%ld\n",get_encoder_total_L());
		delay(1000);
	}*/
		
	while(1){
		Encoder_reset();
		
		ir_flag = 0;//�ԊO��OFF
		
		//���[�h�I��
		while(1){
			led(mode);
			
			mode = get_encoder_total_R() / 50;
			
			if(get_sw() == 1){
				led_up();
				while(get_sw() == 1)nop();
				break;
			}
		}
		
		my_x = Start_x;
		my_y = Start_y;
		my_angle = 1;
		 
		ir_flag = 1;//�ԊO��ON
		
		if(mode != 5){//���H���̃��Z�b�g�łȂ����
			led(6);
			//�X�C�b�`���͑҂�
			while(get_sw() == 0) nop();
			while(get_sw() == 1) nop();
		
			//���ʃZ���T�[�Ɏ��������
			while((get_IR(0) < 20) || (get_IR(3) < 20)) led(1);
  			while((get_IR(0) > 20) || (get_IR(3) > 20)) led(8);
				
			led_up();
			
			GyroSum_reset();
			Encoder_reset();
		}
		
		switch(mode){
			case 1://�T�����[�h
				
				maze_search_adachi(Goal_x,Goal_y);
		
				maze_search_adachi(Start_x,Start_y);
				
				maze_search_adachi(Goal_x,Goal_y);
				
				break;
				
			case 2://�ŒZ���s���[�h
				shortest_path_search_fin();
				
				run_shortest_path_fin(false);
				
				break;
			case 3://�ŒZ���s�i�΂߂���j���[�h
				shortest_path_search_fin();
				remake_shortest_path_list_naname();
				
				run_shortest_path_fin(true);
				
				break;
				
				
				
				
			case 5://���H��񃊃Z�b�g���[�h
				for(int i = 0; i < H;i++)for(int j = 0; j < W; j++)maze_w[i][j] = 0;
				
				//���H�̊O���̊m��ǂ�ݒ�
				//0
  				for(int i = 0; i < W;i++)maze_w[0][i] |= 0x11;
  				//1
  				for(int i = 0; i < H;i++)maze_w[i][W-1] |= 0x22;
  				//2
  				for(int i = 0; i < W;i++)maze_w[H-1][i] |= 0x44;
  				//3
  				for(int i = 0; i < H;i++)maze_w[i][0] |= 0x88;
  
				//�X�^�[�g�n�_�̊m��ǂ�ݒ�
  				maze_w[my_y][my_x] = 0xfd;//13;//15-2;
  				maze_w[my_y+1][my_x] |= 0x11;
				maze_w[my_y][my_x+1] |= 0x80;
				
				break;
			default:
				led(9);
				delay(200);
				led(0);
				delay(200);
				led(9);
				delay(200);
				led(0);
				delay(200);
				break;
		}
		
		ir_flag = 0;//�ԊO��OFF
		
		led(9);
		//�X�C�b�`���͑҂�
		while(get_sw() == 0) nop();
		while(get_sw() == 1) nop();
		
		maze_save();
		
		led_up();
		led_down();
		
		//�X�C�b�`���͑҂�
		while(get_sw() == 0) nop();
		while(get_sw() == 1) nop();
		
		led(9);
		delay(100);
		
	}
}




/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */
/* �� �� �T �v�F������    	                                                                    */
/* �� �� �� �ׁF�e�평�����֐��̂܂Ƃ� �@�@�@�@�@�@�@�@�@�@�@�@�@�@                                 */
/* ��       ���F�Ȃ�										    */
/* ��  ��   �l�F�Ȃ�										    */
/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */ 
void ALL_init(){
	// �}�C�R���@�\�̏����� 
	CLK_init();  // �N���b�N�̏�����
 	IO_init();   // IO�̏�����
	WAIT_init(); //wait(CMT1)�̏�����
 	AD_init();   // A/D�̏�����
	MTU0_init();  //���[�^�[�̏�����
	MTU1_init();  //�G���R�[�_�̏�����
	DataFlash_init();//�f�[�^�t���b�V���̏�����
	
	led(0);
	ir(0);
	
	maze_load();//���H�f�[�^�̓ǂݍ���
  
	//�X�C�b�`���͑҂�
	while(get_sw() == 0) nop();
	while(get_sw() == 1) nop();
	
	Gyro_init();	//�W���C���ASPI�̏����� �@���ӁF�������Ԃ�����܂� �������̓W���C���Z���T�[�𓮂����Ȃ�����

	CMT_init();  // CMT0�̏�����

	initSCI1(SPEED_9600);
	//USB_init();  //USB CDC�̏�����
	
	led(9);
}

/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */
/* �� �� �T �v�F�N���b�N�̏�����                                                                    */
/* �� �� �� �ׁF�V�X�e���N���b�N96MHz,���Ӄ��W���[���N���b�N24MHz                                   */
/* ��       ���F�Ȃ�										    */
/* ��  ��   �l�F�Ȃ�										    */
/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */ 
void CLK_init(void)
{
   SYSTEM.SCKCR.BIT.ICK = 0;		// �V�X�e���N���b�N(ICLK)       EXTAL�~8 (96MHz)
   SYSTEM.SCKCR.BIT.PCK = 2;		// ���Ӄ��W���[���N���b�N(PCLK)	EXTAL�~2 (24MHz)
     
   //SYSTEM.SUBOSCCR = 1;              // 0�F�T�u�N���b�N���U�퓮��i0�F�f�t�H���g�œ��� 1:��~�j
   //RTC.RCR2.BIT.RTCOE = 1;           // 1�FRTCOUT��[�q(P32)����o�͂���
   
}

/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */
/* �� �� �T �v�FI/O�̏�����                                                                    �@�@ */
/* �� �� �� �ׁF                                                                                    */
/* ��       ���F�Ȃ�										    */
/* ��  ��   �l�F�Ȃ�										    */
/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */ 
void IO_init(void)
{
 	PORT1.DDR.BYTE = 0xff;           // 4:PWM_R1 3:MTIOC0B
	
    PORT2.DDR.BYTE = 0xff;           // 6:TxD1	4:PWM_R2	2:MTIOC3B	1:MTIOC1B	0:PWM_L2

    PORT3.DDR.BYTE = 0x10;           // 4:PWM_L1	0:RxD1
	
    PORT4.DDR.BYTE = 0x00;           // 7:AN7	6:AN6	5:AN5	4:AN4	3:AN3	2:AN2	1:AN1	0:AN0  
	
	PORT5.DDR.BYTE = 0x00;			 //4:MTIOC4B-B
	
	PORTA.DDR.BYTE = 0x0f; 			 //4:SW	3:LED4	2:LED3	1:LED2	0:LED1
	 
    PORTB.DDR.BYTE = 0x00; 	     	 // 5:���G���R�[�_B		4:���G���R�[�_A		3:�E�G���R�[�_B		2:�E�G���R�[�_A
	
	//PORTC.DDR.BYTE = 0xff;           // PC���o�͂ɐݒ�
	//  PORTC.PCR.BYTE   = 0x03;        // PC0,1���v���A�b�v�w��
	
    PORTD.DDR.BYTE = 0xff;           // 3:IR4	2:IR3	1:IR2	0:IR1
	
    PORTE.DDR.BYTE = 0x70; 	     	 // 7:SPI_SDO	6:SPI_SDI	5:SPI_SCL	4:SPI_CS    (6,7��IO�̓W���C���ڐ�)
    PORTE.PCR.BYTE = 0x70;           // PE7���v���A�b�v�w��
}



/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */
/* �� �� �T �v�FA/D�̏�����                                                                         */
/* �� �� �� �ׁF                                                                                    */
/* ��       ���F�Ȃ�										    */
/* ��  ��   �l�F�Ȃ�										    */
/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */ 
void AD_init(void)
{
   //A/D�̏�����
   SYSTEM.MSTPCRA.BIT.MSTPA17 = 0; //12bit��A/D�ϊ�����
   
   //MSTP(S12AD) = 0;//���W���[���X�g�b�v��Ԃ�����

   S12AD.ADCSR.BYTE = 0x0c;//���샂�[�h�A�ϊ��J�n�v���A�N���b�N�̐ݒ�
   S12AD.ADANS.WORD = 0x001f;//�X�L�����ϊ��[�q�̐ݒ� AN0-7�̑S�Ďg�p����ꍇ��0xff
   S12AD.ADSTRGR.BYTE = 0x0000;//A/D �ϊ��J�n�v���̐ݒ�
	
}



/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */
/* �� �� �T �v�FCMT0(�R���y�A�}�b�`�^�C�}�[)�̏�����                                                */
/* �� �� �� �ׁF1ms���荞�ݎ���                                                                     */
/* ��       ���F�Ȃ�										    */
/* ��  ��   �l�F�Ȃ�										    */
/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */ 
void CMT_init(void)
{
    MSTP(CMT0) = 0;                // SYSTEM.MSTPCRA.BIT.MSTPA15 = 0; // CMT0 �^�C�}�[�X�^���o�C���� �i0�ŉ����j
    CMT0.CMCR.WORD = 0x0040;       // 4:���荞�݋��@0:PCLK/8 1:PCLK/32 2:PCLK/128 3:PCLK/512
    CMT0.CMCOR = 3000-1;           // 1ms Count�F PCLK = 24MHz/8=3MHz 3M/1mS=3000 (�������J�E���g��-1)
    IPR(CMT0,CMI0) = 3;
    IEN(CMT0,CMI0) = 1;
	
    set_psw(0x00010000);
	
    CMT.CMSTR0.BIT.STR0 = 1;   	   // CMT0�^�C�}�[�X�^�[�g
}
	
/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */
/* �� �� �T �v�F�����[�^�[(MTU0,1�j,�E���[�^�[(MTU3,4�j �^�C�}�[�̏�����	   			            */
/* �� �� �� �ׁF		                                                    		                */
/* ��       ���F�Ȃ�										    									*/
/* ��  ��   �l�F�Ȃ�										    									*/
/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */ 
void MTU0_init(void){
	volatile int C_cycle,duty;
	
	C_cycle = 24e6 / 1000;
	
	SYSTEM.MSTPCRA.BIT.MSTPA9 = 0;  // MTU���j�b�g�O�@���W���[���X�g�b�v����
	MTUA.TSTR.BYTE &= 0x04 ;  // �J�E���^�̒�~  00xx x100�@
	
	MTU0.TCR.BIT.CCLR = 0x1; //
	MTU1.TCR.BIT.CCLR = 0x1; //
	MTU3.TCR.BIT.CCLR = 0x1; //
	MTU4.TCR.BIT.CCLR = 0x1; //
	
	MTU0.TMDR.BIT.MD = 0x2;  // PWM MODE1
	MTU1.TMDR.BIT.MD = 0x2;  // PWM MODE1
	MTU3.TMDR.BIT.MD = 0x2;  // PWM MODE1
	MTU4.TMDR.BIT.MD = 0x2;  // PWM MODE1

	MTU0.TIORH.BIT.IOA = 0x6; //
	MTU0.TIORH.BIT.IOB = 0x5; //
	MTU1.TIOR.BIT.IOA = 0x6; //
	MTU1.TIOR.BIT.IOB = 0x5; //
	MTU3.TIORH.BIT.IOA = 0x6; //
	MTU3.TIORH.BIT.IOB = 0x5; //
	MTU4.TIORH.BIT.IOA = 0x6; //
	MTU4.TIORH.BIT.IOB = 0x5; //
		
	MTU0.TGRA = C_cycle;
	MTU1.TGRA = C_cycle;
	MTU3.TGRA = C_cycle;
	MTU4.TGRA = C_cycle;
	
	MTU0.TGRB = 0;
	MTU1.TGRB = 0;
	MTU3.TGRB = 0;
	MTU4.TGRB = 0;
	
	IOPORT.PFCMTU.BIT.MTUS3=0;
	IOPORT.PFCMTU.BIT.MTUS4=0;
	IOPORT.PFCMTU.BIT.MTUS5=1;
	MTUA.TOER.BIT.OE4A = 1;  //
	
	MTUA.TSTR.BYTE = 0xc3;	// 11xx x011�@�J�E���^�̊J�n
	
  	motor(0,0);

}

/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */
/* �� �� �T �v�FMTU7(���G���R�[�_�j,MTU8(�E�G���R�[�_�j �^�C�}�[�̏�����   		            */
/* �� �� �� �ׁF		                                                                    */
/* ��       ���F�Ȃ�										    */
/* ��  ��   �l�F�Ȃ�										    */
/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */ 
void MTU1_init(void){
	
	SYSTEM.MSTPCRA.BIT.MSTPA8 = 0;  // MTU���j�b�g1�@���W���[���X�g�b�v����
	MTUB.TSTR.BYTE = 0x00;  // �J�E���^�̒�~
	
	IOPORT.PFDMTU.BIT.TCLKS=1;//�s���ݒ�(MTCLKE-B, MTCLKF-B,MTCLKG-B, MTCLKH-B��I��)

	MTU7.TMDR.BYTE=4;//MTU7���ʑ��v�����[�h��
	MTU8.TMDR.BYTE=4;//MTU8���ʑ��v�����[�h��
	
	PORTB.ICR.BIT.B2 = 1;
	PORTB.ICR.BIT.B3 = 1;
	PORTB.ICR.BIT.B4 = 1;
	PORTB.ICR.BIT.B5 = 1;

	MTUB.TSTR.BYTE = 0x06;	// 0000 0110 �J�E���^�̊J�n
	
	Encoder_reset();
}
		


/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */
/* �� �� �T �v�FLED�o��														  			            */
/* �� �� �� �ׁF												                                   */
/* ��       ���F 	3:LED4 2:LED3 1:LED2 0:LED1													    */
/* ��  ��   �l�F�Ȃ�										    									*/
/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */ 
void led(char n){
	
	if((n & 0x01) > 0)PORTA.DR.BIT.B0 = 1;
	else PORTA.DR.BIT.B0 = 0;
	
	if((n & 0x02) > 0)PORTA.DR.BIT.B1 = 1;
	else PORTA.DR.BIT.B1 = 0;
	
	if((n & 0x04) > 0)PORTA.DR.BIT.B2 = 1;
	else PORTA.DR.BIT.B2 = 0;
	
	if((n & 0x08) > 0)PORTA.DR.BIT.B3 = 1;
	else PORTA.DR.BIT.B3 = 0;
}

void led_up(){
	for(int i = 1;i <= 8; i*= 2){
		led(i);
		delay(100);
	}
	led(0);
}

void led_down(){
	for(int i = 8;i > 0; i/= 2){
		led(i);
		delay(100);
	}
	led(0);
}


/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */
/* �� �� �T �v�F�X�C�b�`����												  			            */
/* �� �� �� �ׁF												                                   */
/* ��       ���F �Ȃ�																			    */
/* ��  ��   �l�F on:1 off:0										    									*/
/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */ 
int get_sw(){
	return (PORTA.PORT.BIT.B4 == 1)? 0: 1 ;
}


/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */
/* �� �� �T �v�F���H���̕ۑ�                                                                         */
/* �� �� �� �ׁF                                                                                    */
/* ��       ���F�Ȃ�										    */
/* ��  ��   �l�F�Ȃ�										    */
/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */ 
void maze_save(){
	uint8_t buff[H*W];
	
	for(int i = 0; i < H;i++){
		for(int j = 0; j < W; j++){
			buff[i*W + j] = maze_w[i][j];
		}
	}
	
	DataFlash_write(1,buff,sizeof(buff));
}

/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */
/* �� �� �T �v�F���H���̕���                                                                        */
/* �� �� �� �ׁF                                                                                    */
/* ��       ���F�Ȃ�										    */
/* ��  ��   �l�F�Ȃ�										    */
/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */ 
void maze_load(){
	uint8_t buff[H*W];
	
	DataFlash_read(1,buff,sizeof(buff));
	
	for(int i = 0; i < H;i++){
		for(int j = 0; j < W; j++){
			 maze_w[i][j] = buff[i*W + j];
		}
	}
}

/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */
/* �� �� �T �v�F���H���̍X�V											  			            */
/* �� �� �� �ׁF												                                   */
/* ��       ���F ���ݒn��XY���W�A���p															    */
/* ��  ��   �l�F �Ȃ�										    									*/
/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */ 
void maze_update(char x,char y,char angle){
  if((maze_w[y][x]&0xf0) == 0xf0)return;

  for(short i = -1; i < 3;i++){
    short ii = (4 + angle+i)%4;
    int nx = x+dx[ii], ny = y+dy[ii];

    if((maze_w[y][x] & (1 << (4+ii))) == 0 ){
      maze_w[y][x] |= 1 << (4+ii);
      
      switch(i){
        case -1://L
          if(get_IR(IR_L) > 20)maze_w[y][x] |= 1 << ii;
          break;
        case 0://S
          if(get_IR(IR_FL) > 20)maze_w[y][x] |= 1 << ii;
          break;
        case 1://R
          if(get_IR(IR_R) > 20)maze_w[y][x] |= 1 << ii; 
          break;
      }
      if((0 <= nx && nx < W) && (0 <= ny && ny < H)){
        maze_w[ny][nx] |= 1 << (4+(ii+2)%4);
        if((maze_w[y][x] & (1 << ii)) != 0)maze_w[ny][nx] |= 1 << ((ii+2)%4);
      }
    }
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////
void L_rotate(long long a){
  Tmotor(-a);
  my_angle = (4+my_angle-1)%4;
}

void R_rotate(long long a){
  Tmotor(a);
  my_angle = (4+my_angle+1)%4;
}

void L_curve(long long a,char flag){
  ETmotor(-a,rsls90,flag);
 
  my_angle = (4+my_angle-1)%4;
}

void R_curve(long long a ,char flag ){
  ETmotor(a,rsls90,flag);
  
  my_angle = (4+my_angle+1)%4;
}

void S_run(long long path,int powor, char non_stop,char kabe){
  //GyroSum_reset();
	ESmotor(path,powor,non_stop,kabe);

  	int cnt2 = 0;

  	if(!non_stop && kabe){
   		// GyroSum_reset();
    	if(13 < get_IR(IR_FL) && 13 < get_IR(IR_FR) ){
	 		while(1){
      			if(get_IR(IR_FL) > 90){
        			Smotor(-10,true);

        			cnt2 = 0;
      			}else if(get_IR(IR_FL) < 70){
       	 			Smotor(+10,true);
       				
        			cnt2 = 0;
      			}else {
        			motor(0,0);
        			cnt2++;
      			}
      			if(cnt2 > 100)break;
			}
    	}
  	}
	
  	motor(0,0);
  //GyroSum_reset();
}

void S_run_kabe(int powor, char flag){//�ǐ؂�܂ő��s
  int Lflag = 0,Rflag = 0;
 
  while(1){
    if(Lflag == 0){
      if(get_IR(IR_L) > 20)Lflag = 1;
    }else if(Lflag == 1){
      if(get_IR(IR_L) < 10)break;
    }

    if(Rflag == 0){
      if(get_IR(IR_R) > 20)Rflag = 1;
    }else if(Rflag == 1){
      if(get_IR(IR_R) < 10)break;
    }
    
    Smotor(powor,flag);
  }
 
  ESmotor(160,powor,true,false);
}

void S_run_maze_search(int path,int powor){
	int M_pwm_min = 5;
	int M_pwm = M_pwm_min;
	long long enc_base = get_encoder_total_L();
	long long enc_now = 0;
	
	int path_cnt = 0;
	
	int cnt2 = 0;
	
	GyroSum_reset();
	
	while(1){
		Smotor(M_pwm,true);
		
		
		if(enc_now > (long long)path * s1){//�ڕW�����ɓ��B
		
			//�}�X�̒��S�܂ňړ�(�߂�j
			while(enc_now - ((long long)s1 * path_cnt ) > s1){
				Smotor(-10,true);
				enc_now = get_encoder_total_L() - enc_base;
			}
			
			if(get_IR(IR_FL) > 20 && get_IR(IR_FR) > 20){//�O�ǂ��������ꍇ��
				while(1){//�O�Ǖ␳
      				if(get_IR(IR_FL) > 90){
        				Smotor(-10,true);

	        			cnt2 = 0;
	      			}else if(get_IR(IR_FL) < 70){
	       	 			Smotor(+10,true);
	       
	        			cnt2 = 0;
	      			}else {
	        			motor(0,0);
	        			cnt2++;
	      			}
	      			if(cnt2 > 100)break;
				}
			}
			
			//���ݒn�̍X�V
			my_x += dx[my_angle];
			my_y += dy[my_angle];
			
			maze_update(my_x,my_y,my_angle);//���H���̍X�V
			
			break;
		}
		
		if( path_cnt < path-1 && get_IR(IR_FL) > 15 && get_IR(IR_FR) > 15){//�ڕW�܂łP�}�X�ȏ�c���Ă�@�����@�O�ǂ��o��
			//�}�X�̒��S�܂ňړ�
			while(enc_now - ((long long)s1 * path_cnt ) < s1){
				Smotor(+15,true);
				enc_now = get_encoder_total_L() - enc_base;
			}
		
			while(1){//�O�Ǖ␳
      			if(get_IR(IR_FL) > 90){
        			Smotor(-10,true);

        			cnt2 = 0;
      			}else if(get_IR(IR_FL) < 70){
       	 			Smotor(+10,true);
       
        			cnt2 = 0;
      			}else {
        			motor(0,0);
        			cnt2++;
      			}
      			if(cnt2 > 100)break;
			}
			
			//���ݒn�̍X�V
			my_x += dx[my_angle];
			my_y += dy[my_angle];
			
			maze_update(my_x,my_y,my_angle);//���H���̍X�V
			
			break;
		}
	
		if(enc_now - ((long long)s1 * path_cnt ) > s1){//�P�}�X�i��
			//���ݒn�̍X�V
			my_x += dx[my_angle];
			my_y += dy[my_angle];
			
			maze_update(my_x,my_y,my_angle);//���H���̍X�V
			
			path_cnt++;
		}
		
		if(enc_now < (long long)path * s1 /4){// �i�񂾋��� < �ڕW���� * 1/4�@�� �������
			M_pwm = M_pwm_min + (enc_now / 4);	
			
		}else if(enc_now > (long long)path * s1 * 3/4){// �i�񂾋��� < �ڕW���� * 3/4 = //�������
			M_pwm = M_pwm_min + ( ((long long)path * s1 - enc_now) / 4);	
			
		}else{
			M_pwm = powor;
		}
			
		if(M_pwm > powor)M_pwm = powor;
		if(M_pwm < M_pwm_min)M_pwm = M_pwm_min;
		
		enc_now = get_encoder_total_L() - enc_base;
	}
	
	motor(0,0);
	GyroSum_reset();
	Encoder_reset();
}

////////////////////////////////////////////////////////////////////////////////////////////////////////

/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */
/* �� �� �T �v�F�ŒZ�o�H�T��											  			            */
/* �� �� �� �ׁF												                                   */
/* ��       ���F �ړI�n��XY���W															    */
/* ��  ��   �l�F �Ȃ�										    									*/
/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */ 
void shortest_path_search(short target_x,short target_y){
  queue_reset();
  for(int i = 0; i < H;i++){
    for(int j = 0;j < W; j++){
      for(int k = 0; k < 4; k++){
        maze_d[i][j][k] = maze_d_max;
      }
    }
  }
  for(int k = 0; k < 4; k++){
    if((maze_w[target_y][target_x] & (1<<k)) == 0 )maze_d[target_y][target_x][k] = 0;
  }
  enqueue(target_x*100 + target_y);
  
  while(!queue_empty()){
    short x = dequeue(),y;
    y = x%100;
    x /=100;

    for(char i =0;i<4;i++){
      char update_flag = 0;
      short nx = x+dx[i],ny = y+dy[i];
      if((0 <= nx && nx < W) && (0 <= ny && ny < H) && ((maze_w[y][x] & (1<<i)) == 0 ) ){

        short num = maze_d[y][x][i];
        for(int k = 0; k < 4; k++){
           
          if(i == k){//S
            if(maze_d[ny][nx][k] > num + 1){
              update_flag = true;
              maze_d[ny][nx][k] = num + 1;
            }
          }else if((i+2+4)%4 == k){//B
            if(maze_d[ny][nx][k] > num+1 + r_cost*2){
              update_flag = true;
              maze_d[ny][nx][k] = num+1 + r_cost*2;
            }
          }else{// L or R
            if(maze_d[ny][nx][k] > num+1 + r_cost){
              update_flag = true;
              maze_d[ny][nx][k] = num+1 + r_cost;
            }
          }
        }
        if(update_flag)enqueue(nx*100 + ny);
      }
    }
  }
}

/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */
/* �� �� �T �v�F�ŒZ�o�H�쐬											  			            */
/* �� �� �� �ׁF												                                   */
/* ��       ���F �ړI�n��XY���W															    */
/* ��  ��   �l�F �Ȃ�										    									*/
/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */ 
void make_shortest_path_list(short target_x,short target_y){
	queue_reset();
  	short s_path = 0;
 	short x = my_x,y = my_y,angle = my_angle;
  	short unknown_flag = 0; // 0:�m��ǁ@1:���m���
	
  	while(x != target_x || y != target_y){
    	short num = maze_d[y][x][(angle+2)%4];
    	short n_num = 0;
    	char s_flag = 0;
    	short nx = x+dx[angle],ny = y+dy[angle];
    	if((0 <= nx && nx < W) && (0 <= ny && ny < H) && ((maze_w[y][x] & (1<<angle)) == 0 ) ){
      		short next = maze_d[ny][nx][(angle+2)%4];
      		if(num == next+1){
        		n_num = (angle+2)%4;
        		num = next;
        		s_flag = true;
      		}
    	}

    	if(s_flag == false){
      		for(int i = 0;i < 4;i++){
        		if(i == (angle+2)%4){
        		}else{
          			short next = maze_d[y][x][i];
          			if(num > next){
            			n_num = i;
            			num = next;
          			}
        		}
      		}
    	}
    
    	n_num = (n_num+2)%4;// 0 ~ 4
    	short ni = ((4 + n_num - ((4+angle-1)%4))%4) -1;// -1 ~ 2

    	if((maze_w[y][x] & (1 << (4+n_num))) == 0){//���m��̕�
		
			break;
			
			//if(ni != 0)break;//���i�����łȂ���Αł��؂�//////////////////////////////////////////////////////////////////////////////////////////////////////////////
			//ni = 10;//���m��̒���
		}
	
    	switch(ni){
      		case -1://L
        		if(s_path > 0){
          			enqueue(10 * unknown_flag );
					//enqueue(0);
          			enqueue(s_path);
          			s_path = 0;
        		}

        		enqueue(-1);
        		enqueue(1);

        		s_path += 1;
        
        		angle = (4+angle-1)%4;
		        x += dx[n_num];
		        y += dy[n_num];
				
				unknown_flag = 0;
		        break;
		    case 0://S
   
				if(unknown_flag == 1 && s_path > 0){
          			enqueue(10);
          			enqueue(s_path);
          			s_path = 0;
        		}
				
		        s_path +=1;
		        x += dx[n_num];
		        y += dy[n_num];
				
				unknown_flag = 0;
		        break;
				
			case 10://S ���m��̒���
   				
				if(unknown_flag == 0 && s_path > 0){
          			enqueue(0);
          			enqueue(s_path);
          			s_path = 0;
        		}
				
		        s_path +=1;
		        x += dx[n_num];
		        y += dy[n_num];
				
				unknown_flag = 1;
		        break;
				
      		case 1://R
		        if(s_path > 0){
			    	enqueue(10 * unknown_flag);
					//enqueue(0);
			        enqueue(s_path);
			        s_path = 0;
		        }
        
		        enqueue(1);
		        enqueue(1);

		        s_path += 1;
		         
		        angle = (4+angle+1)%4;
		        x += dx[n_num];
		        y += dy[n_num];
				
				unknown_flag = 0;
		        break;
      		case 2://B
        		if(s_path > 0){
          			enqueue(10 * unknown_flag);
					//enqueue(0);
          			enqueue(s_path);
          			s_path = 0;
        		}
        
        		enqueue(2);
        		enqueue(1);

				s_path += 1;
				 
        		angle = (4+angle+2)%4;
				x += dx[n_num];
		        y += dy[n_num];
				
				unknown_flag = 0;
        		break;
    	}
  	}
 
  	if(s_path > 0){
    	enqueue(10 * unknown_flag);
		//enqueue(0);
    	enqueue(s_path);
    	s_path = 0;
  	}
}

/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */
/* �� �� �T �v�F�ŒZ�o�H�𑖍s									  			            */
/* �� �� �� �ׁF												                                   */
/* ��       ���F �Ȃ�														    */
/* ��  ��   �l�F �Ȃ�										    									*/
/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */ 
void run_shortest_path(){
  GyroSum_reset();
  Encoder_reset();
  
  short comand ,path_num;
  int time = 50;
  
  while(!queue_empty()){
    comand = dequeue();path_num = dequeue();
	
    switch(comand){
      case -1://L
	  	delay(time);
		
        L_rotate(l90);
		
		delay(time);
        break;
      case 0://S
        if(path_num == 1){
	  		S_run(s1 * (long long)path_num,17,false,true);
			
		}else{
			S_run(s1 * (long long)path_num,22,false,true);
		}
		
        switch(my_angle){
          case 0:
            my_y -= path_num;
            break;
          case 1:
            my_x += path_num;
            break;
          case 2:
            my_y += path_num;
            break;
          case 3:
            my_x -= path_num;
            break;
        }
		//delay(time);
        break;
		
	/*  case 10://S ���m��̒���
        if(path_num == 1){
	  		S_run(s1 * (long long)path_num,20,false,true);
			
			my_x += dx[my_angle];
			my_y += dy[my_angle];
		}else{
			S_run_maze_search(path_num,20);
		}
		
        break;*/
      case 1://R
	  	delay(time);
		
        R_rotate(r90);
		
		delay(time);
        break;
      case 2://B
	  	delay(time);
		
        Tmotor(r180);
        my_angle = (4+my_angle+2)%4;
		
		delay(time);
        break;
    }
  }
}

/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */
/* �� �� �T �v�F�����@�ŒT�����s											  			            */
/* �� �� �� �ׁF												                                   */
/* ��       ���F �ړI�n��XY���W															    */
/* ��  ��   �l�F �Ȃ�										    									*/
/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */ 
void maze_search_adachi(short target_x,short target_y){
	GyroSum_reset();
	Encoder_reset();

	led_down();
	
	while(1){
    	maze_update(my_x,my_y,my_angle);
    	if((target_x != Goal_x || target_y != Goal_y) && (target_x != Start_x || target_y != Start_y)){//�X�^�[�g�n�_�A�S�[���n�_�ȊO���ڕW�n�_�̂Ƃ�
      		if((maze_w[target_y][target_x] & 0xf0) == 0xf0)break;    //�ڕW�n�_�̕ǂ����ׂĊm�肵����T������  
    	}
		
    	if(target_x == my_x && target_y == my_y){//�S�[��
			led_up();
			
      		if(target_x == Start_x && target_y == Start_y){
        		Tmotor(r180);
        		my_angle = (4+my_angle+2)%4;
      		}
      		break;
    	}
		
    	shortest_path_search(target_x,target_y);
    	make_shortest_path_list(target_x,target_y);
		run_shortest_path();
  	}
}

/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */
/* �� �� �T �v�F�ŒZ�o�H�T���i�ŏI�Łj											  			            */
/* �� �� �� �ׁF												                                   */
/* ��       ���F �Ȃ�														    */
/* ��  ��   �l�F �Ȃ�										    									*/
/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */ 
void shortest_path_search_fin(){
  queue_reset();
  for(int i = 0; i < H;i++){
    for(int j = 0;j < W; j++){
      for(int k = 0; k < 4; k++){
        maze_d[i][j][k] = maze_d_max;
      }
    }
  }
  for(int k = 0; k < 4; k++){
    if(((maze_w[Goal_y][Goal_x] & (1<<k)) == 0 ) && ((maze_w[Goal_y][Goal_x] & (1<<(4+k))) != 0 )){
      maze_d[Goal_y][Goal_x][k] = 0;
    }
  }
  enqueue(Goal_x*100 + Goal_y);
  
  while(!queue_empty()){
    short x = dequeue(),y;
    y = x%100;
    x /=100;

    for(char i =0;i<4;i++){
      char update_flag = 0;
      short nx = x+dx[i],ny = y+dy[i];
      if((0 <= nx && nx < W) && (0 <= ny && ny < H) && ((maze_w[y][x] & (1<<i)) == 0 )  && ((maze_w[y][x] & (1<<(4+i))) != 0 )  ){

        short num = maze_d[y][x][i];
        for(int k = 0; k < 4; k++){
           
          if(i == k){//S
            if(maze_d[ny][nx][k] > num + 1){
              update_flag = true;
              maze_d[ny][nx][k] = num + 1;
            }
          }else if((i+2+4)%4 == k){//B
            if(maze_d[ny][nx][k] > num+1 + r_cost*2){
              update_flag = true;
              maze_d[ny][nx][k] = num+1 + r_cost*2;
            }
          }else{// L or R
            if(maze_d[ny][nx][k] > num+1 + r_cost){
              update_flag = true;
              maze_d[ny][nx][k] = num+1 + r_cost;
            }
          }
        }
        if(update_flag)enqueue(nx*100 + ny);
      }
    }
  }
  
  for(int i = 1;i <= 8; i*= 2){
	led(i);
	delay(100);
  }
  led(0);
 
  //run_list
  queue_reset();
  short h_path = 0;
  my_x = Start_x;my_y = Start_y;my_angle = 1;
 
  while(my_x != Goal_x || my_y != Goal_y){

    short num = maze_d[my_y][my_x][(my_angle+2)%4];
    short n_num = 0;
    char s_flag = 0;
    int nx = my_x+dx[my_angle],ny = my_y+dy[my_angle];
    if((0 <= nx && nx < W) && (0 <= ny && ny < H) && ((maze_w[my_y][my_x] & (1<<my_angle)) == 0 )  && ((maze_w[my_y][my_x] & (1<<(4+my_angle))) != 0 ) ){
      short next = maze_d[ny][nx][(my_angle+2)%4];
      if(num == next+1){
        n_num = (my_angle+2)%4;
        num = next;
        s_flag = true;
      }
    }

    if(s_flag == false){
      for(int i = 0;i < 4;i++){
        if(i == (my_angle+2)%4){
        }else{
          short next = maze_d[my_y][my_x][i];
          if(num > next){
            n_num = i;
            num = next;
          }
        }
      }
    }
    
    n_num = (n_num+2)%4;// 0 ~ 4
    short ni = ((4 + n_num - ((4+my_angle-1)%4))%4) -1;// -1 ~ 2

    switch(ni){
      case -1://L
        if(h_path > 0){
          if(queue_empty())h_path--;
          enqueue(0);
          enqueue(h_path);
          h_path = 0;
        }

        enqueue(-1);
        enqueue(1);
        my_angle = (4+my_angle-1)%4;
        
        my_x += dx[n_num];
        my_y += dy[n_num];
        break;
      case 0://S
   
        h_path +=2;
        my_x += dx[n_num];
        my_y += dy[n_num];
        break;
      case 1://R
        if(h_path > 0){
          if(queue_empty())h_path--;
          enqueue(0);
          enqueue(h_path);
          h_path = 0;
        }
        
        enqueue(1);
        enqueue(1);
     
        my_angle = (4+my_angle+1)%4;

        my_x += dx[n_num];
        my_y += dy[n_num];
        break;
    }
  }
 
  if(h_path > 0){
    if(queue_empty())h_path--;
    enqueue(0);
    enqueue(h_path+1);
    h_path = 0;
   }else{
    enqueue(0);
    enqueue(1);
   }
  
  led_down();
}

/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */
/* �� �� �T �v�F�ŒZ�o�H�Ɏ΂ߗL����										  			            */
/* �� �� �� �ׁF												                                   */
/* ��       ���F �Ȃ�													    */
/* ��  ��   �l�F �Ȃ�										    									*/
/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */ 
void remake_shortest_path_list_naname(){
  enqueue(99);
  enqueue(99);
  int naname_cnt = 0,lr = 0;
  
  while(1){
    short mode = dequeue(),num = dequeue();
    if(mode == 99)break;

    switch(mode){
      case -1://L
        if(naname_cnt == 0){
          enqueue(-11);
          enqueue(1);
          naname_cnt = 1;
          lr = -1;
        }else{
          if(lr == 1){
            lr = -1;
            naname_cnt++;
          }else{
            enqueue(10);
            enqueue(naname_cnt);
            naname_cnt = 1;
            enqueue(-1);
            enqueue(1);
            lr = -1;
          }
        }
        break;
       case 0://S
        if(naname_cnt > 0){
          if(lr == 1){
            enqueue(10);
            enqueue(naname_cnt);
            enqueue(11);
            enqueue(1);
            naname_cnt = 0;
            lr = 0;
          }else{
            enqueue(10);
            enqueue(naname_cnt);
            enqueue(-11);
            enqueue(1);
            naname_cnt = 0;
            lr = 0;
          }
        }
        enqueue(mode);
        enqueue(num);
        break;
       case 1://R
        if(naname_cnt == 0){
          enqueue(11);
          enqueue(1);
          naname_cnt = 1;
          lr = 1;
        }else{
          if(lr == -1){
            lr = 1;
            naname_cnt++;
          }else{
            enqueue(10);
            enqueue(naname_cnt);
            naname_cnt = 1;
            enqueue(1);
            enqueue(1);
            lr = 1;
          }
        }
        break;
    }
  }
}

/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */
/* �� �� �T �v�F�ŒZ�o�H���s�i�ŏI�Łj											  			            */
/* �� �� �� �ׁF												                                   */
/* ��       ���F 0 �΂߂Ȃ��@�P�΂߂���														    */
/* ��  ��   �l�F �Ȃ�										    									*/
/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */ 
void run_shortest_path_fin(	char naname){
  GyroSum_reset();
  Encoder_reset();
  
  my_x = Start_x;my_y = Start_y;my_angle = 1;
  char non_stop = 0;
  int comand ,path_num;
  int first_flag = 0; //0:�܂����s���ĂȂ� 1:���s��
  
  while(!queue_empty()){
    comand = dequeue();path_num = dequeue();
    switch(comand){
      case -1://L
        if(naname)L_rotate(l90);
        else L_curve(sl90,non_stop);
        non_stop = 1;
        break;
      case -11://L45
        L_rotate(l45);
        non_stop = 1;
        break;
      case 0://S
        if(queue_empty())S_run(h1 * (long long)path_num + (h1/2),32,false,true);
        else {
          path_num--;
          if(path_num > 0){
			  if(first_flag == 0)S_run(h1 *(long long) path_num ,32,3,true); // memo : non_stop = 3 �����͂������@�����͂����Ȃ�
		  	  else S_run(h1 * (long long)path_num ,32,true,true);
		  }
          S_run_kabe(27,true);
        }

         non_stop = 0;
        //my_x = nx;
        //my_y = ny;
        break;
      case 10://Snaname
        S_run(s45 * (long long)path_num ,30,false,false);

         non_stop = 0;
        //my_x = nx;
        //my_y = ny;
        break;
      case 1://R
        if(naname)R_rotate(r90);
        else R_curve(sr90,non_stop);
        non_stop = 1;
        break;
      case 11://R45
        R_rotate(r45);
        non_stop = 1;
        break;
    }
	first_flag = 1;
  }

  led_up();
}

/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */
/* �� �� �T �v�FCMT0���荞�݃��W���[��                                                              */
/* �� �� �� �ׁF                                                                                    */
/* ��       ���F�Ȃ�										    */
/* ��  ��   �l�F�Ȃ�										    */
/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */
#pragma interrupt (Excep_CMT0_CMI0(vect=28))
void Excep_CMT0_CMI0(void)
{	
	static int task = 0;
	task ++;                         // �^�X�N�̍X�V						
  	if (task == 10) task = 0;            
	
	if(ir_flag == 1){
		ir_update();
	}else{
		ir(0);
	}
	
	Gyro_update();
		
	if(abs(Gyro()) > 250){
    	motor_stop_cnt++;
    	if(motor_stop_cnt > 20)motor_stop();
  	}else motor_stop_cnt = 0;
	
	switch(task) {                         			
	case 0:
		encoder_update();
        break;
   
   	case 1:
   		
        break;
		
	default:
		break;
   	}
}





 



