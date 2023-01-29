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

#define PRINT /* �g�p���͗L�������邱��*/

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
void CMT2_init(void);
void MTU0_init(void);
void MTU1_init(void);

void led(char);
void led_up(void);
void led_down(void);
int get_sw(void);

void maze_save(void);
void maze_load(void);
void log_reset(void);
void log_save(void);
void log_load(void);
void maze_update(char,char,char,char);
void maze_search_adachi(short,short);
void maze_search_all(void);
void shortest_path_search_fin(void);
void remake_shortest_path_list_naname(void);
void run_shortest_path_fin(char);
	
void L_rotate(long long);
void R_rotate(long long);
void S_run(long long,int, char,char);
	
/* �萔�ݒ� */
#define true 1
#define false 0

#define LOG_BUF_MAX 32  //DataFlash_write2 ���̐��l�����킹�邱��
#define LOG_MAX 2048


//�O���[�o���ϐ�
short motor_stop_cnt = 0;
char maze_w[H][W] = {0};	//���4bit = �ǂ̊m��bit ����4bit = �ǂ̏��i���m��܂ށj
short maze_d[H][W][4] = {0};	//4�������̏d��

short dx[4] = {0,1,0,-1},dy[4] = {-1,0,1,0};

char my_x = Start_x,my_y = Start_y,my_angle = Start_angle;//0:up 1:right 2:down 3:left

int ir_flag = 0; // 0:�ԊO��OFF 1:�ԊO��ON

int run_fin_speed_offset = 0;

long long time_limit = -1;

uint8_t log[LOG_MAX] = {0};
uint8_t log_buff[LOG_BUF_MAX] = {0};
char log_start = 0;
int log_cnt = 0;
int log_save_cnt = 0;
int log_block_num = 1; //max 15

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
	ir_flag = 0;//�ԊO��OFF
	
	ALL_init();//������
	
	delay(100);
/*
	while(1){
		led(0);
		ir_flag = 1;//�ԊO��ON
		//motor(20,20);

		//printf2("%d\t%d\t%d\t%d :",get_IR_base(0),get_IR_base(1),get_IR_base(2),get_IR_base(3));
		
		printf2("%d\t%d\t%d\t%d\n",get_IR(0),get_IR(1),get_IR(2),get_IR(3));
		//printf2("%ld\n",get_encoder_total_L());
		if(get_sw() == 1){
			Encoder_reset();
		}
		delay(100);
	}
*/			
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
		my_angle = Start_angle;
		
		if(mode != 5 && mode != 6 && mode != 7){//���x�������[�h�ł͂Ȃ��@�����@���H���̃��Z�b�g�łȂ���� �����@���O�o�͂łȂ����
			ir_flag = 1;//�ԊO��ON
			
			led(6);
			//�X�C�b�`���͑҂�
			while(get_sw() == 0){
				if(get_IR(0) > 10 && get_IR(3) < 10){
					led(1);
					
				}else if(get_IR(0) < 10 && get_IR(3) > 10){
					led(8);
				}else if(get_IR(0) > 10 && get_IR(3) > 10){
					led(9);
				}else{
					led(6);
				}
			}
			while(get_sw() == 1) nop();
		
			
			//���ʃZ���T�[�Ɏ��������
			while((get_IR(0) < 20) && (get_IR(3) < 20)) led(1);
  			while((get_IR(0) > 20) || (get_IR(3) > 20)) led(8);
				
			led_up();
			
			GyroSum_reset();
			Encoder_reset();
		}
		
		switch(mode){
			case 1://�T�����[�h
			
				log_reset();//���O�̏�����
				log_start = 1; //���O�L�^�J�n�@30ms�ɂP��L�^
				
				maze_search_adachi(Goal_x,Goal_y);
		
				maze_search_adachi(Start_x,Start_y);
				
				log_start = 0; //���O�L�^�I��
				
				break;
				
				
			case 2://�ŒZ���s���[�h
				shortest_path_search_fin();
				
				log_reset();//���O�̏�����
				log_start = 2; //���O�L�^�J�n 10ms�ɂP��L�^
				
				run_shortest_path_fin(false);
				
				log_start = 0; //���O�L�^�I��
				
				led(9);
				delay(500);
				led_up();
				
				my_x = Goal_x;
				my_y = Goal_y;
				my_angle = Goal_angle;
				
				maze_search_adachi(Pickup_x,Pickup_y);//�E���₷���Ƃ���܂ňړ�����
				
				break;
			case 3://�ŒZ���s�i�΂߂���j���[�h
				shortest_path_search_fin();
				remake_shortest_path_list_naname();
				
				log_reset();//���O�̏�����
				log_start = 2; //���O�L�^�J�n 10ms�ɂP��L�^
				
				run_shortest_path_fin(true);
				
				log_start = 0; //���O�L�^�I��
				
				led(9);
				delay(500);
				led_up();
				
				my_x = Goal_x;
				my_y = Goal_y;
				my_angle = Goal_angle;
				
				maze_search_adachi(Pickup_x,Pickup_y);//�E���₷���Ƃ���܂ňړ�����
				
				break;
				
			case 4://�ŒZ�o�H��̖��m��}�X�����ׂĒT���ɍs��
				maze_search_all();
				
				led(9);
				delay(500);
				led_up();
				
				my_x = Goal_x;
				my_y = Goal_y;
				my_angle = Goal_angle;
				
				maze_search_adachi(Pickup_x,Pickup_y);//�E���₷���Ƃ���܂ňړ�����
				
				break;
				
			case 5://���x�������[�h�i�����l�͕ۑ����Ȃ��A�{�Ԃł̔������p�j
				Encoder_reset();
		
				led(6);
				delay(1000);
				
				//���[�h�I��
				while(1){
					led(run_fin_speed_offset);
			
					run_fin_speed_offset = get_encoder_total_R() / 50;
			
					if(get_sw() == 1){
						led_up();
						while(get_sw() == 1)nop();
						break;
					}
				}
		
				break;
			case 6://���H��񃊃Z�b�g���[�h
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
				
			case 7://���O�o�̓��[�h
				for(int i = 0; i < H;i++){
					for(int j = 0; j < W; j++)printf2("%d\t",maze_w[i][j]&0x0f);
					printf2("\n");
				}
				printf2("\n");
				
				log_load();//���[�h���o��
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
  
	led(6);
	//�X�C�b�`���͑҂�
	while(get_sw() == 0) nop();
	led(0);
	while(get_sw() == 1) nop();
	
	Gyro_init();	//�W���C���ASPI�̏����� �@���ӁF�������Ԃ�����܂� �������̓W���C���Z���T�[�𓮂����Ȃ�����

	CMT_init();  // CMT0�̏�����
	CMT2_init();  // CMT2�̏�����

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
/* �� �� �T �v�FCMT2(�R���y�A�}�b�`�^�C�}�[)�̏�����                                                */
/* �� �� �� �ׁF0.25ms���荞�ݎ���                                                                     */
/* ��       ���F�Ȃ�										    */
/* ��  ��   �l�F�Ȃ�										    */
/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */ 
void CMT2_init(void)
{
    MSTP(CMT2) = 0;                // SYSTEM.MSTPCRA.BIT.MSTPA15 = 0; // CMT0 �^�C�}�[�X�^���o�C���� �i0�ŉ����j
    CMT2.CMCR.WORD = 0x0040;       // 4:���荞�݋��@0:PCLK/8 1:PCLK/32 2:PCLK/128 3:PCLK/512
    CMT2.CMCOR = 750-1;           // 0.25ms Count�F PCLK = 24MHz/8=3MHz 3M/0.25mS=750 (�������J�E���g��-1) 
    IPR(CMT2,CMI2) = 3;
    IEN(CMT2,CMI2) = 1;
	
    set_psw(0x00010000);
	
    CMT.CMSTR1.BIT.STR2 = 1;   	   // CMT2�^�C�}�[�X�^�[�g
}
	
/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */
/* �� �� �T �v�F�����[�^�[(MTU0,1�j,�E���[�^�[(MTU3,4�j �^�C�}�[�̏�����	   			            */
/* �� �� �� �ׁF		                                                    		                */
/* ��       ���F�Ȃ�										    									*/
/* ��  ��   �l�F�Ȃ�										    									*/
/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */ 
void MTU0_init(void){
	volatile int C_cycle;
	
	//C_cycle = 24e6 / 1000;
	C_cycle = 1500;
	
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
		delay(80);
	}
	led(0);
}

void led_down(){
	for(int i = 8;i > 0; i/= 2){
		led(i);
		delay(80);
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
	
	DataFlash_write(0,buff,sizeof(buff));
}

/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */
/* �� �� �T �v�F���H���̕���                                                                        */
/* �� �� �� �ׁF                                                                                    */
/* ��       ���F�Ȃ�										    */
/* ��  ��   �l�F�Ȃ�										    */
/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */ 
void maze_load(){
	uint8_t buff[H*W];
	
	DataFlash_read(0,buff,sizeof(buff));
	
	for(int i = 0; i < H;i++){
		for(int j = 0; j < W; j++){
			 maze_w[i][j] = buff[i*W + j];
		}
	}
}


/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */
/* �� �� �T �v�F���O�̏�����                                                                       */
/* �� �� �� �ׁF                                                                                    */
/* ��       ���F�Ȃ�										    */
/* ��  ��   �l�F�Ȃ�										    */
/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */ 
void log_reset(){
	log_cnt = 0;
	log_save_cnt = 0;
	log_block_num = 1;
	for(int i = 0; i < LOG_MAX;i++)log[i] = 0;
	
	for(int i = 1; i < 16;i++){//����:0�͖��H���Ȃ̂ō폜���Ȃ�
		//DataFlash_write(i,log,sizeof(log));
		R_FlashErase(i + 38);// BLOCK_DB0    38 
	}
}
/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */
/* �� �� �T �v�F���O�̕ۑ�                                                                       */
/* �� �� �� �ׁF                                                                                    */
/* ��       ���F�Ȃ�										    */
/* ��  ��   �l�F�Ȃ�										    */
/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */ 
void log_save(){
	
	DataFlash_write2(log_block_num,log_save_cnt,log_buff,sizeof(log_buff));
}


/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */
/* �� �� �T �v�F���O�̓ǂݍ���                                                                        */
/* �� �� �� �ׁF                                                                                    */
/* ��       ���F�Ȃ�										    */
/* ��  ��   �l�F�Ȃ�										    */
/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */ 
int log_minus(uint8_t data){//�������}�C�i�X�̒l���C������
	
	int ret = data;
	
	if((data & 0x80) != 0){//�}�C�i�X������
			data ^= 0xff;
			data += 1;
			ret = -((int)data);
			
	}
	return ret;
}

void log_load(){
	int i = 0;
	int block_num = 1;
	
	printf2("X\tY\tS1\tS2\tS3\tS4\tPWM_L\tPWM_R\n\n");
	
	while(block_num < 16){
		DataFlash_read(block_num,log,sizeof(log));
		i = 0;
		
		while(1){
			if( i >= LOG_MAX)break;
			if(log[i+2] == 0 && log[i+3] == 0 && log[i+3] == 0 && log[i+5] == 0)break;//�Z���T�[�l�����ׂĂO�Ȃ�I��
			if(log[i+0] > 16 || log[i+1] > 16)break;//X,Y���W���͈͊O�Ȃ�I��
			
			printf2("%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t\n",log[i],log[i+1]
														,log[i+2],log[i+3],log[i+4],log[i+5]
														,log_minus(log[i+6]),log_minus(log[i+7]) );	
			i += 8;
		}
		block_num++;
	}
}

/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */
/* �� �� �T �v�F���H���̍X�V											  			            */
/* �� �� �� �ׁF												                                   */
/* ��       ���F ���ݒn��XY���W�A���p ,�X�V��������i1:�O 2:�� ����ȊO:����)					    */
/* ��  ��   �l�F �Ȃ�										    									*/
/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */ 
void maze_update(char x,char y,char angle, char type){
  if((maze_w[y][x]&0xf0) == 0xf0)return;

  for(short i = -1; i < 3;i++){
	if(type == 1){
		if(i != 0)continue;//�O�ȊO�͍X�V���Ȃ�
	}else if(type == 2){
		if(i == 0)continue;//���ȊO�͍X�V���Ȃ�
	}
	
    short ii = (4 + angle+i)%4;
    int nx = x+dx[ii], ny = y+dy[ii];

 //   if((maze_w[y][x] & (1 << (4+ii))) == 0 ){//���m��̏ꍇ
      maze_w[y][x] |= 1 << (4+ii);
      
      switch(i){
        case -1://L
			//if((maze_w[y][x] & (1 << (4+ii))) == 0 ){//���m��̏ꍇ
          		if(get_IR(IR_L) > 15){
					maze_w[y][x] |= 1 << ii;
		  		}else{
					maze_w[y][x] &= ~(1 << ii);  
		  		}
			//}
         	break;
        case 0://S
			//if((maze_w[y][x] & (1 << (4+ii))) == 0 ){//���m��̏ꍇ
          		if(get_IR(IR_FL) > 13 || get_IR(IR_FR) > 13){
					maze_w[y][x] |= 1 << ii; //���O�����l������?
		  		}else{
					maze_w[y][x] &= ~(1 << ii);
		  		}
			// }
         	 break;
        case 1://R
			//if((maze_w[y][x] & (1 << (4+ii))) == 0 ){//���m��̏ꍇ
          		if(get_IR(IR_R) > 15){
		  			maze_w[y][x] |= 1 << ii; 
		  		}else{
					maze_w[y][x] &= ~(1 << ii);
		  		}
			//}
          	break;
      }
      if((0 <= nx && nx < W) && (0 <= ny && ny < H)){
        maze_w[ny][nx] |= 1 << (4+(ii+2)%4);
		maze_w[ny][nx] &= ~(1 << ((ii+2)%4));
		
       	if((maze_w[y][x] & (1 << ii)) != 0){	
			maze_w[ny][nx] |= 1 << ((ii+2)%4);
		}
      }
 //  }
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
  ETmotor(-a,rslsl90,flag);
 
  my_angle = (4+my_angle-1)%4;
}

void R_curve(long long a ,char flag ){
  ETmotor(a,rslsr90,flag);
  
  my_angle = (4+my_angle+1)%4;
}

void L_rotate_naname(long long a){
  Tmotor_naname(-a);
  my_angle = (4+my_angle-1)%4;
}

void R_rotate_naname(long long a){
  Tmotor_naname(a);
  my_angle = (4+my_angle+1)%4;
}

void S_run(long long path,int powor, char non_stop,char kabe){
  //GyroSum_reset();
	ESmotor(path,powor,non_stop,kabe);

  	int cnt2 = 0;

  	if(!non_stop && kabe == 1){
   		// GyroSum_reset();
    	if(15 < get_IR(IR_FL) && 15 < get_IR(IR_FR) ){
	 		while(1){
      			if(get_IR(IR_FR) > 58){
        			Smotor(-5,false);

        			cnt2 = 0;
      			}else if(get_IR(IR_FR) < 55){
       	 			Smotor(+5,false);
       				
        			cnt2 = 0;
      			}else {
        			motor(0,0);
        			cnt2++;
      			}
      			if(cnt2 > 2000)break;
			}
    	}
  	}
	
  	motor(0,0);
  //GyroSum_reset();
}

void S_run_kabe(int powor, char flag, int LR){//�ǐ؂�܂ő��s
  int Lflag = 0,Rflag = 0;
  long long enc_base = (get_encoder_total_L() + get_encoder_total_R())/2;
   int cnt2 = 0;
	
  while(1){
	if(LR == 3 || LR == 1){
    	if(Lflag == 0){
      		if((get_IR(IR_L) > 20) || (get_IR(IR_L) > 13 && get_IR(IR_R) > 70)){//���΂̕ǂ��߂��Ƃ��͒��̒l���������Ȃ�
				Lflag = 1;
				led(1);
			}
    	}else if(Lflag == 1){
      		if(get_IR(IR_L) < 11){
				led(0);
				break;
			}
    	}
	}

	if(LR == 3 || LR == 2){
    	if(Rflag == 0){
      		if((get_IR(IR_R) > 20) || (get_IR(IR_L) > 70 && get_IR(IR_R) > 13)){//���΂̕ǂ��߂��Ƃ��͒��̒l���������Ȃ�
				Rflag = 1;
				led(8);
			}
    	}else if(Rflag == 1){
      		if(get_IR(IR_R) < 11){
				led(0);
				break;
			}
    	}
	}
    
    Smotor(powor,flag);
	
	if(get_IR(IR_FL) > 70 && get_IR(IR_FR) > 70){//�O�ǂ��߂�����ꍇ��
		while(1){//�O�Ǖ␳
      		if(get_IR(IR_FR) > 58){
        		Smotor(-5,false);
        		cnt2 = 0;
     		}else if(get_IR(IR_FR) < 55){
	      		Smotor(+5,false);
	     
	       		cnt2 = 0;
	    	}else {
	       		motor(0,0);
	       		cnt2++;
	      	}
	      	if(cnt2 > 2000){
				led(0);
				return;
			}
		}
	}
	
	if(abs((get_encoder_total_L() + get_encoder_total_R())/2 -  enc_base) > (s1+h1 )){
		led(9);
		break; //�ǐ؂ꂪ���Ȃ�������u���[�N
	}
  }
  
  ESmotor(170,powor,true,false);
  led(0);
}

void S_run_kabe2(int powor, char flag, int LR){//�ǐ؂�܂ő��s ��������̂S�T�^�[��
  int Lflag = 0,Rflag = 0;
  long long enc_base = (get_encoder_total_L() + get_encoder_total_R())/2;
	
  while(1){
	if(LR == 3 || LR == 1){
    	if(Lflag == 0){
      		if((get_IR(IR_L) > 20) || (get_IR(IR_L) > 13 && get_IR(IR_R) > 70)){//���΂̕ǂ��߂��Ƃ��͒��̒l���������Ȃ�
				Lflag = 1;
				led(1);
			}
    	}else if(Lflag == 1){
      		if(get_IR(IR_L) < 11){
				led(0);
				break;
			}
    	}
	}

	if(LR == 3 || LR == 2){
    	if(Rflag == 0){
      		if((get_IR(IR_R) > 20) || (get_IR(IR_L) > 70 && get_IR(IR_R) > 13)){//���΂̕ǂ��߂��Ƃ��͒��̒l���������Ȃ�
				Rflag = 1;
				led(8);
			}
    	}else if(Rflag == 1){
      		if(get_IR(IR_R) < 11){
				led(0);
				break;
			}
    	}
	}
    
    Smotor(powor,flag);
	
	if(abs((get_encoder_total_L() + get_encoder_total_R())/2 -  enc_base) > (s1+h1 )){
		led(9);
		break; //�ǐ؂ꂪ���Ȃ�������u���[�N
	}
  }
  
  ESmotor(5,powor,true,false);//�@��������̂S�T�^�[�� ����������̂ŕs�v
  led(0);
}

void S_run_kabe_naname(int powor, char flag, int LR){//�ǐ؂�܂ő��s
  int Lflag = 0,Rflag = 0;
  long long enc_base = get_encoder_total_L();
  
  led(6);
  
  while(1){
	if(LR == 3 || LR == 1){
		if(Lflag == 0){
      		if(get_IR(IR_L) > 30){
				led(1);
				Lflag = 1;
			}
    	}else if(Lflag == 1){
      		if(get_IR(IR_L) < 15){
				led(0);
				break;
			}
    	}
	}

	if(LR == 3 || LR == 2){
		if(Rflag == 0){
      		if(get_IR(IR_R) > 30){
				led(8);
				Rflag = 1;
			}
    	}else if(Rflag == 1){
      		if(get_IR(IR_R) < 15){
				led(0);
				break;
			}
    	}
	}
    
    Smotor(powor,flag);
	
	
	
	if(abs(get_encoder_total_L() -  enc_base) > (s45+s45)  ){
		led(9);
		
		break; //�ǐ؂ꂪ���Ȃ�������u���[�N
	}
  }
 
  
  ESmotor(180,powor,true,false);
  led(0);
}

void S_run_maze_search(int path,int powor){
	Encoder_reset();
	
	int M_pwm_min = 5;
	int M_pwm = M_pwm_min;
	long long enc_base = (get_encoder_total_L() + get_encoder_total_R())/2;
	long long enc_now = 0;
	
	int path_cnt = 0;
	int maza_update_flag = 0;
	
	int cnt2 = 0;
	
	int ir_L_now = 0,ir_R_now = 0;
	int ir_L_flag = 0,ir_R_flag = 0;
	int path_cnt_save_L = -1;//�����}�X�ŕǐ؂ꏈ�����Q��ȏサ�Ȃ��悤�Ɋo���Ă����ϐ�
	int path_cnt_save_R = -1;//�����}�X�ŕǐ؂ꏈ�����Q��ȏサ�Ȃ��悤�Ɋo���Ă����ϐ�
//	int hosei_kyori_L = -1,hosei_kyori_R = -1;//�ǐ؂ꎞ�̕␳�����@���قȂ�^�C�~���O�ŕǐ؂ꂵ���ۂɗ��p����
	long long enc_kabe_L,enc_kabe_R;
	
//	int kame_hosei = 230;//170
	
	GyroSum_reset();
	
	while(1){
		
		if(enc_now >= (long long)path * s1){//�ڕW�����ɓ��B
		
			//�}�X�̒��S�܂ňړ�(�߂�j
			while(enc_now - ((long long)s1 * path_cnt ) > s1){
				Smotor(-10,true);
				enc_now = (get_encoder_total_L() + get_encoder_total_R())/2 - enc_base;
			}
			
			if(get_IR(IR_FL) > 15 && get_IR(IR_FR) > 15){//�O�ǂ��������ꍇ��
				while(1){//�O�Ǖ␳
      				if(get_IR(IR_FR) > 58){
        				Smotor(-5,false);

	        			cnt2 = 0;
	      			}else if(get_IR(IR_FR) < 55){
	       	 			Smotor(+5,false);
	       
	        			cnt2 = 0;
	      			}else {
	        			motor(0,0);
	        			cnt2++;
	      			}
	      			if(cnt2 > 2000)break;
				}
			}
			
			//���ݒn�̍X�V
			my_x += dx[my_angle];
			my_y += dy[my_angle];
			
			maze_update(my_x,my_y,my_angle,3);//���H���̍X�V
			
			break;
		}
		
		if( path_cnt < path-1 && get_IR(IR_FL) > 25 && get_IR(IR_FR) > 25){//�ڕW�܂łP�}�X�ȏ�c���Ă�@�����@�O�ǂ��o��
			//�}�X�̒��S�܂ňړ�
			while(enc_now - ((long long)s1 * path_cnt ) < s1 && get_IR(IR_FR) < 50){
				Smotor(+10,true);
				enc_now = (get_encoder_total_L() + get_encoder_total_R())/2- enc_base;
			}
		
			while(1){//�O�Ǖ␳
      			if(get_IR(IR_FR) > 58){
        			Smotor(-5,false);

        			cnt2 = 0;
      			}else if(get_IR(IR_FR) < 55){
       	 			Smotor(+5,false);
       
        			cnt2 = 0;
      			}else {
        			motor(0,0);
        			cnt2++;
      			}
      			if(cnt2 > 2000)break;
			}
			
			if(maza_update_flag != 2){//�Ȃ����ǂ̍X�V���ł��Ă��Ȃ����
				if(maza_update_flag == 0)maze_update(my_x + dx[my_angle],my_y + dy[my_angle],my_angle,3);//���H���̍X�V
				else if(maza_update_flag == 1)maze_update(my_x + dx[my_angle],my_y + dy[my_angle],my_angle,1);//���H���̍X�V
			}
			//���ݒn�̍X�V
			my_x += dx[my_angle];
			my_y += dy[my_angle];
			
			maze_update(my_x,my_y,my_angle,3);//���H���̍X�V
			maza_update_flag = 0;
			
			break;
		}
	
		if( path_cnt < path-1){//�ڕW�܂łP�}�X�ȏ�c���Ă� �����F�Ō�̂P�}�X�͑O�Ǖ␳��ɖ��H�����X�V���邽��
			if(enc_now - ((long long)s1 * path_cnt ) > s1){//�P�}�X�i��
			
				if(maza_update_flag != 2){//�Ȃ����ǂ̍X�V���ł��Ă��Ȃ����
					if(maza_update_flag == 0)maze_update(my_x + dx[my_angle],my_y + dy[my_angle],my_angle,3);//���H���̍X�V
					else if(maza_update_flag == 1)maze_update(my_x + dx[my_angle],my_y + dy[my_angle],my_angle,1);//���H���̍X�V
				}
				//���ݒn�̍X�V
				my_x += dx[my_angle];
				my_y += dy[my_angle];
			
				path_cnt++;
				maza_update_flag = 0;
				ir_L_flag = 0;
				ir_R_flag = 0;
			}
		}	
		
		if(maza_update_flag == 0){//�܂����ǂ̍X�V�����Ă��Ȃ����
			if(enc_now - ((long long)s1 * path_cnt ) > s1 - 250){//�}�X�̒��S�ł͂Ȃ�������O�ŕǂ��`�F�b�N���� �����F���ǃZ���T�[�������΂ߑO�������Ă��邽��
				maze_update(my_x + dx[my_angle],my_y + dy[my_angle],my_angle,2);//���H���̍X�V
				maza_update_flag = 1;
			}
		}else if(maza_update_flag == 1){//�܂��O�ǂ̍X�V�����Ă��Ȃ����
			if(enc_now - ((long long)s1 * path_cnt ) > s1 - 50){//�}�X�̒��S�ł͂Ȃ�������O�ŕǂ��`�F�b�N���� �����F���ǃZ���T�[�������΂ߑO�������Ă��邽��
				maze_update(my_x + dx[my_angle],my_y + dy[my_angle],my_angle,1);//���H���̍X�V
				maza_update_flag = 2;
			}
		}
		
		
		if(enc_now < (long long)path * s1 /4){// �i�񂾋��� < �ڕW���� * 1/4�@�� �������
			M_pwm = M_pwm_min + (enc_now / 6);	
			
		}else if(enc_now > (long long)path * s1 * 3/4){// �i�񂾋��� < �ڕW���� * 3/4 = //�������
			M_pwm = M_pwm_min + ( ((long long)path * s1 - enc_now) / 4);	
			
		}else{
			M_pwm = powor;
		}
			
		if(M_pwm > powor)M_pwm = powor;
		if(M_pwm < M_pwm_min)M_pwm = M_pwm_min;
		
		Smotor(M_pwm,true);
		
		
		//�ǐ؂�̋����␳
		ir_L_now = get_IR(IR_L);
		ir_R_now = get_IR(IR_R);
		if(path_cnt_save_L !=  path_cnt){//���݂̃}�X�ŕǐ؂ꏈ�������s���Ă��Ȃ����
		
			if(ir_L_flag == 0 && ir_L_now > 20 && ir_R_now < 70){
				ir_L_flag = 1;
				
			}else if(ir_L_flag == 1 && ir_L_now < 11 && ir_R_now < 70){
				if((enc_now % s1) < s1 * 2 / 3){//�}�X�̔�������O�ŕǐ؂ꂵ���ꍇ
				
					if(path_cnt == path_cnt_save_R){//������ɉE���ǐ؂�␳���Ă����ꍇ
/*						enc_base -= hosei_kyori_R;//�E�ł̕␳�𖳂��������Ƃɂ���
						enc_now = (get_encoder_total_L() + get_encoder_total_R())/2 - enc_base;
						
						hosei_kyori_L = (enc_now % s1) - kame_hosei;
						
						hosei_kyori_L = (hosei_kyori_L + hosei_kyori_R) / 2;//���E�̕��ϒl���g�p����
*/						
						//�ǐ؂�^�C�~���O�̈Ⴂ�Ŋp�x�␳
						enc_kabe_L = (get_encoder_total_L() + get_encoder_total_R())/2;
						if(abs( (enc_kabe_L - enc_kabe_R) ) < 150){
							GyroSum_add( (enc_kabe_L - enc_kabe_R) * 10);
						}
					}else{
//						hosei_kyori_L = (enc_now % s1) - kame_hosei;
						enc_kabe_L = (get_encoder_total_L() + get_encoder_total_R())/2;
					}
					
//					enc_base += hosei_kyori_L;
					
				}
				ir_L_flag = 0;
				path_cnt_save_L = path_cnt;
			}
		}
		
		if(path_cnt_save_R !=  path_cnt){//���݂̃}�X�ŕǐ؂ꏈ�������s���Ă��Ȃ����
			
			if(ir_R_flag == 0 && ir_R_now > 20 && ir_L_now < 70){
				ir_R_flag = 1;
				
			}else if(ir_R_flag == 1 && ir_R_now < 11 && ir_L_now < 70){
				if((enc_now % s1) < s1 * 2 / 3){//�}�X�̔�������O�ŕǐ؂ꂵ���ꍇ
				
					if(path_cnt == path_cnt_save_L){//�E����ɍ����ǐ؂�␳���Ă����ꍇ
/*						enc_base -= hosei_kyori_L;//���ł̕␳�𖳂��������Ƃɂ���
						enc_now = (get_encoder_total_L() + get_encoder_total_R())/2 - enc_base;
						hosei_kyori_R = (enc_now % s1) - kame_hosei;
						
						hosei_kyori_R = (hosei_kyori_L + hosei_kyori_R) / 2;//���E�̕��ϒl���g�p����
	*/					
						//�ǐ؂�^�C�~���O�̈Ⴂ�Ŋp�x�␳
						enc_kabe_R = (get_encoder_total_L() + get_encoder_total_R())/2;
						if(abs( (enc_kabe_L - enc_kabe_R) ) < 150){
							GyroSum_add( (enc_kabe_L - enc_kabe_R) * 10);
						}
					}else{
//						hosei_kyori_R = (enc_now % s1) - kame_hosei;
						enc_kabe_R = (get_encoder_total_L() + get_encoder_total_R())/2;
					}
					
					//enc_base += hosei_kyori_R;
					
				}
				ir_R_flag = 0;
				path_cnt_save_R = path_cnt;
			}
		}
		
		enc_now = (get_encoder_total_L() + get_encoder_total_R())/2 - enc_base;
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
      if((0 <= nx && nx < W) && (0 <= ny && ny < H) && ((maze_w[y][x] & (1<<i)) == 0 ) ){//���m��̏ꍇ�͕ǖ����Ƃ��čl����

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
  	short unknown_flag = 1; // 0:�m��ǁ@1:���m���
	
	int mas_cnt = 0;//�����ɐi�ރ}�X�̐�
	
  	while(x != target_x || y != target_y){
    	short num = maze_d[y][x][(angle+2)%4];
    	short n_num = 0;
    	char s_flag = 0;
    	short nx = x+dx[angle],ny = y+dy[angle];
    	if((0 <= nx && nx < W) && (0 <= ny && ny < H) && ((maze_w[y][x] & (1<<angle)) == 0 ) ){//�ڂ̑O�����H���@&& �ǂ��Ȃ� 
      		short next = maze_d[ny][nx][(angle+2)%4];
      		if(num == next+1){//�ڂ̑O�̃}�X���S�[���ɋ߂�
        		n_num = (angle+2)%4;
        		num = next;
        		s_flag = true;
      		}
    	}

    	if(s_flag == false){//��]����K�v����
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
    
    	n_num = (n_num+2)%4;// 0 ~ 4�@�i�݂������p
    	short ni = ((4 + n_num - ((4+angle-1)%4))%4) -1;// -1 ~ 2�@�}�V�����猩�����p

    	if((maze_w[y][x] & (1 << (4+n_num))) == 0){//���m��̕�
			//break;
			
			if( unknown_flag == 1){//���߂��疢�m��̒����̏ꍇ�͗L���@��x�ł��m��̃}�X��i�ނƖ���
				if(ni != 0)break;//���i�����łȂ���Αł��؂�//////////////////////////////////////////////////////////////////////////////////////////////////////////////
				ni = 10;//���m��̒���
			}else{
				break;//���m��̕ǂ��m�F����K�v������̂őł��؂�
			}
			
		}else{//�m��
			if(ni == 0){//����
				if(mas_cnt == 0){//�͂��߂Ă̒��i�̂P�}�X��
					ni = 10;//���i�����Ȃ疢�m��̒����̉\������ �����F�ڂ̑O�̕ǂ͊m�肵�Ă邩��
				}else{
					
				}
			}
		}
	
    	switch(ni){
      		case -1://L
        		if(s_path > 0){
          			enqueue(10 * unknown_flag );
					//enqueue(0);
          			enqueue(s_path);
          			s_path = 0;
					
					if(unknown_flag == 1)return;//���m��̒����̂��Ƃ̓��[�g���쐬���Ă͂����Ȃ�
        		}

        		enqueue(-1);
        		enqueue(1);

        		//s_path += 1;
        
        		angle = (4+angle-1)%4;
		       // x += dx[n_num];
		       // y += dy[n_num];
				
				//unknown_flag = 0;
		        break;
		    case 0://S
   
				if(unknown_flag == 1 && s_path > 0 && mas_cnt > 1){
          			enqueue(10);
          			enqueue(s_path);
          			s_path = 0;
					
					return;//���m��̒����̂��Ƃ̓��[�g���쐬���Ă͂����Ȃ�
        		}
				
		        s_path +=1;
		        x += dx[n_num];
		        y += dy[n_num];
				
				unknown_flag = 0;
				mas_cnt++;
		        break;
				
			case 10://S ���m��̒���
				
		        s_path +=1;
		        x += dx[n_num];
		        y += dy[n_num];
				
				mas_cnt++;
		        break;
				
      		case 1://R
		        if(s_path > 0){
			    	enqueue(10 * unknown_flag);
					//enqueue(0);
			        enqueue(s_path);
			        s_path = 0;
					
					if(unknown_flag == 1)return;//���m��̒����̂��Ƃ̓��[�g���쐬���Ă͂����Ȃ�
		        }
        
		        enqueue(1);
		        enqueue(1);

		       // s_path += 1;
		         
		        angle = (4+angle+1)%4;
		       // x += dx[n_num];
		       // y += dy[n_num];
				
				//unknown_flag = 0;
		        break;
      		case 2://B
        		if(s_path > 0){
          			enqueue(10 * unknown_flag);
					//enqueue(0);
          			enqueue(s_path);
          			s_path = 0;
					
					if(unknown_flag == 1)return;//���m��̒����̂��Ƃ̓��[�g���쐬���Ă͂����Ȃ�
        		}
        
        		enqueue(2);
        		enqueue(1);

				//s_path += 1;
				 
        		angle = (4+angle+2)%4;
				//x += dx[n_num];
		        //y += dy[n_num];
				
				//unknown_flag = 0;
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
	  	if(queue_empty()){
			if(path_num == 1){
	  			S_run(s1,20,false,true);
			}else{
				S_run(s1 * (long long)path_num,30,false,true);
			}
			
		}else{
        	if(path_num == 1){
	  			//S_run(s1,18,false,true);
				
				if(queue_next() < 0){//���@��
	          	  S_run_kabe(20,true,1);
				
			  	}else if(queue_next() > 0){//���@�E
				  S_run_kabe(20,true,2);
			  	}else{
				  S_run_kabe(20,true,3);
			  	}
				
				S_run(h1,20,false,true);
				
			}else{
				S_run(s1 * ((long long)path_num - 1),30,true,true);
				
				if(queue_next() < 0){//���@��
	          	  S_run_kabe(20,true,1);
				
			  	}else if(queue_next() > 0){//���@�E
				  S_run_kabe(20,true,2);
			  	}else{
				  S_run_kabe(20,true,3);
			  	}
				
				S_run(h1,20,false,true);
			}
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
		
	  case 10://S ���m��̒���
        if(path_num == 1){
	  		
			S_run_maze_search(path_num,18);
			
		}else{
			S_run_maze_search(path_num,18);
		}
		
        break;
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
	int cnt = 0;
	
	GyroSum_reset();
	Encoder_reset();

	led_down();
	
	while(1){
    	maze_update(my_x,my_y,my_angle,3);
    	/*if((target_x != Goal_x || target_y != Goal_y) && (target_x != Start_x || target_y != Start_y)){//�X�^�[�g�n�_�A�S�[���n�_�ȊO���ڕW�n�_�̂Ƃ�
      		if((maze_w[target_y][target_x] & 0xf0) == 0xf0)break;    //�ڕW�n�_�̕ǂ����ׂĊm�肵����T������  
    	}*/
		
    	if(target_x == my_x && target_y == my_y){//�S�[��
			led_up();
			
      		if(target_x == Start_x && target_y == Start_y){
				
				while(1){//�X�^�[�g�̉��܂Ői��
	      			if(get_IR(IR_FR) > 60){
	        			Smotor(-7,false);

	        			cnt = 0;
	      			}else if(get_IR(IR_FR) < 55){
	       	 			Smotor(+7,false);
	       				
	        			cnt = 0;
	      			}else {
	        			motor(0,0);
	        			cnt++;
	      			}
	      			if(cnt > 2000)break;
				}
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
/* �� �� �T �v�F�ŒZ�o�H�쐬	���m��}�X�̎�O�Ŏ~�܂�					  			            */
/* �� �� �� �ׁF												                                   */
/* ��       ���F �ړI�n��XY���W															    */
/* ��  ��   �l�F �Ȃ�										    									*/
/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */ 
void make_shortest_path_list_simple(short target_x,short target_y){
	queue_reset();
  	short s_path = 0;
 	short x = my_x,y = my_y,angle = my_angle;
	
	
  	while(x != target_x || y != target_y){
    	short num = maze_d[y][x][(angle+2)%4];
    	short n_num = 0;
    	char s_flag = 0;
    	short nx = x+dx[angle],ny = y+dy[angle];
    	if((0 <= nx && nx < W) && (0 <= ny && ny < H) && ((maze_w[y][x] & (1<<angle)) == 0 ) ){//�ڂ̑O�����H���@&& �ǂ��Ȃ� 
      		short next = maze_d[ny][nx][(angle+2)%4];
      		if(num == next+1){//�ڂ̑O�̃}�X���S�[���ɋ߂�
        		n_num = (angle+2)%4;
        		num = next;
        		s_flag = true;
      		}
    	}

    	if(s_flag == false){//��]����K�v����
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
    
    	n_num = (n_num+2)%4;// 0 ~ 4�@�i�݂������p
    	short ni = ((4 + n_num - ((4+angle-1)%4))%4) -1;// -1 ~ 2�@�}�V�����猩�����p

    	if((maze_w[y][x] & (1 << (4+n_num))) == 0){//���m��̕�
			break;
		}
	
    	switch(ni){
      		case -1://L
        		if(s_path > 0){
          			enqueue(0);
          			enqueue(s_path);
          			s_path = 0;		
        		}

        		enqueue(-1);
        		enqueue(1);
        
        		angle = (4+angle-1)%4;
		     
		        break;
		    case 0://S
				
		        s_path +=1;
		        x += dx[n_num];
		        y += dy[n_num];
				
		        break;
				
      		case 1://R
		        if(s_path > 0){
			    	enqueue(0);
			        enqueue(s_path);
			        s_path = 0;
		        }
        
		        enqueue(1);
		        enqueue(1);
		         
		        angle = (4+angle+1)%4;
		     
		        break;
      		case 2://B
        		if(s_path > 0){
          			enqueue(0);
          			enqueue(s_path);
          			s_path = 0;
        		}
        
        		enqueue(2);
        		enqueue(1);
				 
        		angle = (4+angle+2)%4;
        		break;
    	}
  	}
 
  	if(s_path > 0){
    	enqueue(0);
    	enqueue(s_path);
    	s_path = 0;
  	}
}
/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */
/* �� �� �T �v�F�ŒZ�o�H��̖��m��}�X��T��											            */
/* �� �� �� �ׁF												                                   */
/* ��       ���F 																				    */
/* ��  ��   �l�F �Ȃ�										    									*/
/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */ 
void maze_search_unknown(short* target_x,short* target_y){
 
 	short x = Start_x,y = Start_y,angle = Start_angle;
	
	
  	while(x != Goal_x || y != Goal_y){
    	short num = maze_d[y][x][(angle+2)%4];
    	short n_num = 0;
    	char s_flag = 0;
    	short nx = x+dx[angle],ny = y+dy[angle];
    	if((0 <= nx && nx < W) && (0 <= ny && ny < H) && ((maze_w[y][x] & (1<<angle)) == 0 ) ){//�ڂ̑O�����H���@&& �ǂ��Ȃ� 
      		short next = maze_d[ny][nx][(angle+2)%4];
      		if(num == next+1){//�ڂ̑O�̃}�X���S�[���ɋ߂�
        		n_num = (angle+2)%4;
        		num = next;
        		s_flag = true;
      		}
    	}

    	if(s_flag == false){//��]����K�v����
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
    
    	n_num = (n_num+2)%4;// 0 ~ 4�@�i�݂������p
    	short ni = ((4 + n_num - ((4+angle-1)%4))%4) -1;// -1 ~ 2�@�}�V�����猩�����p

    	if((maze_w[y][x] & (1 << (4+n_num))) == 0){//���m��̕�
			*target_x = x;
			*target_y = y;
			return;
			
		}
	
    	switch(ni){
      		case -1://L
        		angle = (4+angle-1)%4;
				
		        break;
		    case 0://S
   
		        x += dx[n_num];
		        y += dy[n_num];
	
		        break;
				
      		case 1://R
		        angle = (4+angle+1)%4;
			
		        break;
      		case 2://B �s�v�̂͂�
        	
        		angle = (4+angle+2)%4;
        		break;
    	}
  	}
	
	//���m��}�X��ʂ炸�ɃS�[���܂Ōo�H���m�F�ł����B
	*target_x = Goal_x;
	*target_y = Goal_y;
}
/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */
/* �� �� �T �v�F�ŒZ�o�H�͂��ׂĊm��}�X�ɂ���  										            */
/* �� �� �� �ׁF												                                   */
/* ��       ���F 																				    */
/* ��  ��   �l�F �Ȃ�										    									*/
/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */ 
void maze_search_all(){
	GyroSum_reset();
	Encoder_reset();

	led_down();
	
	short target_x,target_y;
	
	time_limit = 60000;//60�b
	
	while(time_limit > 0){//�������Ԃ̊ԑ��s�\
	
		maze_update(my_x,my_y,my_angle,3);
		
		shortest_path_search(Goal_x,Goal_y);
		maze_search_unknown(&target_x,&target_y);//�ŒZ�o�H��̖��m��}�X�̍��W���擾
		
    	shortest_path_search(target_x,target_y);
    	make_shortest_path_list_simple(target_x,target_y);
		run_shortest_path();
  	
		if(my_x == Goal_x && my_y == Goal_y){//�ŒZ�o�H��ɖ��m��}�X���Ȃ���ΏI��
			led_down();
			led_up();
			break;
		}
  	}
	
	if(time_limit <= 0){//�@�������ԓ��ɒT���ł��Ȃ������@�S�[���܂Ō�����
		maze_search_adachi(Goal_x,Goal_y);
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
      if((0 <= nx && nx < W) && (0 <= ny && ny < H) && ((maze_w[y][x] & (1<<i)) == 0 )  && ((maze_w[y][x] & (1<<(4+i))) != 0 )  ){//���m��̕ǂ͒ʉ߂��Ȃ�

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
 
  //run_list
  queue_reset();
  short h_path = 0;
  my_x = Start_x;my_y = Start_y;my_angle = Start_angle;
 
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
  enqueue(99);//�ڈ�
  enqueue(99);
  int naname_cnt = 0,lr = 0,first_lr = 0;
  
  while(1){
    short mode = dequeue(),num = dequeue();
    if(mode == 99)break;

    switch(mode){
      case -1://L
	       if(naname_cnt == 0){
	          naname_cnt = 1;
	          lr = -1;
			  first_lr = -1;
	        }else if(naname_cnt == 1 && lr == -1 && queue_next() == 0){//U�^�[��
				enqueue(-1);
	            enqueue(1);
				
				enqueue(-1);
	            enqueue(1);
				
				naname_cnt = 0;
	          	lr = 0;
			}else if(naname_cnt == 1 && lr == -1 && queue_next() == 1){//L L R //�P�ڂ̓X�����[���ɂ���
				enqueue(-1);
	            enqueue(1);
				
			}else if(naname_cnt == 1 && lr == 1 && queue_next() != 1){//R L * //�X�����[���ɂ���
				enqueue(1);
	            enqueue(1);
				
				enqueue(-1);
	            enqueue(1);
				
				naname_cnt = 0;
	          	lr = 0;
				
			}else{
	          if(lr == 1){
	            lr = -1;
	            naname_cnt++;
	          }else{//�΂ߒ���V�^�[��
				if(first_lr == -1){
					enqueue(-11);
	         		enqueue(1);
				}else if(first_lr == 1){
					enqueue(11);
	         		enqueue(1);
				}
				first_lr = 0;
				
	            enqueue(10);
	            enqueue(naname_cnt);
	           
			    enqueue(-11);
	            enqueue(1);
				
				if(queue_next() == 0){//L L (S) //�X�����[���ɂ���
					naname_cnt = 0;
					enqueue(-1);
	           		enqueue(1);
				}else{
	            	naname_cnt = 1;
					lr = -1;
					first_lr = -1;
				}
	          }
	        }
	        break;
       case 0://S
		    if(naname_cnt == 1){//�΂߂�1�}�X�̂Ƃ��̓X�����[���ɂ���
				if(lr == 1){
					enqueue(1);
	            	enqueue(1);
				}else{
					enqueue(-1);
	            	enqueue(1);
				}
				naname_cnt = 0;
	            lr = 0;
				
/*			}else if(naname_cnt == 2){//�΂߂�2�}�X�̂Ƃ��̓X�����[���ɂ���
				if(first_lr == 1){
					enqueue(1);
	            	enqueue(1);
					
					enqueue(-1);
	            	enqueue(1);
				}else{
					enqueue(-1);
	            	enqueue(1);
					
					enqueue(1);
	            	enqueue(1);
				}
				naname_cnt = 0;
	            lr = 0;
*/				
			}else if(naname_cnt > 2){
			  if(first_lr == 1){
				enqueue(11);
	         	enqueue(1);
			  }else if(first_lr == -1){
				enqueue(-11);
	         	enqueue(1);
			  }
			  first_lr = 0;
			  
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
	          naname_cnt = 1;
	          lr = 1;
			  first_lr = 1;
	        }else if(naname_cnt == 1 && lr == 1 && queue_next() == 0){//U�^�[��
				enqueue(1);
	            enqueue(1);
				
				enqueue(1);
	            enqueue(1);
				
				naname_cnt = 0;
	          	lr = 0;
			}else if(naname_cnt == 1 && lr == 1 && queue_next() == -1){//R R L //�P�ڂ̓X�����[���ɂ���
				enqueue(1);
	            enqueue(1);
				
			}else if(naname_cnt == 1 && lr == -1 && queue_next() != -1){//L R * //�X�����[���ɂ���
				enqueue(-1);
	            enqueue(1);
				
				enqueue(1);
	            enqueue(1);
				
				naname_cnt = 0;
	          	lr = 0;
				
			}else{
	          if(lr == -1){
	            lr = 1;
	            naname_cnt++;
	          }else{//�΂ߒ���V�^�[��
				if(first_lr == 1){
					enqueue(11);
	         		enqueue(1);
				}else if(first_lr == -1){
					enqueue(-11);
	         		enqueue(1);
				}
				first_lr = 0;
				
	            enqueue(10);
	            enqueue(naname_cnt);
				
				enqueue(11);
	            enqueue(1);
				
				if(queue_next() == 0){//R R (S) //�X�����[���ɂ���
					naname_cnt = 0;
					enqueue(1);
	           		enqueue(1);
				}else{
	            	naname_cnt = 1;
					lr = 1;
					first_lr = 1;
				}
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
  int comand ,path_num;
  int first_flag = 0; //0:�܂����s���ĂȂ� 1:���s��
  int cnt = 0;
  
  int over_run = -200;//���x�グ��ƃI�[�o�[�������݂Ȃ̂ŏ�����O�Ŏ~�߂�

   
  /*
  while(!queue_empty()){//debug
    comand = dequeue();path_num = dequeue();
	printf2("%d %d\n",comand,path_num);
	delay(1);
  }
  while(1);
 */ 
  while(!queue_empty()){
    comand = dequeue();path_num = dequeue();
    switch(comand){
      case -1://L
        L_curve(sl90,true);
  
		if(queue_next() == -1){//�^�[��
			ESmotor(20,40,true,true);//�����A�X�s�[�h
			
		}else if(queue_next() == 1){//�^�[��
			ESmotor(50,40,true,true);//�����A�X�s�[�h
		}
		
        break;
      case -11://L45
	  	
		if(queue_next() == -11){//V�^�[��
			 L_rotate_naname(l45 * path_num * 0.85);
			 
		}else if(queue_next() == -1){//45�����90�^�[��
			L_rotate_naname(l45 * path_num);
			
			ESmotor(30,40,true,true);//�����A�X�s�[�h
		
		}else{
			 L_rotate_naname(l45 * path_num);
		}
        break;
      case 0://S
        if(queue_empty()){
			
			S_run(h1 * (long long)path_num ,55 + run_fin_speed_offset,false,true);
			
			while(1){//�S�[���̉��܂Ői��
      			if(get_IR(IR_FR) > 60){
        			Smotor(-7,false);

        			cnt = 0;
      			}else if(get_IR(IR_FR) < 55){
       	 			Smotor(+7,false);
       				
        			cnt = 0;
      			}else {
        			motor(0,0);
        			cnt++;
      			}
      			if(cnt > 2000)break;
			}
		}else {
          path_num--;
          if(path_num > 0){
			  if(first_flag == 0)S_run((h1 *(long long) path_num) - over_run ,55 + run_fin_speed_offset,3,true); // memo : non_stop = 3 �����͂������@�����͂����Ȃ�
		  	  else S_run((h1 * (long long)path_num)  - over_run ,55 + run_fin_speed_offset,true,true);
			  
			  if(queue_next() == -11 || queue_next() == 11){//�������45�^�[��
				  if(queue_next() < 0){//���@��
			      	  S_run_kabe2(35 + run_fin_speed_offset,true,1);
						
				  }else if(queue_next() > 0){//���@�E
				  	  S_run_kabe2(35 + run_fin_speed_offset,true,2);
				  }else{
					  S_run_kabe2(35 + run_fin_speed_offset,true,3);
				  }
			  }else{
				   if(queue_next() < 0){//���@��
			      	  S_run_kabe(50 + run_fin_speed_offset,true,1);
						
				  }else if(queue_next() > 0){//���@�E
				  	  S_run_kabe(50 + run_fin_speed_offset,true,2);
				  }else{
					  S_run_kabe(50 + run_fin_speed_offset,true,3);
				  }
				  
			  }
			  
			  
		  }else{//���}�X�������X�^�[�g����ɕǐ؂�@
			  if(queue_next() < 0){//���@��
	          	  S_run_kabe(30 + run_fin_speed_offset,false,1);
				
			  }else if(queue_next() > 0){//���@�E
				  S_run_kabe(30 + run_fin_speed_offset,false,2);
			  }else{
				  S_run_kabe(30 + run_fin_speed_offset,false,3);
			  }
		  }
        }

        //my_x = nx;
        //my_y = ny;
        break;
      case 10://Snaname
	  	path_num-=2;
		 
	    if(path_num <= 0){
			//���݂��Ȃ��͂�
		}else{
			S_run(s45 * (long long)path_num + 300,50 + run_fin_speed_offset,true,3); // w_flag = 3 �΂߂̕Ǖ␳����
		}
		
		
		if(queue_next() < 0){//���@��
        	S_run_kabe_naname(45 + run_fin_speed_offset,3,1);
			
		}else if(queue_next() > 0){//���@�E
			S_run_kabe_naname(45 + run_fin_speed_offset,3,2);
		}else{
			S_run_kabe_naname(45 + run_fin_speed_offset,3,3);
		}
		
        //my_x = nx;
        //my_y = ny;
        break;
      case 1://R
        R_curve(sr90,true);
		
		if(queue_next() == -1){//�^�[��
			ESmotor(50,40,true,true);//�����A�X�s�[�h
			
		}else if(queue_next() == 1){//�^�[��
			ESmotor(20,40,true,true);//�����A�X�s�[�h
		}
        break;
      case 11://R45
        
  		if(queue_next() == 11){//V�^�[��
			R_rotate_naname(r45 * path_num * 0.85);
			
		}else if(queue_next() == 1){//45�����90�^�[��
			R_rotate_naname(r45 * path_num);
			
			ESmotor(30,40,true,true);//�����A�X�s�[�h
			
		}else{
			R_rotate_naname(r45 * path_num);
		}
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
	static char log_flag = 0;
	
	task ++;                         // �^�X�N�̍X�V						
  	if (log_start != 2 && task >= 30) task = 0;       
	if (log_start == 2 && task >= 10) task = 0;       
	
	Gyro_update();
		
	if(ir_flag == 1){//�ԊO���L����=���s���ɃW���C���ɂ����S��~�`�F�b�N���s��
		if(abs(Gyro()) > 300){
    		motor_stop_cnt++;
    		if(motor_stop_cnt > 20)motor_stop();
  		}else motor_stop_cnt = 0;
	}
	
	if(time_limit > 0){
		time_limit--;	
	}
	
	switch(task) {                         			
	case 0:
	case 10:
	case 20:
		encoder_update();
        break;
   
   	case 1:
   		if(log_start != 0){
			log_flag = 1;
				
			
			if((LOG_BUF_MAX * log_save_cnt) + log_cnt +8 <= LOG_MAX){
				log_buff[log_cnt++] = my_x;
				log_buff[log_cnt++] = my_y;
				
				log_buff[log_cnt++] = (char)get_IR(IR_FL);
				log_buff[log_cnt++] = (char)get_IR(IR_L);
				log_buff[log_cnt++] = (char)get_IR(IR_R);
				log_buff[log_cnt++] = (char)get_IR(IR_FR);
				
				log_buff[log_cnt++] = get_pwm_buff_L();
				log_buff[log_cnt++] = get_pwm_buff_R();
				
				if(log_cnt >= LOG_BUF_MAX-2){//���܂�����ۑ�����
					log_save();
					log_save_cnt++;
					log_cnt = 0;
				}	
				
			}else{
				if(log_block_num < 15){
					log_block_num++;
					
					log_cnt = 0;
					log_save_cnt = 0;
	
				}else{
					log_start = 0;//�T�C�Y�I�[�o�[�Ȃ̂ŋL�^�I��
				}
			}
		}else{
			if(log_flag == 1){//���O�̕ۑ��I��
				while(log_cnt < LOG_BUF_MAX){
					log_buff[log_cnt++] = 0;
				}
				log_save();
				log_save_cnt++;
				log_cnt = 0;
			}
		}
        break;
		
	default:
		break;
   	}
}

/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */
/* �� �� �T �v�FCMT2���荞�݃��W���[��                                                              */
/* �� �� �� �ׁF                                                                                    */
/* ��       ���F�Ȃ�										    */
/* ��  ��   �l�F�Ȃ�										    */
/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */
#pragma interrupt (Excep_CMT2_CMI2(vect=30))
void Excep_CMT2_CMI2(){
	
	if(ir_flag == 1){
		ir_update();
	}else{
		ir(0);
	}
}




 



