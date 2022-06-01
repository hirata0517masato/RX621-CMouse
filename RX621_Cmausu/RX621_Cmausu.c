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
#include "wait.h"
#include "Gyro.h"
#include "DataFlash.h"
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
void MTU0_init(void);
void MTU1_init(void);


void motor(float, float);
void AD_update( void );
void led(char);
void ir(char);
int get_sw(void);
void ir_update(void);


/* �萔�ݒ� */
#define	L1_PWM	PORT3.DR.BIT.B4       // CN2-4 : L1���[�^�[PWM�o�� 
#define	L2_PWM	PORT2.DR.BIT.B0       // CN2-16: L2���[�^�[PWM�o�� 

#define R1_PWM	PORT1.DR.BIT.B4       // CN2-10: R1���[�^�[PWM�o��
#define R2_PWM	PORT2.DR.BIT.B4       // CN2-12: R2���[�^�[PWM�o��

#define AD_0	S12AD.ADDR0	     // CN3-9 : AN0(0�`4069)
#define AD_1	S12AD.ADDR1	     // CN3-10: AN1(0�`4069)
#define AD_2	S12AD.ADDR2	     // CN3-11 :AN2(0�`4069)
#define AD_3	S12AD.ADDR3	     // CN3-12 :AN3(0�`4069)
#define AD_4	S12AD.ADDR4	     // AN4(0�`4069)

#define Lenc	MTU7.TCNT	     //A:CN4-15 B:CN4-16
#define Renc	MTU8.TCNT	     //A:CN4-13 B:CN4-14

#define	LED_HIGH	PORTD.DR.BIT.B0 = 1	/* CN3-17 */
#define	LED_LOW		PORTD.DR.BIT.B0 = 0	/* CN3-17 */

//�O���[�o���ϐ�
int s1,s2,s3,s4;


/***********************************************************************/
/* ���C���v���O����                                                    */
/***********************************************************************/
void main(void)
{
	//unsigned char c,buf[256];
	volatile int i = 0;
 	float l=0,r=0;
	int aa;
	
	uint8_t prog_buff[512] = "hello world RX621";
	uint8_t read_buff[512];
	

	ALL_init();//������
	

	/*
	while(1){
		printf2("input: ");
		scanf2("%d",&aa);
		printf2("%d\n",aa);
	}*/
	
	/*
	DataFlash_read(1,read_buff,sizeof(read_buff));
	
	 // Verify 
    for (i = 0; i < sizeof(read_buff); i++){
    	if (read_buff[i] != prog_buff[i]){
       		printf2("NG1\n");
			break;
        }
    }
	
	DataFlash_write(1,prog_buff,sizeof(prog_buff));
	
	DataFlash_read(1,read_buff,sizeof(read_buff));
	
    // Verify 
    for (i = 0; i < sizeof(read_buff); i++){
    	if (read_buff[i] != prog_buff[i]){
        	printf2("NG_2\n");
            break;
        }
    }
	*/
	
	printf2("FIN\n");
//	while(1){
//		 nop();
//	}
	

	
	/*
	i = 1;
	while(1){
		led(i);
		
		i <<= 1;
		if(i > 0x0f)i = 1;
		
		delay(250);
	}
	*/
/*
	l = -100;
	r = 100;
	while(1){
		
		printf2("L:%f \tR:%f\n",l,r);
		motor(l,r);
		
		l += 0.1;
		if( l > 100.0)l = -100;
		r = - l;
		
		//for(i = 0; i < 500; i++);
		
	}
	*/
	

	
	while(1){
		//delay(1000);

		printf2("%ld\n",GyroSum_get());
		
		if(get_sw() == 1)GyroSum_reset();
	}
	

/*
	while(1){
		//motor(10,10);
		printf2("%d\t%d\n",Lenc,Renc);	
		delay(250);
	}
*/



	while(1){
		delay(1);
		printf2("%d\t%d\t%d\t%d\n",s1,s2,s3,s4);
		
		int p = 0;
		
		if(s1 > 30)p += 1;
		if(s2 > 30)p += 2;
		if(s3 > 30)p += 4;
		if(s4 > 30)p += 8;
		
		led(p);
			
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
	
	Gyro_init();	//�W���C���ASPI�̏����� �@���ӁF�������Ԃ�����܂� �������̓W���C���Z���T�[�𓮂����Ȃ�����

	CMT_init();  // CMT0�̏�����

	initSCI1(SPEED_9600);
	//USB_init();  //USB CDC�̏�����
	
	led(0);
	ir(0);
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
/* �� �� �T �v�FA/D�l�̍X�V                                                                         */
/* �� �� �� �ׁFS12AD.ADANS.WORD �Őݒ肳�ꂽ�|�[�g��AD�ϊ�����                                     */
/* ��       ���F�Ȃ�										    */
/* ��  ��   �l�F�Ȃ�										    */
/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */
void AD_update(void)
{	
	S12AD.ADCSR.BIT.ADST = 1;  	// A/D�ϊ��J�n
   	while(S12AD.ADCSR.BIT.ADST == 0); // ���肪�I������܂ő҂�
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
}
		
/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */
/* �� �� �T �v�F���[�^�[��PWM�ݒ�											  			            */
/* �� �� �� �ׁF												                                   */
/* ��       ���F�����[�^�[PWM�A�E���[�^�[PWM	�i-100.0����+100.0�j 							    */
/* ��  ��   �l�F�Ȃ�										    									*/
/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */ 
void motor(float duty_L, float duty_R){
	
    unsigned int dt_L, dt_R;
	int L_PM = 0,R_PM = 0; // 0:�v���X 1:�}�C�i�X
	
    if(duty_L >  100.0) duty_L =  100.0; // duty_L��100�ȏ�ɂ��Ȃ�
    if(duty_L < -100.0) duty_L = -100.0; // duty_L��-100�ȉ��ɂ��Ȃ�
    if(duty_R >  100.0) duty_R =  100.0;
    if(duty_R < -100.0) duty_R = -100.0;
	
	
	if(duty_L < 0.0){
		L_PM = 1;
		duty_L = -duty_L;
	}
	
	/* �f���[�e�B��̎Z�o */
   	dt_L = MTU1.TGRA * duty_L / 100.0;//  dt_L = 0.9445*50/100 = 0.5
		 
	/* �f���[�e�B��̃I�[�o�[�t���[�ی� */
    if(dt_L >= MTU1.TGRA)   dt_L = MTU1.TGRA - 1;  // 
		
	if(L_PM == 0){
		 /* �f���[�e�B��̐ݒ� */
   		 MTU0.TGRB = dt_L;
   		 MTU1.TGRB = 0;
	}else{
		/* �f���[�e�B��̐ݒ� */
   		 MTU0.TGRB = 0;
   		 MTU1.TGRB = dt_L;
	}
	
	if(duty_R < 0.0){
		R_PM = 1;
		duty_R = -duty_R;
	}
	
	/* �f���[�e�B��̎Z�o */
   	dt_R = MTU3.TGRA * duty_R / 100.0;//  dt_R = 0.9445*50/100 = 0.5
		 
	/* �f���[�e�B��̃I�[�o�[�t���[�ی� */
    if(dt_R >= MTU3.TGRA)   dt_R = MTU3.TGRA - 1;  // 
		
	if(R_PM == 0){
		 /* �f���[�e�B��̐ݒ� */
   		 MTU3.TGRB = dt_R;
   		 MTU4.TGRB = 0;
	}else{
		/* �f���[�e�B��̐ݒ� */
   		 MTU3.TGRB = 0;
   		 MTU4.TGRB = dt_R;
	}
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

/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */
/* �� �� �T �v�F�ԊO��LED�o��												  			            */
/* �� �� �� �ׁF												                                   */
/* ��       ���F 	3:LED4 2:LED3 1:LED2 0:LED1													    */
/* ��  ��   �l�F�Ȃ�										    									*/
/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */ 
void ir(char n){
	
	if((n & 0x01) > 0)PORTD.DR.BIT.B0 = 1;
	else PORTD.DR.BIT.B0 = 0;
	
	if((n & 0x02) > 0)PORTD.DR.BIT.B1 = 1;
	else PORTD.DR.BIT.B1 = 0;
	
	if((n & 0x04) > 0)PORTD.DR.BIT.B2 = 1;
	else PORTD.DR.BIT.B2 = 0;
	
	if((n & 0x08) > 0)PORTD.DR.BIT.B3 = 1;
	else PORTD.DR.BIT.B3 = 0;
}

/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */
/* �� �� �T �v�F�ԊO���Z���T�[�l�X�V										  			            */
/* �� �� �� �ׁF												                                   */
/* ��       ���F �Ȃ�																			    */
/* ��  ��   �l�F�Ȃ�										    									*/
/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */ 
void ir_update(){
	
	static int num = 0;
	static int s[4] = {0,0,0,0};
	
	AD_update();
	
	
	switch(num){
		case 0:	//front
			s1 = AD_1 - s[0];
			s4 = AD_4 - s[3];
			
			s[1] = AD_2;
			s[2] = AD_3;
			
			ir(6);//0110
			break;
			
		case 1://side
			s2 = AD_2 - s[1];
			s3 = AD_3 - s[2];
			
			s[0] = AD_1;
			s[3] = AD_4;
			
			ir(9);// 1001
			break;
		
	}
	
	num++;
	if(num > 1)num = 0;
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
  	if (task == 10) task = 0;             // 2�܂ŗ�����0�ɃN���A
	
	Gyro_update();
	
	ir_update();
	
	
	switch(task) {                         // �^�X�N�|�C���^�ɏ]���ď������s��			
	case 0:
        	break;
   
   	case 1:
   		
        	break;
		
	default:
		break;
   	}
}





 



