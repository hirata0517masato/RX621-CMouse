/****************************************************************************************************/
/* プロジェクト名：RX621_SAMPLE     	                　　　　　          			    */
/* モジュール名：  			                     					    */
/* 履    歴    ：										    */
/* 使用マイコン：RX621 R5F56218BDFP (Flash ROM:512KB, SRAM:64KB, DATA Flash 32KB)                   */
/* 作成者      ：hirata0517masato               				                    */
/* バージョン  ：0.00                                                                               */
/* 作成日      ：2022/02/06 									    */
/****************************************************************************************************/                  
#include "iodefine.h"
#include <machine.h>
#include "wait.h"
#include "Gyro.h"
#include "DataFlash.h"
//#include "usb.h"

#include "printf_lib.h"   /* printf2 関連処理    コンパイルおよびライブラリジェネレートオプションにてC99対応が必要  */

#define PRINT /* 使用時は有効化すること*/

#ifndef PRINT
	#define printf2(...)  
	//#define scanf2(...) 　
#endif /* PRINT*/

/*======================================*/
/* プロトタイプ宣言                     */
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


/* 定数設定 */
#define	L1_PWM	PORT3.DR.BIT.B4       // CN2-4 : L1モーターPWM出力 
#define	L2_PWM	PORT2.DR.BIT.B0       // CN2-16: L2モーターPWM出力 

#define R1_PWM	PORT1.DR.BIT.B4       // CN2-10: R1モーターPWM出力
#define R2_PWM	PORT2.DR.BIT.B4       // CN2-12: R2モーターPWM出力

#define AD_0	S12AD.ADDR0	     // CN3-9 : AN0(0～4069)
#define AD_1	S12AD.ADDR1	     // CN3-10: AN1(0～4069)
#define AD_2	S12AD.ADDR2	     // CN3-11 :AN2(0～4069)
#define AD_3	S12AD.ADDR3	     // CN3-12 :AN3(0～4069)
#define AD_4	S12AD.ADDR4	     // AN4(0～4069)

#define Lenc	MTU7.TCNT	     //A:CN4-15 B:CN4-16
#define Renc	MTU8.TCNT	     //A:CN4-13 B:CN4-14

#define	LED_HIGH	PORTD.DR.BIT.B0 = 1	/* CN3-17 */
#define	LED_LOW		PORTD.DR.BIT.B0 = 0	/* CN3-17 */

//グローバル変数
int s1,s2,s3,s4;


/***********************************************************************/
/* メインプログラム                                                    */
/***********************************************************************/
void main(void)
{
	//unsigned char c,buf[256];
	volatile int i = 0;
 	float l=0,r=0;
	int aa;
	
	uint8_t prog_buff[512] = "hello world RX621";
	uint8_t read_buff[512];
	

	ALL_init();//初期化
	

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
/* 関 数 概 要：初期化    	                                                                    */
/* 関 数 詳 細：各種初期化関数のまとめ 　　　　　　　　　　　　　　                                 */
/* 引       数：なし										    */
/* 戻  り   値：なし										    */
/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */ 
void ALL_init(){
	// マイコン機能の初期化 
	CLK_init();  // クロックの初期化
 	IO_init();   // IOの初期化
	WAIT_init(); //wait(CMT1)の初期化
 	AD_init();   // A/Dの初期化
	MTU0_init();  //モーターの初期化
	MTU1_init();  //エンコーダの初期化
	DataFlash_init();//データフラッシュの初期化
	
	Gyro_init();	//ジャイロ、SPIの初期化 　注意：少し時間かかります 処理中はジャイロセンサーを動かさないこと

	CMT_init();  // CMT0の初期化

	initSCI1(SPEED_9600);
	//USB_init();  //USB CDCの初期化
	
	led(0);
	ir(0);
}

/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */
/* 関 数 概 要：クロックの初期化                                                                    */
/* 関 数 詳 細：システムクロック96MHz,周辺モジュールクロック24MHz                                   */
/* 引       数：なし										    */
/* 戻  り   値：なし										    */
/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */ 
void CLK_init(void)
{
   SYSTEM.SCKCR.BIT.ICK = 0;		// システムクロック(ICLK)       EXTAL×8 (96MHz)
   SYSTEM.SCKCR.BIT.PCK = 2;		// 周辺モジュールクロック(PCLK)	EXTAL×2 (24MHz)
     
   //SYSTEM.SUBOSCCR = 1;              // 0：サブクロック発振器動作（0：デフォルトで動作 1:停止）
   //RTC.RCR2.BIT.RTCOE = 1;           // 1：RTCOUTを端子(P32)から出力する
   
}

/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */
/* 関 数 概 要：I/Oの初期化                                                                    　　 */
/* 関 数 詳 細：                                                                                    */
/* 引       数：なし										    */
/* 戻  り   値：なし										    */
/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */ 
void IO_init(void)
{
 	PORT1.DDR.BYTE = 0xff;           // 4:PWM_R1 3:MTIOC0B
	
    PORT2.DDR.BYTE = 0xff;           // 6:TxD1	4:PWM_R2	2:MTIOC3B	1:MTIOC1B	0:PWM_L2

    PORT3.DDR.BYTE = 0x10;           // 4:PWM_L1	0:RxD1
	
    PORT4.DDR.BYTE = 0x00;           // 7:AN7	6:AN6	5:AN5	4:AN4	3:AN3	2:AN2	1:AN1	0:AN0  
	
	PORT5.DDR.BYTE = 0x00;			 //4:MTIOC4B-B
	
	PORTA.DDR.BYTE = 0x0f; 			 //4:SW	3:LED4	2:LED3	1:LED2	0:LED1
	 
    PORTB.DDR.BYTE = 0x00; 	     	 // 5:左エンコーダB		4:左エンコーダA		3:右エンコーダB		2:右エンコーダA
	
	//PORTC.DDR.BYTE = 0xff;           // PCを出力に設定
	//  PORTC.PCR.BYTE   = 0x03;        // PC0,1をプルアップ指定
	
    PORTD.DDR.BYTE = 0xff;           // 3:IR4	2:IR3	1:IR2	0:IR1
	
    PORTE.DDR.BYTE = 0x70; 	     	 // 7:SPI_SDO	6:SPI_SDI	5:SPI_SCL	4:SPI_CS    (6,7のIOはジャイロ目線)
    PORTE.PCR.BYTE = 0x70;           // PE7をプルアップ指定
}



/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */
/* 関 数 概 要：A/Dの初期化                                                                         */
/* 関 数 詳 細：                                                                                    */
/* 引       数：なし										    */
/* 戻  り   値：なし										    */
/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */ 
void AD_init(void)
{
   //A/Dの初期化
   SYSTEM.MSTPCRA.BIT.MSTPA17 = 0; //12bitでA/D変換する
   
   //MSTP(S12AD) = 0;//モジュールストップ状態を解除

   S12AD.ADCSR.BYTE = 0x0c;//動作モード、変換開始要件、クロックの設定
   S12AD.ADANS.WORD = 0x001f;//スキャン変換端子の設定 AN0-7の全て使用する場合は0xff
   S12AD.ADSTRGR.BYTE = 0x0000;//A/D 変換開始要件の設定
	
}

/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */
/* 関 数 概 要：A/D値の更新                                                                         */
/* 関 数 詳 細：S12AD.ADANS.WORD で設定されたポートをAD変換する                                     */
/* 引       数：なし										    */
/* 戻  り   値：なし										    */
/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */
void AD_update(void)
{	
	S12AD.ADCSR.BIT.ADST = 1;  	// A/D変換開始
   	while(S12AD.ADCSR.BIT.ADST == 0); // 測定が終了するまで待つ
}

/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */
/* 関 数 概 要：CMT0(コンペアマッチタイマー)の初期化                                                */
/* 関 数 詳 細：1ms割り込み周期                                                                     */
/* 引       数：なし										    */
/* 戻  り   値：なし										    */
/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */ 
void CMT_init(void)
{
    MSTP(CMT0) = 0;                // SYSTEM.MSTPCRA.BIT.MSTPA15 = 0; // CMT0 タイマースタンバイ解除 （0で解除）
    CMT0.CMCR.WORD = 0x0040;       // 4:割り込み許可　0:PCLK/8 1:PCLK/32 2:PCLK/128 3:PCLK/512
    CMT0.CMCOR = 3000-1;           // 1ms Count： PCLK = 24MHz/8=3MHz 3M/1mS=3000 (得たいカウント数-1)
    IPR(CMT0,CMI0) = 3;
    IEN(CMT0,CMI0) = 1;
	
    set_psw(0x00010000);
	
    CMT.CMSTR0.BIT.STR0 = 1;   	   // CMT0タイマースタート
}
	
/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */
/* 関 数 概 要：左モーター(MTU0,1）,右モーター(MTU3,4） タイマーの初期化	   			            */
/* 関 数 詳 細：		                                                    		                */
/* 引       数：なし										    									*/
/* 戻  り   値：なし										    									*/
/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */ 
void MTU0_init(void){
	volatile int C_cycle,duty;
	
	C_cycle = 24e6 / 1000;
	
	SYSTEM.MSTPCRA.BIT.MSTPA9 = 0;  // MTUユニット０　モジュールストップ解除
	MTUA.TSTR.BYTE &= 0x04 ;  // カウンタの停止  00xx x100　
	
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
	
	MTUA.TSTR.BYTE = 0xc3;	// 11xx x011　カウンタの開始

}

/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */
/* 関 数 概 要：MTU7(左エンコーダ）,MTU8(右エンコーダ） タイマーの初期化   		            */
/* 関 数 詳 細：		                                                                    */
/* 引       数：なし										    */
/* 戻  り   値：なし										    */
/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */ 
void MTU1_init(void){
	
	SYSTEM.MSTPCRA.BIT.MSTPA8 = 0;  // MTUユニット1　モジュールストップ解除
	MTUB.TSTR.BYTE = 0x00;  // カウンタの停止
	
	IOPORT.PFDMTU.BIT.TCLKS=1;//ピン設定(MTCLKE-B, MTCLKF-B,MTCLKG-B, MTCLKH-Bを選択)

	MTU7.TMDR.BYTE=4;//MTU7を位相計数モードに
	MTU8.TMDR.BYTE=4;//MTU8を位相計数モードに
	
	PORTB.ICR.BIT.B2 = 1;
	PORTB.ICR.BIT.B3 = 1;
	PORTB.ICR.BIT.B4 = 1;
	PORTB.ICR.BIT.B5 = 1;

	MTUB.TSTR.BYTE = 0x06;	// 0000 0110 カウンタの開始
}
		
/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */
/* 関 数 概 要：モーターのPWM設定											  			            */
/* 関 数 詳 細：												                                   */
/* 引       数：左モーターPWM、右モーターPWM	（-100.0から+100.0） 							    */
/* 戻  り   値：なし										    									*/
/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */ 
void motor(float duty_L, float duty_R){
	
    unsigned int dt_L, dt_R;
	int L_PM = 0,R_PM = 0; // 0:プラス 1:マイナス
	
    if(duty_L >  100.0) duty_L =  100.0; // duty_Lを100以上にしない
    if(duty_L < -100.0) duty_L = -100.0; // duty_Lを-100以下にしない
    if(duty_R >  100.0) duty_R =  100.0;
    if(duty_R < -100.0) duty_R = -100.0;
	
	
	if(duty_L < 0.0){
		L_PM = 1;
		duty_L = -duty_L;
	}
	
	/* デューティ比の算出 */
   	dt_L = MTU1.TGRA * duty_L / 100.0;//  dt_L = 0.9445*50/100 = 0.5
		 
	/* デューティ比のオーバーフロー保護 */
    if(dt_L >= MTU1.TGRA)   dt_L = MTU1.TGRA - 1;  // 
		
	if(L_PM == 0){
		 /* デューティ比の設定 */
   		 MTU0.TGRB = dt_L;
   		 MTU1.TGRB = 0;
	}else{
		/* デューティ比の設定 */
   		 MTU0.TGRB = 0;
   		 MTU1.TGRB = dt_L;
	}
	
	if(duty_R < 0.0){
		R_PM = 1;
		duty_R = -duty_R;
	}
	
	/* デューティ比の算出 */
   	dt_R = MTU3.TGRA * duty_R / 100.0;//  dt_R = 0.9445*50/100 = 0.5
		 
	/* デューティ比のオーバーフロー保護 */
    if(dt_R >= MTU3.TGRA)   dt_R = MTU3.TGRA - 1;  // 
		
	if(R_PM == 0){
		 /* デューティ比の設定 */
   		 MTU3.TGRB = dt_R;
   		 MTU4.TGRB = 0;
	}else{
		/* デューティ比の設定 */
   		 MTU3.TGRB = 0;
   		 MTU4.TGRB = dt_R;
	}
}

/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */
/* 関 数 概 要：LED出力														  			            */
/* 関 数 詳 細：												                                   */
/* 引       数： 	3:LED4 2:LED3 1:LED2 0:LED1													    */
/* 戻  り   値：なし										    									*/
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
/* 関 数 概 要：赤外線LED出力												  			            */
/* 関 数 詳 細：												                                   */
/* 引       数： 	3:LED4 2:LED3 1:LED2 0:LED1													    */
/* 戻  り   値：なし										    									*/
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
/* 関 数 概 要：赤外線センサー値更新										  			            */
/* 関 数 詳 細：												                                   */
/* 引       数： なし																			    */
/* 戻  り   値：なし										    									*/
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
/* 関 数 概 要：スイッチ入力												  			            */
/* 関 数 詳 細：												                                   */
/* 引       数： なし																			    */
/* 戻  り   値： on:1 off:0										    									*/
/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */ 
int get_sw(){
	return (PORTA.PORT.BIT.B4 == 1)? 0: 1 ;
}

/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */
/* 関 数 概 要：CMT0割り込みモジュール                                                              */
/* 関 数 詳 細：                                                                                    */
/* 引       数：なし										    */
/* 戻  り   値：なし										    */
/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */
#pragma interrupt (Excep_CMT0_CMI0(vect=28))
void Excep_CMT0_CMI0(void)
{	
	static int task = 0;
	task ++;                         // タスクの更新						
  	if (task == 10) task = 0;             // 2まで来たら0にクリア
	
	Gyro_update();
	
	ir_update();
	
	
	switch(task) {                         // タスクポインタに従って処理を行う			
	case 0:
        	break;
   
   	case 1:
   		
        	break;
		
	default:
		break;
   	}
}





 



