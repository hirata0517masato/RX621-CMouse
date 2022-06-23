#include"IR.h"
#include "iodefine.h"
#include <machine.h>

#define AD_0	S12AD.ADDR0	     // CN3-9 : AN0(0〜4069)
#define AD_1	S12AD.ADDR1	     // CN3-10: AN1(0〜4069)
#define AD_2	S12AD.ADDR2	     // CN3-11 :AN2(0〜4069)
#define AD_3	S12AD.ADDR3	     // CN3-12 :AN3(0〜4069)
#define AD_4	S12AD.ADDR4	     // AN4(0〜4069)

int S[4] = {0};	//IRセンサー値

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
/* 関 数 概 要：赤外線LED出力												  			            */
/* 関 数 詳 細：												                                   */
/* 引       数： 	3:LED4 2:LED3 1:LED2 0:LED1													    */
/* 戻  り   値：なし										    									*/
/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */ 
void ir(char n){
	
	if((n & 0x01) > 0)PORTD.DR.BIT.B0 = 0;
	else PORTD.DR.BIT.B0 = 1;
	
	if((n & 0x02) > 0)PORTD.DR.BIT.B1 = 0;
	else PORTD.DR.BIT.B1 = 1;
	
	if((n & 0x04) > 0)PORTD.DR.BIT.B2 = 0;
	else PORTD.DR.BIT.B2 = 1;
	
	if((n & 0x08) > 0)PORTD.DR.BIT.B3 = 0;
	else PORTD.DR.BIT.B3 = 1;
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
	
	//AD_update();
	
	switch(num){
		case 0:	//front
			S[0] = AD_1 - s[0];
			S[3] = AD_0 - s[3];
			
			s[1] = AD_2;
			s[2] = AD_3;
			
			ir(9);//0110
			num = 1;
			break;
			
		case 1://side
			S[1] = AD_2 - s[1];
			S[2] = AD_3 - s[2];
			
			s[0] = AD_1;
			s[3] = AD_0;
			
			ir(6);// 1001
			num = 0;
			break;
	}
	AD_update();
}

/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */
/* 関 数 概 要：赤外線センサー値取得										  			            */
/* 関 数 詳 細：												                                   */
/* 引       数： なし																			    */
/* 戻  り   値：なし										    									*/
/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */ 
int get_IR(int num){
	return S[num];	
}