#include"IR.h"
#include "iodefine.h"
#include <machine.h>

#define AD_0	S12AD.ADDR0	     // CN3-9 : AN0(0～4069)
#define AD_1	S12AD.ADDR1	     // CN3-10: AN1(0～4069)
#define AD_2	S12AD.ADDR2	     // CN3-11 :AN2(0～4069)
#define AD_3	S12AD.ADDR3	     // CN3-12 :AN3(0～4069)
#define AD_4	S12AD.ADDR4	     // AN4(0～4069)
#define AD_5	S12AD.ADDR5	     // AN5(0～4069)
#define AD_6	S12AD.ADDR6	     // AN6(0～4069)

int S[7] = {0,0,0,0,0,0,0};	//IRセンサー値
int s[7] = {0,0,0,0,0,0,0};//IRセンサー値(IR発光なし）

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
/* 引       数： 																				    */
/* 戻  り   値：なし										    									*/
/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */ 
void ir(char n){
	
	if((n & 0x01) > 0)PORTD.DR.BIT.B4 = 1;
	else PORTD.DR.BIT.B4 = 0;
	
	if((n & 0x02) > 0)PORTD.DR.BIT.B3 = 1;
	else PORTD.DR.BIT.B3 = 0;
	
	if((n & 0x04) > 0)PORTD.DR.BIT.B2 = 1;
	else PORTD.DR.BIT.B2 = 0;
	
	if((n & 0x08) > 0)PORTD.DR.BIT.B1 = 1;
	else PORTD.DR.BIT.B1 = 0;
	
	if((n & 0x10) > 0)PORTD.DR.BIT.B0 = 1;
	else PORTD.DR.BIT.B0 = 0;
	
	if((n & 0x20) > 0)PORTD.DR.BIT.B5 = 1;
	else PORTD.DR.BIT.B5 = 0;
	
	if((n & 0x40) > 0)PORTD.DR.BIT.B6 = 1;
	else PORTD.DR.BIT.B6 = 0;

}

void ir_update(){
	
	static int num = 0,tmp[7] = {0},S_old[7] = {0};
	
	//IRを光らせた 次の次 が発光タイミング
	AD_update();

	
	//マスの中央でのセンサー値　123 130 290 130 123 
	switch(num){
		case 0:	//FL R 
			s[0] = AD_4;
			s[3] = AD_1;
			
			ir(0);// 000 0000
			S12AD.ADANS.WORD = 0x001B;//0001 1011
			num = 1;
			break;
		
		case 1://
			tmp[0] = (AD_4 - s[0]) /10;
			tmp[3] = (AD_1 - s[3]) /10;
			
			if(abs(S_old[0] - tmp[0]) > 5){//極端に値が変化したときはノイズと考える
				S[0] = S[0];
			}else{
				S[0] = tmp[0];
			}
			S_old[0] = tmp[0];
			
			if(abs(S_old[3] - tmp[3]) > 5){//極端に値が変化したときはノイズと考える
				S[3] = S[3];
			}else{
				S[3] = tmp[3];
			}
			S_old[3] = tmp[3];
			
			ir(0x12);// 001 0010
			//S12AD.ADANS.WORD = 0x0009;//0000 1001
			
			num = 2;
			break;
			
		case 2://L FR
			s[1] = AD_3;
			s[4] = AD_0;
			
			ir(0);// 000 0000
			S12AD.ADANS.WORD = 0x006D;//0110 1101
			
			num = 3;
			break;
		case 3://
			tmp[1] = (AD_3 - s[1]) /10;
			tmp[4] = (AD_0 - s[4]) /10 * 7 /10;//センサーのばらつきを微調整（本当は自動でやりたい）
			
			
			
			if(abs(S_old[1] - tmp[1]) > 5){//極端に値が変化したときはノイズと考える
				S[1] = S[1];
			}else{
				S[1] = tmp[1];
			}
			S_old[1] = tmp[1];
			
			if(abs(S_old[4] - tmp[4]) > 5){//極端に値が変化したときはノイズと考える
				S[4] = S[4];
			}else{
				S[4] = tmp[4];
			}
			S_old[4] = tmp[4];
			
			//ir(0x04);//000 0100
			ir(0x64);//110 0100
			//S12AD.ADANS.WORD = 0x0064;//0110 0100
			
			num = 4;
			break;
		case 4:// LT F RT
			s[2] = AD_2;
			s[5] = AD_5;
			s[6] = AD_6;
			
			ir(0);// 000 0000
			S12AD.ADANS.WORD = 0x0076;//0111 0110
			
			num = 5;
			break;
		case 5://
			tmp[2] = (AD_2 - s[2]) /10;
			tmp[5] = (AD_5 - s[5])  * 6 /10 /2; //センサーのばらつきを微調整;
			tmp[6] = (AD_6 - s[6])/2; 
			
			
			if(abs(S_old[2] - tmp[2]) > 5){//極端に値が変化したときはノイズと考える
				S[2] = S[2];
			}else{
				S[2] = tmp[2];
			}
			S_old[2] = tmp[2];
			
			
			if(abs(S_old[5] - tmp[5]) > 5){//調子が悪いので極端に値が変化したときはノイズと考える
				S[5] = S[5];
			}else{
				S[5] = (tmp[5])  + (S[5] / 2) ;
			}
			S_old[5] = tmp[5];
			
			
			if(abs(S_old[6] - tmp[6]) > 5){//調子が悪いので極端に値が変化したときはノイズと考える
				S[6] = S[6];
			}else{
				S[6] = (tmp[6])  + (S[6] / 2) ;
			}
			S_old[6] = tmp[6];
			ir(0x09);// 000 1001
			//S12AD.ADANS.WORD = 0x0012;//0001 0010
			
			num = 0;
			break;
	}
}




/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */
/* 関 数 概 要：赤外線センサー値取得										  			            */
/* 関 数 詳 細：												                                   */
/* 引       数： なし																			    */
/* 戻  り   値：なし										    									*/
/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */ 
int get_IR(int num){
	return max(0,S[num]);//マイナス値は０にする	
	//return S[num];
}

int get_IR_base(int num){
	return max(0,s[num]);//マイナス値は０にする	
	//return s[num];
}