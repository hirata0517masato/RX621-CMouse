#include "iodefine.h"
#include <machine.h>

#include "usb.h"

void _INIT_IOLIB( void ); //�v���g�^�C�v�錾

/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */
/* �� �� �T �v�FUSB CDC�̏�����  			                                            */
/* �� �� �� �ׁF		                                                                    */
/* ��       ���F�Ȃ�										    */
/* ��  ��   �l�F�Ȃ�										    */
/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */ 
void USB_init(){
	/*Initialise the USB CDC Class*/
	USBCDC_Init();
   	setpsw_i();							//�����݋��� clrpsw_i()�����݋֎~
	_INIT_IOLIB();						// printf(),scanf()�̏�����

	while(0 == USBCDC_IsConnected());	//USB�ڑ��҂�
}