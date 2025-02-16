/************************************************************************/
/* �Ώۃ}�C�R�� RX621                                                   */
/* ̧�ٓ��e     printf,scanf�֘A����                                    */
/* �o�[�W����   Ver.1.00                                                */
/* Date         		                                                */
/* Copyright    		    						                     */
/************************************************************************/

/*======================================*/
/* �C���N���[�h                         */
/*======================================*/
#include    <stdio.h>
#include 	<stdarg.h>
#include    <machine.h>
#include    "iodefine.h"
#include    "printf_lib.h"              /* printf�֘A����               */


/*======================================*/
/* �V���{����`                         */
/*======================================*/
#define     SEND_BUFF_SIZE  64          /* ���M�o�b�t�@�T�C�Y           */
#define     RECV_BUFF_SIZE  32          /* ��M�o�b�t�@�T�C�Y           */

/*======================================*/
/* �O���[�o���ϐ��̐錾                 */
/*======================================*/
/* ���M�o�b�t�@ */
static volatile char    send_buff[SEND_BUFF_SIZE];
static volatile char    *send_w = send_buff;
static volatile char    *send_r = send_buff;
static volatile int     send_count = 0;

/* ��M�o�b�t�@ */
static int             recvFlag;        /* ��M�t���O                   */
static unsigned char   recvData;        /* ��M�f�[�^                   */
static unsigned char   recvError;       /* ��M�G���[                   */

/*======================================*/
/* �v���g�^�C�v�錾                     */
/*======================================*/
void setSendBuff( char c);
int getSendBuff( char *c );


/************************************************************************/
/* UART0�̏������A�y��printf�֌W��UART0�Ɋ��蓖��                       */
/* �����@ �ʐM���x                                                      */
/* �߂�l �Ȃ�                                                          */
/************************************************************************/
void initSCI1( int sp )
{
    volatile int i;

    MSTP(SCI1) = 0 ;                    // ���W���[���X�g�b�v��Ԃ̉���

	IOPORT.PFFSCI.BIT.SCI1S=0;  //P30��RxD1-A�[�q�Ƃ��Đݒ� P27��SCK1-A�[�q�Ƃ��Đݒ� P26��TxD1-A�[�q�Ƃ��Đݒ�
 	PORT3.DDR.BIT.B0 = 0;       //RxD1����͂ɂ��邽�߂ɕK�v
	PORT3.ICR.BIT.B0 = 1; 		// RxD1�̓��̓o�b�t�@ON
	PORT2.DDR.BIT.B6 = 1;       //TxD1���o�͂ɂ��邽�߂ɕK�v	
	
    SCI1.SCR.BYTE = 0 ;                 // �����N���b�N�̑I��
	
    // BRR = PCLK * 10^6 / ( 64 * 2^(2n-1) * B) - 1
    // PCLK = 12.288*2, n=PCLK�N���b�N�̏ꍇ�ɂ�� B=�{�[���[�g[bps]
    // n = PCLK/1:0 PCLK/4:1 PCLK/16:2 PCLK/64:3
    if( sp == SPEED_4800 ) {
        SCI1.SMR.BYTE = 0x01 ;          // PLCK/4, ��������, bit 8, �p���e�B �Ȃ�
        SCI1.BRR = 40-1;                // 24576000 / ( 64 * 2 * 4800 ) - 1
    } else if( sp == SPEED_9600 ) {
        SCI1.SMR.BYTE = 0x00 ;          // PLCK, ��������, bit 8, �p���e�B �Ȃ�
        SCI1.BRR = 80-1;               //  24576000 / ( 64 * 0.5 * 9600 ) - 1
    } else if( sp == SPEED_19200 ) {
        SCI1.SMR.BYTE = 0x00 ;          // PLCK, ��������, bit 8, �p���e�B �Ȃ�
        SCI1.BRR = 40-1;                // 24576000 / ( 64 * 0.5 * 19200 ) - 1
    } else if( sp == SPEED_38400 ) {
        SCI1.SMR.BYTE = 0x00 ;          // PLCK, ��������, bit 8, �p���e�B �Ȃ�
        SCI1.BRR = 20-1;                // 24576000 / ( 64 * 0.5 * 38400 ) - 1
    }
    for(i=0; i<4000; i++);              // 1�r�b�g����(52��s,ICLK=96MHz)�҂�
    SCI1.SCR.BYTE = 0xf0;               // ����M����,����M���荞�݋���


    IPR(SCI1,    ) = 7 ;                // SCI1 ���荞�ݗD�惌�x��=7
    IEN(SCI1,TXI1) = 1 ;                // SCI1 ���M���荞�݋���(IEN)
    IEN(SCI1,RXI1) = 1 ;                // SCI1 ��M���荞�݋���(IEN)
    IEN(SCI1,ERI1) = 1 ;                // SCI1 ��M�G���[���荞�݋���(IEN)

	printf2("start\r\n");//����̃m�C�Y�H�΍�
}

/************************************************************************/
/*													                     */
/* �����@ 			                                                      */
/* �߂�l �Ȃ�                                                          */
/************************************************************************/
void printf2(const char *format, ...){
    char   put_data;
	char send_data[SEND_BUFF_SIZE];
	va_list va;
	int i = 0;
	volatile int cnt = 0;
	 
    va_start(va, format);
	vsprintf(send_data,format, va );
    va_end(va);
	
	for(i = 0; send_data[i] != '\0';i++){
		
		if( send_data[i] == '\n' )  {
        	setSendBuff( '\r' );
    	} else if( send_data[i] == '\b' ) {
        	setSendBuff( '\b' );
        	setSendBuff( ' ' );
    	}
	
    	setSendBuff( send_data[i]);

    	// �������M���Ă��Ȃ��Ȃ�A1�����ڂ͂����ő��M����
    	// ���̌�͑��M���荞�݂ő��M����
    	if( SCI1.SSR.BIT.TEND == 1 ) {
			for(cnt = 0;cnt < 1000;cnt++);
        	getSendBuff( &put_data );
        	put_sci1( put_data );
    	}
	}

    return;
}

/************************************************************************/
/*													                     */
/* �����@ 			                                                      */
/* �߂�l �Ȃ�                                                          */
/************************************************************************/
void scanf2(const char *format, ...){
    char   c;
	va_list va;
	static char    read_data[RECV_BUFF_SIZE];
	int i = 0;
	
    do {
    	/* ��M�҂� */
        while( get_sci1( &c ) != 1 );

        switch( c ) {
    	    case '\b':/* �o�b�N�X�y�[�X */
                /* �����o�b�t�@�ɂȂ��Ȃ�BS�͖��� */
                if( 0 < i ) continue;
                /* ����Ȃ��߂� */
                i--;
                break;
            case '\r':/* Enter�L�[ */
                read_data[i++] = c = '\n';
                read_data[i++] = '\r';
                break;
            default:
                read_data[i++] = c;
                break;
        }
        /* �G�R�[�o�b�N ���͂��ꂽ������Ԃ�   �Ȃ������̏������K�v*/
		printf2("%c",c);
    } while( c != '\n' && i < RECV_BUFF_SIZE-2);
  
	read_data[i++] = '\0';
	
	va_start(va, format);
	vsscanf(read_data,format, va );
    va_end(va);

    return;
}



/************************************************************************/
/* �P������M                                                           */
/* �����@ ��M�����i�[�A�h���X                                          */
/* �߂�l -1:��M�G���[ 0:��M�Ȃ� 1:��M���� ������*s�Ɋi�[            */
/************************************************************************/
int get_sci1( char *s )
{
    volatile int ret = 0;

    if( recvFlag == 1 ){                // ��M�f�[�^����H
        recvFlag = 0;
        *s = recvData;
        ret = 1;
        if( recvError ) {               // �G���[����H
            recvError = 0;
            ret = -1;
        }
    }
    return ret;
}

/************************************************************************/
/* �P�����o��                                                           */
/* �����@ ���M�f�[�^                                                    */
/* �߂�l 0:���M���̂��߁A���M�ł��� 1:���M�Z�b�g����                   */
/************************************************************************/
int put_sci1( char r )
{
    if( SCI1.SSR.BIT.TDRE == 1) {       // TDR�ɏ������݂��o����܂ő҂�
        SCI1.TDR = r;
        return 1;
    } else {
        /* ���M��(����̃f�[�^�͑��M�����ɏI��) */
        return 0;
    }
}

/************************************************************************/
/* ���M�o�b�t�@�ɕۑ�                                                   */
/* �����@ �i�[����                                                      */
/* �߂�l �Ȃ�                                                          */
/* ����   �o�b�t�@���t���̏ꍇ�A�󂭂܂ő҂��܂�                        */
/************************************************************************/
void setSendBuff( char c )
{
    // �o�b�t�@���󂭂܂ő҂�
    while( SEND_BUFF_SIZE == send_count );

    IEN(SCI1,TXI1) = 0;

    *send_w++ = c;
    if( send_w >= send_buff+SEND_BUFF_SIZE ) send_w = send_buff;
    send_count++;

    IEN(SCI1,TXI1) = 1;
}

/************************************************************************/
/* ���M�o�b�t�@����擾                                                 */
/* �����@ �i�[���镶���̃A�h���X                                        */
/* �߂�l 0:�f�[�^�Ȃ� 1:�f�[�^����                                     */
/************************************************************************/
int getSendBuff( char *c )
{
    volatile int    ret = 0;

    if( send_count ) {                  // �f�[�^������Ȃ�o�b�t�@����o��
        IEN(SCI1,TXI1) = 0;

        *c = *send_r++;
        if( send_r >= send_buff+SEND_BUFF_SIZE ) send_r = send_buff;
        send_count--;
        ret = 1;

        IEN(SCI1,TXI1) = 1;
    }
    return ret;
}

/************************************************************************/
/* SCI1 ERI1 ���荞�ݏ���                                               */
/************************************************************************/
#pragma interrupt Excep_SCI1_ERI1(vect=VECT_SCI1_ERI1)
void Excep_SCI1_ERI1(void)
{
    recvError = SCI1.SSR.BYTE & 0x38;   // ��M�G���[�t���O�ǂݏo��

    SCI1.SSR.BYTE = 0xc0 ;              // ��M�G���[�t���O�N���A
    while( (SCI1.SSR.BYTE & 0x38) );    // �G���[�t���O�́g0�h�N���A�m�F
    while( IR(SCI1,ERI1) );             // ��M�G���[�̊��荞�݃X�e�[�^�X�r�b�g��0���m�F
}

/************************************************************************/
/* SCI1 RXI1 ���荞�ݏ���                                               */
/************************************************************************/
#pragma interrupt Excep_SCI1_RXI1(vect=VECT_SCI1_RXI1)
void Excep_SCI1_RXI1(void)
{
    recvData = SCI1.RDR ;               // ��M�f�[�^�ǂݏo��
    recvFlag = 1 ;                      // �t���O�ϐ����P�ɃZ�b�g
}

/************************************************************************/
/* SCI1 TXI1 ���荞�ݏ���                                               */
/************************************************************************/
#pragma interrupt Excep_SCI1_TXI1(vect=VECT_SCI1_TXI1)
void Excep_SCI1_TXI1(void)
{
    char   c;
    int    ret;

    ret = getSendBuff( &c );            // �f�[�^�擾
    if( ret ) {
        SCI1.TDR = c;                   // �f�[�^����Ȃ瑗�M
    }
}


/************************************************************************/
/* printf�ŌĂяo�����֐�                                             */
/* ���[�U�[����͌Ăяo���܂���                                         */
/* printf���g�p�ł��Ȃ��i*buf�̒l���ُ�j�̂ŏ��print�֐�������        */
/************************************************************************/
long write(long fileno, const unsigned char *buf, long count)
{
    return 1;
}

/************************************************************************/
/* scanf�ŌĂяo�����֐�                                              */
/* ���[�U�[����͌Ăяo���܂���                                         */
/* �Ȃ����g�p�ł��܂���			                                        */
/************************************************************************/
long read(long fileno, unsigned char *buf, long count)
{
    return 1;
}
/************************************************************************/
/* end of file                                                          */
/************************************************************************/

