#include "iodefine.h"
#include <machine.h>
#include "rspi.h"


/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */
/* �� �� �T �v�FSPI�̏�����                                                                         */
/* �� �� �� �ׁF                                                                                    */
/* ��       ���F�Ȃ�										    */
/* ��  ��   �l�F�Ȃ�										    */
/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */ 
void SPI_init(void)
{
	SYSTEM.MSTPCRB.BIT.MSTPB16 = 0; //SPI�X�g�b�v����
 
	RSPI1.SPCR.BIT.SPE = 0;  //SPI�@�\��~
 
	RSPI1.SPCMD0.BIT.BRDV = 0;  //6MHz = PCLK/(2x(SPBR+1)x pow(2,BRDV)) 
	RSPI1.SPBR = 1;  	    //
 
	RSPI1.SPPCR.BYTE = 0x00; // CMOS�o�́A�A�C�h������MOSI�M���l��"0"

	RSPI1.SPDCR.BIT.SLSEL = 1; //SSL0 ����
	RSPI1.SPDCR.BIT.SPRDTD = 0; //0:��M 1:���M�@�o�b�t�@�ǂݏo�����[�h
	RSPI1.SPDCR.BIT.SPLW = 0; //���[�h�A�N�Z�X
 
	RSPI1.SPCMD0.BIT.CPHA = 1; //��G�b�W�Ńf�[�^�ω������ŃT���v��
	RSPI1.SPCMD0.BIT.CPOL = 1; //�A�C�h����RSPCK=1
	RSPI1.SPCMD0.BIT.SSLA = 0; //SSL0�ŃA�T�[�g
	RSPI1.SPCMD0.BIT.SSLKP = 1; //SSL�M����ێ�
	RSPI1.SPCMD0.BIT.SPB = 7; //�f�[�^��8bit
 	RSPI1.SPCMD0.BIT.LSBF = 0; //MSB�t�@�[�X�g
 
	PORTE.ICR.BIT.B7=1;  //���̓o�b�t�@�L��
	IOPORT.PFHSPI.BIT.RSPIS = 1; //SPI�[�qportE�ɕύX
	IOPORT.PFHSPI.BIT.RSPCKE = 1; //RSPCKB�L��
	IOPORT.PFHSPI.BIT.MOSIE = 1; //MOSIB�L��
	IOPORT.PFHSPI.BIT.MISOE = 1; //MISOB�L��
	IOPORT.PFHSPI.BIT.SSL0E = 1; //SSL0B�L��

	RSPI1.SPCR.BYTE |= 0x08;	//�}�X�^���[�h
 
	RSPI1.SPSR.BYTE = 0xA0;  //�X�e�[�^�X�N���A	
}

/*
//�Z���T���W�X�^����1�o�C�g�ǂݏo��
unsigned char rspi_read( unsigned short reg){
	unsigned short	recv;
	RSPI1.SPCR.BIT.SPE = 1;   // enable
	
	RSPI1.SPDR.WORD.H = (reg | 0x80) << 8;  // ���M
	
	while( 0 == RSPI1.SPSR.BIT.SPRF ); // ��M�����҂�
	recv = RSPI1.SPDR.WORD.H;  // ��M�o�b�t�@�Ǐo��
	RSPI1.SPCR.BIT.SPE = 0;   // disable
	
	recv = recv&0x00ff;   //�}�X�N
	return(recv);
}

void rspi_write( unsigned short reg,unsigned short data){
	
	RSPI1.SPCR.BIT.SPE = 1;   // enable
	
	RSPI1.SPDR.WORD.H = (reg << 8) + data;  // ���M
	
	while( 0 == RSPI1.SPSR.BIT.SPTEF ); // ��M�����҂�
	
	RSPI1.SPCR.BIT.SPE = 0;   // disable
}*/

unsigned char rspi_transfer( unsigned char reg){
	unsigned char	recv;

	RSPI1.SPDR.WORD.H = reg ;  // ���M
	
	while( 0 == RSPI1.SPSR.BIT.SPRF ); // ��M�����҂�
	recv = RSPI1.SPDR.WORD.H;  // ��M�o�b�t�@�Ǐo��
	
	//recv = recv&0x00ff;   //�}�X�N
	return(recv);
}

void rspi_select(){
	RSPI1.SPCR.BIT.SPE = 1;   // enable
}

void rspi_deselect(){
	RSPI1.SPCR.BIT.SPE = 0;   // disable
}
