#include"DataFlash.h"

/*
�������݃c�[��(Resesas Flash Prgrammer)�ňȉ��̑��삪�K�v�ɂȂ�܂��B

�E����ݒ�^�u > �x���t�@�C�̃`�F�b�N���O���B
�E�u���b�N�ݒ�^�u > DataFlash 1 ��Erase�`�F�b�N���O���B
�E�ڑ��ݒ� > ID�R�[�h�̐ݒ�@�ŘA������ID�R�[�h������B
*/

void DataFlash_init(void){
/*	const unsigned long id_code[4] = {
	    0x45010203,
    	0x04050607,
    	0x08090A0B,
    	0x0C0D0E0F
	};//  �S�̘A��  = 0x450102030405060708090A0B0C0D0E0F
	
	*( unsigned long*)0xFFFFFFA0 = id_code[0];
	*( unsigned long*)0xFFFFFFA4 = id_code[1];
	*( unsigned long*)0xFFFFFFA8 = id_code[2];
	*( unsigned long*)0xFFFFFFAC = id_code[3];
	*/

	R_FlashDataAreaAccess(0xffff,0xffff);
}

void DataFlash_write(char BlockNum,uint8_t prog_buff[],int buf_num){//BlockNum:0-15
	
	R_FlashErase(BlockNum + 38);// BLOCK_DB0    38 
	

	R_FlashWrite( g_flash_BlockAddresses[BlockNum + 38], (uint32_t)prog_buff, buf_num);
}

void DataFlash_read(char BlockNum,uint8_t read_buff[],int buf_num){
	int i;
	
	/* Read back written area */
    for (i = 0; i <  buf_num; i++){
    	read_buff[i] = *(uint8_t*) (g_flash_BlockAddresses[BlockNum + 38] + i);
    }
}
