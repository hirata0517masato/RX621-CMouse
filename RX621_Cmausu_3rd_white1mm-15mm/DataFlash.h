#ifndef DATA_FLASH_H
#define DATA_FLASH_H

#include "r_flash_api_rx_if.h"

void DataFlash_init(void);
void DataFlash_write(char BlockNum,uint8_t prog_buff[],int buf_num);
void DataFlash_write2(char BlockNum,char shift,uint8_t prog_buff[],int buf_num);
void DataFlash_read(char BlockNum,uint8_t read_buff[],int buf_num);

#endif