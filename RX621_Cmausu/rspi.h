#ifndef SPI_H
#define SPI_H

/*======================================*/
/* �v���g�^�C�v�錾                     */
/*======================================*/
void SPI_init(void);
//unsigned char rspi_read( unsigned short);
//void rspi_write( unsigned short,unsigned short);
unsigned char rspi_transfer( unsigned char reg);
void rspi_select(void);
void rspi_deselect(void);

#endif