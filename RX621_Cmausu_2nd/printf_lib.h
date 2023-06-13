#ifndef PRINT_H
#define PRINT_H

/*======================================*/
/* �V���{����`                         */
/*======================================*/
#define     SPEED_4800      1           /* �ʐM���x 4800bps         */
#define     SPEED_9600      2           /* �ʐM���x 9600bps         */
#define     SPEED_19200     3           /* �ʐM���x 19200bps        */
#define     SPEED_38400     4           /* �ʐM���x 38400bps        */

/*======================================*/
/* �v���g�^�C�v�錾                     */
/*======================================*/
void initSCI1( int sp );
int get_sci1( char *s );
int put_sci1( char r );
void printf2(const char *format, ...);
void scanf2(const char *format, ...);

#endif /* PRINT_H */
