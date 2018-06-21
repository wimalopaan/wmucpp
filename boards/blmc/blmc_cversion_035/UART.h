#ifndef _UART_H
#define _UART_H

#define CMD_FLAG 0x80
#define ESC_CMD  0x40
#define MNT_CMD  0x20
#define MTR_INDEX  0x1C // mask for motor index
#define MYADDR   1 //default address of this controller
#define IS_DATA(X)  (!((X) & CMD_FLAG ))
#define IS_CMD_FOR_ME(X)  (((X) & (CMD_FLAG | ESC_CMD | MNT_CMD | (MYADDR<<2))) == (CMD_FLAG | ESC_CMD | (MYADDR<<2)))
#define IS_MAINT(X)  (((X) & (CMD_FLAG | ESC_CMD | MNT_CMD )) == (CMD_FLAG | ESC_CMD | MNT_CMD ))


void openSerial(void);
int getch(void);
void putch(char);
void putln(char);

#endif // _UART_H
