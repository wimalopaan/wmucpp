#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <fcntl.h>   /* File Control Definitions           */
#include <termios.h> /* POSIX Terminal Control Definitions */
#include <unistd.h>  /* UNIX Standard Definitions 	   */ 
#include <errno.h>   /* ERROR Number Definitions           */

const uint16_t Crc16Tbl[256] = {
	0x0000,0x1021,0x2042,0x3063,0x4084,0x50A5,0x60C6,0x70E7,
	0x8108,0x9129,0xA14A,0xB16B,0xC18C,0xD1AD,0xE1CE,0xF1EF,
	0x1231,0x0210,0x3273,0x2252,0x52B5,0x4294,0x72F7,0x62D6,
	0x9339,0x8318,0xB37B,0xA35A,0xD3BD,0xC39C,0xF3FF,0xE3DE,
	0x2462,0x3443,0x0420,0x1401,0x64E6,0x74C7,0x44A4,0x5485,
	0xA56A,0xB54B,0x8528,0x9509,0xE5EE,0xF5CF,0xC5AC,0xD58D,
	0x3653,0x2672,0x1611,0x0630,0x76D7,0x66F6,0x5695,0x46B4,
	0xB75B,0xA77A,0x9719,0x8738,0xF7DF,0xE7FE,0xD79D,0xC7BC,
	0x48C4,0x58E5,0x6886,0x78A7,0x0840,0x1861,0x2802,0x3823,
	0xC9CC,0xD9ED,0xE98E,0xF9AF,0x8948,0x9969,0xA90A,0xB92B,
	0x5AF5,0x4AD4,0x7AB7,0x6A96,0x1A71,0x0A50,0x3A33,0x2A12,
	0xDBFD,0xCBDC,0xFBBF,0xEB9E,0x9B79,0x8B58,0xBB3B,0xAB1A,
	0x6CA6,0x7C87,0x4CE4,0x5CC5,0x2C22,0x3C03,0x0C60,0x1C41,
	0xEDAE,0xFD8F,0xCDEC,0xDDCD,0xAD2A,0xBD0B,0x8D68,0x9D49,
	0x7E97,0x6EB6,0x5ED5,0x4EF4,0x3E13,0x2E32,0x1E51,0x0E70,
	0xFF9F,0xEFBE,0xDFDD,0xCFFC,0xBF1B,0xAF3A,0x9F59,0x8F78,
	0x9188,0x81A9,0xB1CA,0xA1EB,0xD10C,0xC12D,0xF14E,0xE16F,
	0x1080,0x00A1,0x30C2,0x20E3,0x5004,0x4025,0x7046,0x6067,
	0x83B9,0x9398,0xA3FB,0xB3DA,0xC33D,0xD31C,0xE37F,0xF35E,
	0x02B1,0x1290,0x22F3,0x32D2,0x4235,0x5214,0x6277,0x7256,
	0xB5EA,0xA5CB,0x95A8,0x8589,0xF56E,0xE54F,0xD52C,0xC50D,
	0x34E2,0x24C3,0x14A0,0x0481,0x7466,0x6447,0x5424,0x4405,
	0xA7DB,0xB7FA,0x8799,0x97B8,0xE75F,0xF77E,0xC71D,0xD73C,
	0x26D3,0x36F2,0x0691,0x16B0,0x6657,0x7676,0x4615,0x5634,
	0xD94C,0xC96D,0xF90E,0xE92F,0x99C8,0x89E9,0xB98A,0xA9AB,
	0x5844,0x4865,0x7806,0x6827,0x18C0,0x08E1,0x3882,0x28A3,
	0xCB7D,0xDB5C,0xEB3F,0xFB1E,0x8BF9,0x9BD8,0xABBB,0xBB9A,
	0x4A75,0x5A54,0x6A37,0x7A16,0x0AF1,0x1AD0,0x2AB3,0x3A92,
	0xFD2E,0xED0F,0xDD6C,0xCD4D,0xBDAA,0xAD8B,0x9DE8,0x8DC9,
	0x7C26,0x6C07,0x5C64,0x4C45,0x3CA2,0x2C83,0x1CE0,0x0CC1,
	0xEF1F,0xFF3E,0xCF5D,0xDF7C,0xAF9B,0xBFBA,0x8FD9,0x9FF8,
	0x6E17,0x7E36,0x4E55,0x5E74,0x2E93,0x3EB2,0x0ED1,0x1EF0
};

uint16_t CRC16_Calculat(uint8_t *buf, size_t len) {
	size_t index = 0;
	uint16_t uCrc = 0;

	for(size_t ix=0; ix<len; ix++) {
		index = (((uCrc >> 8) & 0xff) ^ buf[ix]) & 0x00FF;	
		uCrc = ((uCrc & 0xff) * 256) ^ Crc16Tbl[index];
	}
	return uCrc;
}

template<auto N>
void addCRC(uint8_t (&a)[N]) {
    uint16_t crc = CRC16_Calculat(&a[3], N - 2 - 3);
    a[N - 2] = crc & 0xff;
    a[N - 1] = crc >> 8;
}

int main() {
    int fd = -1;
    
    if ((fd = open("/dev/ttyUSB0",O_RDWR | O_NOCTTY)) < 0) {
        perror("open");
        exit(EXIT_FAILURE);
    }
    
    struct termios SerialPortSettings = {};
    
    tcgetattr(fd, &SerialPortSettings);	/* Get the current attributes of the Serial port */
    
    //    cfsetispeed(&SerialPortSettings,B19200); /* Set Read  Speed as 9600                       */
    //    cfsetospeed(&SerialPortSettings,B19200); /* Set Write Speed as 9600                       */
    
    cfsetispeed(&SerialPortSettings,B115200); /* Set Read  Speed as 9600                       */
    cfsetospeed(&SerialPortSettings,B115200); /* Set Write Speed as 9600                       */
    
    SerialPortSettings.c_cflag &= ~PARENB;   /* Disables the Parity Enable bit(PARENB),So No Parity   */
    SerialPortSettings.c_cflag &= ~CSTOPB;   /* CSTOPB = 2 Stop bits,here it is cleared so 1 Stop bit */
    SerialPortSettings.c_cflag &= ~CSIZE;	 /* Clears the mask for setting the data size             */
    SerialPortSettings.c_cflag |=  CS8;      /* Set the data bits = 8                                 */
    
    SerialPortSettings.c_cflag &= ~CRTSCTS;       /* No Hardware flow Control                         */
    SerialPortSettings.c_cflag |= CREAD | CLOCAL; /* Enable receiver,Ignore Modem Control lines       */ 
    
    SerialPortSettings.c_iflag &= ~(IXON | IXOFF | IXANY);          /* Disable XON/XOFF flow control both i/p and o/p */
    SerialPortSettings.c_iflag &= ~(ICANON | ECHO | ECHOE | ISIG);  /* Non Cannonical mode                            */
    
    SerialPortSettings.c_oflag &= ~OPOST;/*No Output Processing*/
    
    if((tcsetattr(fd,TCSANOW,&SerialPortSettings)) != 0) {
        perror("tcsetattr");
    }
    
    
//    unsigned char write_buffer[] = {0x00, 0x03, 0xfc, 0x00, 0x00, 0x04, 0x34, 0x13, 0xba}; // Receiver -> 20 Bytes (ACK = 1)
    unsigned char write_buffer[] = {0x00, 0x03, 0xfc, 0x00, 0x00, 0x04, 0x35, 0x32, 0xaa}; // GAM V4  -> 59 Bytes (59) (ACK = 1)
//    unsigned char write_buffer[] = {0x00, 0x03, 0xfc, 0x00, 0x00, 0x04, 0x36, 0x51, 0x9a}; // Electric V4 -> 65 Bytes (59) (ACK = 1)
//    unsigned char write_buffer[] = {0x00, 0x03, 0xfc, 0x00, 0x00, 0x04, 0x38, 0x9f, 0x7b}; // GPS V4 -> 51 Bytes (50 Doku falsch) (ACK = 1)
//    unsigned char write_buffer[] = {0x00, 0x03, 0xfc, 0x00, 0x00, 0x04, 0x37, 0x70, 0x8a}; // Vario V4 -> 50 Bytes (50) (ACK = 1)
//    unsigned char write_buffer[] = {0x00, 0x03, 0xfc, 0x00, 0x00, 0x04, 0x39, 0xbe, 0x6b}; // Air ESC V4 -> 34 Bytes (30 ???) (ACK = 1) (Doku?)
//    unsigned char write_buffer[] = {0x00, 0x03, 0xfc, 0x00, 0x00, 0x04, 0x33, 0xf4, 0xca}; // DBM -> 234 Bytes (Ack = 1) (234) (Doku falsche crc)
//    unsigned char write_buffer[] = {0x00, 0x03, 0xfc, 0x00, 0x00, 0x04, 0x40, 0x00, 0x84}; // Servo Positions -> 9 Bytes (NACK = 2) (73 ???)
//    unsigned char write_buffer[] = {0x00, 0x03, 0xfc, 0x00, 0x00, 0x04, 0x41, 0x21, 0x94}; // Pupil Control -> 9 Bytes (NACK = 2)(65 ???)
//    unsigned char write_buffer[] = {0x00, 0x03, 0xfc, 0x00, 0x00, 0x04, 0x42, 0x42, 0xa4}; // Control Positions -> 9 Bytes (NACK = 2) (178 ???)
//    unsigned char write_buffer[] = {0x00, 0x03, 0xfc, 0x00, 0x00, 0x04, 0x43, 0x63, 0xb4}; // Control Positions -> 9 Bytes (NACK = 2)(11 ???)
//    unsigned char write_buffer[] = {0x00, 0x03, 0xfc, 0x00, 0x00, 0x04, 0x44, 0x84, 0xc4}; // Text Mode-> 9 Bytes (NACK = 2)(178 ???)
//    unsigned char write_buffer[] = {0x00, 0x03, 0xfc, 0x00, 0x00, 0x04, 0x45, 0xa5, 0xd4}; // Read Model -> 9 Bytes (NACK = 2)(178 ???)
    
    addCRC(write_buffer);
    
    unsigned char fastStart[] = {0x00, 0x03, 0xfc, 0x00, 0x00, 0x04, 0x53, 0x00, 0x00}; // Fast Start -> 9 Bytes (NACK)()
    addCRC(fastStart);
    unsigned char fastStop[] = {0x00, 0x03, 0xfc, 0x00, 0x00, 0x04, 0x54, 0x00, 0x00}; // Fast Stop-> 9 Bytes (NACK)()
    addCRC(fastStop);

    unsigned char fast[50] = {0x00, 0x03, 0xfc, 0x29, 0x00, 0x04, 0x52};
    fast[7] = 1;
    fast[8] = 16;
            
    int bytes_written = 0;
    int bytes_read = 0;
    
    unsigned char read_buffer[200];

//        printf("FastStart\n");
        
//        if ((bytes_written = write(fd,fastStart,sizeof(fastStart))) < 0) {
//            perror("write");
//        }
//        else {
//            printf("written: %d, \n", bytes_written);
            
//        }
//        {
//            bool ready = true;
//            while (ready) {
//                if ((bytes_read = read(fd,read_buffer, 200)) < 0) {
//                    perror("read");
//                    ready = false;
//                }
//                else {
//                    printf("read: %d, \n", bytes_read);
//                }
//            }
//            for(size_t i = 0; i < 16; ++i) {
//                printf("%x ", read_buffer[i]);
//            }
//            printf("---\n");
//        }
    
    
    
    
    for(size_t n = 0; n < 5; ++n) {
//        useconds_t delay = 1000 * 200;
//        if (usleep(delay) < 0) {
//            perror("usleep");
//        }
        
        sleep(1);
    
        ++write_buffer[1];
        write_buffer[2] = ~write_buffer[1];
        
        if ((bytes_written = write(fd,write_buffer,sizeof(write_buffer))) < 0) {
            perror("write");
        }
        else {
            printf("written: %d, \n", bytes_written);
            
        }
        bool ready = true;
        while (ready) {
            if ((bytes_read = read(fd,read_buffer, 200)) < 0) {
                perror("read");
                ready = false;
            }
            else {
                printf("read: %d, \n", bytes_read);
            }
        }
        
        
        for(size_t i = 0; i < 9; ++i) {
            printf("%x ", read_buffer[i]);
        }
        printf("---\n");
    }

    sleep(1);

        printf("FastStop\n");
        
        if ((bytes_written = write(fd,fastStop,sizeof(fastStop))) < 0) {
            perror("write");
        }
        else {
            printf("written: %d, \n", bytes_written);
            
        }
        {
            bool ready = true;
            while (ready) {
                if ((bytes_read = read(fd,read_buffer, 200)) < 0) {
                    perror("read");
                    ready = false;
                }
                else {
                    printf("read: %d, \n", bytes_read);
                }
            }
            for(size_t i = 0; i < 9; ++i) {
                printf("%x ", read_buffer[i]);
            }
            printf("---\n");
        }
    
    printf("FastStart\n");
    
    if ((bytes_written = write(fd,fastStart,sizeof(fastStart))) < 0) {
        perror("write");
    }
    else {
        printf("written: %d, \n", bytes_written);
        
    }
    {
        bool ready = true;
        while (ready) {
            if ((bytes_read = read(fd,read_buffer, 200)) < 0) {
                perror("read");
                ready = false;
            }
            else {
                printf("read: %d, \n", bytes_read);
            }
        }
        for(size_t i = 0; i < 16; ++i) {
            printf("%x ", read_buffer[i]);
        }
        printf("---\n");
    }

    printf("Fast\n");
    
    
    sleep(1);
    
    fast[1] = write_buffer[1];
    
    
    for(size_t r = 0; r < 2; ++r ) {
        usleep(1000 * 900);
        
        ++fast[1];
        fast[2] = ~fast[1];
        
        fast[9] = 0;
        fast[10] = 1;

        fast[11] = 0;
        fast[12] = 1;

        addCRC(fast);    
        
        if ((bytes_written = write(fd,fast,sizeof(fast))) < 0) {
            perror("write");
        }
        else {
            printf("written: %d, \n", bytes_written);
            
        }
        {
            bool ready = true;
            while (ready) {
                if ((bytes_read = read(fd,read_buffer, 200)) < 0) {
                    perror("read");
                    ready = false;
                }
                else {
                    printf("read: %d, \n", bytes_read);
                }
            }
            for(size_t i = 0; i < 32; ++i) {
                printf("%x ", read_buffer[i]);
            }
            printf("---\n");
        }
    }

    
    usleep(1000*20);

//    printf("FastStop\n");
    
//    if ((bytes_written = write(fd,fastStop,sizeof(fastStop))) < 0) {
//        perror("write");
//    }
//    else {
//        printf("written: %d, \n", bytes_written);
        
//    }
//    {
//        bool ready = true;
//        while (ready) {
//            if ((bytes_read = read(fd,read_buffer, 200)) < 0) {
//                perror("read");
//                ready = false;
//            }
//            else {
//                printf("read: %d, \n", bytes_read);
//            }
//        }
//        for(size_t i = 0; i < 9; ++i) {
//            printf("%x ", read_buffer[i]);
//        }
//        printf("---\n");
//    }
    
    
    close(fd);
    
}

