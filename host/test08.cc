#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <fcntl.h>   /* File Control Definitions           */
#include <termios.h> /* POSIX Terminal Control Definitions */
#include <unistd.h>  /* UNIX Standard Definitions 	   */ 
#include <errno.h>   /* E */
#include <iostream>
#include <array>

int main(){
    int fd = -1;
    
    if ((fd = open("/dev/ttyUSB0",O_RDWR | O_NOCTTY)) < 0) {
        std::cout << "open failure" << std::endl;
        exit(EXIT_FAILURE);
    }
    else{
        std::cout <<"Serial Port open" << std::endl;
    }

    struct termios SerialPortSettings = {};
    
    tcgetattr(fd, &SerialPortSettings);	/* Get the current attributes of the Serial port */
    
    //    cfsetispeed(&SerialPortSettings,B19200); /* Set Read  Speed as 9600                       */
    //    cfsetospeed(&SerialPortSettings,B19200); /* Set Write Speed as 9600                       */
    
    cfsetispeed(&SerialPortSettings,B9600); /* Set Read  Speed as 9600                       */
    cfsetospeed(&SerialPortSettings,B9600); /* Set Write Speed as 9600                       */
    
    SerialPortSettings.c_cflag &= ~PARENB;   /* Disables the Parity Enable bit(PARENB),So No Parity   */
    SerialPortSettings.c_cflag &= ~CSTOPB;   /* CSTOPB = 2 Stop bits,here it is cleared so 1 Stop bit */
    SerialPortSettings.c_cflag &= ~CSIZE;	 /* Clears the mask for setting the data size             */
    SerialPortSettings.c_cflag |=  CS8;      /* Set the data bits = 8                                 */
    
    SerialPortSettings.c_cflag &= ~CRTSCTS;       /* No Hardware flow Control                         */
    SerialPortSettings.c_cflag |= CREAD | CLOCAL; /* Enable receiver,Ignore Modem Control lines       */ 
    
    SerialPortSettings.c_iflag &= ~(IXON | IXOFF | IXANY);          /* Disable XON/XOFF flow control both i/p and o/p */

    SerialPortSettings.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);  /* Non Cannonical mode                            */
    
    SerialPortSettings.c_oflag &= ~OPOST;/*No Output Processing*/
    
    if((tcsetattr(fd,TCSANOW,&SerialPortSettings)) != 0) {
        perror("tcsetattr");
    }
        
//    if (fcntl(fd, F_SETFL, 0) < 0) { // blocking mode
//        perror("fcntl");
//        return EXIT_FAILURE;
//    } 

    int bytes_written = 0;
    
    std::array toWrite{std::byte{'a'}, std::byte{'b'}};
//    std::array<std::byte, 100> toWrite{};
    
    std::cout << std::size(toWrite) << std::endl;

    if ((bytes_written = write(fd, &toWrite[0], std::size(toWrite))) < 0) {
        perror("write");
        return EXIT_FAILURE;
    }
    std::cout << "Bytes written: " << bytes_written << std::endl;
    
    std::array<std::byte, 2> toRead;
    auto hasSomethingToRead = std::size(toRead);

    while(hasSomethingToRead > 0){
        std::cout << "try to read " << hasSomethingToRead << " bytes\n";
        if (auto bytesRead = read(fd, &toRead[0], std::size(toRead)); bytesRead > 0) {
            hasSomethingToRead -= bytesRead;
            std::cout << "read " << bytesRead << " bytes\n";
        }
        else {
            if (bytesRead == 0) {
                return EXIT_FAILURE; // device closed
            }
            else {
                perror("read");
                return EXIT_FAILURE;
            }
        }
    }
    std::cout << hasSomethingToRead << std::endl;
    
    close(fd);
}
