/**
* @file fork-uart.c
* @brief Serial port programming in C, with fork processing for writing and reading
* @author Jeremie Roy
* @date 2025-10-31
*/

#define _GNU_SOURCE

#include <stdio.h>
#include <fcntl.h>   // File Control Definitions
#include <termios.h> // POSIX Terminal Control Definitions
#include <unistd.h>  // UNIX Standard Definitions
#include <errno.h>   // ERROR Number Definitions

// device port série à utiliser 
//const char *portTTY = "/dev/ttyGS0"; 
//const char *portTTY = "/dev/ttyS0";
//const char *portTTY = "/dev/ttyS1";
//const char *portTTY = "/dev/ttyS2";
//const char *portTTY = "/dev/ttyS3";
//const char *portTTY = "/dev/ttyS4";
//const char *portTTY = "/dev/ttyS5";
const char *portTTY = "/dev/ttyUSB0"; // ttyUSB0 is the FT232 based USB2SERIAL Converter

void main()
{
    int fd;
    printf("Ouverture du port serie");
    fd = open(portTTY, O_RDWR | O_NOCTTY | O_NDELAY);

    //Opening serial port
    if(fd == -1)//error checking
    {
        printf("\n Erreur: ouverture de %s\n", portTTY);
    }
    else
    {
        printf("\nOuverture de %s reussite", portTTY);
    }

    struct termios SerialPortSettings;
    tcgetattr(fd, &SerialPortSettings);
    //Setting baued rate
    cfsetispeed(&SerialPortSettings, B115200);
    cfsetospeed(&SerialPortSettings, B115200);
    //8N1 mode
    SerialPortSettings.c_cflag &= ~PARENB;   // Disables the Parity Enable bit(PARENB),So No Parity
	SerialPortSettings.c_cflag &= ~CSTOPB;   // CSTOPB = 2 Stop bits,here it is cleared so 1 Stop bit
	SerialPortSettings.c_cflag &= ~CSIZE;	 // Clears the mask for setting the data size
	SerialPortSettings.c_cflag |=  CS8;      //Set the data bits = 8
	SerialPortSettings.c_cflag &= ~CRTSCTS;       // No Hardware flow Control
	SerialPortSettings.c_cflag |= CREAD | CLOCAL; // Enable receiver, Ignore Modem Control lines 

	SerialPortSettings.c_iflag &= ~(IXON | IXOFF | IXANY);	// Disable XON/XOFF flow control both i/p and o/p

	SerialPortSettings.c_lflag &= ~(ECHO | ECHOE | ISIG);  // Cannonical mode, Disable echo, Disable signal  
	SerialPortSettings.c_oflag &= ~OPOST;	//No Output Processing

    // Setting Time outs 
	SerialPortSettings.c_cc[VMIN] = 1; // Read at least X character(s) 
	SerialPortSettings.c_cc[VTIME] = 0; // Wait 3sec (0 for indefinetly) 

    if(fd != -1)
    {
        if(fork() == 0)
        {//Child process
        //Write UART
        printf("Je suis le processus Fils, j'écrit sur le port série ce que j'entends sur la console (termial)");
        char* write_buffer;
        do
        {
             //Lie le terminal
            write_buffer = "";
            if(scanf("%s", &write_buffer) > 0 && (write_buffer!= "q"))//si il y a un caractère de lu différent de q
            {
                write(fd, write_buffer, sizeof(write_buffer));//il st écrit sur le port uart
            }

        }while(write_buffer != "q");//processus fini
        write(fd, "!", 1);
        printf("Fin du Fils"); 
        //
        }
         else
        {//parent process
        //Read UART
            printf("Je suis le processus Père, j'écrit sur la console (terminal) ce que j'entends sur le port série");
            char cBuffer;
            do{
                cBuffer = 0;
                int byte_read = 0;
                tcflush(fd, TCIFLUSH);
                byte_read = read(fd, cBuffer, 1);
                printf("%s", cBuffer);
            }while(cBuffer =! '!');  
        }
        close(fd);
    }
}