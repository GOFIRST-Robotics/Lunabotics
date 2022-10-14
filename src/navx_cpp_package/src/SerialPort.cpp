#include <stdio.h>
#include <sys/ioctl.h>
#include <stdlib.h>
#include <string>
#include <cstring>
#include <unistd.h>
#include <termios.h>
#include <fcntl.h>
#include <errno.h>
#include <iostream>
#include <stdexcept>
#include <thread>
#include <chrono>

#include "ahrs/SerialPort.h"

SerialPort::SerialPort(int baudRate, std::string id)
{
    Init(baudRate, id);
}

void SerialPort::Init(int baudRate, std::string id) {


    int USB = open(id.c_str(), O_RDWR| O_NOCTTY);
    if (USB < 0)
    {
        std::cerr << "Could not open " << id.c_str() << " as a TTY:";
        perror("");
  throw std::runtime_error("");
    }
    
    memset(&tty, 0, sizeof(tty));
    this->baudRate = baudRate;
    this->id = id;
    

    cfsetospeed(&tty, (speed_t)baudRate);
    cfsetispeed(&tty, (speed_t)baudRate);

    tty.c_cflag &= ~PARENB;
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;

    //tty.c_cflag = CS8|CREAD|CLOCAL;

    //tty.c_cflag &= ~CRTSCTS;
    tty.c_cc[VMIN] = 1;
    tty.c_cc[VTIME] = 10;

    tty.c_cflag |= CREAD | CLOCAL;

    cfmakeraw(&tty);
    this->fd = USB;
    tcflush(this->fd, TCIOFLUSH);
    if(tcsetattr(USB,TCSANOW,&tty) != 0) std::cout << "Failed to initialize serial." << std::endl;

}

void SerialPort::SetReadBufferSize(int size) {
    this->ReadBufferSize = size;
}

void SerialPort::SetTimeout(int timeout) {
    this->timeout = timeout;
    tty.c_cc[VTIME] = timeout*10;
    cfmakeraw(&tty);
    if(tcsetattr(this->fd,TCSANOW,&tty) != 0) std::cout << "Failed to initialize serial in SetTimeout." << std::endl;;
}


void SerialPort::EnableTermination(char c) {
    this->termination = true;
    this->terminationChar = c;
}


void SerialPort::Flush() {
    tcflush(this->fd, TCOFLUSH);
}

void SerialPort::Write(char *data, int length) {
    int n_written = 0, spot = 0;
    do {

        n_written = write( this->fd, &data[spot], length );
        if (n_written > 0)
            spot += n_written;
    } while (data[spot-1] != terminationChar); 
}

int SerialPort::GetBytesReceived() {
    int bytes_avail;
    ioctl(this->fd, FIONREAD, &bytes_avail);
    return bytes_avail;
}

int SerialPort::Read(char *data, int size) {
    int n = 0, loc = 0;
    char buf = '\0';
    memset(data, '\0', size);
    err = 0;

    do {
        n = read(this->fd, &buf, 1);
        sprintf( &data[loc], "%c", buf );
        loc += n;

        if(n == 0) err++;

        if(err > 100)
        {
            err = 0;
            Reset();
            Close();
            std::this_thread::sleep_for(std::chrono::milliseconds(30000));	
            Init(this->baudRate, this->id);
            SetTimeout(this->timeout);
            SetReadBufferSize(this->ReadBufferSize);
            EnableTermination(this->terminationChar);
            Reset();
            break;

        }

    } while( buf != terminationChar && loc < size);

    if (n < 0) {
        std::cout << "Error reading: " << strerror(errno) << std::endl;
    }
    else if (n == 0) {
        std::cout << "Read nothing!" << std::endl;
    }        
    else {
        //std::cout << "Response: " << data  << std::endl;
    }
    return loc;
}

void SerialPort::WaitForData()
{
    fd_set readfds;
    struct timeval tv;
    FD_ZERO(&readfds);
    FD_SET(this->fd, &readfds);
    tv.tv_sec = 0;
    tv.tv_usec = 100000;
    select(this->fd + 1, &readfds, NULL, NULL, &tv);
}

void SerialPort::Reset() {
    tcflush(this->fd, TCIOFLUSH);
}

void SerialPort::Close() {
    close(this->fd);
}
