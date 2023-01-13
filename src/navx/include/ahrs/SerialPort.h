/*
 * SerialPort.h
 *
 * Created on: Dec 10, 2015
 *     Author: Jude Sauve
 */
#ifndef SRC_SERIALPORT_H_
#define SRC_SERIALPORT_H_

#include <termios.h>
#include <string>

class SerialPort {
  public:
    // Calls Init
    SerialPort(int baudRate, std::string id);

    void Init(int baudRate, std::string id);

    void SetReadBufferSize(int size);

    void SetTimeout(int timeout);

    void EnableTermination(char c);

    void Flush(void);

    void Write(char *data, int length);

    int GetBytesReceived(void);

    int Read(char *data, int size);

    void WaitForData(void);

    void Reset(void);

    void Close();

  private:
    int ReadBufferSize;
    int timeout;
    char terminationChar;
    bool termination;
    std::string id;
    int baudRate;
    int fd;
    struct termios tty;
    int err;
};
#endif
