#include <cstdio>
#include <cstring>
#include <string>
//http://wjwwood.io/serial/doc/1.1.0/classserial_1_1_serial.html
#include "telecom/telecom.h"
#define ERR_CHECK \
  do { if (com.status() != 0){ \
    fprintf(stdout, "Error: %s\n", com.verboseStatus().c_str()); \
    return com.status(); \
  } } while(0)

int main(int argc, char *argv[]){
  if (argc != 4){
    fprintf(stderr, "usage: ./programname dst-hostname dst-udpport src-udpport\n");
    exit(1);
  }

  Telecom com(argv[1], atoi(argv[2]), atoi(argv[3]));
  com.setFailureAction(false);
  com.setBlockingTime(0,0);
  ERR_CHECK;

  time_t timer;
  time(&timer);

  // Joystick js(); // #include "joystick.hh"
  // Exit if !js.isFound()
  // com.fdAdd(js.fd());

  while(1){
    com.update();
    ERR_CHECK;

    // JoystickEvent event;

    // Receive from remote
    if(com.recvAvail()){
      std::string msg = com.recv();
      //ERR_CHECK; // This makes it crash???

      if(!msg.empty())
        printf("Received message: %s\n", msg.c_str());
      
      if(!msg.compare("EOM\n")){ break; } // delete if not want remote close
    }

    // Reboot if comunication channel closed
    while(com.isComClosed()){
      printf("Rebooting connection\n");
      com.reboot();
    }

    // Get user stdio input to send
    if(com.stdioReadAvail()){
      std::string msg = com.stdioRead();
      ERR_CHECK;

      if(!msg.compare("EOM\n")){ // Assume desired for connection to close
        com.send(msg); // Delete for debug connection testing
        printf("Received EOM closing\n");
        break;
      }
      
      com.send(msg);
      ERR_CHECK;
    }

    // Example if including joystick
    // if(com.fdReadAvail(js.fd()) && js.sample(&event)){
    //   ... process buttons and axis of event ... }

    // heartbeat
    if(difftime(time(NULL), timer) > 10){
      timer = time(NULL);
      printf(".\n");
    }
  }
}
