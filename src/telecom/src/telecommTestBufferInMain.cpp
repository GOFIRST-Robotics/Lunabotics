#include <cstdio>
#include <cstring>
#include <string>
#include <queue>
//http://wjwwood.io/serial/doc/1.1.0/classserial_1_1_serial.html
#include "Telecomm.h"
#define ERR_CHECK \
  do { if (comm.status() != 0){ \
    fprintf(stdout, "Error: %s\n", comm.verboseStatus().c_str()); \
    return comm.status(); \
  } } while(0)
// Should add goto LBL_REBOOT to this, that's what it's really good for

int main(int argc, char *argv[]){
  if (argc != 4){
    fprintf(stderr, "usage: ./programname dst-hostname dst-udpport src-udpport\n");
    exit(1);
  }

  Telecomm comm(argv[1], atoi(argv[2]), atoi(argv[3]));
  comm.setFailureAction(false);
  comm.setBlockingTime(0,0);
  ERR_CHECK;

  time_t timer;
  time(&timer);

  // This is to catch dropped messages
  std::queue<std::string> dropBuffer;
  bool rebooting = false;

  while(1){
    comm.update();
    ERR_CHECK;

    // Receive from remote
    if(comm.recvAvail()){
      std::string msg = comm.recv();
      //ERR_CHECK; // This makes it crash???

      if(!msg.empty())
        fprintf(stdout, "Received message: %s\n", msg.c_str());
      
      if(!msg.compare("EOM\n")){ break; } // delete if not want remote close

      // if connection good, then no dropped messages, empty
      while(!comm.isCommClosed() && !dropBuffer.empty())
        dropBuffer.pop();
    }

    // Reboot if communication channel closed
    LBL_REBOOT:
    while(comm.isCommClosed()){
      printf("Rebooting connection\n");
      comm.reboot();
      rebooting = true;
    }
    if(rebooting){ // After rebooting, send dropped msg buffer
      rebooting = false;
      std::queue<std::string> tmpBuff (dropBuffer);
      int rv = 0;
      while(!dropBuffer.empty() && ((rv = comm.send(dropBuffer.front())) == 0)){
        dropBuffer.pop();
      }
      if(rv != 0 || dropBuffer.empty()){
        dropBuffer.swap(tmpBuff);
      }
      //ERR_CHECK; why do these keep causing the destructor, double deletes->crash
      goto LBL_REBOOT;
    }

    // Get user stdio input to send
    if(comm.stdioReadAvail()){
      std::string msg = comm.stdioRead();
      ERR_CHECK;

      if(!msg.compare("EOM\n")){ // Assume desired for connection to close
        comm.send(msg); // Delete for debug connection testing
        printf("Received EOM closing\n");
        break;
      }
      
      comm.send(msg);
      while(!dropBuffer.empty()) // successful send should clear
        dropBuffer.pop();
      dropBuffer.push(msg); // Add msg
      ERR_CHECK;
    }

    // heartbeat
    if(difftime(time(NULL), timer) > 10){
      timer = time(NULL);
      printf(".\n");
    }
  }
}
