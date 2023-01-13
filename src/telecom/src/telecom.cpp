// telecom.cpp
// VERSION: 2.0.0 Last Change: 2019-04-28
// Author: Jude Sauve <sauve031@umn.edu>

//#include <stdio.h>
#include <cstdio>
//#include <stdlib.h>
#include <cstdlib> 
#include <unistd.h> // Actually good
//#include <errno.h>
#include <cerrno>
//#include <string.h>
#include <string>
#include <cstring>
#include <sys/types.h> // Also good, not part of c spec
#include <sys/socket.h> // Has recv, socket, send, etc
#include <sys/time.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <netdb.h>
#include <fcntl.h>

#include "telecom/telecom.h"

#define STDIN 0

int sockfd = -1, maxfd;
fd_set readfds, resultfds;
struct addrinfo hints, *dstinfo = NULL, *srcinfo = NULL, *p = NULL;
std::string dst_addr;
int dst_port, src_port;
int rv = -1, ret = -1, iof = -1;
int len = -1,  numbytes = 0, sec = -1, usec = -1, yes = 1;
//struct timeval tv;
timeval *tv_ptr = NULL; 
char buffer[256] = {0};
bool throwError = true;

void Telecom::setFailureAction(bool thrwErr){
  throwError = thrwErr;
}

void Telecom::setBlockingTime(int sec_, int usec_){
  sec = sec_;
  usec = usec_;
}

void deleteWithException(bool throwit){
  if(dstinfo)
    freeaddrinfo(dstinfo);
  if(srcinfo)
    freeaddrinfo(srcinfo);
  close(sockfd);
  if(throwit && throwError)
    exit(1);
}

Telecom::Telecom(std::string dst_addr_, int dst_port_, int src_port_){
  dst_addr = dst_addr_;
  dst_port = dst_port_;
  src_port = src_port_;

  FD_ZERO(&readfds);
  FD_ZERO(&resultfds);
  FD_SET(STDIN, &readfds);
  maxfd = STDIN;

  reboot();
}

void resetMaxfd(){
  for(; !FD_ISSET(maxfd, &readfds); --maxfd);
  maxfd = (maxfd >= STDIN) ? maxfd : STDIN;
}

void Telecom::reboot(){
  // Clean up old sockfd
  FD_CLR(sockfd, &readfds);
  resetMaxfd();

  // Set dst info
  memset(&hints, 0, sizeof hints);
  hints.ai_family = AF_UNSPEC;
  hints.ai_socktype = SOCK_DGRAM;

  if((rv = getaddrinfo(dst_addr.c_str(), std::to_string(dst_port).c_str(), &hints, &dstinfo)) != 0){
    fprintf(stderr, "getaddrinfo for dst address %s\n", gai_strerror(rv));
    ret = 1;
    goto LBL_RET;
  }

  // Set src info
  memset(&hints, 0, sizeof hints);
  hints.ai_family = AF_UNSPEC;
  hints.ai_socktype = SOCK_DGRAM;
  hints.ai_flags = AI_PASSIVE;

    // Getting src_addr, could do by hand, should be 192.168.1.50; try replace
  if((rv = getaddrinfo(NULL, std::to_string(src_port).c_str(), &hints, &srcinfo)) != 0){
    fprintf(stderr, "getaddrinfo for src address %s\n", gai_strerror(rv));
    ret = 4;
    goto LBL_RET;
  }
  
  if(srcinfo == NULL){ // DEBUG
    fprintf(stderr, "srcinfo is null??");
  }

  for(p = dstinfo; p != NULL; p = p->ai_next){
    // Get socket from dst info
    if((sockfd = socket(p->ai_family, p->ai_socktype, p->ai_protocol)) == -1){
      perror("socket");
      continue;
    }
    // Bind socket src_addr info */
    if((rv = bind(sockfd, srcinfo->ai_addr, srcinfo->ai_addrlen)) != 0) {
      perror("bind");
      continue;
    }
    break;
  }

  if(p == NULL){ // Either failed to get socket to all entries
    fprintf(stderr, "Failed to get socket\n");
    ret = 2;
    goto LBL_RET;
  } else if(rv != 0){ // Bind could've faild
    fprintf(stderr, "bind: %s\n", gai_strerror(rv));
    ret = 5;
    goto LBL_RET;
  }
 
  // Set sock addr to be reusable
  if(setsockopt(sockfd, SOL_SOCKET, SO_REUSEADDR, &yes, sizeof(yes)) == -1){
    perror("setsockopt");
    ret = 11;
    goto LBL_RET;
  }

  // Or connect socket to dst
  if((rv= connect(sockfd, p->ai_addr, p->ai_addrlen)) != 0) {
    fprintf(stderr, "connect: %s\n", gai_strerror(rv));
    ret = 3;
    goto LBL_RET;
  }

  // End init(?)
  ret = 0;

  // Since sockfd is good, no errors, add to &readfds, update maxfd
  FD_SET(sockfd, &readfds);
  maxfd = (maxfd < sockfd) ? sockfd : maxfd;

  LBL_RET:
    if(ret == 0){
      return;
    }else{
      deleteWithException(true);
  }

}

Telecom::~Telecom(){
  ret = 0;
  deleteWithException(false);
}

int Telecom::getErrno(){ return errno; }

int Telecom::status(){ return ret; }

bool Telecom::isComClosed(){ return ret != 0; }

void Telecom::fdAdd(int fd){
  FD_SET(fd, &readfds);
  maxfd = (maxfd < fd) ? fd : maxfd;
}

void Telecom::fdRemove(int fd){ FD_CLR(fd, &readfds); }

bool Telecom::fdReadAvail(int fd){ return FD_ISSET(fd, &resultfds); }

int Telecom::update(){
  // To be called at the beginning of every loop
  memcpy(&resultfds, &readfds, sizeof(resultfds));

  if(sec < 0 || usec < 0){
    tv_ptr = NULL;
  }else{
    timeval tv = {0};
    tv.tv_sec = sec;
    tv.tv_usec = usec;
    tv_ptr = &tv;
  }

  if(select(maxfd + 1, &resultfds, NULL, NULL, tv_ptr) < 0 && errno != EINTR){
    perror("select");
    ret = 6;
    deleteWithException(true);
  }
  return ret;
}

int sendall(int s, char *buf, int *len) {
  int total = 0;        // how many bytes we've sent
  int bytesleft = *len; // how many we have left to send
  int n = 0;

  while(total < *len) {
    n = ::send(s, buf+total, bytesleft, 0);
    //fprintf(stdout,"Sendall: %s\n",buf+total);
    if (n == -1) { break; }
    total += n;
    bytesleft -= n;
  }

  *len = total; // return number actually sent here

  return n==-1?-1:0; // return -1 on failure, 0 on success
}

bool Telecom::stdioReadAvail(){
  return FD_ISSET(STDIN, &resultfds);
}

std::string Telecom::stdioRead(){
  memset(buffer, 0, sizeof(buffer));
  len = 0;
  if(0 >= (len = read(STDIN, buffer, sizeof(buffer)))){
    perror("read STDIN");
    ret = 7;
    deleteWithException(true);
  }
  return std::string(buffer, len); 
}

int Telecom::send(std::string msg){
  memset(buffer, 0, sizeof(buffer));
  std::strcpy(buffer, msg.c_str());
  len = msg.length();
  if(sendall(sockfd, buffer, &len) == -1) {
    perror("sendall");
    fprintf(stderr, "We only sent %d bytes b/c of error\n", len);
    ret = 8;
    deleteWithException(true);
  }
  return ret;
}

int Telecom::sendBytes(char* bytes, int len){
  if(sendall(sockfd, bytes, &len) == -1) {
    perror("sendall");
    fprintf(stderr, "We only sent %d bytes b/c of error\n", len);
    ret = 8;
    deleteWithException(true);
  }
  return ret;
}

bool Telecom::recvAvail(){
  return FD_ISSET(sockfd, &resultfds);
}

std::string Telecom::recv(){
  memset(buffer, 0, sizeof(buffer));
  // overwrite/define recv, want #include, use ::recv to get one def'd in global namespace
  // Set non-blocking mode
  if ((iof = fcntl(sockfd, F_GETFL, 0)) != -1)
    fcntl(sockfd, F_SETFL, iof | O_NONBLOCK);
  // Receive
  numbytes = ::recv(sockfd, buffer, sizeof(buffer), 0);
  // Set flags as before
  if (iof != -1)
    fcntl(sockfd, F_SETFL, iof);
  // Error checking
  if(0 == numbytes || (errno == ECONNREFUSED && -1 == numbytes)){
    printf("Destination closed\n");
    ret = 9;
    deleteWithException(true);
    return "";
  }else if(-1 == numbytes){ // aka errno 111
    perror("recv");
    printf("Receive error check firewall settings, numbytes: %d\n", numbytes);
    ret = 10;
    deleteWithException(true);
    return "";
  }
  return std::string(buffer, numbytes);
}

int Telecom::recv(char*& buf){
  memset(buf, 0, sizeof(buf));
  // overwrite/define recv, want #include, use ::recv to get one def'd in global namespace
  // Set non-blocking mode
  if ((iof = fcntl(sockfd, F_GETFL, 0)) != -1)
    fcntl(sockfd, F_SETFL, iof | O_NONBLOCK);
  // Receive
  numbytes = ::recv(sockfd, buf, sizeof(buf), 0);
  // Set flags as before
  if (iof != -1)
    fcntl(sockfd, F_SETFL, iof);
  // Error checking
  if(0 == numbytes || (errno == ECONNREFUSED && -1 == numbytes)){
    printf("Destination closed\n");
    ret = 9;
    deleteWithException(true);
    return -1;
  }else if(-1 == numbytes){ // aka errno 111
    perror("recv");
    printf("Receive error check firewall settings, numbytes: %d\n", numbytes);
    ret = 10;
    deleteWithException(true);
    return -1;
  }
  return numbytes;
}

std::string Telecom::simpleStatus(int i){
  switch(i){
    case -1: return "INIT_ERR";
    case 0: return "OK";
    case 1: return "GETADDRINFO_DST";
    case 2: return "SOCK_ERR";
    case 3: return "CONNECT_DST_ERR";
    case 4: return "GETADDRINFO_SRC";
    case 5: return "BIND_SRC_ERR";
    case 6: return "SELECT_ERR";
    case 7: return "STDIN_READ_ERR";
    case 8: return "SEND_ERR";
    case 9: return "DST_CLOSED";
    case 10: return "RECV_ERR";
    case 11: return "SETSOCKOPT";
    default: return "SIMPLESTATUS_ERR";
  }
}

std::string Telecom::simpleStatus(){
  return simpleStatus(ret);
}

std::string Telecom::verboseStatus(int i){
  switch(i){
    case -1: return "INIT_ERR";
    case 0: return "OK";
    case 1: return ("GETADDRINFO_DST: " + std::string(gai_strerror(rv)));
    case 2: return "SOCK_ERR: failed to get socket, no addrinfo for dst";
    case 3: return ("CONNECT_DST_ERR: " + std::string(gai_strerror(rv)));
    case 4: return ("GETADDRINFO_SRC: " + std::string(gai_strerror(rv)));
    case 5: return ("BIND_SRC_ERR: " + std::string(gai_strerror(rv)));
    case 6: return "SELECT_ERR";
    case 7: return "STDIN_READ_ERR";
    case 8: return "SEND_ERR";
    case 9: return "DST_CLOSED";
    case 10: return "RECV_ERR: check firewall?";
    case 11: return "SETSOCKOPT, IDK";
    default: return ("SIMPLESTATUS_ERR: UNKNOWN CODE " + std::to_string(i));
  }
}

std::string Telecom::verboseStatus(){
  return verboseStatus(ret);
}