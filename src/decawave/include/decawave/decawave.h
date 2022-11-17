#ifndef DECAWAVE_H
#define DECAWAVE_H
// VERSION 1.0.0

#include <serial/serial.h>
#include <memory>

struct decawave_coordinate{
  double x;
  double y;
};

class Decawave{
public:
  Decawave(int port_num);
  ~Decawave();
  void updateSamples();
  decawave_coordinate getPos();
  double anchor1[8];//was unsigned long int *was a double before...
  double anchor2[8];
private:
  int index;
  std::shared_ptr<serial::Serial> my_serial;
  //
  decawave_coordinate anchor1Pos;
  decawave_coordinate anchor2Pos;
  decawave_coordinate tagPos;
  double anchorSeparation;
};

#endif