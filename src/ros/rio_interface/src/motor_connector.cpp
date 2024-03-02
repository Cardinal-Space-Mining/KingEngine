#include <cstdio>
#include <mutex> //std::mutex
#include <utility> //std::pair

#include "SerialMotorCtrl.h"

constexpr const char* RIO_PORT = "/dev/ttyUSB2";

using std::pair;
using std::mutex;
int main(int argc, char ** argv)
{
  (void) argc;
  (void) argv;

  std::mutex rio_lock;
  MotorSerialConnection rio_con(RIO_PORT);
  
  printf("hello world rio_interface package\n");
  return 0;
}
