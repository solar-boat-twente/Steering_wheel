#include "stdio.h"
#include <fcntl.h>
#include "errno.h"
#include "termios.h"
#include "unistd.h"    
#include <stdlib.h>

int fd1;
char buff[100];
int rd, nbytes, tries;

int main(){
  fd1 = open("/dev/ttyUSB0", O_RDWR | O_NOCTTY | O_NDELAY);
  printf("Opened reader with file descirption %d\n", fd1);
  fcntl(fd1, F_SETFL, 0);
  printf("fnctl was successfull\n");
while(true){
  rd = read(fd1, buff, 100);
  printf("Bytes sent are %s with rd = %d\n\n", buff, rd);
  sleep(0.1);
}
  close(fd1);
  return 0;
}
