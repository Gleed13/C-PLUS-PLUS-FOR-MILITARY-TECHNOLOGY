#include <drone_link.h>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <cstdio>
#include <cstring>

using namespace dlink;
 
// Прапор O_NONBLOCK при open() робить наступні read() неблокуючими
// (read поверне -1/EAGAIN, якщо даних нема, замість того щоб чекати).
int openUart(const char* dev) {            // "/tmp/ttyA" (sim) або "/dev/ttyAMA1" (плата)
    int fd = open(dev, O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd < 0) { perror("open"); return -1; }
    termios tio{};
    tcgetattr(fd, &tio);
    cfmakeraw(&tio);                     	// 8N1, без обробки символів
    cfsetispeed(&tio, B115200);
    cfsetospeed(&tio, B115200);          	// швидкість з обох боків однакова!
    tio.c_cflag |= (CLOCAL | CREAD);
    tcsetattr(fd, TCSANOW, &tio);
    return fd;
}
