#include <iostream>
#include <iostream>
#include <errno.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <termios.h>
#include <unistd.h>
#include <thread>

//‘A’ ‘T’ [29bit ID, big endian ] [3bit: 110 for remote 100 for data] [8bit datalength: MSB to LSB] [n*8bit: data(big endian)] <CR> <LF>

using namespace std;

const uint8_t exteneded_mask = 0b00000100;
const uint8_t remote_mask = 0b00000010;

int set_interface_attribs(int fd, int speed)
{
    struct termios tty;

    if (tcgetattr(fd, &tty) < 0)
    {
        printf("Error from tcgetattr: %s\n", strerror(errno));
        return -1;
    }

    cfsetospeed(&tty, (speed_t)speed);
    cfsetispeed(&tty, (speed_t)speed);

    tty.c_cflag |= (CLOCAL | CREAD); /* ignore modem controls */
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;      /* 8-bit characters */
    tty.c_cflag &= ~PARENB;  /* no parity bit */
    tty.c_cflag &= ~CSTOPB;  /* only need 1 stop bit */
    tty.c_cflag &= ~CRTSCTS; /* no hardware flowcontrol */

    /* setup for non-canonical mode */
    tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL | IXON);
    tty.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);
    tty.c_oflag &= ~OPOST;

    /* fetch bytes as they become available */
    tty.c_cc[VMIN] = 1;
    tty.c_cc[VTIME] = 0;

    if (tcsetattr(fd, TCSANOW, &tty) != 0)
    {
        printf("Error from tcsetattr: %s\n", strerror(errno));
        return -1;
    }
    return 0;
}

void sendData(int fd, uint32_t ID, bool isExtended, bool isRemote, uint8_t numOfData, uint8_t data[])
{
    int buf_length = 2 + 4 + 1 + numOfData + 2;
    unsigned char send[buf_length];

    send[0] = 'A';
    send[1] = 'T';

    if (isExtended)
    {
        uint32_t temp = (ID) << 3;
        send[5] = (temp);
        send[4] = (temp >> 8);
        send[3] = (temp >> 16);
        send[2] = (temp >> 24);
        send[5] |= exteneded_mask;
    }
    else
    {
        uint32_t temp = (ID) << 21;
        send[5] = (temp);
        send[4] = (temp >> 8);
        send[3] = (temp >> 16);
        send[2] = (temp >> 24);
        send[5] &= ~exteneded_mask;
    }

    if (isRemote)
        send[5] |= remote_mask;

    send[6] = numOfData;

    memcpy(&send[7], data, numOfData);

    send[buf_length - 2] = '\r';
    send[buf_length - 1] = '\n';

    int wlen = write(fd, send, buf_length);
    if (wlen != buf_length)
    {
        printf("Error from write: %d, %d\n", wlen, errno);
    }

    printf("sent: ");
    for (int i = 0; i < buf_length; i++)
        printf("%x ", send[i]);
    printf("\n");
}

void send_thread(int fd)
{
    do
    {
        int rdlen;
        uint32_t testID = 0x7ff;
        uint8_t test_data[] = {0x01, 0x02, 0x03, 0x04, 0x05};

        sendData(fd, testID, false, false, sizeof(test_data), test_data);
        usleep(500000); //1ms
    } while (1);
}

void receive_thread(int fd)
{
    do
    {
        unsigned char buf[100];
        int rdlen = read(fd, buf, 1);
        printf("Read");
        if (rdlen > 0)
        {
            unsigned char *p;
            printf("Read %d:", rdlen);
            for (p = buf; rdlen-- > 0; p++)
                printf(" %x", *p);
            printf("\n");
        }
        else if (rdlen < 0)
        {
            printf("Error from read: %d: %s\n", rdlen, strerror(errno));
        }
    } while (1);
    /* repeat read to get full message */
}

int main(int argc, char **argv)
{
    string portname = "/dev/ttyUSB";
    if (argc > 1)
    {
        std::cout << argv[1][0] << endl;
        portname += argv[1][0];
        cout << "opening " << portname << endl;
    }

    int fd = open(portname.c_str(), O_RDWR | O_NOCTTY | O_SYNC);
    if (fd < 0)
    {
        printf("Error opening %s: %s\n", portname.c_str(), strerror(errno));
        return -1;
    }

    /*baudrate 115200, 8 bits, no parity, 1 stop bit */
    set_interface_attribs(fd, B115200);

    usleep(500000); //0.5s
    char setmodestr[] = "AT+AT\r\n";
    int wlen = write(fd, setmodestr, sizeof(setmodestr) - 1);
    if (wlen != sizeof(setmodestr) - 1)
    {
        printf("Error from write: %d, %d\n", wlen, errno);
    }
    usleep(500000); //0.5s

    std::thread task1(send_thread, fd);
    std::thread task2(receive_thread, fd);

    task1.join();
    task2.join();
}
