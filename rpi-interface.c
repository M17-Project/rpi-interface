#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <math.h>

#include <netinet/ip_icmp.h>
#include <netinet/udp.h>
#include <netinet/tcp.h>
#include <netinet/ip.h>
#include <sys/socket.h>
#include <arpa/inet.h>

#include <fcntl.h> 
#include <termios.h>
#include <unistd.h>

#include "interface_cmds.h"

#define PORT 17000
#define MAX_UDP_LEN 65535

//internet
struct sockaddr_in source, dest; 
int16_t sockt;
uint32_t saddr_size, data_size;
struct iphdr *iph;
struct sockaddr_in saddr;
struct sockaddr_in daddr;
struct sockaddr_in serv_addr;

uint8_t tx_buff[512]={0};
uint8_t rx_buff[65536]={0};
int16_t tx_len=0, rx_len=0;

//M17
struct lsf_t
{
	uint64_t dst;
	uint64_t src;
	uint16_t type;
	uint8_t meta[14]; //112-bit
};

struct m17stream_t
{
	uint16_t sid;
	struct lsf_t lsf;
	uint16_t fn;
	uint8_t pld[16];
} m17stream;

uint64_t enc_callsign=0;

//device stuff
uint8_t cmd[8];

//UART magic
int fd; //UART handle

int set_interface_attribs (int fd, int speed, int parity)
{
	struct termios tty;
	if (tcgetattr (fd, &tty) != 0)
	{
		//error_message ("error %d from tcgetattr", errno);
		return -1;
 	}

	cfsetospeed (&tty, speed);
	cfsetispeed (&tty, speed);

	tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;     // 8-bit chars
	// disable IGNBRK for mismatched speed tests; otherwise receive break
	// as \000 chars
	tty.c_iflag &= ~IGNBRK;			// disable break processing
	tty.c_lflag = 0;				// no signaling chars, no echo,
									// no canonical processing
	tty.c_oflag = 0;                // no remapping, no delays
	tty.c_cc[VMIN]  = 0;            // read doesn't block
	tty.c_cc[VTIME] = 5;            // 0.5 seconds read timeout

	tty.c_iflag &= ~(IXON | IXOFF | IXANY); // shut off xon/xoff ctrl

	tty.c_cflag |= (CLOCAL | CREAD);// ignore modem controls,
                                        // enable reading
	tty.c_cflag &= ~(PARENB | PARODD);      // shut off parity
	tty.c_cflag |= parity;
	tty.c_cflag &= ~CSTOPB;
	tty.c_cflag &= ~CRTSCTS;

	if (tcsetattr (fd, TCSANOW, &tty) != 0)
	{
		//error_message ("error %d from tcsetattr", errno);
		return -1;
	}
	
	return 0;
}

void set_blocking (int fd, int should_block)
{
	struct termios tty;
	memset (&tty, 0, sizeof tty);
	if (tcgetattr (fd, &tty) != 0)
	{
		//error_message ("error %d from tggetattr", errno);
		return;
	}

	tty.c_cc[VMIN]  = should_block ? 1 : 0;
	tty.c_cc[VTIME] = 5;            // 0.5 seconds read timeout

	if (tcsetattr (fd, TCSANOW, &tty) != 0)
		;//error_message ("error %d setting term attributes", errno);
}

//M17 stuff
uint8_t refl_send(const uint8_t* msg, uint16_t len)
{
	if(sendto(sockt, msg, len, 0, (const struct sockaddr*)&serv_addr, sizeof(serv_addr))<0)
    {
        //fprintf(stderr, "Error connecting with reflector\nExiting\n");
        return 1;
    }

	return 0;
}

//decodes a 48-bit value to a callsign
void decode_callsign(uint8_t *outp, const uint64_t inp)
{
	uint64_t encoded=inp;

	//repack the data to a uint64_t
	/*for(uint8_t i=0; i<6; i++)
		encoded|=(uint64_t)inp[5-i]<<(8*i);*/

	//check if the value is reserved (not a callsign)
	if(encoded>=262144000000000ULL)
	{
        if(encoded==0xFFFFFFFFFFFF) //broadcast
        {
            sprintf((char*)outp, "#BCAST");
        }
        else
        {
            outp[0]=0;
        }

        return;
	}

	//decode the callsign
	uint8_t i=0;
	while(encoded>0)
	{
		outp[i]=" ABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789-/."[encoded%40];
		encoded/=40;
		i++;
	}
	outp[i]=0;
}

//encode callsign
uint8_t encode_callsign(uint64_t* out, const uint8_t* inp)
{
    //assert inp length
    if(strlen((const char*)inp)>9)
    {
        return -1;
    }

    const uint8_t charMap[40]=" ABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789-/.";

    uint64_t tmp=0;

    if(strcmp((const char*)inp, "ALL")==0)
    {
        *out=0xFFFFFFFFFFFF;
        return 0;
    }

    for(int8_t i=strlen((const char*)inp)-1; i>=0; i--)
    {
        for(uint8_t j=0; j<40; j++)
        {
            if(inp[i]==charMap[j])
            {
                tmp=tmp*40+j;
                break;
            }
        }
    }

    *out=tmp;
    return 0;
}

//device config funcs
void dev_set_rx_freq(uint32_t freq)
{
	uint8_t cmd[6];
	cmd[0]=CMD_SET_RX_FREQ;		//RX freq
	cmd[1]=6;
	*((uint32_t*)&cmd[2])=freq;
	write(fd, cmd, cmd[1]);
	usleep(10000);
}

void dev_set_tx_freq(uint32_t freq)
{
	uint8_t cmd[6];
	cmd[0]=CMD_SET_TX_FREQ;		//TX freq
	cmd[1]=6;
	*((uint32_t*)&cmd[2])=freq;
	write(fd, cmd, cmd[1]);
	usleep(10000);
}

void dev_set_freq_corr(int16_t corr)
{
	uint8_t cmd[4];
	cmd[0]=CMD_SET_FREQ_CORR;	//freq correction
	cmd[1]=4;
	*((int16_t*)&cmd[2])=corr;
	write(fd, cmd, cmd[1]);
	usleep(10000);
}

void dev_set_tx_power(float power) //powr in dBm
{
	uint8_t cmd[3];
	cmd[0]=CMD_SET_TX_POWER;	//transmit power
	cmd[1]=3;
	cmd[2]=roundf(power*4.0f);
	write(fd, cmd, cmd[1]);
	usleep(10000);
}

int main(int argc, char* argv[])
{
	if(argc<4)
	{
		printf("Not enough params\nExiting\n");
		return 1;
	}

	printf("Connecting... ");

	//server
	serv_addr.sin_family = AF_INET;
	serv_addr.sin_addr.s_addr = inet_addr(argv[1]);
	serv_addr.sin_port = htons(PORT);

	//Create a socket
	sockt = socket(AF_INET, SOCK_DGRAM, 0);
	if(sockt<0)
	{
		printf("Socket error\nExiting\n");
		return 1;
	}
	memset((char*)&daddr, 0, sizeof(daddr));

	//encode M17 callsign from argv
	encode_callsign(&enc_callsign, (uint8_t*)argv[2]);

	//send "CONN"
	sprintf((char*)tx_buff, "CONN123456%s", argv[3]);
	for(uint8_t i=0; i<6; i++) //memcpy doesn't work here - endianness issue
		tx_buff[4+5-i]=*((uint8_t*)&enc_callsign+i);
	refl_send(tx_buff, 4+6+1);

	printf("done\n");

	fd=open((char*)"/dev/ttyS0", O_RDWR | O_NOCTTY | O_SYNC);
	set_blocking(fd, 0);
	set_interface_attribs(fd, B460800, 0);

	//config the device
	dev_set_rx_freq(433475000U);
	dev_set_tx_freq(435000000U);
	dev_set_freq_corr(-9);
	dev_set_tx_power(37.0f);

	while(1)
	{
		//Receive a packet
		saddr_size=sizeof(saddr);
		rx_len = recvfrom(sockt, rx_buff, MAX_UDP_LEN, 0, (struct sockaddr*)&saddr, (socklen_t*)&saddr_size);
		if(rx_len<0)
		{
			printf("Packets not recieved\n");
			return 1;
		}

		//debug
		//printf("Size:%d\nPayload:%s\n", rx_len, rx_buff);

		if(rx_buff[0]=='P' && rx_buff[1]=='I' && rx_buff[2]=='N' && rx_buff[3]=='G') //strstr() won't work here, as the PING string may occur in the payload
		{
			sprintf((char*)tx_buff, "PONG123456");
			for(uint8_t i=0; i<6; i++) //memcpy doesn't work here - endianness issue
				tx_buff[4+5-i]=*((uint8_t*)&enc_callsign+i);
			refl_send(tx_buff, 4+6); //PONG
		}
		else if(rx_buff[0]=='M' && rx_buff[1]=='1' && rx_buff[2]=='7' && rx_buff[3]==' ')
		{
			m17stream.sid=((uint16_t)rx_buff[4]<<8)|rx_buff[5];
			m17stream.fn=((uint16_t)rx_buff[34]<<8)|rx_buff[35];
			static uint8_t dst_call[10]={0};
			static uint8_t src_call[10]={0};

			if(m17stream.fn==0) //update LSF at FN=0
			{
				m17stream.lsf.dst=0;
				m17stream.lsf.src=0;
				for(uint8_t i=0; i<6; i++)
					m17stream.lsf.dst|=((uint64_t)rx_buff[6+i]<<((5-i)*8));
				for(uint8_t i=0; i<6; i++)
					m17stream.lsf.src|=((uint64_t)rx_buff[12+i]<<((5-i)*8));
				
				m17stream.lsf.type=((uint16_t)rx_buff[18]<<8)|rx_buff[19];

				memset((uint8_t*)m17stream.lsf.meta, 0, 14);
				for(uint8_t i=0; i<14; i++)
					m17stream.lsf.meta[i]=rx_buff[20+i];

				decode_callsign(dst_call, m17stream.lsf.dst);
				decode_callsign(src_call, m17stream.lsf.src);

				//test, short TX
				cmd[0]=7;
				cmd[1]=2;
				write(fd, cmd, cmd[1]);
			}
			
			printf("SID: %04X FN: %d DST: %s SRC: %s TYPE: %04X META: ",
					m17stream.sid, m17stream.fn&0x7FFFU, dst_call, src_call, m17stream.lsf.type);
			for(uint8_t i=0; i<14; i++)
				printf("%02X", m17stream.lsf.meta[i]);
			printf("\n");

			if(m17stream.fn&0x8000U)
				printf("Stream end\n");
		}

		memset((uint8_t*)rx_buff, 0, rx_len);
	}
	
	//should never get here	
	return 0;
}
