/*
 * rpi-interface.c
 *
 *  Created on: Dec 27, 2023
 *      Author: SP5WWP
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <math.h>
#include <stdarg.h>

#include <netinet/ip_icmp.h>
#include <netinet/udp.h>
#include <netinet/tcp.h>
#include <netinet/ip.h>
#include <sys/socket.h>
#include <arpa/inet.h>

#include <fcntl.h> 
#include <termios.h>
#include <unistd.h>
#include <sys/ioctl.h>

//rpi-interface commands
#include "interface_cmds.h"

//libm17
#include <m17/m17.h>
#include "term.h" //colored terminal font

#define PORT		17000
#define MAX_UDP_LEN	65535

#define nRST		17
#define PA_EN		18

//internet
struct sockaddr_in source, dest; 
int sockt;
struct iphdr *iph;
struct sockaddr_in saddr;
struct sockaddr_in daddr;
struct sockaddr_in serv_addr;
uint32_t saddr_size=sizeof(saddr);

uint8_t tx_buff[512]={0};
uint8_t rx_buff[65536]={0};
int tx_len=0, rx_len=0;
int socket_byte_count=0; //data available for reading at the socket

//config stuff
uint8_t cfg_uart[64]={0}, cfg_call[15]={0}, cfg_module[5]={0};
uint64_t enc_callsign=0;

//device stuff
uint8_t cmd[8];

//M17
struct m17stream_t
{
	uint16_t sid;
	struct LSF lsf;
	uint16_t fn;
	uint8_t pld[16];
} m17stream;

enum rx_state_t
{
	RX_IDLE,
	RX_SYNCD
};

int8_t flt_buff[8*5+1];				//length of this has to match RRC filter's length
float f_flt_buff[1000];				//this can be longer
enum rx_state_t rx_state=RX_IDLE;

//debug printf
void dbg_print(const char* color_code, const char* fmt, ...)
{
	char str[100];
	va_list ap;

	va_start(ap, fmt);
	vsprintf(str, fmt, ap);
	va_end(ap);

	if(color_code!=NULL)
	{
		printf(color_code);
		printf(str);
		printf(TERM_DEFAULT);
	}
	else
	{
		printf(str);
	}
}

//UART magic
int fd; //UART handle

int set_interface_attribs(int fd, int speed, int parity)
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
	tty.c_cc[VTIME] = 0;            // 0.0 seconds read timeout

	tty.c_iflag &= ~(IXON | IXOFF | IXANY); // shut off xon/xoff ctrl

	tty.c_cflag |= (CLOCAL | CREAD);// ignore modem controls,
                                        // enable reading
	tty.c_cflag &= ~(PARENB | PARODD);      // shut off parity
	tty.c_cflag |= parity;
	tty.c_cflag &= ~CSTOPB;
	tty.c_cflag &= ~CRTSCTS;

	if(tcsetattr(fd, TCSANOW, &tty) != 0)
	{
		dbg_print(TERM_RED, "Error from tcsetattr\n");
		return -1;
	}
	
	return 0;
}

void set_blocking(int fd, int should_block)
{
	struct termios tty;
	memset(&tty, 0, sizeof tty);
	if(tcgetattr(fd, &tty) != 0)
	{
		dbg_print(TERM_RED, "Error from tggetattr\n");
		return;
	}

	tty.c_cc[VMIN]  = should_block ? 1 : 0;
	tty.c_cc[VTIME] = 5;            // 0.5 seconds read timeout

	if(tcsetattr(fd, TCSANOW, &tty) != 0)
	{
		dbg_print(TERM_YELLOW, "Error setting UART attributes\n");
	}
}

/**
 * @brief Replaces the first character with ASCII code under 0x20 with 0x00 (null termination).
 * rtrim() scans the input string left to right.
 * 
 * @param inp Pointer to a string with text to trim.
 */
void rtrim(uint8_t* inp)
{
	for(uint8_t i=0; i<strlen((char*)inp); i++)
	{
		if(inp[i]<' ')
		{
			inp[i]=0;
			break;
		}
	}
}

//GPIO - library-less, guerrilla style - we assume that only GPIO17 and 18 will be used
void gpio_init(void)
{
	FILE* fp;
	
	//enable GPIO17 and GPIO18
	fp=fopen("/sys/class/gpio/export", "wb");
	if(fp!=NULL)
	{
		fwrite("17", 2, 1, fp);
		fclose(fp);
	}
	else
	{
		dbg_print(TERM_RED, " can not initialize GPIO\nExiting\n");
		exit(1);
	}

	fp=fopen("/sys/class/gpio/export", "wb");
	if(fp!=NULL)
	{
		fwrite("18", 2, 1, fp);
		fclose(fp);
	}
	else
	{
		dbg_print(TERM_RED, " can not initialize GPIO\nExiting\n");
		exit(1);
	}

	usleep(250000U); //give it 250ms

	//set as output, default value is logic low
	fp=fopen("/sys/class/gpio/gpio17/direction", "wb");
	if(fp!=NULL)
	{
		fwrite("out", 3, 1, fp);
		fclose(fp);
	}
	else
	{
		dbg_print(TERM_RED, " can not initialize GPIO\nExiting\n");
		exit(1);
	}

	fp=fopen("/sys/class/gpio/gpio18/direction", "wb");
	if(fp!=NULL)
	{
		fwrite("out", 3, 1, fp);
		fclose(fp);
	}
	else
	{
		dbg_print(TERM_RED, " can not initialize GPIO\nExiting\n");
		exit(1);
	}
}

void gpio_set(uint8_t gpio, uint8_t state)
{
	FILE* fp=NULL;

	switch(gpio)
	{
		case 17:
			fp=fopen("/sys/class/gpio/gpio17/value", "wb");
		break;

		case 18:
			fp=fopen("/sys/class/gpio/gpio18/value", "wb");
		break;

		default:
			;
		break;
	}

	if(fp!=NULL)
	{
		if(state)
		{
			fwrite("1", 1, 1, fp);
		}
		else
		{
			fwrite("0", 1, 1, fp);
		}

		fclose(fp);
	}
	else
	{
		dbg_print(TERM_YELLOW, "Error - can not set GPIO%d value\n", gpio);
	}
}

//M17 stuff
uint8_t refl_send(const uint8_t* msg, uint16_t len)
{
	if(sendto(sockt, msg, len, 0, (const struct sockaddr*)&serv_addr, sizeof(serv_addr))<0)
    {
        dbg_print(TERM_RED, "Error connecting with reflector.\nExiting.\n");
        return 1;
    }

	return 0;
}

//device config funcs
void dev_ping(void)
{
	uint8_t cmd[2];
	cmd[0]=CMD_PING;			//PING
	cmd[1]=2;
	write(fd, cmd, cmd[1]);
}

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

void dev_set_afc(uint8_t en)
{
	uint8_t cmd[3];
	cmd[0]=CMD_SET_AFC;
	cmd[1]=3;
	cmd[2]=en?1:0;

	write(fd, cmd, cmd[1]);
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

void dev_start_tx(void)
{
	uint8_t cmd[3];
	cmd[0]=CMD_SET_TX_START;	//start tranmission
	cmd[1]=2;
	write(fd, cmd, cmd[1]);
}

void dev_start_rx(void)
{
	uint8_t cmd[3];
	cmd[0]=CMD_SET_RX;			//start reception
	cmd[1]=3;
	cmd[2]=1;
	write(fd, cmd, cmd[1]);
}

void dev_stop_rx(void)
{
	uint8_t cmd[3];
	cmd[0]=CMD_SET_RX;			//stop reception
	cmd[1]=3;
	cmd[2]=0;
	write(fd, cmd, cmd[1]);
}

int main(int argc, char* argv[])
{
	if(argc!=3)
	{
		dbg_print(TERM_RED, "Invalid params\nExiting\n");
		return 1;
	}

	dbg_print(TERM_GREEN, "Starting up rpi-interface\n");

	//-----------------------------------config read-----------------------------------
	dbg_print(0, "Reading config file...");
	FILE* cfg_fp=fopen(argv[2], "r");
	if(cfg_fp!=NULL)
	{
		fgets((char*)cfg_uart, sizeof(cfg_uart), cfg_fp);
		fgets((char*)cfg_call, sizeof(cfg_call), cfg_fp);
		fgets((char*)cfg_module, sizeof(cfg_module), cfg_fp);
		rtrim(cfg_uart);
		rtrim(cfg_call);
		fclose(cfg_fp);
		dbg_print(TERM_GREEN, " OK\n");
	}
	else
	{
		dbg_print(TERM_RED, " error reading %s\nExiting\n", argv[2]);
		return 1;
	}

	//------------------------------------gpio init------------------------------------
	dbg_print(0, "GPIO init...");
	gpio_init();
	gpio_set(nRST, 0); //both pins should be at logic low already, but better be safe than sorry
	gpio_set(PA_EN, 0);
	usleep(50000U); //50ms
	gpio_set(nRST, 1);
	usleep(1000000U); //1s for RRU boot-up
	dbg_print(TERM_GREEN, " OK\n");

	//-----------------------------------device part-----------------------------------
	dbg_print(0, "UART init (%s)...", (char*)cfg_uart);
	fd=open((char*)cfg_uart, O_RDWR | O_NOCTTY | O_SYNC);
	if(fd==0)
	{
		dbg_print(TERM_RED, " error\nExiting\n");
		exit(1);
	}
	
	set_blocking(fd, 0);
	set_interface_attribs(fd, B460800, 0);
	dbg_print(TERM_GREEN, " OK\n");

	//PING-PONG test
	dbg_print(0, "Device's reply to PING...");
	uint8_t trash;
	while(read(fd, &trash, 1)); //read all trash

	uint8_t ping_test[3];
	dev_ping();
	while(read(fd, &ping_test[0], 1)==0);
	while(read(fd, &ping_test[1], 1)==0);
	while(read(fd, &ping_test[2], 1)==0);
	if(ping_test[0]==0 && ping_test[1]==3 && ping_test[2]==0)
		dbg_print(TERM_GREEN, " OK\n");
	else
	{
		dbg_print(TERM_YELLOW, " PONG error code: %d\n", ping_test[2]);
		//return 1;
	}

	//config the device
	dbg_print(0, "Configuring device... ");
	dev_set_rx_freq(433475000U);
	dev_set_tx_freq(435000000U);
	dev_set_freq_corr(-9);
	dev_set_tx_power(37.0f);
	dbg_print(TERM_GREEN, " done\n");

	dev_set_afc(1);
	dbg_print(TERM_GREEN, "AFC enabled\n");

	//-----------------------------------internet part-----------------------------------
	dbg_print(0, "Connecting to %s", argv[1]);

	//server
	serv_addr.sin_family = AF_INET;
	serv_addr.sin_addr.s_addr = inet_addr(argv[1]);
	serv_addr.sin_port = htons(PORT);

	//Create a socket
	sockt = socket(AF_INET, SOCK_DGRAM, 0);
	if(sockt<0)
	{
		dbg_print(TERM_RED, "\nSocket error\nExiting\n");
		return 1;
	}
	memset((char*)&daddr, 0, sizeof(daddr));

	//encode M17 callsign from argv
	encode_callsign_value(&enc_callsign, cfg_call);

	//send "CONN"
	sprintf((char*)tx_buff, "CONN123456%c", cfg_module[0]);
	for(uint8_t i=0; i<6; i++) //memcpy doesn't work here - endianness issue
		tx_buff[4+5-i]=*((uint8_t*)&enc_callsign+i);
	refl_send(tx_buff, 4+6+1);

	dbg_print(TERM_GREEN, " OK\n");

	//start RX
	dev_start_rx();
	dbg_print(0, "RX start\n");

	//UART comms
	int8_t rx_bsb_sample=0;

	float f_sample;
	FILE *fp=fopen("test.out", "wb");

	while(1)
	{
		if(read(fd, (uint8_t*)&rx_bsb_sample, 1)==1)
		{
			//push buffer
			for(uint8_t i=0; i<sizeof(flt_buff)-1; i++)
				flt_buff[i]=flt_buff[i+1];
			flt_buff[sizeof(flt_buff)-1]=rx_bsb_sample;

			f_sample=0.0f;
			for(uint8_t i=0; i<sizeof(flt_buff); i++)
				f_sample+=rrc_taps_5[i]*(float)flt_buff[i];
			f_sample*=0.028846153846154f; //map +104 to +3 (works for CC1200 only)

			for(uint16_t i=0; i<sizeof(f_flt_buff)/sizeof(float)-1; i++)
				f_flt_buff[i]=f_flt_buff[i+1];
			f_flt_buff[sizeof(f_flt_buff)/sizeof(float)-1]=f_sample;

			//L2 norm check against syncword
			float symbols[16];
			for(uint8_t i=0; i<15; i++)
				symbols[i]=f_flt_buff[i*5];
			symbols[15]=f_flt_buff[15*5];

			int8_t stream[16];
			stream[0]=3; stream[1]=-3; stream[2]=3; stream[3]=-3;
			stream[4]=3; stream[5]=-3; stream[6]=3; stream[7]=-3;
			memcpy(&stream[8], lsf_sync_symbols, 8);
			float dist_lsf=eucl_norm(symbols, stream, 16);
			float dist_str=eucl_norm(&symbols[8], str_sync_symbols, 8);

			fwrite(&dist_str, 4, 1, fp);
			if(dist_lsf<=4.5f)
			{
				dbg_print(TERM_YELLOW, "LSF syncword detected, %1.2f\n", dist_lsf);
				;
			}
			else if(dist_str<=2.0f)
			{
				dbg_print(TERM_YELLOW, "Stream syncword detected\n");
				;
			}
				
		}

		//receive a packet - non-blocking
		ioctl(sockt, FIONREAD, &socket_byte_count);
		if(socket_byte_count>0)
		{
			rx_len = recvfrom(sockt, rx_buff, MAX_UDP_LEN, 0, (struct sockaddr*)&saddr, (socklen_t*)&saddr_size);

			//debug
			//dbg_print(0, "Size:%d\nPayload:%s\n", rx_len, rx_buff);

			if(strstr((char*)rx_buff, "PING")==(char*)rx_buff)
			{
				sprintf((char*)tx_buff, "PONG123456"); //that "123456" is just a placeholder
				for(uint8_t i=0; i<6; i++) //memcpy doesn't work here - endianness issue
					tx_buff[4+5-i]=*((uint8_t*)&enc_callsign+i);
				refl_send(tx_buff, 4+6); //PONG
				memset((uint8_t*)rx_buff, 0, rx_len);
				//dbg_print(TERM_YELLOW, "PING\n");
			}
			else if(strstr((char*)rx_buff, "M17 ")==(char*)rx_buff)
			{
				m17stream.sid=((uint16_t)rx_buff[4]<<8)|rx_buff[5];
				m17stream.fn=((uint16_t)rx_buff[34]<<8)|rx_buff[35];
				static uint8_t dst_call[10]={0};
				static uint8_t src_call[10]={0};

				if(m17stream.fn==0) //update LSF at FN=0
				{
					//exytract data with correct endianness
					for(uint8_t i=0; i<6; i++)
						m17stream.lsf.dst[i]=rx_buff[6+5-i];
					for(uint8_t i=0; i<6; i++)
						m17stream.lsf.src[i]=rx_buff[12+5-i];

					m17stream.lsf.type[1]=rx_buff[18];
					m17stream.lsf.type[0]=rx_buff[19];

					memset((uint8_t*)m17stream.lsf.meta, 0, 14);
					for(uint8_t i=0; i<14; i++)
						m17stream.lsf.meta[i]=rx_buff[20+i];

					decode_callsign_bytes(dst_call, m17stream.lsf.dst);
					decode_callsign_bytes(src_call, m17stream.lsf.src);

					//set PA_EN=1 and initialize TX
					//gpio_set(PA_EN, 1);
					dev_start_tx();
				}
				
				int8_t samples[960];
				memset(samples, 0, 960);
				write(fd, (uint8_t*)samples, 960);

				printf("SID: %04X FN: %04X DST: %s SRC: %s TYPE: %04X META: ",
						m17stream.sid, m17stream.fn&0x7FFFU, dst_call, src_call, *((uint16_t*)m17stream.lsf.type));
				for(uint8_t i=0; i<14; i++)
					printf("%02X", m17stream.lsf.meta[i]);
				printf("\n");

				if(m17stream.fn&0x8000U)
				{
					printf("Stream end\n");
					usleep(200000U); //wait 200ms (5 M17 frames)
					gpio_set(PA_EN, 0);
				}

				memset((uint8_t*)rx_buff, 0, rx_len);
			}
		}
	}
	
	//should never get here	
	return 0;
}
