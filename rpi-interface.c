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
#include <time.h>

//rpi-interface commands
#include "interface_cmds.h"

//libm17
#include "libm17/m17.h"
#include "term.h" //colored terminal font
#define DEBUG_HALT				while(1);

#define PORT					17000
#define MAX_UDP_LEN				65535

#define nRST					17
#define PA_EN					18

#define SYMBOL_SCALING_COEFF	3.0f/(2.4f/(40.0e3f/2097152*0x9F)*129.0f) //CC1200 User's Guide, p. 24, 0x9F is `DEVIATION_M`, 2097152=2^21

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
struct config_t
{
	uint8_t log_path[128];
	uint8_t uart[64];
	uint32_t uart_rate;
	uint8_t node[15];
	uint8_t module;
	uint64_t enc_node;
	int16_t freq_corr;
	float tx_pwr;
	uint32_t rx_freq;
	uint32_t tx_freq;
	uint8_t afc;
} config;

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

int8_t flt_buff[8*5+1];						//length of this has to match RRC filter's length
float f_flt_buff[8*5+2*(8*5+4800/25*5)+2];	//8 preamble symbols, 8 for the syncword, and 960 for the payload.
											//floor(sps/2)=2 extra samples for timing error correction
enum rx_state_t rx_state=RX_IDLE;
int8_t lsf_sync_ext[16];					//extended LSF syncword
struct LSF lsf; 							//recovered LSF
uint16_t sample_cnt=0;						//sample counter (for RX sync timeout)
uint16_t fn, last_fn=0xFFFFU;				//current and last received FN
uint8_t lsf_b[30+1];						//raw decoded LSF (including 1 flushing byte)
uint8_t first_frame=1;						//first decoded frame after SYNC?
uint8_t lich_parts=0;						//LICH chunks received (bit flags)
uint8_t got_lsf=0;							//got LSF? either from LSF or reconstructed from LICH

uint8_t uart_byte_count;					//how many bytes are available on UART

//debug printf
void dbg_print(const char* color_code, const char* fmt, ...)
{
	char str[200];
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

int get_baud(uint32_t baud)
{
    switch(baud)
	{
		case 9600:
			return B9600;
		case 19200:
			return B19200;
		case 38400:
			return B38400;
		case 57600:
			return B57600;
		case 115200:
			return B115200;
		case 230400:
			return B230400;
		case 460800:
			return B460800;
		case 500000:
			return B500000;
		case 576000:
			return B576000;
		case 921600:
			return B921600;
		case 1000000:
			return B1000000;
		case 1152000:
			return B1152000;
		case 1500000:
			return B1500000;
		case 2000000:
			return B2000000;
		case 2500000:
			return B2500000;
		case 3000000:
			return B3000000;
		case 3500000:
			return B3500000;
		case 4000000:
			return B4000000;
		default: 
			return -1;
    }
}

int set_interface_attribs(int fd, uint32_t speed, int parity)
{
	struct termios tty;
	if (tcgetattr (fd, &tty) != 0)
	{
		//error_message ("error %d from tcgetattr", errno);
		return -1;
 	}

	cfsetospeed(&tty, get_baud(speed));
	cfsetispeed(&tty, get_baud(speed));

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

	if(tcsetattr(fd, TCSANOW, &tty)!=0)
	{		
		dbg_print(TERM_RED, " Error from tcsetattr\n");
		return -1;
	}
	
	return 0;
}

void set_blocking(int fd, int should_block)
{
	struct termios tty;
	memset(&tty, 0, sizeof tty);
	if(tcgetattr(fd, &tty)!=0)
	{
		dbg_print(TERM_YELLOW, " Error from tggetattr\n");
		return;
	}

	tty.c_cc[VMIN]  = should_block ? 1 : 0;
	tty.c_cc[VTIME] = 5;            // 0.5 seconds read timeout

	if(tcsetattr(fd, TCSANOW, &tty)!=0)
	{
		dbg_print(TERM_YELLOW, " Error setting UART attributes\n");
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

int8_t load_config(struct config_t *cfg, char *path)
{
	FILE* cfg_fp=fopen(path, "r");
	char line[128];

	//load defaults
	sprintf((char*)cfg->log_path, "/var/www/html/files/log.txt");
	sprintf((char*)cfg->uart, "/dev/ttyS0");
	cfg->uart_rate=460800;
	sprintf((char*)cfg->node, "N0CALL H");
	cfg->module='A';
	cfg->rx_freq=433475000U;
	cfg->tx_freq=435000000U;
	cfg->freq_corr=0;
	cfg->tx_pwr=37.0f;
	cfg->afc=0;

	//overwrite settings
	if(cfg_fp!=NULL)
	{
		//mindlessly load all the values, we will perform sanity checks later
		while(fgets((char*)line, sizeof(line), cfg_fp)>(char*)0)
		{
			if(strstr(line, "log_path")!=NULL)
			{
				memcpy((char*)&(cfg->uart), &line[strstr(line, "\"")-line+1], strstr(&line[strstr(line, "\"")-line+1], "\"")-&line[strstr(line, "\"")-line+1]);
				cfg->log_path[strstr(&line[strstr(line, "\"")-line+1], "\"")-&line[strstr(line, "\"")-line+1]]=0;
			}
			else if(strstr(line, "device")!=NULL)
			{
				memcpy((char*)&(cfg->uart), &line[strstr(line, "\"")-line+1], strstr(&line[strstr(line, "\"")-line+1], "\"")-&line[strstr(line, "\"")-line+1]);
				cfg->uart[strstr(&line[strstr(line, "\"")-line+1], "\"")-&line[strstr(line, "\"")-line+1]]=0;
			}
			else if(strstr(line, "speed")!=NULL)
			{
				cfg->uart_rate=atoi(&line[strstr(line, "=")-line+1]);
			}
			else if(strstr(line, "name")!=NULL)
			{
				memcpy((char*)&(cfg->node), &line[strstr(line, "\"")-line+1], strstr(&line[strstr(line, "\"")-line+1], "\"")-&line[strstr(line, "\"")-line+1]);
				cfg->node[strstr(&line[strstr(line, "\"")-line+1], "\"")-&line[strstr(line, "\"")-line+1]]=0;
			}
			else if(strstr(line, "module")!=NULL)
			{
				cfg->module=line[strstr(line, "\"")-line+1];
			}
			else if(strstr(line, "tx_freq")!=NULL)
			{
				cfg->tx_freq=atoi(&line[strstr(line, "=")-line+1]);
			}
			else if(strstr(line, "rx_freq")!=NULL)
			{
				cfg->rx_freq=atoi(&line[strstr(line, "=")-line+1]);
			}
			else if(strstr(line, "freq_corr")!=NULL)
			{
				cfg->freq_corr=atoi(&line[strstr(line, "=")-line+1]);
			}
			else if(strstr(line, "tx_pwr")!=NULL)
			{
				cfg->tx_pwr=atof(&line[strstr(line, "=")-line+1]);
			}
			else if(strstr(line, "afc")!=NULL)
			{
				if(line[strstr(line, "=")-line+1]=='1')
					cfg->afc=1;
				else
					cfg->afc=0;
			}
		}

		fclose(cfg_fp);
		return 0; //file read OK
	}
	else
	{
		return -1; //error reading file
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

uint8_t gpio_set(uint8_t gpio, uint8_t state)
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
		return 0;
	}
	else
	{
		dbg_print(TERM_YELLOW, " Error - can not set GPIO%d value\n", gpio);
		return 1;
	}
}

//M17 stuff
uint8_t refl_send(const uint8_t* msg, uint16_t len)
{
	if(sendto(sockt, msg, len, 0, (const struct sockaddr*)&serv_addr, sizeof(serv_addr))<0)
    {
        dbg_print(TERM_RED, " Error connecting with reflector.\nExiting.\n");
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
	usleep(5000U);
}

void dev_set_tx_freq(uint32_t freq)
{
	uint8_t cmd[6];
	cmd[0]=CMD_SET_TX_FREQ;		//TX freq
	cmd[1]=6;
	*((uint32_t*)&cmd[2])=freq;
	write(fd, cmd, cmd[1]);
	usleep(5000U);
}

void dev_set_freq_corr(int16_t corr)
{
	uint8_t cmd[4];
	cmd[0]=CMD_SET_FREQ_CORR;	//freq correction
	cmd[1]=4;
	*((int16_t*)&cmd[2])=corr;
	write(fd, cmd, cmd[1]);
	usleep(5000U);
}

void dev_set_afc(uint8_t en)
{
	uint8_t cmd[3];
	cmd[0]=CMD_SET_AFC;
	cmd[1]=3;
	cmd[2]=en?1:0;

	write(fd, cmd, cmd[1]);
	usleep(5000U);
}

void dev_set_tx_power(float power) //powr in dBm
{
	uint8_t cmd[3];
	cmd[0]=CMD_SET_TX_POWER;	//transmit power
	cmd[1]=3;
	cmd[2]=roundf(power*4.0f);
	write(fd, cmd, cmd[1]);
	usleep(5000U);
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
	if(argc==2)
	{
		if(strstr(argv[1], (char*)"-r")) //device reset
		{
			uint8_t gpio_err=0;
			gpio_init();
			gpio_err|=gpio_set(nRST, 0); //both pins should be at logic low already, but better be safe than sorry
			gpio_err|=gpio_set(PA_EN, 0);
			usleep(50000U); //50ms
			gpio_err|=gpio_set(nRST, 1);
			dbg_print(0, "Device reset");
			if(gpio_err)
				dbg_print(TERM_RED, " error\n");
			else
				dbg_print(TERM_GREEN, " OK\n");
			return (int)gpio_err;
		}
	}

	if(argc!=3)
	{
		dbg_print(TERM_RED, "Invalid params\nExiting\n");
		return 1;
	}

	srand(time(NULL));
	dbg_print(TERM_GREEN, "Starting up rpi-interface\n");

	//-----------------------------------config read-----------------------------------
	dbg_print(0, "Reading config file...");
	if(load_config(&config, argv[2])==0)
	{
		dbg_print(TERM_GREEN, " OK\n");
	}
	else
	{
		dbg_print(TERM_RED, " error reading %s\nExiting\n", argv[2]);
		return 1;
	}

	//basic sanity checks
	if(config.rx_freq<420000000U || config.rx_freq>450000000U)
	{
		dbg_print(TERM_RED, "Invalid RX frequency\nExiting\n");
		return 1;
	}
	if(config.tx_freq<420000000U || config.tx_freq>450000000U)
	{
		dbg_print(TERM_RED, "Invalid TX frequency\nExiting\n");
		return 1;
	}
	if(config.tx_pwr<0.0f || config.tx_pwr>47.0f)
	{
		dbg_print(TERM_RED, "Invalid TX power\nExiting\n");
		return 1;
	}

	dbg_print(0, "Storing traffic in %s\n", config.log_path);

	//------------------------------------gpio init------------------------------------
	dbg_print(0, "GPIO init...");
	uint8_t gpio_err=0;
	gpio_init();
	gpio_err|=gpio_set(nRST, 0); //both pins should be at logic low already, but better be safe than sorry
	gpio_err|=gpio_set(PA_EN, 0);
	usleep(50000U); //50ms
	gpio_err|=gpio_set(nRST, 1);
	usleep(1000000U); //1s for RRU boot-up
	if(gpio_err==0)
		dbg_print(TERM_GREEN, " OK\n");

	//-----------------------------------device part-----------------------------------
	dbg_print(0, "UART init: %s at %d...", (char*)config.uart, config.uart_rate);
	fd=open((char*)config.uart, O_RDWR | O_NOCTTY | O_SYNC);
	if(fd==0)
	{
		dbg_print(TERM_RED, " error\nExiting\n");
		exit(1);
	}
	
	set_blocking(fd, 0);
	set_interface_attribs(fd, config.uart_rate, 0);
	dbg_print(TERM_GREEN, " OK\n");

	//PING-PONG test
	dbg_print(0, "Device's reply to PING...");

	dev_ping();
	do
	{
		ioctl(fd, FIONREAD, &uart_byte_count);
	}
	while(uart_byte_count!=6);
	uint8_t ping_test[6]={0};
	read(fd, ping_test, 6);

	uint32_t dev_err=((uint32_t)ping_test[5]<<24)|((uint32_t)ping_test[4]<<16)|((uint32_t)ping_test[3]<<8)|ping_test[2];
	if(ping_test[0]==0 && ping_test[1]==6 && dev_err==0)
		dbg_print(TERM_GREEN, " PONG OK\n");
	else
	{
		dbg_print(TERM_YELLOW, " PONG error code: 0x%04X\n", dev_err);
		//return 1;
	}

	//config the device
	dbg_print(0, "RX frequency: %lu Hz\n", config.rx_freq); dev_set_rx_freq(config.rx_freq);
	dbg_print(0, "TX frequency: %lu Hz\n", config.tx_freq); dev_set_tx_freq(config.tx_freq);
	dbg_print(0, "Frequency correction: %d\n", config.freq_corr); dev_set_freq_corr(config.freq_corr);
	dbg_print(0, "TX power: %2.2f dBm\n", config.tx_pwr); dev_set_tx_power(config.tx_pwr);
	dbg_print(0, "AFC "); dev_set_afc(config.afc);
	if(config.afc)
		dbg_print(TERM_GREEN, "enabled\n");
	else
		dbg_print(TERM_YELLOW, "disabled\n");

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
	encode_callsign_value(&(config.enc_node), config.node);

	//send "CONN"
	sprintf((char*)tx_buff, "CONN123456%c", config.module);
	for(uint8_t i=0; i<6; i++) //memcpy doesn't work here - endianness issue
		tx_buff[4+5-i]=*((uint8_t*)&config.enc_node+i);
	refl_send(tx_buff, 4+6+1);
	dbg_print(TERM_GREEN, " OK\n");

	//extend the LSF syncword pattern with 8 symbols from the preamble
	lsf_sync_ext[0]=3; lsf_sync_ext[1]=-3; lsf_sync_ext[2]=3; lsf_sync_ext[3]=-3;
	lsf_sync_ext[4]=3; lsf_sync_ext[5]=-3; lsf_sync_ext[6]=3; lsf_sync_ext[7]=-3;
	memcpy(&lsf_sync_ext[8], lsf_sync_symbols, 8);

	//start RX
	dev_start_rx();
	dbg_print(0, "RX start\n");

	//UART comms
	int8_t rx_bsb_sample=0;

	float f_sample;
	//FILE *fp=fopen("test.out", "wb");

	//time
	time_t rawtime;
    struct tm * timeinfo;

	while(1)
	{
		//are there any new baseband samples to process?
		ioctl(fd, FIONREAD, &uart_byte_count);
		if(uart_byte_count>0)
		{
			read(fd, (uint8_t*)&rx_bsb_sample, 1);

			//push buffer
			for(uint8_t i=0; i<sizeof(flt_buff)-1; i++)
				flt_buff[i]=flt_buff[i+1];
			flt_buff[sizeof(flt_buff)-1]=rx_bsb_sample;

			f_sample=0.0f;
			for(uint8_t i=0; i<sizeof(flt_buff); i++)
				f_sample+=rrc_taps_5[i]*(float)flt_buff[i];
			f_sample*=SYMBOL_SCALING_COEFF; //map +104 to +3 (works for CC1200 only)

			for(uint16_t i=0; i<sizeof(f_flt_buff)/sizeof(float)-1; i++)
				f_flt_buff[i]=f_flt_buff[i+1];
			f_flt_buff[sizeof(f_flt_buff)/sizeof(float)-1]=f_sample;

			//L2 norm check against syncword
			float symbols[16];
			for(uint8_t i=0; i<15; i++)
				symbols[i]=f_flt_buff[i*5];
			symbols[15]=f_flt_buff[15*5];

			float dist_lsf=eucl_norm(&symbols[0], lsf_sync_ext, 16); //check against extended LSF syncword (8 symbols, alternating -3/+3)
			float dist_str_a=eucl_norm(&symbols[8], str_sync_symbols, 8);
			for(uint8_t i=0; i<15; i++)
				symbols[i]=f_flt_buff[960+i*5];
			symbols[15]=f_flt_buff[960+15*5];
			float dist_str_b=eucl_norm(&symbols[8], str_sync_symbols, 8);
			float dist_str=sqrtf(dist_str_a*dist_str_a+dist_str_b*dist_str_b);

			//fwrite(&dist_str, 4, 1, fp);
			if(dist_lsf<=4.5f && rx_state==RX_IDLE)
			{
				//find L2's minimum
				uint8_t sample_offset=0;
				for(uint8_t i=1; i<=2; i++)
				{
					for(uint8_t j=0; j<15; j++)
						symbols[j]=f_flt_buff[j*5+i];
					symbols[15]=f_flt_buff[15*5+i];
					float tmp=eucl_norm(&symbols[0], lsf_sync_ext, 16);
					if(tmp<dist_lsf)
						sample_offset=i;
				}

				float pld[SYM_PER_PLD];
				uint16_t soft_bit[2*SYM_PER_PLD], d_soft_bit[2*SYM_PER_PLD];

				for(uint16_t i=0; i<SYM_PER_PLD; i++)
				{
					pld[i]=f_flt_buff[16*5+i*5+sample_offset]; //add symbol timing correction
				}

				slice_symbols(soft_bit, pld);
				randomize_soft_bits(soft_bit);
				reorder_soft_bits(d_soft_bit, soft_bit);
				uint32_t e=viterbi_decode_punctured(lsf_b, d_soft_bit, puncture_pattern_1, 2*SYM_PER_PLD, sizeof(puncture_pattern_1));
				//shift the buffer 1 position left - get rid of the encoded flushing bits
                for(uint8_t i=0; i<30; i++)
                    lsf_b[i]=lsf_b[i+1];
				for(uint8_t i=0; i<6; i++)
				{
					lsf.dst[i]=lsf_b[5-i];
					lsf.src[i]=lsf_b[11-i];
				}
				lsf.type[0]=lsf_b[13];
				lsf.type[1]=lsf_b[12];

				uint8_t call_dst[10], call_src[10], can;
				decode_callsign_bytes(call_dst, lsf.dst);
                decode_callsign_bytes(call_src, lsf.src);
				can=(*((uint16_t*)lsf.type)>>7)&0xFU;

				time(&rawtime);
    			timeinfo=localtime(&rawtime);
				dbg_print(0, "[%02d:%02d:%02d]",
					timeinfo->tm_hour, timeinfo->tm_min, timeinfo->tm_sec);
				dbg_print(TERM_YELLOW, " RF LSF:");

				if(!CRC_M17(lsf_b, 30)) //if CRC valid
				{
					got_lsf=1;
					rx_state=RX_SYNCD;	//change RX state
					sample_cnt=0;		//reset rx timeout timer

					last_fn=0xFFFFU;

					m17stream.fn=0;
					m17stream.sid=rand()%0x10000U;

					uint8_t refl_pld[(32+16+224+16+128+16)/8];					//single frame
					sprintf((char*)&refl_pld[0], "M17 ");						//MAGIC
					*((uint16_t*)&refl_pld[4])=m17stream.sid;					//SID
					memcpy(&refl_pld[6], &lsf_b[0], 224/8);						//LSF
					*((uint16_t*)&refl_pld[34])=m17stream.fn;					//FN
					memset(&refl_pld[36], 0, 128/8);							//payload (zeros, because this is LSF)
					uint16_t crc_val=CRC_M17(refl_pld, 52);						//CRC
					*((uint16_t*)&refl_pld[52])=(crc_val>>8)|(crc_val<<8);		//endianness swap
					refl_send(refl_pld, sizeof(refl_pld));						//send a single frame to the reflector

					if(*((uint16_t*)lsf.type)&1) //if stream
					{
						dbg_print(TERM_GREEN, " CRC OK ");
						dbg_print(TERM_YELLOW, "| DST: %-9s | SRC: %-9s | CAN: %02d | MER: %-3.1f%%\n",
							call_dst, call_src, can, (float)e/0xFFFFU/SYM_PER_PLD/2.0f*100.0f);
						FILE* logfile=fopen((char*)config.log_path, "awb");
						if(logfile!=NULL)
						{
							time(&rawtime);
    						timeinfo=localtime(&rawtime);
							fprintf(logfile, "\"%02d:%02d:%02d\" \"%s\" \"%s\" \"RF\" \"%d\" \"%3.1f%%\"\n",
								timeinfo->tm_hour, timeinfo->tm_min, timeinfo->tm_sec,
								call_src, call_dst, can, (float)e/0xFFFFU/SYM_PER_PLD/2.0f*100.0f);
							fclose(logfile);
						}
					}
				}
				else
				{
					dbg_print(TERM_RED, " CRC ERR\n");
				}
			}
			else if(dist_str<=5.0f)
			{
				rx_state=RX_SYNCD;
				sample_cnt=0;		//reset rx timeout timer

				//find L2's minimum
				uint8_t sample_offset=0;
				for(uint8_t i=1; i<=2; i++)
				{
					for(uint8_t j=0; j<15; j++)
						symbols[j]=f_flt_buff[j*5+i];
					symbols[15]=f_flt_buff[15*5+i];
					float tmp_a=eucl_norm(&symbols[8], str_sync_symbols, 8);
					for(uint8_t j=0; j<15; j++)
						symbols[j]=f_flt_buff[960+j*5+i];
					symbols[15]=f_flt_buff[960+15*5+i];
					float tmp_b=eucl_norm(&symbols[8], str_sync_symbols, 8);

					if(sqrtf(tmp_a*tmp_a+tmp_b*tmp_b)<dist_str)
						sample_offset=i;
				}

				float pld[SYM_PER_PLD];
				uint16_t soft_bit[2*SYM_PER_PLD], d_soft_bit[2*SYM_PER_PLD];
				uint8_t frame_data[(16+128)/8+1]; //1 byte extra for flushing

				for(uint16_t i=0; i<SYM_PER_PLD; i++)
				{
					pld[i]=f_flt_buff[16*5+i*5+sample_offset];
				}

				slice_symbols(soft_bit, pld);
				randomize_soft_bits(soft_bit);
				reorder_soft_bits(d_soft_bit, soft_bit);

				//decode LICH
				uint8_t lich[6];
				decode_LICH(lich, d_soft_bit);
                uint8_t lich_cnt=lich[5]>>5;

				if(lich_parts!=0x3FU) //6 chunks = 0b111111
				{
					//reconstruct LSF chunk by chunk
					memcpy(&lsf_b[lich_cnt*5], lich, 40/8); //40 bits
					lich_parts|=(1<<lich_cnt);
					if(lich_parts==0x3FU && got_lsf==0) //collected all of them?
					{
						if(!CRC_M17(lsf_b, 30)) //CRC check
						{
							got_lsf=1;
							m17stream.sid=rand()%0x10000U;

							uint8_t call_dst[12]={0}, call_src[12]={0};
							uint8_t can=(*((uint16_t*)&lsf_b[12])>>7)&0xF;

							//swap order
							for(uint8_t i=0; i<3; i++)
							{
								uint8_t tmp;
								tmp=lsf_b[i]; lsf_b[i]=lsf_b[5-i]; lsf_b[5-i]=tmp;
								tmp=lsf_b[6+i]; lsf_b[6+i]=lsf_b[6+5-i]; lsf_b[6+5-i]=tmp;
							}

							decode_callsign_bytes(call_dst, &lsf_b[0]);
							decode_callsign_bytes(call_src, &lsf_b[6]);

							time(&rawtime);
							timeinfo=localtime(&rawtime);
							dbg_print(0, "[%02d:%02d:%02d] ",
								timeinfo->tm_hour, timeinfo->tm_min, timeinfo->tm_sec);
							dbg_print(TERM_YELLOW, "LSF REC: DST: %-9s | SRC: %-9s | CAN: %02d\n",
								call_dst, call_src, can);

							FILE* logfile=fopen((char*)config.log_path, "awb");
							if(logfile!=NULL)
							{
								time(&rawtime);
								timeinfo=localtime(&rawtime);
								fprintf(logfile, "\"%02d:%02d:%02d\" \"%s\" \"%s\" \"RF\" \"%d\" \"--\"\n",
									timeinfo->tm_hour, timeinfo->tm_min, timeinfo->tm_sec,
									call_src, call_dst, can);
								fclose(logfile);
							}
						}
						else
						{
							lich_parts=0; //reset flags
						}
					}
				}

				uint16_t enc_data[272];
				for(uint16_t i=0; i<272; i++)
                {
                    enc_data[i]=d_soft_bit[96+i];
                }

				uint32_t e=viterbi_decode_punctured(frame_data, enc_data, puncture_pattern_2, 2*SYM_PER_PLD-96, sizeof(puncture_pattern_2));
				//shift the buffer 1 position left - get rid of the encoded flushing bits
                for(uint8_t i=0; i<19-1; i++)
                    frame_data[i]=frame_data[i+1];
				fn=(frame_data[0]<<8)|frame_data[1];
				
				//set the last FN number to FN-1 if this is a late-join and the frame data is valid
				if(first_frame==1 && (fn%6)==lich_cnt)
				{
					last_fn=fn-1;
				}
				
				if(((last_fn+1)&0xFFFFU)==fn) //TODO: maybe a timeout would be better
				{
					time(&rawtime);
    				timeinfo=localtime(&rawtime);

					dbg_print(0, "[%02d:%02d:%02d]",
						timeinfo->tm_hour, timeinfo->tm_min, timeinfo->tm_sec);
					dbg_print(TERM_YELLOW, " RF FRM: ");
					dbg_print(TERM_YELLOW, " FN:%04X | LICH_CNT:%d", fn, lich_cnt);
					/*dbg_print(TERM_YELLOW, " | PLD: ");
					for(uint8_t i=0; i<128/8; i++)
						dbg_print(TERM_YELLOW, "%02X", frame_data[2+i]);*/
					dbg_print(TERM_YELLOW, " | MER: %-3.1f%%\n",
						(float)e/0xFFFFU/SYM_PER_PLD/2.0f*100.0f);

					if(got_lsf)
					{
						m17stream.fn=fn;
						uint8_t refl_pld[(32+16+224+16+128+16)/8];					//single frame
						sprintf((char*)&refl_pld[0], "M17 ");						//MAGIC
						*((uint16_t*)&refl_pld[4])=m17stream.sid;					//SID
						memcpy(&refl_pld[6], &lsf_b[0], 224/8);						//LSF
						*((uint16_t*)&refl_pld[34])=m17stream.fn;					//FN
						memcpy(&refl_pld[36], &frame_data[2], 128/8);				//payload (zeros)
						uint16_t crc_val=CRC_M17(refl_pld, 52);						//CRC
						*((uint16_t*)&refl_pld[52])=(crc_val>>8)|(crc_val<<8);		//endianness swap
						refl_send(refl_pld, sizeof(refl_pld));						//send a single frame to the reflector
					}

					last_fn=fn;
				}

				first_frame=0;
			}
			
			//RX sync timeout
			if(rx_state==RX_SYNCD)
			{
				sample_cnt++;
				if(sample_cnt==960*2)
				{
					rx_state=RX_IDLE;
					sample_cnt=0;
					first_frame=1;
					last_fn=0xFFFFU;
					lich_parts=0;
					got_lsf=0;
				}
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
					tx_buff[4+5-i]=*((uint8_t*)&config.enc_node+i);
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

				time(&rawtime);
    			timeinfo=localtime(&rawtime);

				dbg_print(TERM_YELLOW, "[%02d:%02d:%02d] NET FRM: ",
						timeinfo->tm_hour, timeinfo->tm_min, timeinfo->tm_sec);
				dbg_print(TERM_YELLOW, "SID: %04X | FN: %04X | DST: %-9s | SRC: %-9s | TYPE: %04X | META: ",
						m17stream.sid, m17stream.fn&0x7FFFU, dst_call, src_call, *((uint16_t*)m17stream.lsf.type));
				for(uint8_t i=0; i<14; i++)
					dbg_print(TERM_YELLOW, "%02X", m17stream.lsf.meta[i]);
				dbg_print(TERM_YELLOW, "\n");

				if(m17stream.fn==0U)
				{
					FILE* logfile=fopen((char*)config.log_path, "awb");
					if(logfile!=NULL)
					{
						time(&rawtime);
    					timeinfo=localtime(&rawtime);

						fprintf(logfile, "\"%02d:%02d:%02d\" \"%s\" \"%s\" \"Internet\" \"--\" \"--\"\n",
							timeinfo->tm_hour, timeinfo->tm_min, timeinfo->tm_sec,
							src_call, dst_call);
						fclose(logfile);
					}
				}

				if(m17stream.fn&0x8000U)
				{
					time(&rawtime);
    				timeinfo=localtime(&rawtime);

					dbg_print(TERM_YELLOW, "[%02d:%02d:%02d] Stream end\n",
						timeinfo->tm_hour, timeinfo->tm_min, timeinfo->tm_sec);
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
