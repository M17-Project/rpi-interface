/*
 * rpi-interface.c
 *
 *  Edited on: Oct 11, 2024
 *     Author: Wojciech Kaczmarski, SP5WWP
 *             M17 Project
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
#include <signal.h>

#include <zmq.h>

//rpi-interface commands
#include "interface_cmds.h"

//libm17
#include "libm17/m17.h"
#include "term.h" //colored terminal font
#define DEBUG_HALT				while(1)

#define PORT					17000										//should be fine for most of the reflectors TODO: maybe parametrize this?
#define MAX_UDP_LEN				65535
#define ZMQ_RX_BUFF_SIZE		960											//how many RX baseband samples do we want to publish over ZMQ at once?

#define RX_SYMBOL_SCALING_COEFF	(1.0f/(0.8f/(40.0e3f/2097152*0xAD)*129.0f))	//CC1200 User's Guide, p. 24
																			//0xAD is `DEVIATION_M`, 2097152=2^21
																			//+1.0 is the symbol for +0.8kHz
																			//40.0e3 is F_TCXO in kHz
																			//129 is `CFM_RX_DATA_OUT` register value at max. F_DEV
																			//datasheet might have this wrong (it says 64)
#define TX_SYMBOL_SCALING_COEFF	(0.8f/((40.0e3f/2097152)*0xAD)*64.0f)		//0xAD is `DEVIATION_M`, 2097152=2^21
																			//+0.8kHz is the deviation for symbol +1
																			//40.0e3 is F_TCXO in kHz
																			//64 is `CFM_TX_DATA_IN` register value for max. F_DEV

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
char refl_addr[20]={0}; //reflector's IP address

//config stuff
struct config_t
{
	uint8_t log_path[128];
	uint8_t uart[64];
	uint32_t uart_rate;
	uint8_t node[10];
	uint8_t reflector[8]; //TODO: add IP address field
	uint8_t module;
	uint8_t enc_node[6];
	int16_t freq_corr;
	float tx_pwr;
	uint32_t rx_freq;
	uint32_t tx_freq;
	uint8_t afc;
	//GPIO
	uint16_t pa_en;
	uint16_t boot0;
	uint16_t nrst;
} config;

//device stuff
uint8_t cmd[8];

//M17
struct m17stream_t
{
	uint16_t sid;
	lsf_t lsf;
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
lsf_t lsf; 									//recovered LSF
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
	sprintf((char*)cfg->uart, "/dev/ttyAMA0");
	cfg->uart_rate=460800;
	sprintf((char*)cfg->node, "N0CALL H");
	cfg->module='A';
	cfg->rx_freq=433475000U;
	cfg->tx_freq=435000000U;
	cfg->freq_corr=0;
	cfg->tx_pwr=37.0f;
	cfg->afc=0;
	cfg->nrst=17;
	cfg->pa_en=18;
	cfg->boot0=27;

	//overwrite settings
	if(cfg_fp!=NULL)
	{
		//mindlessly load all the values, we will perform sanity checks later
		while(fgets((char*)line, sizeof(line), cfg_fp)>(char*)0)
		{
			uint8_t len;
			if(strstr(line, "log_path")!=NULL)
			{
				len=strstr(strstr(line, "\"")+1, "\"")-strstr(line, "\"")-1;
				memcpy((char*)&(cfg->log_path), strstr(line, "\"")+1, len);
				cfg->log_path[len]=0;
			}
			else if(strstr(line, "device")!=NULL)
			{
				len=strstr(strstr(line, "\"")+1, "\"")-strstr(line, "\"")-1;
				memcpy((char*)&(cfg->uart), strstr(line, "\"")+1, len);
				cfg->uart[len]=0;
			}
			else if(strstr(line, "speed")!=NULL)
			{
				cfg->uart_rate=atoi(strstr(line, "=")+1);
			}
			else if(strstr(line, "node")!=NULL)
			{
				len=strstr(strstr(line, "\"")+1, "\"")-strstr(line, "\"")-1;
				memcpy((char*)&(cfg->node), strstr(line, "\"")+1, len);
				cfg->node[len]=0;
			}
			else if(strstr(line, "refelctor")!=NULL)
			{
				len=strstr(strstr(line, "\"")+1, "\"")-strstr(line, "\"")-1;
				memcpy((char*)&(cfg->reflector), strstr(line, "\"")+1, len);
				cfg->reflector[len]=0;
			}
			else if(strstr(line, "module")!=NULL)
			{
				cfg->module=*(strstr(line, "\"")+1);
			}

			else if(strstr(line, "nrst")!=NULL)
			{
				cfg->nrst=atoi(strstr(line, "=")+1);
			}
			else if(strstr(line, "pa_en")!=NULL)
			{
				cfg->pa_en=atoi(strstr(line, "=")+1);
			}
			else if(strstr(line, "boot0")!=NULL)
			{
				cfg->boot0=atoi(strstr(line, "=")+1);
			}

			else if(strstr(line, "tx_freq")!=NULL)
			{
				cfg->tx_freq=atoi(strstr(line, "=")+1);
			}
			else if(strstr(line, "rx_freq")!=NULL)
			{
				cfg->rx_freq=atoi(strstr(line, "=")+1);
			}
			else if(strstr(line, "freq_corr")!=NULL)
			{
				cfg->freq_corr=atoi(strstr(line, "=")+1);
			}
			else if(strstr(line, "tx_pwr")!=NULL)
			{
				cfg->tx_pwr=atof(strstr(line, "=")+1);
			}
			else if(strstr(line, "afc")!=NULL)
			{
				if(*(strstr(line, "=")+1)=='1')
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

//GPIO - library-less, guerrilla style - we assume that only 3 GPIOs will be used
void gpio_init(void)
{
	FILE* fp;
	char tmp[256];
	
	//enable
	fp=fopen("/sys/class/gpio/export", "wb");
	if(fp!=NULL)
	{
		sprintf(tmp, "%d", config.pa_en);
		fwrite(tmp, strlen(tmp), 1, fp);
		fclose(fp);
	}
	else
	{
		dbg_print(TERM_RED, " can not initialize PA_EN (GPIO%d)\nExiting\n", config.pa_en);
		exit(1);
	}

	fp=fopen("/sys/class/gpio/export", "wb");
	if(fp!=NULL)
	{
		sprintf(tmp, "%d", config.boot0);
		fwrite(tmp, strlen(tmp), 1, fp);
		fclose(fp);
	}
	else
	{
		dbg_print(TERM_RED, " can not initialize BOOT0 (GPIO%d)\nExiting\n", config.boot0);
		exit(1);
	}
	
	fp=fopen("/sys/class/gpio/export", "wb");
	if(fp!=NULL)
	{
		sprintf(tmp, "%d", config.nrst);
		fwrite(tmp, strlen(tmp), 1, fp);
		fclose(fp);
	}
	else
	{
		dbg_print(TERM_RED, " can not initialize nRST (GPIO%d)\nExiting\n", config.nrst);
		exit(1);
	}

	usleep(250000U); //give it 250ms

	//set as output, default value is logic low
	sprintf(tmp, "/sys/class/gpio/gpio%d/direction", config.pa_en);
	fp=fopen(tmp, "wb");
	if(fp!=NULL)
	{
		fwrite("out", 3, 1, fp);
		fclose(fp);
	}
	else
	{
		dbg_print(TERM_RED, " can not initialize PA_EN (GPIO%d)\nExiting\n");
		exit(1);
	}

	sprintf(tmp, "/sys/class/gpio/gpio%d/direction", config.boot0);
	fp=fopen(tmp, "wb");
	if(fp!=NULL)
	{
		fwrite("out", 3, 1, fp);
		fclose(fp);
	}
	else
	{
		dbg_print(TERM_RED, " can not initialize BOOT0 (GPIO%d)\nExiting\n");
		exit(1);
	}

	sprintf(tmp, "/sys/class/gpio/gpio%d/direction", config.nrst);
	fp=fopen(tmp, "wb");
	if(fp!=NULL)
	{
		fwrite("out", 3, 1, fp);
		fclose(fp);
	}
	else
	{
		dbg_print(TERM_RED, " can not initialize nRST (GPIO%d)\nExiting\n");
		exit(1);
	}
}

uint8_t gpio_set(uint16_t gpio, uint8_t state)
{
	FILE* fp=NULL;
	char tmp[256];

	sprintf(tmp, "/sys/class/gpio/gpio%d/value", gpio);
	fp=fopen(tmp, "wb");

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
	
	//wait for device's response
	do
	{
		ioctl(fd, FIONREAD, &uart_byte_count);
	}
	while(uart_byte_count!=3);
	uint8_t resp[3]={0};
	read(fd, resp, 3);
	
	if(resp[2]==0)
	{
		dbg_print(0, "RX frequency: %lu Hz\n", config.rx_freq); //OK
	}
	else
	{
		dbg_print(TERM_YELLOW, "Error %d setting RX frequency: %lu Hz\n", resp[2], config.rx_freq); //error
	}
}

void dev_set_tx_freq(uint32_t freq)
{
	uint8_t cmd[6];
	cmd[0]=CMD_SET_TX_FREQ;		//TX freq
	cmd[1]=6;
	*((uint32_t*)&cmd[2])=freq;
	write(fd, cmd, cmd[1]);

	//wait for device's response
	do
	{
		ioctl(fd, FIONREAD, &uart_byte_count);
	}
	while(uart_byte_count!=3);
	uint8_t resp[3]={0};
	read(fd, resp, 3);
	
	if(resp[2]==0)
	{
		dbg_print(0, "TX frequency: %lu Hz\n", config.tx_freq); //OK
	}
	else
	{
		dbg_print(TERM_YELLOW, "Error %d setting TX frequency: %lu Hz\n", resp[2], config.tx_freq); //error
	}
}

void dev_set_freq_corr(int16_t corr)
{
	uint8_t cmd[4];
	cmd[0]=CMD_SET_FREQ_CORR;	//freq correction
	cmd[1]=4;
	*((int16_t*)&cmd[2])=corr;
	write(fd, cmd, cmd[1]);

	//wait for device's response
	do
	{
		ioctl(fd, FIONREAD, &uart_byte_count);
	}
	while(uart_byte_count!=3);
	uint8_t resp[3]={0};
	read(fd, resp, 3);
	
	if(resp[2]==0)
	{
		dbg_print(0, "Frequency correction: %d\n", config.freq_corr); //OK
	}
	else
	{
		dbg_print(TERM_YELLOW, "Error %d setting frequency correction: %d\n", resp[2], config.freq_corr); //OK //error
	}
}

void dev_set_afc(uint8_t en)
{
	uint8_t cmd[3];
	cmd[0]=CMD_SET_AFC;
	cmd[1]=3;
	cmd[2]=en?1:0;

	write(fd, cmd, cmd[1]);

	//wait for device's response
	do
	{
		ioctl(fd, FIONREAD, &uart_byte_count);
	}
	while(uart_byte_count!=3);
	uint8_t resp[3]={0};
	read(fd, resp, 3);
	
	if(resp[2]==0)
	{
		; //OK
	}
	else
	{
		; //error
	}
}

void dev_set_tx_power(float power) //powr in dBm
{
	uint8_t cmd[3];
	cmd[0]=CMD_SET_TX_POWER;	//transmit power
	cmd[1]=3;
	cmd[2]=roundf(power*4.0f);
	write(fd, cmd, cmd[1]);

	//wait for device's response
	do
	{
		ioctl(fd, FIONREAD, &uart_byte_count);
	}
	while(uart_byte_count!=3);
	uint8_t resp[3]={0};
	read(fd, resp, 3);
	
	if(resp[2]==0)
	{
		dbg_print(0, "TX power: %2.2f dBm\n", config.tx_pwr); //OK
	}
	else
	{
		dbg_print(TERM_YELLOW, "Error %d setting TX power: %2.2f dBm\n", resp[2], config.tx_pwr); //error
	}
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

void sigint_handler(int val)
{
	if(val){}; //get rid of unused variable warning
	dbg_print(TERM_YELLOW, "\nSIGINT caught, disconnecting\n");
	sprintf((char*)tx_buff, "DISCxxxxxx"); //that "xxxxxx" is just a placeholder
	memcpy(&tx_buff[4], config.enc_node, sizeof(config.enc_node));
	refl_send(tx_buff, 4+6); //DISC
	dbg_print(TERM_YELLOW, "Exiting\n");
	exit(0);
}

int main(int argc, char* argv[])
{
	signal(SIGINT, sigint_handler);

	if(argc<3)
	{
		dbg_print(TERM_RED, "Invalid params\nExiting\n");
		return 1;
	}

	//-----------------------------------args parse------------------------------------
	uint8_t reset=0;
	for(uint8_t i=1; i<argc; i++)
	{
		if(argv[i][0]=='-')
		{
			if(argv[i][1]=='r') //device reset
			{
				reset=1; //reset pending
			}
			else if(argv[i][1]=='i') //reflector's address
			{
				memcpy(refl_addr, argv[i+1], strlen(argv[i+1]));
				i++;
			}
			else if(argv[i][1]=='c') //config file
			{
				dbg_print(0, "Config:");
				if(load_config(&config, argv[i+1])==0)
				{
					dbg_print(TERM_GREEN, " OK\n");
					i++; //skip next arg
				}
				else
				{
					dbg_print(TERM_RED, " error reading %s\nExiting\n", argv[i+1]);
					return 1;
				}
			}
		}
	}

	//reset the device and exit
	if(reset)
	{
		dbg_print(0, "Device reset...");
		uint8_t gpio_err=0;
		gpio_init();
		gpio_err|=gpio_set(config.boot0, 0); //all pins should be at logic low already, but better be safe than sorry
		gpio_err|=gpio_set(config.pa_en, 0);
		gpio_err|=gpio_set(config.nrst, 0);
		usleep(50000U); //50ms
		gpio_err|=gpio_set(config.nrst, 1);

		if(gpio_err)
			dbg_print(TERM_RED, " error\n");
		else
			dbg_print(TERM_GREEN, " OK\n");
		
		return (int)gpio_err;
	}

	//---------------------------config's basic sanity checks--------------------------
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
	if(config.tx_pwr<0.0f || config.tx_pwr>47.75f)
	{
		dbg_print(TERM_RED, "Invalid TX power\nExiting\n");
		return 1;
	}

	srand(time(NULL));
	dbg_print(TERM_GREEN, "Starting up rpi-interface\n");

	//check write access to the log file
	FILE* logfile=fopen((char*)config.log_path, "awb");
	if(logfile!=NULL)
	{
		dbg_print(0, "Storing traffic in %s\n", config.log_path);
		fclose(logfile);
	}
	else
	{
		dbg_print(TERM_RED, "Cannot access %s\nExiting\n", config.log_path);
		return 1;
	}

	//------------------------------------gpio init------------------------------------
	dbg_print(0, "GPIO init...");
	uint8_t gpio_err=0;
	gpio_init();
	gpio_err|=gpio_set(config.nrst, 0); //both pins should be at logic low already, but better be safe than sorry
	usleep(50000U); //50ms
	gpio_err|=gpio_set(config.nrst, 1);
	usleep(1000000U); //1s for device boot-up
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

	uint32_t dev_err=*((uint32_t*)&ping_test[2]);
	if(ping_test[0]==0 && ping_test[1]==6 && dev_err==0)
		dbg_print(TERM_GREEN, " PONG OK\n");
	else
	{
		dbg_print(TERM_YELLOW, " PONG error code: 0x%04X\n", dev_err);
		//return 1;
	}

	//config the device
	dev_set_rx_freq(config.rx_freq);
	dev_set_tx_freq(config.tx_freq);
	dev_set_freq_corr(config.freq_corr);
	dev_set_tx_power(config.tx_pwr);
	dbg_print(0, "AFC "); dev_set_afc(config.afc);
	if(config.afc)
		dbg_print(TERM_GREEN, "enabled\n");
	else
		dbg_print(TERM_YELLOW, "disabled\n");

	//-----------------------------------internet part-----------------------------------
	dbg_print(0, "Connecting to %s", refl_addr);

	//server
	serv_addr.sin_family = AF_INET;
	serv_addr.sin_addr.s_addr = inet_addr(refl_addr);
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
	encode_callsign_bytes(config.enc_node, config.node);

	//send "CONN"
	sprintf((char*)tx_buff, "CONNxxxxxx%c", config.module);
	memcpy(&tx_buff[4], config.enc_node, sizeof(config.enc_node));
	refl_send(tx_buff, 4+6+1);
	dbg_print(TERM_GREEN, " OK\n");

	//extend the LSF syncword pattern with 8 symbols from the preamble
	lsf_sync_ext[0]=3; lsf_sync_ext[1]=-3; lsf_sync_ext[2]=3; lsf_sync_ext[3]=-3;
	lsf_sync_ext[4]=3; lsf_sync_ext[5]=-3; lsf_sync_ext[6]=3; lsf_sync_ext[7]=-3;
	memcpy(&lsf_sync_ext[8], lsf_sync_symbols, 8);

	//ZMQ
	void *zmq_ctx = zmq_ctx_new();
    void *bsb_downlink = zmq_socket(zmq_ctx, ZMQ_PUB);
	dbg_print(0, "ZeroMQ ");
	if(zmq_bind(bsb_downlink, "tcp://*:17017")==0) //TODO: make the port number configurable
		dbg_print(TERM_GREEN, "OK\n");
	else
		dbg_print(TERM_RED, "ERROR\n");
	int8_t zmq_samp_buff[ZMQ_RX_BUFF_SIZE];
	uint16_t zmq_samples=0;

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

			//publish over ZMQ
			zmq_samp_buff[zmq_samples++]=rx_bsb_sample;
			if(zmq_samples==ZMQ_RX_BUFF_SIZE)
			{
				zmq_send(bsb_downlink, (char*)zmq_samp_buff, ZMQ_RX_BUFF_SIZE, 0);
				zmq_samples=0;
			}

			//push buffer
			for(uint8_t i=0; i<sizeof(flt_buff)-1; i++)
				flt_buff[i]=flt_buff[i+1];
			flt_buff[sizeof(flt_buff)-1]=rx_bsb_sample;

			f_sample=0.0f;
			for(uint8_t i=0; i<sizeof(flt_buff); i++)
				f_sample+=rrc_taps_5[i]*(float)flt_buff[i];
			f_sample*=RX_SYMBOL_SCALING_COEFF; //symbol map (works for CC1200 only)

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

				//copy SRC and DST as-is, big-endian
				memcpy(lsf.dst, &lsf_b[0], 6);
				memcpy(lsf.src, &lsf_b[6], 6);

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
						dbg_print(TERM_YELLOW, "| DST: %-9s | SRC: %-9s | TYPE: %04X (CAN=%d) | MER: %-3.1f%%\n",
							call_dst, call_src, *((uint16_t*)lsf.type), can, (float)e/0xFFFFU/SYM_PER_PLD/2.0f*100.0f);

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
				
				if(((last_fn+1)&0xFFFFU)==fn) //new frame. TODO: maybe a timeout would be better
				{
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
								dbg_print(TERM_YELLOW, "LSF CRC ERR\n");
								lich_parts=0; //reset flags
							}
						}
					}

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
						m17stream.fn=(fn>>8)|((fn&0xFF)<<8);
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
				sprintf((char*)tx_buff, "PONGxxxxxx"); //that "xxxxxx" is just a placeholder
				memcpy(&tx_buff[4], config.enc_node, sizeof(config.enc_node));
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
				memcpy(m17stream.pld, &rx_buff[(32+16+224+16)/8U], 128/8);

				if(m17stream.fn==0U) //update some parts of the LSF at FN=0
				{
					//dev_stop_rx();
					//dbg_print(0, "RX stop\n");
					usleep(10*1000U);

					//extract data
					memcpy(m17stream.lsf.dst, "\xFF\xFF\xFF\xFF\xFF\xFF", 6);
					memcpy(m17stream.lsf.src, &rx_buff[6+6], 6);
					decode_callsign_bytes(dst_call, m17stream.lsf.dst);
					decode_callsign_bytes(src_call, m17stream.lsf.src);

					//set TYPE field
					memcpy(m17stream.lsf.type, &rx_buff[18], 2);
					m17stream.lsf.type[1]|=0x2U<<5; //no encryption, so the subtype field defines the META field contents: extended callsign data

					//generate META field
					//remove trailing spaces and suffixes
					uint8_t trimmed_src[12], enc_trimmed_src[6];
					for(uint8_t i=0; i<12; i++)
					{
						if(src_call[i]!=' ')
							trimmed_src[i]=src_call[i];
						else
						{
							trimmed_src[i]=0;
							break;
						}
					}
					encode_callsign_bytes(enc_trimmed_src, trimmed_src);

					uint8_t ext_ref[12], enc_ext_ref[6];
					sprintf((char*)ext_ref, "M17-M17 %c", config.module); //hardcoded for now
					encode_callsign_bytes(enc_ext_ref, ext_ref);

					memcpy(&m17stream.lsf.meta[0], m17stream.lsf.src, 6); //originator
					memcpy(&m17stream.lsf.meta[6], enc_ext_ref, 6); //reflector
					memset(&m17stream.lsf.meta[12], 0, 2);
					memcpy(m17stream.lsf.src, enc_trimmed_src, 6);

					//append CRC
					uint16_t ccrc=LSF_CRC(&m17stream.lsf);
            		m17stream.lsf.crc[0]=ccrc>>8;
            		m17stream.lsf.crc[1]=ccrc&0xFF;

					//stop RX, set PA_EN=1 and initialize TX
					dev_stop_rx();
					usleep(2*1000U);
					gpio_set(config.pa_en, 1);
					dev_start_tx();
					usleep(10*1000U);
				}
				
				//generate frame symbols, filter them and send out to the device
				float frame_symbols[SYM_PER_FRA];	//raw frame symbols
				uint32_t frame_buff_cnt;			//buffer counter
				int8_t samples[SYM_PER_FRA*5];		//samples=symbols*sps
				uint8_t lich[6];                    //48 bits packed raw, unencoded LICH
				uint8_t lich_encoded[12];           //96 bits packed, encoded LICH
				uint8_t enc_bits[SYM_PER_PLD*2];    //type-2 bits, unpacked
				uint8_t rf_bits[SYM_PER_PLD*2];     //type-4 bits, unpacked
				static float flt_buff[sizeof(rrc_taps_5)/sizeof(float)-1 + SYM_PER_FRA*5]; //lookback samples plus a whole frame
				if(m17stream.fn==0U)
				{
					//we need to prepare 3 frames to begin the transmission - preamble, LSF and stream frame 0
					//let's start with the preamble
					frame_buff_cnt=0;
					send_preamble(frame_symbols, &frame_buff_cnt, 0); //0 - LSF preamble
					//filter and send out to the device
					memset((uint8_t*)&flt_buff[0], 0U, sizeof(rrc_taps_5)-sizeof(float)); //lookback
					for(uint16_t i=0; i<SYM_PER_FRA; i++)
					{
						for(uint8_t j=0; j<5; j++) //upsample
						{
							if(j==0)
								flt_buff[sizeof(rrc_taps_5)/sizeof(float)-1 + 5*i]=(float)frame_symbols[i];
							else
								flt_buff[sizeof(rrc_taps_5)/sizeof(float)-1 + 5*i+j]=0.0f;
						}
					}
					for(uint16_t i=0; i<sizeof(samples); i++)
					{
						float acc=0.0f;
						for(uint16_t j=0; j<sizeof(rrc_taps_5)/sizeof(float); j++)
						{
							acc+=flt_buff[i+j]*rrc_taps_5[j];
						}
						samples[i]=acc*TX_SYMBOL_SCALING_COEFF*sqrtf(5.0f); //crank up the gain
					}
					write(fd, (uint8_t*)samples, sizeof(samples));

					//now the LSF
					frame_buff_cnt=0;
					send_syncword(frame_symbols, &frame_buff_cnt, SYNC_LSF);
					conv_encode_LSF(enc_bits, &(m17stream.lsf));
					reorder_bits(rf_bits, enc_bits);
					randomize_bits(rf_bits);
					send_data(frame_symbols, &frame_buff_cnt, rf_bits);
					//filter and send out to the device
					memcpy((uint8_t*)&flt_buff[0], (uint8_t*)&flt_buff[sizeof(flt_buff)/sizeof(float)-(sizeof(rrc_taps_5)/sizeof(float)-1)], sizeof(rrc_taps_5)-sizeof(float)); //lookback
					for(uint16_t i=0; i<SYM_PER_FRA; i++)
					{
						for(uint8_t j=0; j<5; j++) //upsample
						{
							if(j==0)
								flt_buff[sizeof(rrc_taps_5)/sizeof(float)-1 + 5*i]=(float)frame_symbols[i];
							else
								flt_buff[sizeof(rrc_taps_5)/sizeof(float)-1 + 5*i+j]=0.0f;
						}
					}
					for(uint16_t i=0; i<sizeof(samples); i++)
					{
						float acc=0.0f;
						for(uint16_t j=0; j<sizeof(rrc_taps_5)/sizeof(float); j++)
						{
							acc+=flt_buff[i+j]*rrc_taps_5[j];
						}
						samples[i]=acc*TX_SYMBOL_SCALING_COEFF*sqrtf(5.0f); //crank up the gain
					}
					write(fd, (uint8_t*)samples, sizeof(samples));

					//finally, frame 0
					frame_buff_cnt=0;
					send_syncword(frame_symbols, &frame_buff_cnt, SYNC_STR);
					extract_LICH(lich, 0, &(m17stream.lsf)); //m17stream.fn % 6 = 0
					encode_LICH(lich_encoded, lich);
					unpack_LICH(enc_bits, lich_encoded);
					conv_encode_stream_frame(&enc_bits[96], m17stream.pld, 0); //m17stream.fn = 0
					reorder_bits(rf_bits, enc_bits);
            		randomize_bits(rf_bits);
					send_data(frame_symbols, &frame_buff_cnt, rf_bits);
					//filter and send out to the device
					memcpy((uint8_t*)&flt_buff[0], (uint8_t*)&flt_buff[sizeof(flt_buff)/sizeof(float)-(sizeof(rrc_taps_5)/sizeof(float)-1)], sizeof(rrc_taps_5)-sizeof(float)); //lookback
					for(uint16_t i=0; i<SYM_PER_FRA; i++)
					{
						for(uint8_t j=0; j<5; j++) //upsample
						{
							if(j==0)
								flt_buff[sizeof(rrc_taps_5)/sizeof(float)-1 + 5*i]=(float)frame_symbols[i];
							else
								flt_buff[sizeof(rrc_taps_5)/sizeof(float)-1 + 5*i+j]=0.0f;
						}
					}
					for(uint16_t i=0; i<sizeof(samples); i++)
					{
						float acc=0.0f;
						for(uint16_t j=0; j<sizeof(rrc_taps_5)/sizeof(float); j++)
						{
							acc+=flt_buff[i+j]*rrc_taps_5[j];
						}
						samples[i]=acc*TX_SYMBOL_SCALING_COEFF*sqrtf(5.0f); //crank up the gain
					}
					write(fd, (uint8_t*)samples, sizeof(samples));
				}
				else
				{
					//only one frame is needed
					frame_buff_cnt=0;
					send_syncword(frame_symbols, &frame_buff_cnt, SYNC_STR);
					extract_LICH(lich, m17stream.fn%6, &(m17stream.lsf)); //m17stream.fn % 6 - LICH_CNT
					encode_LICH(lich_encoded, lich);
					unpack_LICH(enc_bits, lich_encoded);
					conv_encode_stream_frame(&enc_bits[96], m17stream.pld, m17stream.fn);
					reorder_bits(rf_bits, enc_bits);
            		randomize_bits(rf_bits);
					send_data(frame_symbols, &frame_buff_cnt, rf_bits);
					//filter and send out to the device
					memcpy((uint8_t*)&flt_buff[0], (uint8_t*)&flt_buff[sizeof(flt_buff)/sizeof(float)-(sizeof(rrc_taps_5)/sizeof(float)-1)], sizeof(rrc_taps_5)-sizeof(float)); //lookback
					for(uint16_t i=0; i<SYM_PER_FRA; i++)
					{
						for(uint8_t j=0; j<5; j++) //upsample
						{
							if(j==0)
								flt_buff[sizeof(rrc_taps_5)/sizeof(float)-1 + 5*i]=(float)frame_symbols[i];
							else
								flt_buff[sizeof(rrc_taps_5)/sizeof(float)-1 + 5*i+j]=0.0f;
						}
					}
					for(uint16_t i=0; i<sizeof(samples); i++)
					{
						float acc=0.0f;
						for(uint16_t j=0; j<sizeof(rrc_taps_5)/sizeof(float); j++)
						{
							acc+=flt_buff[i+j]*rrc_taps_5[j];
						}
						samples[i]=acc*TX_SYMBOL_SCALING_COEFF*sqrtf(5.0f); //crank up the gain
					}
					write(fd, (uint8_t*)samples, sizeof(samples));
				}

				time(&rawtime);
    			timeinfo=localtime(&rawtime);

				dbg_print(TERM_YELLOW, "[%02d:%02d:%02d] NET FRM: ",
						timeinfo->tm_hour, timeinfo->tm_min, timeinfo->tm_sec);
				dbg_print(TERM_YELLOW, "SID: %04X | FN: %04X | DST: %-9s | SRC: %-9s | TYPE: %04X | META: ",
						m17stream.sid, m17stream.fn&0x7FFFU, dst_call, src_call, ((uint16_t)m17stream.lsf.type[0]<<8)|m17stream.lsf.type[1]);
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

				if(m17stream.fn&0x8000U) //last stream frame
				{
					//two frames need to be sent - last stream frame and EOT marker
					//last frame
					frame_buff_cnt=0;
					send_syncword(frame_symbols, &frame_buff_cnt, SYNC_STR);
					extract_LICH(lich, (m17stream.fn&0x7FFFU)%6, &(m17stream.lsf)); //(m17stream.fn&0x7FFF) % 6 - LICH_CNT
					encode_LICH(lich_encoded, lich);
					unpack_LICH(enc_bits, lich_encoded);
					conv_encode_stream_frame(&enc_bits[96], m17stream.pld, m17stream.fn);
					reorder_bits(rf_bits, enc_bits);
            		randomize_bits(rf_bits);
					send_data(frame_symbols, &frame_buff_cnt, rf_bits);
					//filter and send out to the device
					memcpy((uint8_t*)&flt_buff[0], (uint8_t*)&flt_buff[sizeof(flt_buff)/sizeof(float)-(sizeof(rrc_taps_5)/sizeof(float)-1)], sizeof(rrc_taps_5)-sizeof(float)); //lookback
					for(uint16_t i=0; i<SYM_PER_FRA; i++)
					{
						for(uint8_t j=0; j<5; j++) //upsample
						{
							if(j==0)
								flt_buff[sizeof(rrc_taps_5)/sizeof(float)-1 + 5*i]=(float)frame_symbols[i];
							else
								flt_buff[sizeof(rrc_taps_5)/sizeof(float)-1 + 5*i+j]=0.0f;
						}
					}
					for(uint16_t i=0; i<sizeof(samples); i++)
					{
						float acc=0.0f;
						for(uint16_t j=0; j<sizeof(rrc_taps_5)/sizeof(float); j++)
						{
							acc+=flt_buff[i+j]*rrc_taps_5[j];
						}
						samples[i]=acc*TX_SYMBOL_SCALING_COEFF*sqrtf(5.0f); //crank up the gain
					}
					write(fd, (uint8_t*)samples, sizeof(samples));

					//now the final EOT marker
					frame_buff_cnt=0;
					send_eot(frame_symbols, &frame_buff_cnt);
					//filter and send out to the device
					memcpy((uint8_t*)&flt_buff[0], (uint8_t*)&flt_buff[sizeof(flt_buff)/sizeof(float)-(sizeof(rrc_taps_5)/sizeof(float)-1)], sizeof(rrc_taps_5)-sizeof(float)); //lookback
					for(uint16_t i=0; i<SYM_PER_FRA; i++)
					{
						for(uint8_t j=0; j<5; j++) //upsample
						{
							if(j==0)
								flt_buff[sizeof(rrc_taps_5)/sizeof(float)-1 + 5*i]=(float)frame_symbols[i];
							else
								flt_buff[sizeof(rrc_taps_5)/sizeof(float)-1 + 5*i+j]=0.0f;
						}
					}
					for(uint16_t i=0; i<sizeof(samples); i++)
					{
						float acc=0.0f;
						for(uint16_t j=0; j<sizeof(rrc_taps_5)/sizeof(float); j++)
						{
							acc+=flt_buff[i+j]*rrc_taps_5[j];
						}
						samples[i]=acc*TX_SYMBOL_SCALING_COEFF*sqrtf(5.0f); //crank up the gain
					}
					write(fd, (uint8_t*)samples, sizeof(samples));

					time(&rawtime);
    				timeinfo=localtime(&rawtime);

					dbg_print(TERM_YELLOW, "[%02d:%02d:%02d] Stream end\n",
						timeinfo->tm_hour, timeinfo->tm_min, timeinfo->tm_sec);
					usleep(10*40000U); //wait 400ms (10 M17 frames)
					
					//disable TX
					gpio_set(config.pa_en, 0);

					//restart RX
					dev_start_rx();
					time(&rawtime);
    				timeinfo=localtime(&rawtime);
					dbg_print(TERM_YELLOW, "[%02d:%02d:%02d] RX start\n",
						timeinfo->tm_hour, timeinfo->tm_min, timeinfo->tm_sec);
				}

				memset((uint8_t*)rx_buff, 0, rx_len);
			}
		}
	}
	
	//should never get here	
	return 0;
}
