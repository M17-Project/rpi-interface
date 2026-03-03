/*
 * rpi-interface.c
 *
 * Edited on: Mar 3, 2026
 * Author: Wojciech Kaczmarski, SP5WWP
 *         M17 Foundation
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
#include <gpiod.h>

// rpi-interface commands
#include "interface_cmds.h"

// libm17
#include <m17.h>

#include "term.h" //colored terminal font

#define DEBUG_HALT while (1)

#define MAX_UDP_LEN 65535
#define ZMQ_RX_BUFF_SIZE 1024 // how many RX baseband samples do we want to publish over ZMQ at once?

#define RX_SYMBOL_SCALING_COEFF (1.0f / (0.8f / (40.0e3f / 2097152 * 0xAD) * 130.0f)) // CC1200 User's Guide, p. 24
																					  // 0xAD is `DEVIATION_M`, 2097152=2^21
																					  //+1.0 is the symbol for +0.8kHz
																					  // 40.0e3 is F_TCXO in kHz
																					  // 129 is `CFM_RX_DATA_OUT` register value at max. F_DEV (130 is 1 off but offers a better symbol map)
																					  // datasheet might have this wrong (it says 64)
#define TX_SYMBOL_SCALING_COEFF (0.8f / ((40.0e3f / 2097152) * 0xAD) * 64.0f)		  // 0xAD is `DEVIATION_M`, 2097152=2^21
																					  //+0.8kHz is the deviation for symbol +1
																					  // 40.0e3 is F_TCXO in kHz
																					  // 64 is `CFM_TX_DATA_IN` register value for max. F_DEV

// internet
struct sockaddr_in source, dest;
int sockt;
struct iphdr *iph;
struct sockaddr_in saddr;
struct sockaddr_in daddr;
struct sockaddr_in serv_addr;
uint32_t saddr_size = sizeof(saddr);

uint8_t tx_buff[512] = {0};
uint8_t rx_buff[65536] = {0};
int tx_len = 0, rx_len = 0;

// config stuff
struct config_t
{
	char log_path[128];
	char uart[64];
	uint32_t uart_rate;
	char node[10];
	char refl_addr[20];
	uint16_t refl_port;
	char reflector[8];
	char module;
	uint8_t enc_node[6];
	int16_t freq_corr;
	float tx_pwr;
	uint32_t rx_freq;
	uint32_t tx_freq;
	uint8_t afc;
	uint16_t zmq_port;

	// GPIO Pins
	uint16_t pa_en;
	uint16_t boot0;
	uint16_t nrst;

	// GPIO resources (handles)
	struct gpiod_chip *gpio_chip;
	struct gpiod_line_request *pa_en_req;
	struct gpiod_line_request *boot0_req;
	struct gpiod_line_request *nrst_req;
} config;

// device stuff
uint8_t cmd[8];

// M17
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

enum tx_state_t
{
	TX_IDLE,
	TX_ACTIVE
};

enum err_t
{
	ERR_OK,			 // all good
	ERR_TRX_PLL,	 // TRX PLL lock error
	ERR_TRX_SPI,	 // TRX SPI comms error
	ERR_RANGE,		 // value out of range
	ERR_CMD_MALFORM, // malformed command
	ERR_BUSY,		 // busy!
	ERR_BUFF_FULL,	 // buffer full
	ERR_NOP,		 // nothing to do
	ERR_OTHER
};

int8_t flt_buff[8 * 5 + 1];								   // length of this has to match RRC filter's length
float f_flt_buff[8 * 5 + 2 * (8 * 5 + 4800 / 25 * 5) + 2]; // 8 preamble symbols, 8 for the syncword, and 960 for the payload.
														   // floor(sps/2)=2 extra samples for timing error correction

uint8_t rx_samp_buff[1024];
int8_t raw_bsb_rx[960];
uint16_t rx_buff_cnt;
uint8_t uart_rx_sync;
uint8_t uart_rx_data_valid;
volatile uint8_t uart_lock;

enum rx_state_t rx_state = RX_IDLE;
enum tx_state_t tx_state = TX_IDLE;

int8_t lsf_sync_ext[16];			// extended LSF syncword
lsf_t lsf;							// recovered LSF
uint16_t sample_cnt = 0;			// sample counter (for RX sync timeout)
uint16_t fn, last_fn = 0xFFFFU;		// current and last received FN (stream mode)
uint8_t pkt_fn, last_pkt_fn = 0xFF; // current and last received FN (packet mode)
uint8_t lsf_b[30];					// raw decoded LSF
uint8_t first_frame = 1;			// first decoded frame after SYNC?
uint8_t lich_parts = 0;				// LICH chunks received (bit flags)
uint8_t got_lsf = 0;				// got LSF? either from LSF or reconstructed from LICH
const int8_t eot_symbols[8] = {+3, +3, +3, +3, +3, +3, -3, +3};

// timer for timeouts
uint32_t tx_timer = 0;

// log traffic to file
FILE *logfile = NULL;

// ZMQ PUB for the baseband
char zmq_addr[64];
void *zmq_ctx;
void *bsb_downlink;
int8_t zmq_samp_buff[ZMQ_RX_BUFF_SIZE];
uint16_t zmq_samples = 0;

time_t last_refl_ping;

// debug printf
void dbg_print(const char *color_code, const char *fmt, ...)
{
	char str[1000]; // 1k chars is probably an overkill, but - oh well :)
	va_list ap;

	va_start(ap, fmt);
	vsprintf(str, fmt, ap);
	va_end(ap);

	if (color_code != NULL)
	{
		fputs(color_code, stdout);
		fputs(str, stdout);
		fputs(TERM_DEFAULT, stdout);
	}
	else
	{
		fputs(str, stdout);
	}
}

void move_cursor(uint8_t x, uint8_t y)
{
	printf("\033[%d;%dH", y, x);
}

uint32_t get_ms(void)
{
	struct timespec spec;

	clock_gettime(CLOCK_REALTIME, &spec);

	time_t s = spec.tv_sec;
	uint32_t ms = roundf(spec.tv_nsec / 1.0e6); // convert nanoseconds to milliseconds
	if (ms > 999)
	{
		s++;
		ms = 0;
	}

	return s * 1000 + ms;
}

// UART magic
int fd; // UART handle

int get_baud(uint32_t baud)
{
	switch (baud)
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
	if (tcgetattr(fd, &tty) != 0)
	{
		dbg_print(TERM_YELLOW, " Error from tcgetattr\n");
		exit(1);
	}

	cfsetospeed(&tty, get_baud(speed));
	cfsetispeed(&tty, get_baud(speed));

	tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8; // 8-bit chars
	// disable IGNBRK for mismatched speed tests; otherwise receive break
	// as \000 chars
	tty.c_iflag &= ~IGNBRK; // disable break processing
	tty.c_lflag = 0;		// no signaling chars, no echo,
							// no canonical processing
	tty.c_oflag = 0;		// no remapping, no delays
	tty.c_cc[VMIN] = 1;		// read returns when 1 byte available
	tty.c_cc[VTIME] = 5;	// 5*0.5=0.5 seconds read timeout

	tty.c_iflag &= ~(IXON | IXOFF | IXANY); // shut off xon/xoff ctrl

	tty.c_cflag |= (CLOCAL | CREAD);   // ignore modem controls,
									   // enable reading
	tty.c_cflag &= ~(PARENB | PARODD); // shut off parity
	tty.c_cflag |= parity;
	tty.c_cflag &= ~CSTOPB;
	tty.c_cflag &= ~CRTSCTS;

	if (tcsetattr(fd, TCSANOW, &tty) != 0)
	{
		dbg_print(TERM_RED, " Error from tcsetattr\n");
		exit(1);
	}

	return 0;
}

/**
 * @brief Replaces the first character with ASCII code under 0x20 with 0x00 (null termination).
 * rtrim() scans the input string left to right.
 *
 * @param inp Pointer to a string with text to trim.
 */
void rtrim(uint8_t *inp)
{
	for (uint8_t i = 0; i < strlen((char *)inp); i++)
	{
		if (inp[i] < ' ')
		{
			inp[i] = 0;
			break;
		}
	}
}

int8_t load_config(struct config_t *cfg, char *path)
{
	FILE *cfg_fp = fopen(path, "r");
	char line[128];

	// load defaults
	cfg->log_path[0] = 0; // empty string - disabled
	sprintf(cfg->uart, "/dev/ttyAMA0");
	cfg->uart_rate = 460800;
	sprintf(cfg->node, "N0CALL H");
	sprintf(cfg->refl_addr, "152.70.192.70");
	cfg->refl_port = 17000;
	sprintf(cfg->reflector, "M17-M17");
	cfg->module = 'A';
	cfg->rx_freq = 433475000U;
	cfg->tx_freq = 433475000U;
	cfg->freq_corr = 0;
	cfg->tx_pwr = 10.0f;
	cfg->afc = 0;
	cfg->zmq_port = 0; // 0 - disabled
	cfg->nrst = 21;
	cfg->pa_en = 18;
	cfg->boot0 = 20;

	// overwrite settings
	if (cfg_fp != NULL)
	{
		// mindlessly load all the values, we will perform sanity checks later
		while (fgets((char *)line, sizeof(line), cfg_fp) > (char *)0)
		{
			uint8_t len;
			if (strstr(line, "log_path") == line)
			{
				len = strstr(strstr(line, "\"") + 1, "\"") - strstr(line, "\"") - 1;
				memcpy(cfg->log_path, strstr(line, "\"") + 1, len);
				cfg->log_path[len] = 0;
			}
			else if (strstr(line, "device") == line)
			{
				len = strstr(strstr(line, "\"") + 1, "\"") - strstr(line, "\"") - 1;
				memcpy(cfg->uart, strstr(line, "\"") + 1, len);
				cfg->uart[len] = 0;
			}
			else if (strstr(line, "speed") == line)
			{
				cfg->uart_rate = atoi(strstr(line, "=") + 1);
			}
			else if (strstr(line, "node") == line)
			{
				len = strstr(strstr(line, "\"") + 1, "\"") - strstr(line, "\"") - 1;
				memcpy(cfg->node, strstr(line, "\"") + 1, len);
				cfg->node[len] = 0;
			}
			else if (strstr(line, "ipv4") == line)
			{
				len = strstr(strstr(line, "\"") + 1, "\"") - strstr(line, "\"") - 1;
				memcpy(cfg->refl_addr, strstr(line, "\"") + 1, len);
				cfg->refl_addr[len] = 0;
			}
			else if (strstr(line, "port") == line)
			{
				cfg->refl_port = atoi(strstr(line, "=") + 1);
			}
			else if (strstr(line, "reflector") == line)
			{
				len = strstr(strstr(line, "\"") + 1, "\"") - strstr(line, "\"") - 1;
				memcpy(cfg->reflector, strstr(line, "\"") + 1, len);
				cfg->reflector[len] = 0;
			}
			else if (strstr(line, "module") == line)
			{
				cfg->module = *(strstr(line, "\"") + 1);
			}

			else if (strstr(line, "nrst") == line)
			{
				cfg->nrst = atoi(strstr(line, "=") + 1);
			}
			else if (strstr(line, "pa_en") == line)
			{
				cfg->pa_en = atoi(strstr(line, "=") + 1);
			}
			else if (strstr(line, "boot0") == line)
			{
				cfg->boot0 = atoi(strstr(line, "=") + 1);
			}

			else if (strstr(line, "tx_freq") == line)
			{
				cfg->tx_freq = atoi(strstr(line, "=") + 1);
			}
			else if (strstr(line, "rx_freq") == line)
			{
				cfg->rx_freq = atoi(strstr(line, "=") + 1);
			}
			else if (strstr(line, "freq_corr") == line)
			{
				cfg->freq_corr = atoi(strstr(line, "=") + 1);
			}
			else if (strstr(line, "tx_pwr") == line)
			{
				cfg->tx_pwr = atof(strstr(line, "=") + 1);
			}
			else if (strstr(line, "afc") == line)
			{
				if (*(strstr(line, "=") + 1) == '1')
					cfg->afc = 1;
				else
					cfg->afc = 0;
			}
			else if (strstr(line, "zmq_port") == line)
			{
				cfg->zmq_port = atoi(strstr(line, "=") + 1);
			}
		}

		fclose(cfg_fp);
		return 0; // file read OK
	}
	else
	{
		return -1; // error reading file
	}
}

// Release GPIO resources
void gpio_cleanup(void)
{
	// Release all GPIO lines
	if (config.pa_en_req)
	{
		gpiod_line_request_release(config.pa_en_req);
		config.pa_en_req = NULL;
	}

	if (config.boot0_req)
	{
		gpiod_line_request_release(config.boot0_req);
		config.boot0_req = NULL;
	}

	if (config.nrst_req)
	{
		gpiod_line_request_release(config.nrst_req);
		config.nrst_req = NULL;
	}

	if (config.gpio_chip)
	{
		gpiod_chip_close(config.gpio_chip);
		config.gpio_chip = NULL;
	}

	dbg_print(TERM_GREEN, "GPIO resources released\n");
}

void gpio_init(const char *program_name)
{
	config.gpio_chip = NULL;
	config.pa_en_req = NULL;
	config.boot0_req = NULL;
	config.nrst_req = NULL;

	config.gpio_chip = gpiod_chip_open("/dev/gpiochip0");
	if (!config.gpio_chip)
	{
		dbg_print(TERM_RED, "\nError opening GPIO chip\n");
		exit(1);
	}

	struct gpiod_line_settings *settings = gpiod_line_settings_new();
	struct gpiod_line_config *line_cfg = gpiod_line_config_new();
	struct gpiod_request_config *req_cfg = gpiod_request_config_new();

	if (!settings || !line_cfg || !req_cfg)
	{
		dbg_print(TERM_RED, "\nGPIO allocation error\n");
		exit(1);
	}

	gpiod_line_settings_set_direction(settings, GPIOD_LINE_DIRECTION_OUTPUT);
	gpiod_line_settings_set_output_value(settings, GPIOD_LINE_VALUE_INACTIVE);

	gpiod_request_config_set_consumer(req_cfg, program_name);

	unsigned int offset;

	// PA_EN
	offset = config.pa_en;
	gpiod_line_config_add_line_settings(line_cfg, &offset, 1, settings);
	config.pa_en_req = gpiod_chip_request_lines(config.gpio_chip, req_cfg, line_cfg);
	if (!config.pa_en_req)
	{
		dbg_print(TERM_RED, "\nError requesting PA_EN GPIO%d\n", config.pa_en);
		gpio_cleanup();
		exit(1);
	}
	gpiod_line_config_reset(line_cfg);

	// BOOT0
	offset = config.boot0;
	gpiod_line_config_add_line_settings(line_cfg, &offset, 1, settings);
	config.boot0_req = gpiod_chip_request_lines(config.gpio_chip, req_cfg, line_cfg);
	if (!config.boot0_req)
	{
		dbg_print(TERM_RED, "\nError requesting BOOT0 GPIO%d\n", config.boot0);
		gpio_cleanup();
		exit(1);
	}
	gpiod_line_config_reset(line_cfg);

	// nRST
	offset = config.nrst;
	gpiod_line_config_add_line_settings(line_cfg, &offset, 1, settings);
	config.nrst_req = gpiod_chip_request_lines(config.gpio_chip, req_cfg, line_cfg);
	if (!config.nrst_req)
	{
		dbg_print(TERM_RED, "\nError requesting nRST GPIO%d\n", config.nrst);
		gpio_cleanup();
		exit(1);
	}

	gpiod_line_settings_free(settings);
	gpiod_line_config_free(line_cfg);
	gpiod_request_config_free(req_cfg);
}

uint8_t gpio_set(uint16_t gpio, uint8_t state)
{
	struct gpiod_line_request *req = NULL;
	enum gpiod_line_value val = state ? GPIOD_LINE_VALUE_ACTIVE : GPIOD_LINE_VALUE_INACTIVE;

	if (gpio == config.pa_en)
	{
		req = config.pa_en_req;
	}
	else if (gpio == config.boot0)
	{
		req = config.boot0_req;
	}
	else if (gpio == config.nrst)
	{
		req = config.nrst_req;
	}

	if (!req)
	{
		dbg_print(TERM_RED, "Error: Invalid GPIO number %d or GPIO not initialized\n", gpio);
		return 1;
	}

	if (gpiod_line_request_set_value(req, gpio, val) < 0)
	{
		dbg_print(TERM_RED, "Error setting GPIO line %d\n", gpio);
		return 1;
	}

	return 0;
}

// M17 stuff
void refl_send(const uint8_t *msg, uint16_t len)
{
	if (sendto(sockt, msg, len, 0, (const struct sockaddr *)&serv_addr, sizeof(serv_addr)) < 0)
	{
		dbg_print(TERM_RED, "\nError while sending data to reflector.\nExiting.\n");
		exit(EXIT_FAILURE);
	}
}

// device config funcs
int8_t dev_ping(void)
{
	uint8_t cid = CMD_PING;
	uint8_t cmd[3] = {cid, 3, 0};
	uint8_t resp[7] = {0};

	uart_lock = 1;		   // prevent main loop from reading
	tcflush(fd, TCIFLUSH); // clear leftover bytes

	write(fd, cmd, 3);

	int rd = 0;
	while (rd < 7)
	{
		int r = read(fd, resp + rd, 7 - rd);
		if (r <= 0)
		{
			uart_lock = 0;
			dbg_print(TERM_RED, "PING: Timeout waiting for device response\n");
			return -1;
		}
		rd += r;
	}

	uart_lock = 0;

	if (memcmp(resp, (uint8_t[]){cid, 7, 0, 0, 0, 0, 0}, 7) == 0)
	{
		dbg_print(TERM_GREEN, "PONG OK\n"); // OK
		return 0;
	}

	uint32_t dev_err;
	memcpy((uint8_t *)&dev_err, &resp[3], sizeof(uint32_t));
	dbg_print(TERM_YELLOW, "PONG error code: 0x%04X\n", dev_err);
	return -1;
}

int8_t dev_set_rx_freq(uint32_t freq)
{
	uint8_t cid = CMD_SET_RX_FREQ;
	uint8_t cmd[3 + 4] = {cid, 7, 0};
	memcpy(&cmd[3], (uint8_t *)&freq, sizeof(freq));
	uint8_t resp[4] = {0};

	uart_lock = 1;		   // prevent main loop from reading
	tcflush(fd, TCIFLUSH); // clear leftover bytes

	write(fd, cmd, 7);

	int rd = 0;
	while (rd < 4)
	{
		int r = read(fd, resp + rd, 4 - rd);
		if (r <= 0)
		{
			uart_lock = 0;
			// dbg_print(TERM_RED, "%s(): Timeout waiting for device response\n", __func__);
			return -1;
		}
		rd += r;
	}

	uart_lock = 0;

	if (memcmp(resp, (uint8_t[]){cid, 4, 0, ERR_OK}, 4) == 0)
	{
		dbg_print(0, "RX frequency: ");
		dbg_print(TERM_GREEN, "%lu Hz\n", freq); // OK
		return 0;
	}

	dbg_print(TERM_YELLOW, "Error %d setting RX frequency: %lu Hz\n", resp[3], freq); // error
	return -1;
}

int8_t dev_set_tx_freq(uint32_t freq)
{
	uint8_t cid = CMD_SET_TX_FREQ;
	uint8_t cmd[3 + 4] = {cid, 7, 0};
	memcpy(&cmd[3], (uint8_t *)&freq, sizeof(freq));
	uint8_t resp[4] = {0};

	uart_lock = 1;		   // prevent main loop from reading
	tcflush(fd, TCIFLUSH); // clear leftover bytes

	write(fd, cmd, 7);

	int rd = 0;
	while (rd < 4)
	{
		int r = read(fd, resp + rd, 4 - rd);
		if (r <= 0)
		{
			uart_lock = 0;
			// dbg_print(TERM_RED, "%s(): Timeout waiting for device response\n", __func__);
			return -1;
		}
		rd += r;
	}

	uart_lock = 0;

	if (memcmp(resp, (uint8_t[]){cid, 4, 0, ERR_OK}, 4) == 0)
	{
		dbg_print(0, "TX frequency: ");
		dbg_print(TERM_GREEN, "%lu Hz\n", freq); // OK
		return 0;
	}

	dbg_print(TERM_YELLOW, "Error %d setting TX frequency: %lu Hz\n", resp[3], freq); // error
	return -1;
}

/*int8_t dev_set_freq_corr(int16_t corr)
{
	uint8_t cid = CMD_SET_FREQ_CORR;
	uint8_t cmd[3 + 2] = {cid, 5, 0, corr & 0xFF, (corr >> 8) & 0xFF};
	uint8_t resp[4] = {0};

	uart_lock = 1;		   // prevent main loop from reading
	tcflush(fd, TCIFLUSH); // clear leftover bytes

	write(fd, cmd, 5);

	int rd = 0;
	while (rd < 4)
	{
		int r = read(fd, resp + rd, 4 - rd);
		if (r <= 0)
		{
			uart_lock = 0;
			// dbg_print(TERM_RED, "%s(): Timeout\n", __func__);
			return -1;
		}
		rd += r;
	}

	uart_lock = 0;

	if (memcmp(resp, (uint8_t[]){cid, 4, 0, ERR_OK}, 4) == 0)
	{
		dbg_print(0, "Frequency correction: ");
		dbg_print(TERM_GREEN, "%d\n", corr); // OK
		return 0;
	}

	dbg_print(TERM_YELLOW, "Error %d setting frequency correction: %d\n", resp[3], corr); // error
	return -1;
}*/

int8_t dev_set_afc(uint8_t en)
{
	uint8_t cid = CMD_SET_AFC;
	uint8_t cmd[3 + 1] = {cid, 4, 0, en == 0 ? 0 : 1};
	uint8_t resp[4] = {0};

	uart_lock = 1;		   // prevent main loop from reading
	tcflush(fd, TCIFLUSH); // clear leftover bytes

	write(fd, cmd, 4);

	int rd = 0;
	while (rd < 4)
	{
		int r = read(fd, resp + rd, 4 - rd);
		if (r <= 0)
		{
			uart_lock = 0;
			// dbg_print(TERM_RED, "%s(): Timeout\n", __func__);
			return -1;
		}
		rd += r;
	}

	uart_lock = 0;

	if (memcmp(resp, (uint8_t[]){cid, 4, 0, ERR_OK}, 4) == 0)
	{
		dbg_print(0, "AFC: ");
		dbg_print(TERM_GREEN, "%s\n", en == 0 ? "disabled" : "enabled"); // OK
		return 0;
	}

	dbg_print(TERM_YELLOW, "Error setting AFC\n"); // error
	return -1;
}

int8_t dev_set_tx_power(float power) // powr in dBm
{
	uint8_t cid = CMD_SET_TX_POWER;
	uint8_t cmd[3 + 1] = {cid, 4, 0, roundf(power * 4.0f)};
	uint8_t resp[4] = {0};

	uart_lock = 1;		   // prevent main loop from reading
	tcflush(fd, TCIFLUSH); // clear leftover bytes

	write(fd, cmd, 4);

	int rd = 0;
	while (rd < 4)
	{
		int r = read(fd, resp + rd, 4 - rd);
		if (r <= 0)
		{
			uart_lock = 0;
			// dbg_print(TERM_RED, "%s(): Timeout\n", __func__);
			return -1;
		}
		rd += r;
	}

	uart_lock = 0;

	if (memcmp(resp, (uint8_t[]){cid, 4, 0, ERR_OK}, 4) == 0)
	{
		dbg_print(0, "TX power: ");
		dbg_print(TERM_GREEN, "%2.2f dBm\n", power); // OK
		return 0;
	}

	dbg_print(TERM_YELLOW, "Error %d setting TX power: %2.2f dBm\n", resp[3], power); // error
	return -1;
}

int8_t dev_start_tx(void)
{
	uint8_t cid = CMD_TX_START;
	uint8_t cmd[4] = {cid, 4, 0, 1};
	uint8_t resp[4] = {0};

	uart_lock = 1;		   // prevent main loop from reading
	tcflush(fd, TCIFLUSH); // clear leftover bytes

	write(fd, cmd, 4);

	int rd = 0;
	while (rd < 4)
	{
		int r = read(fd, resp + rd, 4 - rd);
		if (r <= 0)
		{
			uart_lock = 0;
			// dbg_print(TERM_RED, "%s(): Timeout\n", __func__);
			return -1;
		}
		rd += r;
	}

	uart_lock = 0;

	if (memcmp(resp, (uint8_t[]){cid, 4, 0, ERR_OK}, 4) == 0 ||
		memcmp(resp, (uint8_t[]){cid, 4, 0, ERR_NOP}, 4) == 0)
	{
		// dbg_print(TERM_GREEN, "%s(): OK\n", __func__);
		return 0;
	}

	// dbg_print(TERM_RED, "%s(): Bad resp = %02X %02X %02X %02X\n", __func__, resp[0], resp[1], resp[2], resp[3]);
	return -1;
}

int8_t dev_stop_tx(void)
{
	uint8_t cid = CMD_TX_START;
	uint8_t cmd[4] = {cid, 4, 0, 0};
	uint8_t resp[4] = {0};

	uart_lock = 1;		   // prevent main loop from reading
	tcflush(fd, TCIFLUSH); // clear leftover bytes

	write(fd, cmd, 4);

	int rd = 0;
	while (rd < 4)
	{
		int r = read(fd, resp + rd, 4 - rd);
		if (r <= 0)
		{
			uart_lock = 0;
			// dbg_print(TERM_RED, "%s(): Timeout\n", __func__);
			return -1;
		}
		rd += r;
	}

	uart_lock = 0;

	if (memcmp(resp, (uint8_t[]){cid, 4, 0, ERR_OK}, 4) == 0 ||
		memcmp(resp, (uint8_t[]){cid, 4, 0, ERR_NOP}, 4) == 0)
	{
		// dbg_print(TERM_GREEN, "%s(): OK\n", __func__);
		return 0;
	}

	// dbg_print(TERM_RED, "%s(): Bad resp = %02X %02X %02X %02X\n", __func__, resp[0], resp[1], resp[2], resp[3]);
	return -1;
}

int8_t dev_start_rx(void) // start reception
{
	uint8_t cid = CMD_RX_START;
	uint8_t cmd[4] = {cid, 4, 0, 1};
	uint8_t resp[4] = {0};

	uart_lock = 1;		   // prevent main loop from reading
	tcflush(fd, TCIFLUSH); // clear leftover bytes

	write(fd, cmd, 4);

	int rd = 0;
	while (rd < 4)
	{
		int r = read(fd, resp + rd, 4 - rd);
		if (r <= 0)
		{
			uart_lock = 0;
			// dbg_print(TERM_RED, "%s(): Timeout\n", __func__);
			return -1;
		}
		rd += r;
	}

	uart_lock = 0;

	if (memcmp(resp, (uint8_t[]){cid, 4, 0, ERR_OK}, 4) == 0 ||
		memcmp(resp, (uint8_t[]){cid, 4, 0, ERR_NOP}, 4) == 0)
	{
		// dbg_print(TERM_GREEN, "%s(): OK\n", __func__);
		return 0;
	}

	// dbg_print(TERM_RED, "%s(): Bad resp = %02X %02X %02X %02X\n", __func__, resp[0], resp[1], resp[2], resp[3]);
	return -1;
}

int8_t dev_stop_rx(void) // stop reception
{
	uint8_t cid = CMD_RX_START;
	uint8_t cmd[4] = {cid, 4, 0, 0};
	uint8_t resp[4] = {0};

	uart_lock = 1;		   // prevent main loop from reading
	tcflush(fd, TCIFLUSH); // clear leftover bytes

	write(fd, cmd, 4);

	int rd = 0;
	while (rd < 4)
	{
		int r = read(fd, resp + rd, 4 - rd);
		if (r <= 0)
		{
			uart_lock = 0;
			// dbg_print(TERM_RED, "%s(): Timeout\n", __func__);
			return -1;
		}
		rd += r;
	}

	uart_lock = 0;

	if (memcmp(resp, (uint8_t[]){cid, 4, 0, ERR_OK}, 4) == 0 ||
		memcmp(resp, (uint8_t[]){cid, 4, 0, ERR_NOP}, 4) == 0)
	{
		// dbg_print(TERM_GREEN, "%s(): OK\n", __func__);
		return 0;
	}

	// dbg_print(TERM_RED, "%s(): Bad resp = %02X %02X %02X %02X\n", __func__, resp[0], resp[1], resp[2], resp[3]);
	return -1;
}

void sigint_handler(int val)
{
	(void)val; // get rid of unused variable warning
	dbg_print(TERM_YELLOW, "\nSIGINT caught, disconnecting\n");
	sprintf((char *)tx_buff, "DISCxxxxxx"); // that "xxxxxx" is just a placeholder
	memcpy(&tx_buff[4], config.enc_node, sizeof(config.enc_node));
	refl_send(tx_buff, 4 + 6); // DISC

	// Clean up GPIO resources
	gpio_cleanup();

	// close log file if necessary
	if (logfile != NULL)
	{
		fclose(logfile);
	}

	dbg_print(TERM_YELLOW, "Exiting\n");
	exit(EXIT_SUCCESS);
}

// samples per symbol (sps) = 5
// old code - deprecated
/*void filter_symbols(int8_t *out, const int8_t *in, const float* flt, uint8_t phase_inv)
{
	#define FLT_LEN 41
	static int8_t last[FLT_LEN]; //memory for last symbols

	if(out!=NULL)
	{
		for(uint8_t i=0; i<SYM_PER_FRA; i++)
		{
			for(uint8_t j=0; j<5; j++)
			{
				for(uint8_t k=0; k<FLT_LEN-1; k++)
					last[k]=last[k+1];

				if(j==0)
				{
					if(phase_inv) //optional phase inversion
						last[FLT_LEN-1]=-in[i];
					else
						last[FLT_LEN-1]= in[i];
				}
				else
					last[FLT_LEN-1]=0;

				float acc=0.0f;
				for(uint8_t k=0; k<FLT_LEN; k++)
					acc+=last[k]*flt[k];

				out[i*5+j]=acc*TX_SYMBOL_SCALING_COEFF*sqrtf(5.0f); //crank up the gain
			}
		}
	}
	else
	{
		for(uint8_t i=0; i<FLT_LEN; i++)
			last[i]=0;
	}
}*/

// new, polyphase filter implementation
void filter_symbols(int8_t *__restrict out, const int8_t *__restrict in, const float *__restrict flt, uint8_t phase_inv)
{
#define FLT_LEN 41
#define TAPS_PER_PHASE 9

	// history
	static float sr[TAPS_PER_PHASE * 2] = {0};
	static uint8_t w = 0;

	// flush filter state
	if (in == NULL)
	{
		memset(sr, 0, sizeof(sr));
		w = 0;
		return;
	}

	// precompute gain and sign once
	static const float gain = TX_SYMBOL_SCALING_COEFF * sqrtf(5.0f);
	const float sign = phase_inv ? -1.0f : 1.0f;

	for (uint16_t i = 0; i < SYM_PER_FRA; i++)
	{
		// insert new sample per symbol
		const float x = (float)in[i] * sign;

		// store once, duplicated for linear access
		float *__restrict hp = &sr[w];
		hp[0] = x;
		hp[TAPS_PER_PHASE] = x;

		// phase pointer
		const float *__restrict tp = flt;

		// generate sps (5) output samples
		for (uint8_t ph = 0; ph < 5; ph++)
		{
			float acc;

			// fully unrolled 9-tap dot product
			acc = hp[0] * tp[0];
			acc += hp[1] * tp[1];
			acc += hp[2] * tp[2];
			acc += hp[3] * tp[3];
			acc += hp[4] * tp[4];
			acc += hp[5] * tp[5];
			acc += hp[6] * tp[6];
			acc += hp[7] * tp[7];
			acc += hp[8] * tp[8];

			out[i * 5 + ph] = (int8_t)(acc * gain);

			// advance to next phase coefficients
			tp += TAPS_PER_PHASE;
		}

		// circular index update without modulo
		if (w == 0)
			w = TAPS_PER_PHASE - 1;
		else
			w--;
	}
}

int main(int argc, char *argv[])
{
	signal(SIGINT, sigint_handler);

	// time
	time_t rawtime;
	struct tm *timeinfo;

	if (argc < 3)
	{
		dbg_print(TERM_RED, "Invalid params\nExiting\n");
		return 1;
	}

	//-----------------------------------args parse------------------------------------
	uint8_t reset = 0;
	for (uint8_t i = 1; i < argc; i++)
	{
		if (argv[i][0] == '-') // TODO: replace this with getopt
		{
			if (argv[i][1] == 'r') // device reset
			{
				reset = 1; // reset pending
			}
			else if (argv[i][1] == 'c') // config file
			{
				dbg_print(0, "Config:");
				if (load_config(&config, argv[i + 1]) == 0)
				{
					dbg_print(TERM_GREEN, " OK\n");
					i++; // skip next arg
				}
				else
				{
					dbg_print(TERM_RED, " error reading %s\nExiting\n", argv[i + 1]);
					return 1;
				}
			}
		}
	}

	// reset the device and exit
	if (reset)
	{
		dbg_print(0, "Device reset...");
		uint8_t gpio_err = 0;
		gpio_init(argv[0]);
		gpio_err |= gpio_set(config.boot0, 0); // all pins should be at logic low already, but better be safe than sorry
		gpio_err |= gpio_set(config.pa_en, 0);
		gpio_err |= gpio_set(config.nrst, 0);
		usleep(250000U); // 250ms
		gpio_err |= gpio_set(config.nrst, 1);

		if (gpio_err)
			dbg_print(TERM_RED, " error\n");
		else
			dbg_print(TERM_GREEN, " OK\n");

		return (int)gpio_err;
	}

	// check if the reflector's address looks valid
	if (strlen(config.refl_addr) < 7)
	{
		dbg_print(TERM_RED, "Invalid reflector's IPv4 address\nExiting\n");
		return 1;
	}

	//---------------------------config's basic sanity checks--------------------------
	if (config.rx_freq < 420000000U || config.rx_freq > 450000000U)
	{
		dbg_print(TERM_RED, "Invalid RX frequency\nExiting\n");
		return 1;
	}
	if (config.tx_freq < 420000000U || config.tx_freq > 450000000U)
	{
		dbg_print(TERM_RED, "Invalid TX frequency\nExiting\n");
		return 1;
	}
	if (config.tx_pwr < 0.0f || config.tx_pwr > 47.75f)
	{
		dbg_print(TERM_RED, "Invalid TX power\nExiting\n");
		return 1;
	}

	srand(time(NULL));
	dbg_print(TERM_GREEN, "Starting up rpi-interface\n");

	// check write access to the log file
	if (strlen(config.log_path) > 0)
	{
		logfile = fopen(config.log_path, "awb");
		if (logfile != NULL)
		{
			dbg_print(0, "Storing traffic in %s\n", config.log_path);
		}
		else
		{
			dbg_print(TERM_RED, "Cannot access %s\nExiting\n", config.log_path);
			return 1;
		}
	}
	else
	{
		dbg_print(0, "Traffic logging ");
		dbg_print(TERM_GREEN, "disabled\n");
	}

	//------------------------------------gpio init------------------------------------
	dbg_print(0, "GPIO init...");
	uint8_t gpio_err = 0;
	gpio_init(argv[0]);
	gpio_err |= gpio_set(config.nrst, 0); // both pins should be at logic low already, but better be safe than sorry
	usleep(250000U);					  // 250ms
	gpio_err |= gpio_set(config.nrst, 1);
	usleep(2000000U); // 2s for device boot-up
	if (gpio_err == 0)
		dbg_print(TERM_GREEN, " OK\n");

	//-----------------------------------device part-----------------------------------
	dbg_print(0, "UART init: %s at %d...", (char *)config.uart, config.uart_rate);
	fd = open((char *)config.uart, O_RDWR | O_NOCTTY | O_SYNC);
	if (fd < 0)
	{
		dbg_print(TERM_RED, " error\nExiting\n");
		exit(1);
	}

	set_interface_attribs(fd, config.uart_rate, 0);
	dbg_print(TERM_GREEN, " OK\n");

	// PING-PONG test
	dbg_print(0, "Radio board's reply to PING... ");
	dev_ping();

	// config the device
	dev_set_rx_freq(config.rx_freq);
	dev_set_tx_freq(config.tx_freq);
	//dev_set_freq_corr(config.freq_corr);
	dev_set_tx_power(config.tx_pwr);
	dev_set_afc(config.afc);

	//-----------------------------------internet part-----------------------------------
	dbg_print(0, "Connecting to %s:%d (%s) module %c as \"%s\"", config.refl_addr, config.refl_port, config.reflector, config.module, config.node);

	// server
	serv_addr.sin_family = AF_INET;
	serv_addr.sin_addr.s_addr = inet_addr(config.refl_addr);
	serv_addr.sin_port = htons(config.refl_port);

	// Create a socket
	sockt = socket(AF_INET, SOCK_DGRAM, 0);
	if (sockt < 0)
	{
		dbg_print(TERM_RED, "\nSocket error\nExiting\n");
		return 1;
	}
	memset((char *)&daddr, 0, sizeof(daddr));

	// encode M17 callsign from argv
	encode_callsign_bytes(config.enc_node, config.node);

	// send "CONN"
	sprintf((char *)tx_buff, "CONNxxxxxx%c", config.module);
	memcpy(&tx_buff[4], config.enc_node, sizeof(config.enc_node));
	refl_send(tx_buff, 4 + 6 + 1);
	dbg_print(TERM_GREEN, " OK\n");

	// extend the LSF syncword pattern with 8 symbols from the preamble
	lsf_sync_ext[0] = 3;
	lsf_sync_ext[1] = -3;
	lsf_sync_ext[2] = 3;
	lsf_sync_ext[3] = -3;
	lsf_sync_ext[4] = 3;
	lsf_sync_ext[5] = -3;
	lsf_sync_ext[6] = 3;
	lsf_sync_ext[7] = -3;
	memcpy(&lsf_sync_ext[8], lsf_sync_symbols, 8);

	// ZMQ
	dbg_print(0, "ZeroMQ ");
	if (config.zmq_port != 0)
	{
		sprintf(zmq_addr, "tcp://*:%d", config.zmq_port);
		zmq_ctx = zmq_ctx_new();
		bsb_downlink = zmq_socket(zmq_ctx, ZMQ_PUB);
		if (zmq_bind(bsb_downlink, zmq_addr) == 0)
			dbg_print(TERM_GREEN, "OK\n");
		else
			dbg_print(TERM_RED, "ERROR\n");
	}
	else
	{
		dbg_print(TERM_GREEN, "disabled\n");
	}

	// start RX
	dev_start_rx();
	time(&rawtime);
	timeinfo = localtime(&rawtime);
	dbg_print(TERM_SKYBLUE, "[%02d:%02d:%02d]",
			  timeinfo->tm_hour, timeinfo->tm_min, timeinfo->tm_sec);
	dbg_print(TERM_GREEN, " Device start - RX\n");

	// UART comms
	int8_t rx_bsb_sample = 0;

	float f_sample;

	// file for debug data dumping
	// FILE *fp=fopen("test_dump.bin", "wb");

	last_refl_ping = time(NULL);

	fd_set rfds;
	int maxfd = fd > sockt ? fd : sockt;

	while (1)
	{
		FD_ZERO(&rfds);
		FD_SET(fd, &rfds);
		FD_SET(sockt, &rfds);

		select(maxfd + 1, &rfds, NULL, NULL, NULL);

		// are there any new baseband samples to process?
		if (!uart_lock && FD_ISSET(fd, &rfds))
		{
			read(fd, (uint8_t *)&rx_bsb_sample, 1);

			// wait for rx baseband data header
			if (!uart_rx_sync)
			{
				rx_samp_buff[0] = rx_samp_buff[1];
				rx_samp_buff[1] = rx_samp_buff[2];
				rx_samp_buff[2] = rx_bsb_sample;

				if (rx_samp_buff[0] == CMD_RX_DATA && rx_samp_buff[1] == 0xC3 && rx_samp_buff[2] == 0x03)
				{
					uart_rx_sync = 1;
					rx_buff_cnt = 3;
				}
			}
			else
			{
				rx_samp_buff[rx_buff_cnt] = rx_bsb_sample;
				rx_buff_cnt++;
			}

			if (uart_rx_sync && rx_buff_cnt == 963)
			{
				// dbg_print(TERM_YELLOW, "Baseband packet received\n");
				memcpy(raw_bsb_rx, &rx_samp_buff[3], sizeof(raw_bsb_rx));
				memset(rx_samp_buff, 0, sizeof(rx_samp_buff));
				uart_rx_data_valid = 1;
				uart_rx_sync = 0;
				rx_buff_cnt = 0;
			}

			if (rx_buff_cnt > 1024)
				dbg_print(TERM_RED, "Input buffer overflow\n");
		}

		if (uart_rx_data_valid)
		{
			// publish over ZMQ
			if (config.zmq_port != 0)
			{
				for (uint16_t i = 0; i < 960; i++)
				{
					zmq_samp_buff[zmq_samples++] = raw_bsb_rx[i];
					if (zmq_samples == ZMQ_RX_BUFF_SIZE)
					{
						zmq_send(bsb_downlink, (char *)zmq_samp_buff, ZMQ_RX_BUFF_SIZE, 0);
						zmq_samples = 0;
					}
				}
			}

			for (uint16_t i = 0; i < 960; i++)
			{
				// push buffer TODO: please optimize this. eyes hurt
				for (uint8_t i = 0; i < sizeof(flt_buff) - 1; i++)
					flt_buff[i] = flt_buff[i + 1];
				flt_buff[sizeof(flt_buff) - 1] = raw_bsb_rx[i];

				f_sample = 0.0f;
				for (uint8_t i = 0; i < sizeof(flt_buff); i++)
					f_sample += rrc_taps_5[i] * (float)flt_buff[i];
				f_sample *= RX_SYMBOL_SCALING_COEFF; // symbol map (works for CC1200 only)

				for (uint16_t i = 0; i < sizeof(f_flt_buff) / sizeof(float) - 1; i++)
					f_flt_buff[i] = f_flt_buff[i + 1];
				f_flt_buff[sizeof(f_flt_buff) / sizeof(float) - 1] = f_sample;

				// L2 norm check against syncword
				float symbols[16];
				for (uint8_t i = 0; i < 16; i++)
					symbols[i] = f_flt_buff[i * 5];

				float dist_lsf = sq_eucl_norm(&symbols[0], lsf_sync_ext, 16); // check against extended LSF syncword (8 symbols, alternating -3/+3)
				float dist_pkt = sq_eucl_norm(&symbols[0], pkt_sync_symbols, 8);
				float dist_str = sq_eucl_norm(&symbols[8], str_sync_symbols, 8);
				for (uint8_t i = 0; i < 16; i++)
					symbols[i] = f_flt_buff[960 + i * 5];
				float dist_stb = sq_eucl_norm(&symbols[8], str_sync_symbols, 8);
				float dist_eot = sq_eucl_norm(&symbols[8], eot_symbols, 0);
				dist_str += ((dist_stb < dist_eot) ? dist_stb : dist_eot);

				// fwrite(&dist_str, 4, 1, fp);

				// LSF received at idle state
				if (dist_lsf <= 22.5f && rx_state == RX_IDLE)
				{
					// find L2's minimum
					uint8_t sample_offset = 0;
					for (uint8_t i = 1; i <= 2; i++)
					{
						for (uint8_t j = 0; j < 16; j++)
							symbols[j] = f_flt_buff[j * 5 + i];

						float d = sq_eucl_norm(symbols, lsf_sync_ext, 16);

						if (d < dist_lsf)
						{
							dist_lsf = d;
							sample_offset = i;
						}
					}

					float pld[SYM_PER_PLD];

					for (uint16_t i = 0; i < SYM_PER_PLD; i++)
					{
						pld[i] = f_flt_buff[16 * 5 + i * 5 + sample_offset]; // add symbol timing correction
					}

					uint32_t e = decode_LSF(&lsf, pld);

					char call_dst[10], call_src[10], can;
					uint16_t type, crc;
					decode_callsign_bytes(call_dst, lsf.dst);
					decode_callsign_bytes(call_src, lsf.src);
					type = ((uint16_t)lsf.type[0] << 8 | lsf.type[1]);
					can = (type >> 7) & 0xFU;
					crc = (((uint16_t)lsf.crc[0] << 8) | lsf.crc[1]);

					time(&rawtime);
					timeinfo = localtime(&rawtime);
					dbg_print(TERM_SKYBLUE, "[%02d:%02d:%02d]",
							  timeinfo->tm_hour, timeinfo->tm_min, timeinfo->tm_sec);
					dbg_print(TERM_YELLOW, " RF LSF:");

					if (LSF_CRC(&lsf) == crc) // if CRC valid
					{
						got_lsf = 1;
						rx_state = RX_SYNCD; // change RX state
						sample_cnt = 0;		 // reset rx timeout timer

						last_fn = 0xFFFFU;

						dbg_print(TERM_GREEN, " CRC OK ");
						dbg_print(TERM_YELLOW, "| DST: %-9s | SRC: %-9s | TYPE: %04X (CAN=%d) | MER: %-3.1f%%\n",
								  call_dst, call_src, type, can, (float)e / 0xFFFFU / SYM_PER_PLD / 2.0f * 100.0f);

						if (type & 1) // if stream
						{
							m17stream.fn = 0;
							m17stream.sid = rand() % 0x10000U;

							uint8_t refl_pld[(32 + 16 + 224 + 16 + 128 + 16) / 8];			// single frame
							sprintf((char *)&refl_pld[0], "M17 ");							// MAGIC
							*((uint16_t *)&refl_pld[4]) = m17stream.sid;					// SID
							memcpy(&refl_pld[6], &lsf, 224 / 8);							// LSD
							*((uint16_t *)&refl_pld[34]) = m17stream.fn;					// FN
							memset(&refl_pld[36], 0, 128 / 8);								// payload (zeros, because this is LSF)
							uint16_t crc_val = CRC_M17(refl_pld, 52);						// CRC
							*((uint16_t *)&refl_pld[52]) = (crc_val >> 8) | (crc_val << 8); // endianness swap
							refl_send(refl_pld, sizeof(refl_pld));							// send a single frame to the reflector

							if (logfile != NULL)
							{
								time(&rawtime);
								timeinfo = localtime(&rawtime);
								fprintf(logfile, "\"%02d:%02d:%02d\" \"%s\" \"%s\" \"RF\" \"%d\" \"%3.1f%%\"\n",
										timeinfo->tm_hour, timeinfo->tm_min, timeinfo->tm_sec,
										call_src, call_dst, can, (float)e / 0xFFFFU / SYM_PER_PLD / 2.0f * 100.0f);
							}
						}
					}
					else
					{
						dbg_print(TERM_RED, " CRC ERR\n");
					}
				}

				// stream frame received
				else if (dist_str <= 25.0f)
				{
					rx_state = RX_SYNCD;
					sample_cnt = 0; // reset rx timeout timer

					// find L2's minimum
					uint8_t sample_offset = 0;
					for (uint8_t i = 1; i <= 2; i++)
					{
						for (uint8_t j = 0; j < 16; j++)
							symbols[j] = f_flt_buff[j * 5 + i];

						float tmp_a = sq_eucl_norm(&symbols[8], str_sync_symbols, 8);
						for (uint8_t j = 0; j < 16; j++)
							symbols[j] = f_flt_buff[960 + j * 5 + i];
						float tmp_b = sq_eucl_norm(&symbols[8], str_sync_symbols, 8);
						float tmp_e = sq_eucl_norm(&symbols[8], eot_symbols, 8);

						float d = tmp_a + ((tmp_b < tmp_e) ? tmp_b : tmp_e);

						if (d < dist_str)
						{
							dist_str = d;
							sample_offset = i;
						}
					}

					float pld[SYM_PER_PLD];

					for (uint16_t i = 0; i < SYM_PER_PLD; i++)
					{
						pld[i] = f_flt_buff[16 * 5 + i * 5 + sample_offset];
					}

					uint8_t lich[6];
					uint8_t lich_cnt;
					uint8_t frame_data[128 / 8];
					uint32_t e = decode_str_frame(frame_data, lich, &fn, &lich_cnt, pld);
					uint16_t frame_count = fn & 0x7FFFU;
					// set the last FN number to FN-1 if this is a late-join and the frame data is valid
					if (first_frame == 1 && (frame_count % 6) == lich_cnt)
					{
						last_fn = frame_count - 1;
					}

					if (((last_fn + 1) & 0xFFFFU) == frame_count) // new frame. TODO: maybe a timeout would be better
					{
						if (lich_parts != 0x3FU) // 6 chunks = 0b111111
						{
							// reconstruct LSF chunk by chunk
							memcpy(&lsf_b[lich_cnt * 5], lich, 40 / 8); // 40 bits
							lich_parts |= (1 << lich_cnt);
							if (lich_parts == 0x3FU && got_lsf == 0) // collected all of them?
							{
								if (!CRC_M17(lsf_b, 30)) // CRC check
								{
									got_lsf = 1;
									m17stream.sid = rand() % 0x10000U;

									char call_dst[12] = {0}, call_src[12] = {0};
									uint16_t type = ((uint16_t)lsf_b[12] << 8) | lsf_b[13];
									uint8_t can = (type >> 7) & 0xF;

									decode_callsign_bytes(call_dst, &lsf_b[0]);
									decode_callsign_bytes(call_src, &lsf_b[6]);

									time(&rawtime);
									timeinfo = localtime(&rawtime);
									dbg_print(TERM_SKYBLUE, "[%02d:%02d:%02d] ",
											  timeinfo->tm_hour, timeinfo->tm_min, timeinfo->tm_sec);
									dbg_print(TERM_YELLOW, "LSF REC: DST: %-9s | SRC: %-9s | TYPE: %04X (CAN=%d)\n",
											  call_dst, call_src, type, can);

									if (logfile != NULL)
									{
										time(&rawtime);
										timeinfo = localtime(&rawtime);
										fprintf(logfile, "\"%02d:%02d:%02d\" \"%s\" \"%s\" \"RF\" \"%d\" \"--\"\n",
												timeinfo->tm_hour, timeinfo->tm_min, timeinfo->tm_sec,
												call_src, call_dst, can);
									}
								}
								else
								{
									dbg_print(TERM_YELLOW, "LSF CRC ERR\n");
									lich_parts = 0; // reset flags
								}
							}
						}

						time(&rawtime);
						timeinfo = localtime(&rawtime);

						dbg_print(TERM_SKYBLUE, "[%02d:%02d:%02d]",
								  timeinfo->tm_hour, timeinfo->tm_min, timeinfo->tm_sec);
						dbg_print(TERM_YELLOW, " RF FRM: ");
						dbg_print(TERM_YELLOW, " FN:%04X | LICH_CNT:%d", fn, lich_cnt);
						/*dbg_print(TERM_YELLOW, " | PLD: ");
						for(uint8_t i=0; i<128/8; i++)
							dbg_print(TERM_YELLOW, "%02X", frame_data[2+i]);*/
						dbg_print(TERM_YELLOW, " | MER: %-3.1f%%\n",
								  (float)e / 0xFFFFU / SYM_PER_PLD / 2.0f * 100.0f);

						if (got_lsf)
						{
							m17stream.fn = (fn >> 8) | ((fn & 0xFF) << 8);
							uint8_t refl_pld[(32 + 16 + 224 + 16 + 128 + 16) / 8];			// single frame
							sprintf((char *)&refl_pld[0], "M17 ");							// MAGIC
							*((uint16_t *)&refl_pld[4]) = m17stream.sid;					// SID
							memcpy(&refl_pld[6], &lsf_b[0], 224 / 8);						// LSD
							*((uint16_t *)&refl_pld[34]) = m17stream.fn;					// FN
							memcpy(&refl_pld[36], frame_data, 128 / 8);						// payload
							uint16_t crc_val = CRC_M17(refl_pld, 52);						// CRC
							*((uint16_t *)&refl_pld[52]) = (crc_val >> 8) | (crc_val << 8); // endianness swap
							refl_send(refl_pld, sizeof(refl_pld));							// send a single frame to the reflector
						}

						last_fn = fn;
					}

					first_frame = 0;
				}

				// TODO: handle packet mode reception over RF
				else if (dist_pkt <= 25.0f && rx_state == RX_SYNCD)
				{
					// find L2's minimum
					uint8_t sample_offset = 0;
					for (uint8_t i = 1; i <= 2; i++)
					{
						for (uint8_t j = 0; j < 8; j++)
							symbols[j] = f_flt_buff[j * 5 + i];
						float d = sq_eucl_norm(symbols, pkt_sync_symbols, 8);
						for (uint8_t j = 0; j < 16; j++)
							symbols[j] = f_flt_buff[960 + j * 5 + i];
						float p = sq_eucl_norm(symbols, str_sync_symbols, 8);
						float e = sq_eucl_norm(symbols, eot_symbols, 8);
						d += ((p < e) ? p : e);

						if (d < dist_pkt)
						{
							dist_pkt = d;
							sample_offset = i;
						}
					}

					float pld[SYM_PER_PLD];
					uint8_t pkt_frame_data[25] = {0};
					uint8_t eof = 0;

					for (uint16_t i = 0; i < SYM_PER_PLD; i++)
					{
						pld[i] = f_flt_buff[8 * 5 + i * 5 + sample_offset];
					}

					// debug data dump
					// fwrite((uint8_t*)&f_flt_buff[sample_offset], SYM_PER_FRA*5*sizeof(float), 1, fp);

					/*uint32_t e = */ decode_pkt_frame(pkt_frame_data, &eof, &pkt_fn, pld);

					// TODO: this will only properly decode single-framed packets
					if (last_pkt_fn == 0xFF && eof == 1 && CRC_M17(pkt_frame_data, strlen((char *)pkt_frame_data) + 3) == 0)
					{
						sample_cnt = 0; // reset rx timeout timer
						last_pkt_fn = pkt_fn;

						time(&rawtime);
						timeinfo = localtime(&rawtime);

						dbg_print(TERM_SKYBLUE, "[%02d:%02d:%02d]",
								  timeinfo->tm_hour, timeinfo->tm_min, timeinfo->tm_sec);
						dbg_print(TERM_YELLOW, " RF PKT: ");
						/*for(uint8_t i=0; i<25; i++)
							dbg_print(0, "%02X ", pkt_frame_data[i]);
						dbg_print(0, "\n");*/
						dbg_print(0, "%s\n", (char *)&pkt_frame_data[1]);
						uint8_t refl_pld[4 + sizeof(lsf) + strlen((char *)pkt_frame_data) + 3];		// single frame
						sprintf((char *)&refl_pld[0], "M17P");										// MAGIC
						memcpy(&refl_pld[4], &lsf, sizeof(lsf));									// LSF
						memcpy(&refl_pld[34], &pkt_frame_data, strlen((char *)pkt_frame_data) + 3); // PKT data + CRC
						/*debug logging
						time(&rawtime);
						timeinfo=localtime(&rawtime);

						dbg_print(TERM_SKYBLUE, "[%02d:%02d:%02d]",
							timeinfo->tm_hour, timeinfo->tm_min, timeinfo->tm_sec);
						dbg_print(TERM_YELLOW, " refl_pld: ");
						for(uint8_t i=0; i<sizeof(refl_pld); i++)
							dbg_print(0, "%02X ", refl_pld[i]);
						dbg_print(0, "\n");
						*/
						refl_send(refl_pld, 4 + sizeof(lsf) + strlen((char *)pkt_frame_data) + 3); // send to the reflector
					}
				}

				// RX sync timeout
				if (rx_state == RX_SYNCD)
				{
					sample_cnt++;
					if (sample_cnt == 960 * 2)
					{
						rx_state = RX_IDLE;
						sample_cnt = 0;
						first_frame = 1;
						last_fn = 0xFFFFU; // TODO: there's a small chance that this will cause problems (it's a valid frame number)
						last_pkt_fn = 0xFF;
						lich_parts = 0;
						got_lsf = 0;
					}
				}
			}

			// all data has been used
			uart_rx_data_valid = 0;
		}

		// receive a packet - blocking
		if (FD_ISSET(sockt, &rfds))
		{
			rx_len = recvfrom(sockt, rx_buff, MAX_UDP_LEN, 0, (struct sockaddr *)&saddr, (socklen_t *)&saddr_size);

			// debug
			// dbg_print(0, "Size:%d\nPayload:%s\n", rx_len, rx_buff);

			// PING-PONG
			if (strstr((char *)rx_buff, "PING") == (char *)rx_buff)
			{
				last_refl_ping = time(NULL);
				sprintf((char *)tx_buff, "PONGxxxxxx"); // that "xxxxxx" is just a placeholder
				memcpy(&tx_buff[4], config.enc_node, sizeof(config.enc_node));
				refl_send(tx_buff, 4 + 6); // PONG
										   // dbg_print(TERM_YELLOW, "PING\n");
			}

			// M17 stream frame data - "Steaming Mode IP Packet, Single Packet Method"
			else if (strstr((char *)rx_buff, "M17 ") == (char *)rx_buff)
			{
				tx_timer = get_ms();

				m17stream.sid = ((uint16_t)rx_buff[4] << 8) | rx_buff[5];
				m17stream.fn = ((uint16_t)rx_buff[34] << 8) | rx_buff[35];
				static char dst_call[10] = {0};
				static char src_call[10] = {0};
				memcpy(m17stream.pld, &rx_buff[(32 + 16 + 224 + 16) / 8U], 128 / 8);

				int8_t frame_symbols[SYM_PER_FRA];					 // raw frame symbols
				int8_t bsb_samples[963] = {CMD_TX_DATA, 0xC3, 0x03}; // baseband samples wrapped in a frame

				if (tx_state == TX_IDLE) // first received frame
				{
					tx_state = TX_ACTIVE;

					// TODO: this needs to happen every time a new transmission appears
					// dev_stop_rx();
					// dbg_print(0, "RX stop\n");
					usleep(10e3);

					// extract data
					memcpy(m17stream.lsf.dst, "\xFF\xFF\xFF\xFF\xFF\xFF", 6);
					memcpy(m17stream.lsf.src, &rx_buff[6 + 6], 6);
					decode_callsign_bytes(dst_call, m17stream.lsf.dst);
					decode_callsign_bytes(src_call, m17stream.lsf.src);

					// set TYPE field
					memcpy(m17stream.lsf.type, &rx_buff[18], 2);
					m17stream.lsf.type[1] |= 0x2U << 5; // no encryption, so the subtype field defines the META field contents: extended callsign data

					// generate META field
					// remove trailing spaces and suffixes
					char trimmed_src[10];
					uint8_t enc_trimmed_src[6];
					for (uint8_t i = 0; i < 10; i++)
					{
						if (src_call[i] != ' ')
							trimmed_src[i] = src_call[i];
						else
						{
							trimmed_src[i] = 0;
							break;
						}
					}
					encode_callsign_bytes(enc_trimmed_src, trimmed_src);

					char ext_ref[12];
					uint8_t enc_ext_ref[6];
					sprintf((char *)ext_ref, "%s %c", config.reflector, config.module);
					encode_callsign_bytes(enc_ext_ref, ext_ref);

					memcpy(&m17stream.lsf.meta[0], m17stream.lsf.src, 6); // originator
					memcpy(&m17stream.lsf.meta[6], enc_ext_ref, 6);		  // reflector
					memset(&m17stream.lsf.meta[12], 0, 2);
					memcpy(m17stream.lsf.src, enc_trimmed_src, 6);

					// append CRC
					uint16_t ccrc = LSF_CRC(&m17stream.lsf);
					m17stream.lsf.crc[0] = ccrc >> 8;
					m17stream.lsf.crc[1] = ccrc & 0xFF;

					// log to file
					if (logfile != NULL)
					{
						time(&rawtime);
						timeinfo = localtime(&rawtime);
						fprintf(logfile, "\"%02d:%02d:%02d\" \"%s\" \"%s\" \"Internet\" \"--\" \"--\"\n",
								timeinfo->tm_hour, timeinfo->tm_min, timeinfo->tm_sec,
								src_call, dst_call);
					}

					time(&rawtime);
					timeinfo = localtime(&rawtime);
					dbg_print(TERM_SKYBLUE, "[%02d:%02d:%02d]",
							  timeinfo->tm_hour, timeinfo->tm_min, timeinfo->tm_sec);
					dbg_print(TERM_GREEN, " Stream TX start\n");

					// stop RX, set PA_EN=1 and initialize TX
					while (dev_stop_rx() != 0)
						usleep(40e3);
					usleep(2e3);
					gpio_set(config.pa_en, 1);
					while (dev_start_tx() != 0)
						usleep(40e3);
					usleep(10e3);

					// flush the RRC baseband filter
					filter_symbols(NULL, NULL, NULL, 0);

					// generate frame symbols, filter them and send out to the device
					// we need to prepare 3 frames to begin the transmission - preamble, LSF and stream frame 0
					// let's start with the preamble
					uint32_t frame_buff_cnt = 0;
					gen_preamble_i8(frame_symbols, &frame_buff_cnt, PREAM_LSF);

					// filter and send out to the device
					filter_symbols(bsb_samples + 3, frame_symbols, rrc_taps_5_poly, 0);
					write(fd, bsb_samples, sizeof(bsb_samples));

					// now the LSF
					gen_frame_i8(frame_symbols, NULL, FRAME_LSF, &(m17stream.lsf), 0, 0);

					// filter and send out to the device
					filter_symbols(bsb_samples + 3, frame_symbols, rrc_taps_5_poly, 0);
					write(fd, bsb_samples, sizeof(bsb_samples));

					// finally, the first frame
					gen_frame_i8(frame_symbols, m17stream.pld, FRAME_STR, &(m17stream.lsf), (m17stream.fn & 0x7FFFU) % 6, m17stream.fn);

					// filter and send out to the device
					filter_symbols(bsb_samples + 3, frame_symbols, rrc_taps_5_poly, 0);
					write(fd, bsb_samples, sizeof(bsb_samples));
				}
				else
				{
					// only one frame is needed
					gen_frame_i8(frame_symbols, m17stream.pld, FRAME_STR, &(m17stream.lsf), (m17stream.fn & 0x7FFFU) % 6, m17stream.fn);

					// filter and send out to the device
					filter_symbols(bsb_samples + 3, frame_symbols, rrc_taps_5_poly, 0);
					write(fd, bsb_samples, sizeof(bsb_samples));
				}

				time(&rawtime);
				timeinfo = localtime(&rawtime);

				/*dbg_print(TERM_YELLOW, "[%02d:%02d:%02d] NET FRM: ",
						timeinfo->tm_hour, timeinfo->tm_min, timeinfo->tm_sec);
				dbg_print(TERM_YELLOW, "SID: %04X | FN: %04X | DST: %-9s | SRC: %-9s | TYPE: %04X | META: ",
						m17stream.sid, m17stream.fn&0x7FFFU, dst_call, src_call, ((uint16_t)m17stream.lsf.type[0]<<8)|m17stream.lsf.type[1]);
				for(uint8_t i=0; i<14; i++)
					dbg_print(TERM_YELLOW, "%02X", m17stream.lsf.meta[i]);
				dbg_print(TERM_YELLOW, "\n");*/

				if (m17stream.fn & 0x8000U) // last stream frame
				{
					// send the final EOT marker
					uint32_t frame_buff_cnt = 0;
					gen_eot_i8(frame_symbols, &frame_buff_cnt);

					// filter and send out to the device
					filter_symbols(bsb_samples + 3, frame_symbols, rrc_taps_5_poly, 0);
					write(fd, bsb_samples, sizeof(bsb_samples));

					time(&rawtime);
					timeinfo = localtime(&rawtime);

					dbg_print(TERM_SKYBLUE, "[%02d:%02d:%02d]",
							  timeinfo->tm_hour, timeinfo->tm_min, timeinfo->tm_sec);
					dbg_print(TERM_GREEN, " Stream TX end\n");
					usleep(8 * 40e3); // wait 320ms (8 M17 frames) - let the transmitter consume all the buffered samples

					// disable TX
					gpio_set(config.pa_en, 0);

					// restart RX
					dev_stop_tx();
					dev_start_rx();
					time(&rawtime);
					timeinfo = localtime(&rawtime);
					dbg_print(TERM_SKYBLUE, "[%02d:%02d:%02d]",
							  timeinfo->tm_hour, timeinfo->tm_min, timeinfo->tm_sec);
					dbg_print(TERM_GREEN, " RX start\n");

					tx_state = TX_IDLE;
				}
			}

			// M17 packet data - "Packet Mode IP Packet"
			else if (strstr((char *)rx_buff, "M17P") == (char *)rx_buff)
			{
				time(&rawtime);
				timeinfo = localtime(&rawtime);
				dbg_print(TERM_SKYBLUE, "[%02d:%02d:%02d]", timeinfo->tm_hour, timeinfo->tm_min, timeinfo->tm_sec);
				dbg_print(TERM_GREEN, " M17 Inet packet received\n");

				char call_dst[10], call_src[10], can, type;
				decode_callsign_bytes(call_dst, &rx_buff[4 + 0]);
				decode_callsign_bytes(call_src, &rx_buff[4 + 6]);
				can = (*((uint16_t *)&rx_buff[4 + 6 + 6]) >> 7) & 0xF;
				type = rx_buff[4 + 240 / 8];

				dbg_print(TERM_DEFAULT, " ├ ");
				dbg_print(TERM_YELLOW, "DST: ");
				dbg_print(TERM_DEFAULT, "%s\n", call_dst);
				dbg_print(TERM_DEFAULT, " ├ ");
				dbg_print(TERM_YELLOW, "SRC: ");
				dbg_print(TERM_DEFAULT, "%s\n", call_src);
				dbg_print(TERM_DEFAULT, " ├ ");
				dbg_print(TERM_YELLOW, "CAN: ");
				dbg_print(TERM_DEFAULT, "%d\n", can);
				if (type != 5) // assuming 1-byte type specifier
				{
					dbg_print(TERM_DEFAULT, " └ ");
					dbg_print(TERM_YELLOW, "TYPE: ");
					dbg_print(TERM_DEFAULT, "%d\n", type);
				}
				else
				{
					dbg_print(TERM_DEFAULT, " ├ ");
					dbg_print(TERM_YELLOW, "TYPE: ");
					dbg_print(TERM_DEFAULT, "SMS\n");
					dbg_print(TERM_DEFAULT, " └ ");
					dbg_print(TERM_YELLOW, "MSG: ");
					dbg_print(TERM_DEFAULT, "%s\n", &rx_buff[4 + 240 / 8 + 1]);
				}

				// TODO: handle TX here
				int8_t frame_symbols[SYM_PER_FRA];					// raw frame symbols
				int8_t bsb_samples[SYM_PER_FRA * 5];				// filtered baseband samples = symbols*sps
				uint8_t bsb_chunk[963] = {CMD_TX_DATA, 0xC3, 0x03}; // baseband samples wrapped in a frame

				// log to file
				FILE *logfile = fopen((char *)config.log_path, "awb");
				if (logfile != NULL)
				{
					time(&rawtime);
					timeinfo = localtime(&rawtime);
					fprintf(logfile, "\"%02d:%02d:%02d\" \"%s\" \"%s\" \"Internet\" \"--\" \"--\"\n",
							timeinfo->tm_hour, timeinfo->tm_min, timeinfo->tm_sec,
							call_src, call_dst);
				}

				time(&rawtime);
				timeinfo = localtime(&rawtime);
				dbg_print(TERM_SKYBLUE, "[%02d:%02d:%02d]",
						  timeinfo->tm_hour, timeinfo->tm_min, timeinfo->tm_sec);
				dbg_print(TERM_GREEN, " Packet TX start\n");

				// stop RX, set PA_EN=1 and initialize TX
				while (dev_stop_rx() != 0)
					usleep(40e3);
				usleep(2e3);
				gpio_set(config.pa_en, 1);
				while (dev_start_tx() != 0)
					usleep(40e3);
				usleep(10e3);

				// flush the RRC baseband filter
				filter_symbols(NULL, NULL, NULL, 0);

				// generate frame symbols, filter them and send out to the device
				// we need to prepare 3 frames to begin the transmission - preamble, LSF and stream frame 0
				// let's start with the preamble
				uint32_t frame_buff_cnt = 0;
				gen_preamble_i8(frame_symbols, &frame_buff_cnt, PREAM_LSF);

				// filter and send out to the device
				filter_symbols(bsb_samples, frame_symbols, rrc_taps_5_poly, 0);
				memcpy(&bsb_chunk[3], bsb_samples, sizeof(bsb_samples));
				write(fd, bsb_samples, sizeof(bsb_samples));

				// now the LSF
				gen_frame_i8(frame_symbols, NULL, FRAME_LSF, (lsf_t *)&rx_buff[4], 0, 0);

				// filter and send out to the device
				filter_symbols(bsb_samples, frame_symbols, rrc_taps_5_poly, 0);
				memcpy(&bsb_chunk[3], bsb_samples, sizeof(bsb_samples));
				write(fd, bsb_samples, sizeof(bsb_samples));

				// packet frames
				uint16_t pld_len = rx_len - (4 + 240 / 8); //"M17P" plus 240-bit LSD
				uint8_t frame = 0;
				uint8_t pld[26];

				while (pld_len > 25)
				{
					memcpy(pld, &rx_buff[4 + 240 / 8 + frame * 25], 25);
					pld[25] = frame << 2;
					gen_frame_i8(frame_symbols, pld, FRAME_PKT, NULL, 0, 0);
					filter_symbols(bsb_samples, frame_symbols, rrc_taps_5_poly, 0);
					memcpy(&bsb_chunk[3], bsb_samples, sizeof(bsb_samples));
					write(fd, bsb_samples, sizeof(bsb_samples));
					pld_len -= 25;
					frame++;
					usleep(40 * 1000U);
				}
				memset(pld, 0, 26);
				memcpy(pld, &rx_buff[4 + 240 / 8 + frame * 25], pld_len);
				pld[25] = (1 << 7) | (pld_len << 2); // EoT flag set, amount of remaining data in the 'frame number' field
				gen_frame_i8(frame_symbols, pld, FRAME_PKT, NULL, 0, 0);
				filter_symbols(bsb_samples, frame_symbols, rrc_taps_5_poly, 0);
				memcpy(&bsb_chunk[3], bsb_samples, sizeof(bsb_samples));
				write(fd, bsb_samples, sizeof(bsb_samples));
				usleep(40 * 1000U);

				// now the final EOT marker
				frame_buff_cnt = 0;
				gen_eot_i8(frame_symbols, &frame_buff_cnt);

				// filter and send out to the device
				filter_symbols(bsb_samples, frame_symbols, rrc_taps_5_poly, 0);
				memcpy(&bsb_chunk[3], bsb_samples, sizeof(bsb_samples));
				write(fd, bsb_samples, sizeof(bsb_samples));

				time(&rawtime);
				timeinfo = localtime(&rawtime);

				dbg_print(TERM_SKYBLUE, "[%02d:%02d:%02d]",
						  timeinfo->tm_hour, timeinfo->tm_min, timeinfo->tm_sec);
				dbg_print(TERM_GREEN, " PKT TX end\n");
				usleep(3 * 40e3); // wait 120ms (3 M17 frames)

				// disable TX
				gpio_set(config.pa_en, 0);

				// restart RX
				dev_stop_tx();
				dev_start_rx();
				time(&rawtime);
				timeinfo = localtime(&rawtime);
				dbg_print(TERM_SKYBLUE, "[%02d:%02d:%02d]",
						  timeinfo->tm_hour, timeinfo->tm_min, timeinfo->tm_sec);
				dbg_print(TERM_GREEN, " RX start\n");

				tx_state = TX_IDLE;
			}

			// clear the rx_buff
			memset((uint8_t *)rx_buff, 0, rx_len);
		}

		// tx timeout
		if (tx_state == TX_ACTIVE && (get_ms() - tx_timer) > 240) // 240ms timeout
		{
			time(&rawtime);
			timeinfo = localtime(&rawtime);

			dbg_print(TERM_SKYBLUE, "[%02d:%02d:%02d]",
					  timeinfo->tm_hour, timeinfo->tm_min, timeinfo->tm_sec);
			dbg_print(TERM_GREEN, " TX timeout\n");
			// usleep(10*40e3); //wait 400ms (10 M17 frames)

			// disable TX
			gpio_set(config.pa_en, 0);

			// restart RX
			while (dev_stop_tx() != 0)
				usleep(40e3);
			while (dev_start_rx() != 0)
				usleep(40e3);
			time(&rawtime);
			timeinfo = localtime(&rawtime);
			dbg_print(TERM_SKYBLUE, "[%02d:%02d:%02d]",
					  timeinfo->tm_hour, timeinfo->tm_min, timeinfo->tm_sec);
			dbg_print(TERM_GREEN, " RX start\n");

			tx_state = TX_IDLE;
		}

		// connection with the reflector borken
		if (time(NULL) - last_refl_ping > 30)
		{
			// for now, just cry about it and quit
			dbg_print(TERM_RED, "Lost connection with the reflector\nExiting\n");

			// cleanup gpios
			gpio_cleanup();

			// close log file if necessary
			if (logfile != NULL)
			{
				fclose(logfile);
			}

			exit(EXIT_FAILURE);
		}
	}

	// should never get here
	return 0;
}

