#define BR1200 (1200)
#define BR2400 (2400)
#define BR4800 (4800)
#define BR9600 (9600)
#define BR19200 (19200)
#define BR28800 (28800)
#define BR38400 (38400)
#define BR57600 (57600)
#define BR115200 (115200)
#define BR625000 (625000)
#define BR1000000 (1000000)
#define BR1250000 (1250000)
#define BR2500000 (2500000)
#define BR3000000 (3000000)
#define BR3125000 (3125000)
#define PARITY_NONE (0)
#define PARITY_ODD (1)
#define PARITY_EVEN (2)
#define UART_COM (1)
#define UART_RX (2)
#define UART_SIO1 (3)
#define UART_SIO2 (4)
#define UART_SIO3 (5)
#define UART_SIO4 (6)
typedef enum {
	UART = 0,
	RS485 = 1,
	ICS = 2
} uart_mode;

extern bool uart_init(int port, uart_mode mode, unsigned int baudrate, unsigned
	int data, unsigned int parity);
extern bool uart_tx(int port, unsigned char* tx, int start, int length);
extern bool uart_rx(int port, unsigned char* rx, int length, unsigned long timeout);
