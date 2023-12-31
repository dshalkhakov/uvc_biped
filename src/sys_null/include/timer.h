#define TIMER (0xFF) // ポート番号に指定するため Specify timer port
#define TIMER_INT (0xAA) //ポートを指定せず、一定時間割り込みにつかう Use for interrupts for a certain amount of time without specifying the port
#define TIMER1 (0x00) // PA0
#define TIMER2 (0x01) // PA1
#define TIMER3 (0x06) // PA6
#define TIMER4 (0x07) // PA7
#define TIMER5 (0x10) // PB0
#define TIMER6 (0x11) // PB1

// モード 1
// モード 2
// モード 1 と OR で使用して T 端子から出力、入力を決める
// 不可能な組み合わせは初期化関数でエラーを発生させる
// PWM の INPUT はインプット・キャプチャ(開発中)
typedef enum {
	TIMER_MODE_TIMER16 = 0x0100,
	TIMER_MODE_TIMER32 = 0x0200,
	TIMER_MODE_ONEPULSE = 0x0400,
	TIMER_MODE_PWM = 0x0800,
	TIMER_MODE_OUTPUT = 0x1000,
	TIMER_MODE_INPUT = 0x2000,
	TIMER_MODE_INTERRUPT = 0x4000
} timer_mode;

// Function Initializes the timer.
// Arguments int port
// mode
// frequency: timer period. Specify the timer period in microseconds. For example, for a 1ms timer, please specify 1000.
extern bool timer_init(int port, timer_mode mode, int frequency);

// Function Writes the timer value of the specified port.
// Arguments int port.
// unsigned int data Timer value to write.
// Return value Always returns true.
extern bool timer_write(int port, unsigned int data);

// Function Starts the timer for the specified port.
// Arguments int port Specify the port on which to start the timer.
// Return value Always returns true.
extern int timer_start(int port);

// Reads the timer value of the specified port.
// Arguments int port Select the port to read from.
// Return value Returns the current timer value for the specified port.
extern unsigned int timer_read(int port);
