#define ICS20 (20)
#define ICS21 (30)
#define ICS22 (35)
#define ICS_MAX_ID (32)
#define ICS_MAX_POS_VALUE (0x3FFF)
#define ICS_POS_CMD (0x80) // ポジション設定コマンド Position setting command
#define ICS_POS_BYTE (3)
#define ICS_GET_PARAM_CMD (0xA0) // パラメータ読み書きコマンド Parameter read/write command
#define ICS_SET_PARAM_CMD (0xC0)
#define ICS_EEPROM_SC (0) // パラメータ読み書きサブコマンド Parameter read/write subcommand
#define ICS_STRC_SC (1) // STRETCH 値読み書きサブコマンド STRETCH value read/write subcommand
#define ICS_SPD_SC (2) // SPEED 値読み書きサブコマンド SPEED value read/write subcommand
#define ICS_CURNT_SC (3) // 電流値・電流制限値読み書きサブコマンド Current value/current limit value read/write subcommand
#define ICS_TMPR_SC (4) // 温度値・温度制限値読み書きサブコマンド Temperature value/temperature limit value reading/writing subcommand
#define ICS_GENE_SC (0x7F) // 汎用データ読み書きサブコマンド General purpose data read/write subcommand
#define ICS_PARAM_BYTE (2) // パラメータ読み書きデータサイズ Parameter read/write data size
#define ICS_EEPROM_BYTE (60)
#define ICS35_EEPROM_BYTE (66)
#define ICS_ID_CMD (0xE0) // ID コマンド  ID command
#define ICS_GET_ID_SC (0) // ID 取得サブコマンド ID acquisition subcommand
#define ICS_SET_ID_SC (1) // ID 設定サブコマンド ID setting subcommand

// Function Sets the position of the ICS servo motor and retrieves the current position.
// Arguments: int port Select the port from UART_SIO1, UART_SIO2, UART_SIO3, or UART_SIO4.
// byte id Specify her ID number for the device (0-31)
// int pos Specify the position in the range from 3500 to 11500. Align with the origin (neutral) of the ICS servo motor. (which is 7500? -DS)
// If you send 0, the servo motor will be de-energized and the current position will be returned.
// Return value The current position(3500 to 11500) of the ICS servo motor is returned. -1 is returned if acquisition fails.
// Remarks If the return value is -1, check the ID number and communication speed of the ICS servo motor.
extern int ics_set_pos(int port, unsigned char id, unsigned short pos);

// Function Sets the parameters of the ICS servo motor
// Argument int port Select the port from UART_SIO1, UART_SIO2, UART_SIO3, or UART_SIO4.
// byte id Specify her ID number for the device(0 - 31)
// byte sc Select from ICS_STRC_SC, ICS_SPD_SC, ICS_CURNT_SC, ICS_TMPR_SC
// (ICS_EEPROM_SC cannot be used to prevent rewriting)
// byte param Enter the value of the parameter
// Return value The parameter specified by sc is returned.If no data is returned, -1 is returned.
// Remarks Please note that the parameters that can be set are different between ICS3.5 and ICS 3.0.
extern int ics_set_param(int port, unsigned char id, unsigned char sc, unsigned char param);

// Function Initializes the UART_SIO port for ICS servo use.
// Argument int port Select the port from UART_SIO1, UART_SIO2, UART_SIO3, or UART_SIO4.
// int baudrate Select the communication speed from 115200, 625000, or 1250000. Set the communication speed on the servo motor side.
// please confirm.
// Return value Returns true if port initialization is successful.
extern bool sio_init(int port, int baudrate);
