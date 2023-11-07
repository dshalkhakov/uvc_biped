typedef float vec2_t[2];

#define Vec2Length(x,y)		(sqrt((x)*(x) + (y)*(y)))
#define Vec2LengthSquared(x,y)	( (x)*(x) + (y)*(y) )
#define Vec2Set(v,x,y)		( (v)[0]=(x), (v)[1]=(y) )

extern vec2_t vec2_zero;
extern float clamp(float value, float min, float max);

// svangle_t is radians * SVANGLE_MULTIPLIER. range is -4000..4000, representing max 
// servo rotation of 270 degrees, midpoint is zero.
typedef int16_t svangle_t;

// angle expressed in radians
typedef float radangle_t;

// hwsvangle_t is radians * SV_ANGLE_MULTIPLIER + 7500. range is 3500..11500, representing max 
// servo rotation of 270 degrees, midpoint is 7500.
typedef int16_t hwsvangle_t;

// hwangle_t is representation of angle used by BNO55. degrees * 16.0
typedef int16_t	hwangle_t;

#define HWANGLE_180			(2880)
#define HWANGLE_MULTIPLIER	(16.0f)
#define HWANGLE_RECIPROCAL	(1.0f/16.0f)
#define TO_HWANGLE(x)		((x) * HWANGLE_MULTIPLIER)
#define FROM_HWANGLE(x)		((x) * HWANGLE_RECIPROCAL)
#define HWANGLE_TORAD(x)	((float)x*(M_PI/(180.0*16.0)))

#define SVANGLE_180			(5331) // 180 degrees. might not be entirely accurate.
#define SVANGLE_MULTIPLIER	(29.62f)
#define SVANGLE_RECIPROCAL	(1.0f/29.62f)
#define ANGLE2SVANGLE(x)	((x) * SVANGLE_MULTIPLIER)
#define SVANGLE2ANGLE(x)	((x) * SVANGLE_RECIPROCAL)

#define DEGREES2RADIANS(deg)	((deg) * (M_PI/180.0f))
#define RADIANS2DEGREES(rad)	((rad) * (180.0f/M_PI))

// biped servo state. these variables seem to represent servo angles in 'normalized' form 
typedef struct state_s {
	svangle_t K0W[2];			// 股関節前後方向書込用 hip pitch
	svangle_t K1W[2];			// 股関節横方向書込用 hip roll
	svangle_t K2W[2];			// 股関節横方向書込用 hip yaw
	svangle_t HW[2];			// 膝関節書込用 For knee joint writing
	svangle_t A0W[2];			// 足首上下方向書込用 ankle pitch
	svangle_t A1W[2];			// 足首横方向書込用 ankle roll
	svangle_t U0W[2];			// 肩前後方向書込用 shoulder pitch
	svangle_t U1W[2];			// 肩横後方向書込用 shoulder roll
	svangle_t U2W[2];			// 肩ヨー向書込用 shoulder yaw
	svangle_t EW[2];			// 肘書込用 For elbow writing
	svangle_t WESTW;			// 腰回転書込用 For waist rotation writing
	svangle_t HEADW;			// 頭回転書込用 For head rotation writing
	svangle_t K0R, K0RB, U0R, U0L, U1R, U1L, EWS;
	svangle_t K0L, K0LB, U0WB;
	svangle_t K0WB;
} state_t;

typedef struct core_s {
	// jikuasi is a Japanese translation of the pivot foot and represents the foot on the grounded side.
	// When jikuasi is 0, the right foot is in contact with the ground, and when it is 1,
	// the left foot is in contact with the ground.
	int16_t jikuasi;
	int16_t motCt, motCtBak, motCtBak2, motCtdat;
	int16_t mode, modeNxt, subMode; // keyMode
	int16_t	pitch_gyr, roll_gyr, yaw_gyr;
	int16_t	cycle, tst0;
	int16_t	walkCt, walkCtLim;		// 歩数 number of steps
	hwangle_t	p_ofs, r_ofs;
	int16_t ir, ip, ira, ipa;
	int16_t irb, ipb, ct;
	hwangle_t	pitchs, rolls, pitch_ofs, roll_ofs, yaw, yaw_ofs;
	// landB is always 0
	int16_t	landF, landB;

	int32_t	tBak, pitchi;
	uint32_t tNow;

	//uint8_t cmd[2];
	//uint8_t ff[45];
	//uint8_t dsp[110];
	//uint8_t krr[4];
	//int8_t	kn;
	int8_t	LEDct;	// LED点灯カウンタ LED lighting counter

	float fwctEnd;					// DS: 一周期最大カウント数 Maximum count in one cycle
	float fwct;						// DS: 一周期カウンタ One cycle counter
	float fwctUp;
	radangle_t pitch, roll, pitcht, rollt;
	float pitch_gyrg, roll_gyrg;
	float wk, wt;
	// dyi is equivalent to I in PID control, and is for converting the tilt angle in the roll direction into the distance in the left-right direction and integrating it
	float dyi, dyib, dyis;
	// dxi is equivalent to I in PID control, and is for converting the tilt angle in the pitch direction into the distance in the front - back direction and integrating it
	float dxi, dxib, dxis;
	float rollg, fw, fwi, fws, sw, freeBak, tt0;
	float supportingLeg, swingLeg;	// 支持脚、遊脚股関節振出角度 Support leg, swing leg hip joint swing angle
	float footH;					// 脚上げ高さ leg lift height
	float swx, swy, swMax;			// 横振り巾 Horizontal width
	float autoH, fh, fhMax;			// 脚上げ高さ leg lift height
} core_t;

typedef struct input_s {
	uint8_t cmd[2];
	uint8_t ff[45];
	uint8_t krr[4];
	int8_t	kn;
	int16_t keyMode;
} input_t;

typedef struct {
	uint8_t dsp[110];
} globals_t;

extern uint8_t read8(input_t* input, uint8_t reg);
extern bool readLen(input_t* input, uint8_t reg, uint8_t len);
extern bool write8(input_t* input, uint8_t reg, uint8_t dat);

extern void movSv(core_t* core, short* s, int d);
extern void angAdj(core_t* core);
extern void detAng(core_t* core);
extern void uvcSub(core_t* core);
extern void uvcSub2(core_t* core, state_t* state);
extern void uvc(core_t* core);
extern void footUp(core_t* core);
extern void swCont(core_t* core);
extern void armCont(core_t* core, state_t* state);
extern void footCont(core_t* core, state_t* state, vec2_t p0, float h, int s);
extern void feetCont1(core_t* core, state_t* state, vec2_t p0, vec2_t p1, int s);
extern void feetCont2(core_t* core, state_t* state, int s);
extern void counterCont(core_t* core);
extern void walk(core_t* core, state_t* state);
extern void keyCont(input_t* input, core_t* core, state_t* state);
extern int32_t main_init(state_t* state, core_t* core, input_t* input);
extern void state_init(state_t* state);
extern void core_init(core_t* core);

extern int32_t main_step(state_t* state, core_t* core, input_t* input, int initialI);

extern int controllerMain();
