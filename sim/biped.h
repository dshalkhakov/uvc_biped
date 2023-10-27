#pragma once

typedef char bodyType_t;

#define BODYTYPE_BOX		('b')
#define BODYTYPE_CAPSULE	('c')
#define BODYTYPE_SPHERE		('s')
#define BODYTYPE_CYLINDER	('y')

typedef char bodyColor_t;

#define COLOR_WHITE			('w')
#define COLOR_RED			('r')
#define COLOR_GREEN			('g')
#define COLOR_YELLOW		('y')
#define COLOR_GREY			('d')
#define COLOR_BODY			('b')	// color depends on input uvc flag state

class bodyStr{
public:
	dBodyID	bodyId;		// ボディのID Body ID
	int		enabled;		// ボディ有効 body valid
	dGeomID	geometry;		// ジオメトリのID Geometry ID
	int		hasGeometry;		// ジオメトリ設定 有/無 Geometry settings presence/absence
	bodyType_t	bodyType;		// ボディの種類 Body type
	bodyColor_t	color;		// 色の種類 Color type
	double	length;		// 長さ Length
	double	width;		// 幅 Width
	double	height;		// 高さ Height
	double	radius;		// 半径 Radius
};

typedef char jointType_t;

#define JOINT_HINGE		('h')
#define JOINT_SLIDER	('d')
#define JOINT_FIXED		('f')
#define JOINT_ENVFIXED	('g')

typedef enum axis_s {
	AXIS_X	= 0,
	AXIS_Y	= 1,
	AXIS_Z	= 2
} axis_t;

class jointStr{
public:
	dJointID jointId;		// ジョイントID Joint ID
	jointType_t	jointType;		// ジョイントの種類 Joint type
	int		enabled;		// ジョイント有効 joint enabled
	double	origin_x;		// X座標 X coordinate
	double	origin_y;		// Y座標 Y coordinate
	double	origin_z;		// Z座標 Z coordinate
	double	dn;		// 下限角度 lower limit angle
	double	up;		// 上限角度 upper limit angle
	double	t_jointAngle;		// 関節角度 Joint angle
	double	t2_jointAngle;		// 関節2角度(ユニバーサル時) Joint 2 angle (when universal)
	double	tm;		// 最大角速度 maximum angular velocity
	double	tm2;	// 最大角速度 maximum angular velocity
	double	torque_tk;		// 関節トルク Joint torque
	double	torque_tk2;	// 関節トルク Joint torque
	int		s_flag;		// 特別制御フラグ(1の場合、歩行制御特別ジョイント) Special control flag (if 1, walking control special joint)
	int		c_counter;		// 汎用カウンタ General purpose counter
	int		mode;	// 駆動モード drive mode
	int		pn_footPressureCounter;		// 足圧力カウンタ foot pressure counter
	int		sv_servoId;		// サーボ識別 servo identification
};

typedef struct state_s {
	// 間接角度 indirect angle
	double K0W[2];	// 股関節前後方向書込用 For writing in the anteroposterior direction of the hip joint
	double K1W[2];	// 股関節横方向書込用 For hip joint lateral writing
	// double K2W[2];	// 股関節横方向書込用 For hip joint lateral writing
	double HW[2];	// 膝関節書込用 For knee joint writing
	double A0W[2];	// 足首上下方向書込用 For writing in the upper and lower direction of the ankle
	double A1W[2];	// 足首横方向書込用 For ankle lateral writing
	double U0W[2];	// 肩前後方向書込用 For writing in shoulder anteroposterior direction
	double U1W[2];	// 肩横後方向書込用 For shoulder horizontal and posterior writing
	double U2W[2];	// 肩ヨー向書込用 For writing in shoulder yaw direction

	// センサ関連 Sensor related
	double fbRad = 0;			// 頭前後角度 head front and back angle
	double lrRad = 0;			// 頭左右角度 head left and right angle
	double fbAV = 0;			// 頭前後角速度 head front and rear angular velocity
	double lrAV = 0;			// 頭左右角速度 Head left and right angular velocity
	double asiPress_r = 0;	// 右足裏圧力 right foot pressure
	double asiPress_l = 0;	// 左足裏圧力 left foot pressure
} state_t;

extern state_t state;

typedef struct input_s {
	int		uvcOff = 0;		// UVC起動フラグ UVC activation flag
	unsigned char walkF = 0;	// 歩行フラグ	（b0:歩行  b1:未  b2:未）walk flag (b0:walk b1:not yet b2:not yet)
	double	frRatI;			// 上体角補正用積分係数 Integral coefficient for body angle correction
	double	frRatA;			// 上体角オフセット値 Upper body angle offset value
	double	fhRat;			// 足上げ高さ補正値 Foot lift height correction value
	double	fwMax;			// 歩幅最大値 Maximum stride length
} input_t;

extern input_t input;
