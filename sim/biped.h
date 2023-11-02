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

typedef struct bipedstate_s {
	// 間接角度 indirect angle
	double K0W[2];	// 股関節前後方向書込用 hip pitch
	double K1W[2];	// 股関節横方向書込用 hip roll
	// double K2W[2];	// 股関節横方向書込用 hip yaw
	double HW[2];	// 膝関節書込用 For knee joint writing
	double A0W[2];	// 足首上下方向書込用 ankle pitch
	double A1W[2];	// 足首横方向書込用 ankle roll
	double U0W[2];	// 肩前後方向書込用 shoulder pitch
	double U1W[2];	// 肩横後方向書込用 shoulder roll
	double U2W[2];	// 肩ヨー向書込用 shoulder yaw

	// センサ関連 Sensor related
	double fbRad = 0;			// 頭前後角度 head front and back angle
	double lrRad = 0;			// 頭左右角度 head left and right angle
	double fbAV = 0;			// 頭前後角速度 head front and rear angular velocity
	double lrAV = 0;			// 頭左右角速度 Head left and right angular velocity
	double asiPress_r = 0;	// 右足裏圧力 right foot pressure
	double asiPress_l = 0;	// 左足裏圧力 left foot pressure
} bipedstate_t;

extern bipedstate_t bipedstate;

typedef struct bipedinput_s {
	int		uvcOff = 0;		// UVC起動フラグ UVC activation flag
	unsigned char walkF = 0;	// 歩行フラグ	（b0:歩行  b1:未  b2:未）walk flag (b0:walk b1:not yet b2:not yet)
	double	frRatI;			// 上体角補正用積分係数 Integral coefficient for body angle correction
	double	frRatA;			// 上体角オフセット値 Upper body angle offset value
	double	fhRat;			// 足上げ高さ補正値 Foot lift height correction value
	double	fwMax;			// 歩幅最大値 Maximum stride length
} bipedinput_t;

extern bipedinput_t bipedinput;

typedef struct biped_s {
	bodyStr solep_r;	// 足裏圧力センサ Sole pressure sensor
	bodyStr solep_l;
	bodyStr sole_r;	// 足裏 sole
	bodyStr sole_l;
	bodyStr A1_r;		// 足首ロール	 ankle roll
	bodyStr A1_l;
	bodyStr A0_r;		// 足首ピッチ ankle pitch
	bodyStr A0_l;
	bodyStr S_r;		// 脛 shin
	bodyStr S_l;
	bodyStr H_r;		// 膝 knees
	bodyStr H_l;
	bodyStr M_r;		// 腿 leg
	bodyStr M_l;
	bodyStr K0_r;		// 股関節ピッチ hip pitch
	bodyStr K0_l;
	bodyStr K1_r;		// 股関節ロール hip roll
	bodyStr K1_l;
	bodyStr DOU;		// 胴 torso
	bodyStr HEADT;	// 頭 head

	jointStr soleJ_r;	// 足裏センサ sole sensor
	jointStr soleJ_l;
	jointStr A1J_r;		// 足首ロール Ankle roll
	jointStr A1J_l;
	jointStr A0J_r;		// 足首ピッチ Ankle pitch
	jointStr A0J_l;
	jointStr SJ_r;		// 脛固定 Shin fixation
	jointStr SJ_l;
	jointStr HJ_r;		// 膝 knee HW
	jointStr HJ_l;		// knee HW
	jointStr MJ_r;		// 腿結合 Thigh joint
	jointStr MJ_l;
	jointStr M2J_r;		// 腿結合2 Thigh joint 2
	jointStr M2J_l;
	jointStr K0J_r;		// 股関節ピッチ Hip joint pitch
	jointStr K0J_l;
	jointStr K1J_r;		// 股関節ロール Hip roll
	jointStr K1J_l;
	jointStr K2J_r;		// 股関節ヨー Hip joint yaw
	jointStr K2J_l;
	jointStr HEADJ;		// 頭固定 Head fixation
} biped_t;

typedef struct world_s {
	int		bodyCount;		// ボディ配列カウント値 Body array count value
	int		jointCount;		// ジョイント配列カウント値 Joint array count value

	bodyStr* body[50];	// bodyStrアドレス格納配列 bodyStr address storage array
	jointStr* joint[50];// jointStrアドレス格納配列 jointStr address storage array

	double softERP;			// 柔らかさ、沈み込み softness, sinking
	double softCFM;			// 柔らかさ、弾力 softness, elasticity
	double bounce;			// 反発係数 coefficient of repulsion
	double bounce_vel;		// 反発最低速度 minimum bounce speed
} world_t;

extern world_t g_world;
