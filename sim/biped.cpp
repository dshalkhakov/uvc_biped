/*
June 1,2021
Delete variable K2Ｗ[2]
Fixed restart ()

ODEによるUVC(上体垂直制御)の検証 Verification of UVC (upper body vertical control) by ODE
UVCの応用、登坂実験 2021 4/29 Application of UVC, hill climbing experiment 2021 4/29
*/

#define USING_MAIN (1)

#include < ode/ode.h >
#include < drawstuff/drawstuff.h >
#include < stdio.h >
#include < stdlib.h >
#include < windows.h >
#include < fstream >
#include < math.h >
#include < conio.h >
#include  "biped.h"
#include  "core.h"

#ifdef USING_MAIN
extern "C" {
#include <uart.h>
#include "../src/main.h"
#include "../src/sys_sim/sim_ics.h"
#include "../src/sys_sim/sim_i2c.h"
}

state_t	g_mainstate;
input_t	g_maininput;
core_t	g_maincore;
int16_t g_main_initialI;
#endif // !USING_MAIN

// 関数定義 Function definition
static	void command		(int cmd);
static	void nearCallback	(void *data, dGeomID o1, dGeomID o2);
static	void simLoop		(int pause);
static	void setJoint		(jointStr *j, jointType_t k, bodyStr *b1, bodyStr *b2, axis_t a, double x, double y, double z);
static	void setBody		(bodyStr  *b, bodyType_t k, bodyColor_t	 c,	 double l,	 double w, double h, double r, double x, double y, double z,  int ge,   double ma);
static	void createBody		(biped_t* biped);
void		 destroyBot		();
void		 restart		();
static	void start			();

// select correct drawing functions
#ifdef  dDOUBLE			// 単精度と倍精度の両方に対応するためのおまじない Character for supporting both single precision and double precision
#define dsDrawBox		dsDrawBoxD
#define dsDrawSphere	dsDrawSphereD
#define dsDrawCylinder	dsDrawCylinderD
#define dsDrawCapsule	dsDrawCapsuleD
#endif

bipedstate_t bipedstate;

void bipedstate_init(bipedstate_t* state) {
	// 間接角度 indirect angle
	state->K0W[0] = 0;	// 股関節前後方向書込用 For writing in the anteroposterior direction of the hip joint
	state->K0W[1] = 0;
	state->K1W[0] = 0;	// 股関節横方向書込用 For hip joint lateral writing
	state->K1W[1] = 0;
	// double state->K2W={0,0};	// 股関節横方向書込用 For hip joint lateral writing
	state->HW[0] = 0;	// 膝関節書込用 For knee joint writing
	state->HW[1] = 0;
	state->A0W[0] = 0;	// 足首上下方向書込用 For writing in the upper and lower direction of the ankle
	state->A0W[1] = 0;
	state->A1W[0] = 0;	// 足首横方向書込用 For ankle lateral writing
	state->A1W[1] = 0;
	state->U0W[0] = 0;	// 肩前後方向書込用 For writing in shoulder anteroposterior direction
	state->U0W[1] = 0;
	state->U1W[0] = 0;	// 肩横後方向書込用 For shoulder horizontal and posterior writing
	state->U1W[1] = 0;
	state->U2W[0] = 0;	// 肩ヨー向書込用 For writing in shoulder yaw direction
	state->U2W[1] = 0;

	// センサ関連 Sensor related
	state->fbRad = 0;			// 頭前後角度 head front and back angle
	state->lrRad = 0;			// 頭左右角度 head left and right angle
	state->fbAV = 0;			// 頭前後角速度 head front and rear angular velocity
	state->lrAV = 0;			// 頭左右角速度 Head left and right angular velocity
	state->asiPress_r = 0;	// 右足裏圧力 right foot pressure
	state->asiPress_l = 0;	// 左足裏圧力 left foot pressure
}

// 各種変数定義 Various variable definitions
world_t g_world;

void world_addBody(world_t* world, bodyStr* body) {
	world->body[world->bodyCount] = body;	// 構造体のアドレスを格納する Store address of structure
	++world->bodyCount;			// ボディ数カウントアップ body count up
}

void world_addJoint(world_t* world, jointStr* joint) {
	world->joint[world->jointCount] = joint;	// 配列のアドレスを格納する Storing the address of the array
	++world->jointCount;			// 配列カウンタ、カウントアップ array counter, count up
}

static dWorldID world;				// 動力学計算用ワールド World for dynamics calculation
static dSpaceID world_space;				// 衝突検出用スペース Space for collision detection
static dJointGroupID world_contactgroup;	// コンタクトグループ Contact group
static dGeomID world_ground;				// 地面 ground

dMatrix3 world_R;
const double* temp_Rot;		// 回転行列取得 Get rotation matrix
static struct dJointFeedback feedback[50];	// ジョイントフィードバック構造体 Joint feedback structure
bipedinput_t bipedinput;

void bipedinput_init(bipedinput_t* bipedinput) {
	bipedinput->uvcOff = 0;		// UVC起動フラグ UVC activation flag
	bipedinput->walkF = 0;		// 歩行フラグ	（b0:歩行  b1:未  b2:未）walk flag (b0:walk b1:not yet b2:not yet)

	bipedinput->walkF = 0;
	bipedinput->frRatI = 0;
	bipedinput->frRatA = 0;
}

//###############  各種構造体 Various structures ###############
biped_t biped;

bodyStr platform_DAI;		// 台 platform
bodyStr platform_DAI2;		// 台 platform
bodyStr barrier_base;		// 遮断機柱 barrier pillar
bodyStr barrier_pole;		// 遮断機棒 barrier rod

jointStr platform_DAIJ;		// 台の固定 Fixing the stand
jointStr platform_DAI2J;		// 台2の固定 Fixing stand 2
jointStr barrier_baseJ;		// 柱の固定 Fixing the column
jointStr barrier_poleJ;		// ポールのジョイント pole joint

//###############  クラスの実体化　class instantiation ###############
core g_co;			// 歩行制御クラスの実体化 Instantiation of gait control class

//--------------------------------- command ----------------------------------------
static void command (int cmd){
	static int mag = 30;

	switch (cmd) {
#ifdef USING_MAIN
		case 'g':case 'G':
			printf("=============== main walk\n");
			g_maincore.mode = 740; // switch to walking
			break;
#endif
		//// 外力印加 External input ////
		case 'j':case 'J':
			printf("F<-\n"); // Ｆ←\n
			dBodyAddForce(biped.DOU.bodyId, -20,0,0);
			break;
		case 'k':case 'K':
			printf("F->\n"); // Ｆ→\n
			dBodyAddForce(biped.DOU.bodyId,  20,0,0);
			break;
		case 'p':case 'P':
			printf("F^\n"); // Ｆ↑
			dBodyAddForce(biped.DOU.bodyId, 0,20,0);
			break;
		case 'l':case 'L':
			printf("F.\n"); // Ｆ↓
			dBodyAddForce(biped.DOU.bodyId,  0,-20,0);
			break;

		//// 操作 operate ////
		case 'w':				// 歩行開始 Start walking
			printf("Start walking\n"); // 歩行開始
			bipedinput.walkF=0x01;
			barrier_poleJ.t_jointAngle=5;
			break;

		case '1':				//外力実験 External force experiment
			printf("External force experiment\n"); // 外力実験
			restart ();
			bipedinput.fwMax=30;
			bipedinput.frRatA=0;
			bipedinput.frRatI=0;
			bipedinput.fhRat=0;
			setBody  (&barrier_base,		BODYTYPE_CYLINDER,COLOR_GREY,	260,	 0,	  0,	24,		0,	180,	110,		0,	0.01);	//遮断機棒
			setBody  (&barrier_pole,		BODYTYPE_CYLINDER,COLOR_YELLOW,	320,	 0,	  0,	8,		0,	300,	210,		1,	0.0001);	//ボール
			dRFromAxisAndAngle(world_R, 1, 0, 0, -M_PI_2);// 回転 rotate
			dBodySetRotation(barrier_pole.bodyId, world_R);
			setJoint(&barrier_baseJ,			JOINT_ENVFIXED,	&barrier_base,	&barrier_base,	AXIS_X,		0,	180,	0);		//遮断機柱固定用
			setJoint(&barrier_poleJ,			JOINT_HINGE,	&barrier_pole,	&barrier_base,	AXIS_Z,		0,	180,	210);	//遮断機棒ヒンジ
			barrier_poleJ.tm=1;
			barrier_poleJ.torque_tk=0.25;
			break;

		case '2':				// 前進実験 forward experiment
			printf("Forward experiment\n"); // 前進実験
			restart ();
			bipedinput.fwMax=30;
			bipedinput.frRatA=0.0015;
			bipedinput.frRatI=0;
			bipedinput.fhRat=0;
			goto saka;
			
		case '3':				// 登坂実験 Climbing experiment
			printf("Climbing experiment\n"); // 登坂実験
			restart ();
			bipedinput.fwMax=30;
			bipedinput.frRatA=0.0015;
			bipedinput.frRatI=0.2;
			bipedinput.fhRat=0;
			goto saka;

		case '4':				// 段差実験 step experiment
			printf("Step experiment\n"); // 段差実験
			restart ();
			bipedinput.fwMax=30;
			bipedinput.frRatA=0.0015;
			bipedinput.frRatI=0.2;
			bipedinput.fhRat=7;
			setBody  (&platform_DAI2,		BODYTYPE_BOX,COLOR_GREEN,500,300,	 20,0,		0,	0,	-10,		 1,	100);	//水平台
			setJoint(&platform_DAI2J,		JOINT_ENVFIXED,	&platform_DAI2,&platform_DAI2,	AXIS_X,	50,	0,	0);						//台固定用
			break;

		case '5':				// 最終実験 final experiment
			printf("Final experiment\n"); // 最終実験
			restart ();
			bipedinput.fwMax=30;
			bipedinput.frRatI=0.2;
			bipedinput.frRatA=0.0015;
			bipedinput.fhRat=7;
saka:	// slope/hill
			setBody  (&platform_DAI,			BODYTYPE_BOX,COLOR_GREEN,680,300,	 100,   0,	 480,	0,   -70,	1,	100);		//傾斜台
			dRFromAxisAndAngle(world_R, 0, 1, 0, -0.05);// 回転 rotate
			dBodySetRotation(platform_DAI.bodyId, world_R);
			setJoint(&platform_DAIJ,			JOINT_ENVFIXED,	&platform_DAI,		&platform_DAI,	AXIS_X,	  50,	0,	   0);				//台固定用
			break;

		case 'r':case 'R':		// 初期化
			printf("Initialization\n"); // 初期化
			restart ();
			break;
		case 'q':case 'Q':		// 終了
			printf("End\n"); // 終了
			exit(0);
			break;
		case 'u':case 'U':		// UVC ON/OFF
			printf("UVC ON/OFF\n");
			if(bipedinput.uvcOff==0){
				bipedinput.uvcOff=1;
			}
			else{
				bipedinput.uvcOff=0;
			}
			break;
	}
}


//----------------------------------- nearCallback --------------------------------------
static void nearCallback (void *data, dGeomID o1, dGeomID o2){
	int i,n;
	const int N = 10;
	dContact contact[N];
	dBodyID b1 = dGeomGetBody(o1);
	dBodyID b2 = dGeomGetBody(o2);

	if (b1 && b2 && dAreConnectedExcluding(b1,b2,dJointTypeContact)) return;
	n = dCollide (o1,o2,N,&contact[0].geom,sizeof(dContact));
	if (n > 0) {
		for (i=0; i<n; i++) {
			contact[i].surface.mode		= dContactBounce | dContactSoftERP | dContactSoftCFM;
			contact[i].surface.soft_cfm	= 0.00005;												// 柔らかさ、弾力 softness, elasticity
			contact[i].surface.soft_erp	= 0.1;													// 柔らかさ、沈み込み softness, sinking
			if(0 != barrier_pole.bodyId &&(b1==barrier_pole.bodyId||b2==barrier_pole.bodyId))	contact[i].surface.mu		= 0.2;			// ポールの摩擦 pole friction
			else									contact[i].surface.mu		= 5;			// 地面間摩擦 friction between floors
			contact[i].surface.bounce		= 0;												// bouncing the objects
			contact[i].surface.bounce_vel	= 0;												// bouncing velocity
			dJointID c = dJointCreateContact (world,world_contactgroup,&contact[i]);					// ジョイント生成 Joint generation
			dJointAttach (c,dGeomGetBody(contact[i].geom.g1),dGeomGetBody(contact[i].geom.g2));	// ジョイントを結合する join joints
	    }
	}
}


//--------------------------------- control ----------------------------------------
// hinge control 
static void control(){
	double kp = 100.0;
	double k;
	int i;
	jointStr* joint;
	for (i=0;i<g_world.jointCount;i++){
		joint = g_world.joint[i];
		switch (joint->jointType) {
			case JOINT_HINGE:
				k = kp * (joint->t_jointAngle - dJointGetHingeAngle(joint->jointId));
				if(abs(k) > joint->tm){
					if(k > 0){k = joint->tm;}
					else{k = -joint->tm;}
				}
				dJointSetHingeParam(joint->jointId, dParamVel, k );
				dJointSetHingeParam(joint->jointId, dParamFMax, joint->torque_tk);
				break;
			case JOINT_SLIDER:
				k = joint->t_jointAngle - dJointGetSliderPosition(joint->jointId);
				dJointSetSliderParam(joint->jointId, dParamVel,  k * 100);
				dJointSetSliderParam(joint->jointId, dParamFMax, 300);
				break;
		}
	}
}

//--------------------------------- simLoop ----------------------------------------
//	simulation loop
static void simLoop (int pause){
	int i;
	char inputKey;
	static int mag = 3;

	double sides[3];
	dVector3 headVel1;
	dVector3 headVel2;

//	Sleep(1);			// 描画速度の調整 Adjusting drawing speed
	if(_kbhit()){
		inputKey=getchar();	// キー読込 key reading
		command (inputKey);
	}

	if (!pause) {
		//******** この３行は最初に置くべし These three lines should be placed first ********
		dSpaceCollide (world_space,0,&nearCallback);	// 衝突しそうなジオメトリのペア集団を探す Find pairs of geometries that are likely to collide
		dWorldStep (world,0.01);				// シミュレーションを１ステップ指定時間進める Advance the simulation by one step for the specified time
		dJointGroupEmpty (world_contactgroup);		// ジョイントグループを空にする Empty the joint group

		//******** 足裏圧力検出 Sole pressure detection ********
		dJointFeedback* fb1 = dJointGetFeedback(biped.soleJ_r.jointId);
		bipedstate.asiPress_r = fb1->f1[2];				// 右足(足首Ｚ)圧力 Right foot (ankle Z) pressure
		dJointFeedback* fb2 = dJointGetFeedback(biped.soleJ_l.jointId);
		bipedstate.asiPress_l = fb2->f1[2];				// 左足(足首Ｚ)圧力 Left foot (ankle Z) pressure

		//******** 頭前後左右角度検出 Head front, back, left, right, and right angles ********
		temp_Rot = dBodyGetRotation(biped.HEADT.bodyId);		// 回転行列取得 Get rotation matrix
		bipedstate.fbRad = asin(temp_Rot[8]);					// 頭前後角度(後ろに仰向きが正) Anteroposterior angle of the head (positive when facing backwards)
		bipedstate.lrRad = asin(temp_Rot[9]);					// 頭左右角度（右傾きが正） Head left/right angle (right tilt is positive)

		//******** 頭前後左右角速度検出 Head front, rear, left and right angular velocity ********
		temp_Rot = dBodyGetAngularVel(biped.HEADT.bodyId);	// 回転行列取得 Get rotation matrix
		bipedstate.fbAV = temp_Rot[1];						// 頭前後角度(後ろに仰向きが負) Head anteroposterior angle (negative when facing backwards)
		bipedstate.lrAV = temp_Rot[0];						// 頭左右角度（右傾きが正） Head left/right angle (right tilt is positive)

#ifdef USING_MAIN
		bipedstate.K0W[0] = DEGREES2RADIANS(SVANGLE2ANGLE(g_mainstate.K0W[0]));
		bipedstate.K0W[1] = DEGREES2RADIANS(SVANGLE2ANGLE(g_mainstate.K0W[1]));
		bipedstate.K1W[0] = DEGREES2RADIANS(SVANGLE2ANGLE(g_mainstate.K1W[0]));
		bipedstate.K1W[1] = DEGREES2RADIANS(SVANGLE2ANGLE(g_mainstate.K1W[1]));
		//bipedstate.K2W[0] = DEGREES2RADIANS(SVANGLE2ANGLE(g_mainstate.K2W[0]));
		bipedstate.HW[0] = DEGREES2RADIANS(SVANGLE2ANGLE(g_mainstate.HW[0]));
		bipedstate.HW[1] = DEGREES2RADIANS(SVANGLE2ANGLE(g_mainstate.HW[1]));
		bipedstate.A0W[0] = DEGREES2RADIANS(SVANGLE2ANGLE(g_mainstate.A0W[0]));
		bipedstate.A0W[1] = DEGREES2RADIANS(SVANGLE2ANGLE(g_mainstate.A0W[1]));
		bipedstate.A1W[0] = DEGREES2RADIANS(SVANGLE2ANGLE(g_mainstate.A1W[0]));
		bipedstate.A1W[1] = DEGREES2RADIANS(SVANGLE2ANGLE(g_mainstate.A1W[1]));
#endif

		biped.K0J_r.t_jointAngle	=bipedstate.K0W[0];			// 股関節前後方向書込用 For writing in hip joint anteroposterior direction
		biped.K1J_r.t_jointAngle	=bipedstate.K1W[0];			// 股関節横方向書込用 For hip joint lateral writing
		biped.HJ_r.t_jointAngle	=bipedstate.HW [0];			// 膝関節書込用 For writing knee joint
		biped.A0J_r.t_jointAngle	=bipedstate.A0W[0];			// 足首上下方向書込用 For writing in ankle up and down direction
		biped.A1J_r.t_jointAngle	=bipedstate.A1W[0];			// 足首横方向書込用 For ankle lateral writing

		biped.K0J_l.t_jointAngle	=bipedstate.K0W[1];			// 股関節前後方向書込用 For writing in hip joint anteroposterior direction
		biped.K1J_l.t_jointAngle	=bipedstate.K1W[1];			// 股関節横方向書込用 For hip joint lateral writing
		biped.HJ_l.t_jointAngle	=bipedstate.HW [1];			// 膝関節書込用  For writing knee joint
		biped.A0J_l.t_jointAngle	=bipedstate.A0W[1];			// 足首上下方向書込用 For writing in ankle up and down direction
		biped.A1J_l.t_jointAngle	=bipedstate.A1W[1];			// 足首横方向書込用 For ankle lateral writing

#ifdef USING_MAIN
		// map input to maininput
		// map core to maincore
		g_main_initialI = main_step(&g_mainstate, &g_maincore, &g_maininput, g_main_initialI);
#else
		g_co.walk(&bipedstate, &bipedinput);				// 歩行制御 Walk control
#endif
		control();				// モータ駆動 motor drive
	}

	for (i=0;i<g_world.bodyCount;i++){
		bodyStr* body = g_world.body[i];
		switch (body->color) {
			case COLOR_GREEN:
				dsSetColor (0,1,0);
				break;
			case COLOR_RED:
				dsSetColor (1,0,0);
				break;
			case COLOR_BODY:
				if(bipedinput.uvcOff==0)	dsSetColor(0.3 ,0.3, 2.0);			// 青 green
				else			dsSetColor(2.0, 0.3, 0.3);			// 赤 red
				break;
			case COLOR_YELLOW:
	 			dsSetColor (1,1,0);
				break;
			case COLOR_WHITE:
	 			dsSetColor (1,1,1);
				break;
			case COLOR_GREY:
	 			dsSetColor (0.8,0.8,0.8);
				break;
			default:
				break;
		}
		switch (body->bodyType) {
			case BODYTYPE_BOX: // 'b':
				sides[0] = body->length; sides[1] = body->width; sides[2] = body->height;
				dsDrawBox (dBodyGetPosition(body->bodyId),dBodyGetRotation(body->bodyId),sides);						//箱形表示
				break;
			case BODYTYPE_SPHERE: // 's':
	 			dsDrawSphere (dBodyGetPosition(body->bodyId),dBodyGetRotation(body->bodyId),body->radius);				//球形表示
				break;
			case BODYTYPE_CAPSULE: // 'c':
				dsDrawCapsule (dBodyGetPosition(body->bodyId),dBodyGetRotation(body->bodyId),body->length,body->radius);	//カプセル形表示
				break;
			case BODYTYPE_CYLINDER: // 'y':
				dsDrawCylinder (dBodyGetPosition(body->bodyId),dBodyGetRotation(body->bodyId), body->length, body->radius);	//円柱形表示
				break;
			default:
				break;
		}
	}
}


//----------------------------------- setBody --------------------------------------
//	配列にボディ情報を設定する Set body information in array

static void setBody (bodyStr *b, bodyType_t k,    bodyColor_t c, double l, double w, double h, double r, double x, double y, double z, int ge, double ma){
// 引数：　                   ボディの種類     色　    長さ　     幅　      高さ     半径　  前後位置   左右位置　上下位置  ジオメト　重量
// Arguments:				 Body      type       Color   Length    Width     Height    Radius    Fore/aft  Left/right Up/down  Geometry Weight
	dMass m;

	z += 20;

	// スケール調整 convert cm to m
	l/=1000;
	w/=1000;
	h/=1000;
	r/=1000;
	x/=1000;
	y/=1000;
	z/=1000;

	// 構造体に記録する record in structure
	b-> bodyType = k;			// ボディの種類を記録 Record body type
	b-> color = c;			// ボディの色の種類を記録 Record body color type
	b-> length = l;			// ボディの長さを記録 Record body length
	b-> width = w;			// ボディの幅さを記録 Record the width of the body
	b-> height = h;			// ボディの高さを記録 Record body height
	b-> radius = r;			// ボディの半径を記録 record body radius
	b-> hasGeometry = ge;			// ジオメトリ設定 有/無 Geometry settings Yes/No
	b-> enabled = 1;			// ボディ有効設定 Body enable setting

	x += 2;				// 2m手前に置いて地面障害物を避ける Place 2m in front to avoid ground obstacles

	world_addBody(&g_world, b);

	// ボディとジオメトリの生成と、質量、位置、関連性を設定 Generate bodies and geometry and set mass, position, and relationships
	switch (b->bodyType) {
		case BODYTYPE_BOX: //'b':	// 箱型 Box-shaped
			b->bodyId = dBodyCreate (world);						// 物体の存在を生成(ボディID設定) Generate the existence of an object (Body ID setting)
			dMassSetZero(&m);								// 質量パラメータ初期化 Mass parameter initialization
			dMassSetBoxTotal (&m,ma,b->length,b->width,b->height);		// 物体の重量設定 Setting the weight of an object
			dBodySetMass (b->bodyId,&m);							// 物体の重量分布設定 Object weight distribution settings
			dBodySetPosition (b->bodyId,x,y,(z));				// 物体の初期位置 initial position of object
			if(ge > 0){
				b->geometry = dCreateBox (world_space,b->length,b->width,b->height);	// 物体の幾何情報を生成（ジオメトリID設定） Generate geometric information of object (geometry ID setting)
				dGeomSetBody (b->geometry,b->bodyId);					// 物体の『存在』と『幾何情報』の一致 Matching the “existence” of an object and its “geometric information”
			}
			break;
		case BODYTYPE_SPHERE: // 's':	// 球形 spherical
			b->bodyId = dBodyCreate (world);						// 物体の存在を生成(ボディID設定) create object (set body id)
			dMassSetZero(&m);								// 質量パラメータ初期化 Mass parameter initialization
			dMassSetSphereTotal (&m,ma,b->radius);				// 物体の重量設定 Setting the weight of the object
			dBodySetMass (b->bodyId,&m);							// 物体の重量分布設定 Object weight distribution settings
			dBodySetPosition (b->bodyId,x,y,z);					// 物体の初期位置 Initial position of the object
			if(ge > 0){
				b->geometry = dCreateSphere (world_space,b->radius);			// 物体の幾何情報を生成（ジオメトリID設定） generate geometric information
				dGeomSetBody (b->geometry,b->bodyId);					// 物体の『存在』と『幾何情報』の一致 Match between the object's "existence" and "geometric information"
			}
			break;
		case BODYTYPE_CAPSULE: // 'c':	// カプセル形 capsule shape
			b->bodyId = dBodyCreate (world);						// 物体の存在を生成(ボディID設定) Create existence of object (set body ID
			dMassSetZero(&m);								// 質量パラメータ初期化 Mass parameter initialization
			dMassSetCapsuleTotal(&m,ma,3,b->radius,b->length);		// 物体の重量設定 Setting the weight of the object
			dBodySetMass (b->bodyId,&m);							// 物体の重量分布設定 Object weight distribution settings
			dBodySetPosition (b->bodyId,x,y,(b->length/2+z));			// 物体の初期位置 Initial position of the object
			if(ge > 0){
				b->geometry = dCreateCapsule (world_space,b->radius,b->length);	// 物体の幾何情報を生成（ジオメトリID設定） Generate geometric information of object (geometry ID setting)
				dGeomSetBody (b->geometry,b->bodyId);					// 物体の『存在』と『幾何情報』の一致 Match between the object's "existence" and "geometric information"
			}
			break;
		case BODYTYPE_CYLINDER: // 'y':	// 円柱形 cylindrical
			b->bodyId = dBodyCreate (world);						// 物体の存在を生成(ボディID設定) Create existence of object (set body ID)
			dMassSetZero(&m);								// 質量パラメータ初期化 Mass parameter initialization
			dMassSetCylinderTotal(&m,ma,3,b->radius,b->length);		// 物体の重量設定 Setting the weight of the object
			dBodySetMass (b->bodyId,&m);							// 物体の重量分布設定 Object weight distribution settings
			dBodySetPosition (b->bodyId,x,y,(z));				// 物体の初期位置 Initial position of the object
			if(ge > 0){
				b->geometry = dCreateCylinder (world_space,b->radius,b->length);	// 物体の幾何情報を生成（ジオメトリID設定）Generate geometric information of object (geometry ID setting)
				dGeomSetBody (b->geometry,b->bodyId);					// 物体の『存在』と『幾何情報』の一致 Match between the object's "existence" and "geometric information"
			}
			break;
		default:
			break;
	}
}


//---------------------------------- setJoint ---------------------------------------
//	ジョイントを生成する generate joint
// Parameters x, y, z are in cm

static void setJoint (jointStr *j, jointType_t k, bodyStr *b1, bodyStr *b2, axis_t a, double x, double y, double z){
// 引数：　            対象Joint　Joint種類   Body番号1  　Body番号2　 設定軸  前後位置  左右位置　上下位置
// Argument:      Target Joint  Joint type  Body #1    Body #2  Setting axis Front-back Left-right Up-down
	z+= 20; // place 20cm above ground

	// convert from cm to m
	x/=1000;
	y/=1000;
	z/=1000;

	// 構造体に記録する record in structure
	j -> jointType	= k;			// 種類を記録 record type
	j -> origin_x	= x;			// X座標を記録 record the x coordinate
	j -> origin_y	= y;			// X座標を記録 record the y coordinate
	j -> origin_z	= z;			// X座標を記録 record the z coordinate
	j -> c_counter	= 0;			// 汎用カウンタ General purpose counter
	j -> t_jointAngle	= 0;			// 間接角度 indirect angle
	j -> t2_jointAngle	= 0;			// 間接角度2 indirect angle 2
	j -> mode = 0;			// 駆動モード drive mode
	j -> pn_footPressureCounter	= 0;			// 足圧力カウンタ foot pressure counter
	j -> tm	= 44.06;		// 8.06最大角速度 -- 8.06 maximum angular velocity
	j -> tm2	= 8.06;		// 8.06最大角速度 -- //8.06 maximum angular velocity
	j -> torque_tk	= 2.45;		// 2.45トルク 25kgfcm   (25/100)/9.8=2.45 -- 2.45 torque 25kgfcm (25/100)/9.8=2.45
	j -> torque_tk2	= 2.45;		// トルク2 torque 2

	x += 2;					// 2m手前に置いて地面障害物を避ける Place 2m in front to avoid ground obstacles
	world_addJoint(&g_world, j);

	switch (k) {
		case JOINT_HINGE:	// ヒンジジョイント hinge joint
			j -> jointId = dJointCreateHinge(world, 0);			// ヒンジジョイントの生成と記録 generate and record hinge joints
			dJointAttach(j -> jointId, b1->bodyId, b2->bodyId);				// ヒンジジョイントの取付 Installation of hinge joint
			dJointSetHingeAnchor(j -> jointId, x, y, z);			// 中心点の設定 Setting the center point
			switch (a) {		// 軸を設定 set axis
				case AXIS_X: dJointSetHingeAxis(j -> jointId, 1, 0, 0); break;	// X軸の設定 set X axis
				case AXIS_Z: dJointSetHingeAxis(j -> jointId, 0, 0, 1); break;	// Z軸の設定 set Z axis
				default : dJointSetHingeAxis(j -> jointId, 0, 1, 0); break;	// Y軸の設定 set Y axis
			}
			break;
		case JOINT_SLIDER:	// スライダージョイント（ダンパー） slider joint (damper)
			j -> jointId = dJointCreateSlider(world, 0);		// スライダージョイントの生成と記録 Generating and recording slider joints
			dJointAttach(j -> jointId, b1->bodyId, b2->bodyId);			// スライダージョイントの取付 Installing the slider joint
			dJointSetSliderAxis(j -> jointId, 0, 0, 1);		// 中心点の設定 Setting the center point
			break;
		case JOINT_FIXED:	// 固定ジョイント fixed joint
			j -> jointId = dJointCreateFixed(world, 0);		// 固定ジョイントの生成と記録 Generate and record fixed joints
			dJointAttach(j -> jointId, b1->bodyId, b2->bodyId);			// 固定ジョイントの取付 Installing the fixed joint
			dJointSetFixed(j -> jointId);						// 固定ジョイントにの設定 Setting to fixed joint
			break;
		case JOINT_ENVFIXED:	// 環境固定ジョイント environment fixed joint
			j -> jointId = dJointCreateFixed(world, 0);		// 固定ジョイントの生成と記録 Generate and record fixed joints
			dJointAttach(j -> jointId, b1->bodyId, 0);				// 固定ジョイントの取付（環境固定） Installing a fixed joint (fixed environment)
			dJointSetFixed(j -> jointId);						// 固定ジョイントにの設定 Setting to fixed joint
			break;
		default:
			break;
	}
}

void world_init(world_t* world) {
	world->softERP = 0.2;		// 弾力 Elasticity
	world->softCFM = 0.0001;	// 沈み込み sinking
	world->bounce = 0.01;		// 反発係数 coefficient of restitution
	world->bounce_vel = 0.02;		// 反発最低速度 minimum bounce speed

	world->bodyCount = 0;			// ボディ配列カウント値 body array count value
	world->jointCount = 0;			// ジョイント配列カウント値 joint array count
}

//---------------------------------- createBody ---------------------------------------
//	各部のパーツサイズ等を指定してロボットを組み立てる
// Assemble the robot by specifying the parts size etc. of each part
static void createBody (biped_t* biped){
	double	fw	= 21;		// 脚の間隔（中心からの距離） Leg spacing (distance from center)
	double	fmax= 1000;		// 駆動トルク標準値 Drive torque standard value
	double	kmax= 1000;		// 最大角速度 maximum angular velocity

	world_init(&g_world);

//	####################
//	#### ボディ生成 body generation ####
//	####################
//						    種類 色		L　 W	H   R		X	Y	Z	ジオメト 重量
//                         Type Color   L   W   H   R       X   Y   Z  Geometry Weight

	setBody  (&biped->HEADT,		BODYTYPE_CAPSULE, COLOR_WHITE,	15,	0,	0,	21,		0,	0,	340,	0,	0.16);	// 頭 head
	setBody  (&biped->DOU,		BODYTYPE_BOX,COLOR_BODY,	40, 84, 130,0,		0,	0,	260,	1,	1.24);	// 胴 torso
	setBody  (&biped->K0_r,		BODYTYPE_CYLINDER,COLOR_GREY,	34,	0,	0,	12,		0,	-fw,195,	0,	0.05);	// 股関節ピッチ hip pitch
	dRFromAxisAndAngle(world_R, 1, 0, 0, -M_PI_2);// 回転 rotate
	dBodySetRotation(biped->K0_r.bodyId, world_R);
	setBody  (&biped->K0_l,		BODYTYPE_CYLINDER, COLOR_GREY,	34,	0,	0,	12,		0,	fw,	195,	0,	0.05);
	dRFromAxisAndAngle(world_R, 1, 0, 0, -M_PI_2);// 回転 rotate
	dBodySetRotation(biped->K0_l.bodyId, world_R);
	setBody  (&biped->K1_r,		BODYTYPE_CYLINDER,COLOR_WHITE,	34,	0,	0,	12,		0,	-fw,195,	0,	0.05);	// 股関節ロール hip roll
	dRFromAxisAndAngle(world_R, 0, 1, 0, -M_PI_2);// 回転 rotate
	dBodySetRotation(biped->K1_r.bodyId, world_R);
	setBody  (&biped->K1_l,		BODYTYPE_CYLINDER,COLOR_WHITE,	34,	0,	0,	12,		0,	fw,	195,	0,	0.05);
	dRFromAxisAndAngle(world_R, 0, 1, 0, -M_PI_2);// 回転 rotate
	dBodySetRotation(biped->K1_l.bodyId, world_R);
	setBody  (&biped->M_r,			BODYTYPE_BOX, COLOR_GREY,	20,	26,	90,	0,		0,	-fw,150,	0,	0.08);	// 腿 leg
	setBody  (&biped->M_l,			BODYTYPE_BOX, COLOR_GREY,	20,	26,	90, 0,		0,	fw,	150,	0,	0.08);
	setBody  (&biped->H_r,			BODYTYPE_CYLINDER, COLOR_GREY,	34,	0,	0,  12,		0,	-fw,105,	0,	0.03);	// 膝 knees
	dRFromAxisAndAngle(world_R, 1, 0, 0, -M_PI_2);// 回転 rotate
	dBodySetRotation(biped->H_r.bodyId, world_R);
	setBody  (&biped->H_l,			BODYTYPE_CYLINDER, COLOR_GREY,	34,	0,	0,	12,		0,	fw,	105,	0,	0.03);
	dRFromAxisAndAngle(world_R, 1, 0, 0, -M_PI_2);// 回転 rotate
	dBodySetRotation(biped->H_l.bodyId, world_R);
	setBody  (&biped->S_r,			BODYTYPE_BOX, COLOR_GREY,	20,	26, 90,	0,		0,	-fw,60,		1,	0.04);	// 脛 shin
	setBody  (&biped->S_l,			BODYTYPE_BOX, COLOR_GREY,	20,	26, 90,	0,		0,	fw,	60,		1,	0.04);
	setBody  (&biped->A0_r,			BODYTYPE_CYLINDER, COLOR_GREY,	34,	 0, 0,	12,		0,	-fw,15,		0,	0.02);	// 足首ピッチ ankle pitch
	dRFromAxisAndAngle(world_R, 1, 0, 0, -M_PI_2);// 回転 rotate pitch
	dBodySetRotation(biped->A0_r.bodyId, world_R);
	setBody  (&biped->A0_l,			BODYTYPE_CYLINDER, COLOR_GREY,	34,	 0,	0,	12,		0,	fw,	15,		0,	0.02);
	dRFromAxisAndAngle(world_R, 1, 0, 0, -M_PI_2);// 回転 rotate pitch
	dBodySetRotation(biped->A0_l.bodyId, world_R);
	setBody  (&biped->A1_r,			BODYTYPE_CYLINDER,COLOR_WHITE,	34,	 0,	0,	12,		0,	-fw,15,		0,	0.02);	// 足首ロール ankle roll
	dRFromAxisAndAngle(world_R, 0, 1, 0, -M_PI_2);// 回転 rotate roll
	dBodySetRotation(biped->A1_r.bodyId, world_R);
	setBody  (&biped->A1_l,			BODYTYPE_CYLINDER,COLOR_WHITE,	34,	 0,	0,	12,		0,	fw,	15,		0,	0.02);
	dRFromAxisAndAngle(world_R, 0, 1, 0, -M_PI_2);// 回転 rotate
	dBodySetRotation(biped->A1_l.bodyId, world_R);
	setBody  (&biped->sole_r,		BODYTYPE_BOX, COLOR_WHITE,	55,	 40,2,	0,		0,	-fw,6.0,	0,	0.01);	// 足平 foot
	setBody  (&biped->sole_l,		BODYTYPE_BOX, COLOR_WHITE,	55,	 40,2,	0,		0,  fw,	6.0,	0,	0.01);
	setBody  (&biped->solep_r,		BODYTYPE_BOX, COLOR_RED,	55,	 40,6,	0,		0,	-fw,3.0,	1,	0.01);	// ソールセンサ sole sensor
	setBody  (&biped->solep_l,		BODYTYPE_BOX, COLOR_RED,	55,	 40,6,	0,		0,  fw,	3.0,	1,	0.01);
	barrier_pole.bodyId=0;// 摩擦係数設定条件 Friction coefficient setting conditions

//	######################
//	####ジョイント生成 Joint generation ####
//	######################
//								種類				B番号1			B番号2			軸			X		Y		Z
//						        Type			B number 1		B number 2		Axis        X       Y       Z

	setJoint(&biped->HEADJ,		JOINT_FIXED,	&biped->HEADT,	&biped->DOU,	AXIS_Z,		0,		0,		360);	// 頭固定用 For head fixation
	setJoint(&biped->K0J_r,		JOINT_HINGE,	&biped->K1_r,	&biped->K0_r,	AXIS_Y,		0,		-fw,	195);	// 股関節ピッチ hip joint pitch
	setJoint(&biped->K0J_l,		JOINT_HINGE,	&biped->K1_l,	&biped->K0_l,	AXIS_Y,		0,		fw,		195);
	setJoint(&biped->K1J_r,		JOINT_HINGE,	&biped->DOU,	&biped->K1_r,	AXIS_X,		0,		-fw+11,	195);	// 股関節ロール hip roll
	setJoint(&biped->K1J_l,		JOINT_HINGE,	&biped->K1_l,	&biped->DOU,	AXIS_X,		0,		fw-11,	195);
	setJoint(&biped->MJ_r,		JOINT_FIXED,	&biped->M_r,	&biped->K0_r,	AXIS_Y,		0,		-fw,	128);	// 腿固定用 for leg fixation
	setJoint(&biped->MJ_l,		JOINT_FIXED,	&biped->M_l,	&biped->K0_l,	AXIS_Y,		0,		fw,		128);
	setJoint(&biped->M2J_r,		JOINT_FIXED,	&biped->H_r,	&biped->M_r,	AXIS_Y,		0,		-fw,	128);	// 腿固定用 for leg fixation
	setJoint(&biped->M2J_l,		JOINT_FIXED,	&biped->H_l,	&biped->M_l,	AXIS_Y,		0,		fw,		128);
	setJoint(&biped->HJ_r,		JOINT_HINGE,	&biped->S_r,	&biped->H_r,	AXIS_Y,		0,		-fw,	105);	// 膝関節 Knee fixation
	setJoint(&biped->HJ_l,		JOINT_HINGE,	&biped->S_l,	&biped->H_l,	AXIS_Y,		0,		fw,		105);
	setJoint(&biped->SJ_r,		JOINT_FIXED,	&biped->S_r,	&biped->A0_r,	AXIS_Y,		0,		-fw,	60);	// 脛固定用 For shin fixation
	setJoint(&biped->SJ_l,		JOINT_FIXED,	&biped->S_l,	&biped->A0_l,	AXIS_Y,		0,		fw,		60);
	setJoint(&biped->A0J_r,		JOINT_HINGE,	&biped->A0_r,	&biped->A1_r,	AXIS_Y,		0,		-fw,	15);	// 足首ピッチ Ankle pitch
	setJoint(&biped->A0J_l,		JOINT_HINGE,	&biped->A0_l,	&biped->A1_l,	AXIS_Y,		0,		fw,		15);
	setJoint(&biped->A1J_r,		JOINT_HINGE,	&biped->A1_r,	&biped->sole_r,	AXIS_X,		0,		-fw+11,	15);	// 足首ロール Ankle roll
	setJoint(&biped->A1J_l,		JOINT_HINGE,	&biped->sole_l,	&biped->A1_l,	AXIS_X,		0,		fw-11,	15);
	setJoint(&biped->soleJ_r,	JOINT_SLIDER,	&biped->solep_r,&biped->sole_r,	AXIS_X,		0,		-fw,	6);		// ソール圧力センサ sole pressure sensor
	setJoint(&biped->soleJ_l,	JOINT_SLIDER,	&biped->solep_l,&biped->sole_l,	AXIS_X,		0,		fw,		6);

	dJointSetFeedback(biped->soleJ_r.jointId,			&feedback[0]);
	dJointSetFeedback(biped->soleJ_l.jointId,			&feedback[1]);

#ifdef USING_MAIN
	g_main_initialI = main_init(&g_mainstate, &g_maincore, &g_maininput);
#endif
}


//--------------------------------- destroy ----------------------------------------
//	ロボットの破壊 robot destruction
void destroyBot (){
	int i;
	for (i=0;i<g_world.jointCount;i++){
		jointStr* joint = g_world.joint[i];
		if(joint->enabled > 0){dJointDestroy (joint->jointId);}	// ジョイント破壊 Joint destruction
	}
	for (i=0;i< g_world.bodyCount;i++){
		bodyStr* body = g_world.body[i];
		if(body->enabled > 0){dBodyDestroy (body->bodyId);}		// ボディ有効なら破壊 Destroy if body is valid
		if(body->hasGeometry > 0){dGeomDestroy (body->geometry);}		// ジオメトリ設定されてたら破壊 Destroyed if geometry is set
	}
	dJointGroupDestroy (world_contactgroup);
}


//--------------------------------- restart ----------------------------------------
//	リスタート restart
void restart (){
	destroyBot ();
	world_contactgroup = dJointGroupCreate (0);	// 接触点のグループを格納する入れ物 A container that stores a group of contact points
	createBody(&biped);						// ロボット生成 robot generation
	dWorldSetGravity (world, 0, 0, -9.8);	// 重力設定 Gravity settings
	g_co.mode=0;
	g_co.autoHs=180;
	bipedinput.walkF=0;
	bipedinput.uvcOff=0;
	bipedinput.frRatI=0;
	bipedinput.frRatA=0;
	bipedinput.fhRat=0;
	bipedinput.fwMax=0;
 
	bipedstate.K0W[0]=0;			//股関節前後方向 hip joint anteroposterior direction
	bipedstate.K1W[0]=0;			//股関節横方向 Hip joint lateral direction
	bipedstate.HW [0]=0;			//膝関節 knee joint
	bipedstate.A0W[0]=0;			//足首上下方向 Ankle up and down direction
	bipedstate.A1W[0]=0;			//足首横方向 Ankle lateral direction
	bipedstate.K0W[1]=0;			//股関節前後方向 hip joint anteroposterior direction
	bipedstate.K1W[1]=0;			//股関節横方向 Hip joint lateral direction
	bipedstate.HW [1]=0;			//膝関節 knee joint
	bipedstate.A0W[1]=0;			//足首上下方向 Ankle up and down direction
	bipedstate.A1W[1]=0;			//足首横方向 Ankle lateral direction
}


//--------------------------------- start ----------------------------------------
//	start simulation - set viewpoint
static void start(){
	static float xyz[3] = {2.3, -0.3, 0.18};	// 視点の位置 View position
	static float hpr[3] = {135,0,0};	// 視線の方向 direction of gaze
	dsSetViewpoint (xyz,hpr); // カメラの設定 Camera settings
}

#ifdef USING_MAIN
int sim_ics_set_pos(int port, unsigned char id, unsigned short pos) {
	bipedstate_t* state = &bipedstate;
	float	angle = DEGREES2RADIANS( SVANGLE2ANGLE(-pos + 7500) );
	double*	dest = NULL;

	switch (port) {
	case UART_SIO1:
		switch (id) {
		case 5:  /*dest = &state->K2W[0]; */ break;
		case 6:  dest = &state->K1W[0]; break;
		case 7:  dest = &state->K0W[0]; break;
		case 8:  dest = &state->HW[0]; break;
		case 9:  dest = &state->A0W[0]; break;
		case 10: dest = &state->A1W[0]; break;
		default: break;
		}
		break;

	case UART_SIO2:
		switch (id) {
		case 1: dest = &state->U0W[0]; break;
		case 2: dest = &state->U1W[0]; break;
		case 3: dest = &state->U2W[0]; break;
		case 4: /*dest = &state->EW[0];*/ break;
		case 0: /*dest = &state->WESTW;*/ break;
		default: break;
		}
		break;

	case UART_SIO3:
		switch (id) {
		case 5:  /*dest = &state->K2W[1];*/ break;
		case 6:  dest = &state->K1W[1]; break;
		case 7:  dest = &state->K0W[1]; break;
		case 8:  dest = &state->HW[1]; break;
		case 9:  dest = &state->A0W[1]; break;
		case 10: dest = &state->A1W[1]; break;
		default: break;
		}
		break;

	case UART_SIO4:
		switch (id) {
		case 1: dest = &state->U0W[1]; break;
		case 2: dest = &state->U1W[1]; break;
		case 3: dest = &state->U2W[1]; break;
		//case 4: dest = &state->EW[1]; break;
		//case 0: dest = &state->HEADW; break;
		default: break;
		}
		break;

	default:
		break;
	}

	if (dest != NULL) {
		if (pos == 0) {
			return -ANGLE2SVANGLE(RADIANS2DEGREES(*dest)) + 7500;
		}
		else {
			*dest = (double)angle;
			return pos;
		}
	}

	return pos;
}

int sim_bno55_read(unsigned char* command, size_t c_size, unsigned char* data, size_t r_size) {
	if (c_size == 1 && command[0] == 0X1A && r_size == 6) {
		// return gyroscope data in degrees*16.0
		// core->yaw	= ((int16_t)input->ff[0]) | (((int16_t)input->ff[1]) << 8); // Standing upright and turning clockwise +
		// core->pitchs = ((int16_t)input->ff[2]) | (((int16_t)input->ff[3]) << 8); // Standing upright and leaning forward -
		// core->rolls = ((int16_t)input->ff[4]) | (((int16_t)input->ff[5]) << 8); // Upright with right tilt +
		int16_t yaw = 0, pitch = RADIANS2DEGREES(bipedstate.fbRad) * 16.0,
			roll = (RADIANS2DEGREES(bipedstate.lrRad) + 180.0f) * 16.0;
		((int16_t*)data)[0] = 0;
		((int16_t*)data)[1] = pitch;
		((int16_t*)data)[2] = roll;
		return 1;
	}
	else if (c_size == 1 && command[0] == 0X14 && r_size == 6) {
		// return angular velocity in degrees*16.0
		int16_t fbAV = RADIANS2DEGREES(bipedstate.fbAV) * 16.0f;
		int16_t lrAV = RADIANS2DEGREES(bipedstate.lrAV) * 16.0f;
		((int16_t*)data)[0] = 0;
		((int16_t*)data)[1] = fbAV;
		((int16_t*)data)[2] = lrAV;
		return 1;
	}

	return 1;
}
#endif // !USING_MAIN

//------------------------------------ main -------------------------------------
int main (int argc, char **argv){
#ifdef USING_MAIN
	g_ics_set_pos_callback = &sim_ics_set_pos;
	g_bno55_read = &sim_bno55_read;
#endif

	bipedstate_init(&bipedstate);
	bipedinput_init(&bipedinput);
	dMass m;
	dInitODE(); // ODEの初期化 Initializing ODE

	// setup pointers to drawstuff callback functions
	dsFunctions fn;
	fn.version = DS_VERSION;
	fn.start = &start;
	fn.step = &simLoop;
	fn.command = &command; // Windows10から利用できなくなった No longer available from Windows 10
	fn.stop = 0;
	fn.path_to_textures = "c:/ode-0.13/drawstuff/textures";
	if(argc==2)fn.path_to_textures = argv[1];

	//   create world
	world = dWorldCreate();					// シミュレーションワールド生成 Create simulation world
	world_space = dHashSpaceCreate (0);			// 衝突検出用スペース生成 Create space for collision detection
	world_contactgroup = dJointGroupCreate (0);	// 接触点のグループを格納する入れ物 Container for storing groups of contact points
	dWorldSetGravity (world, 0, 0, -9.8);	// 重力設定 Gravity settings
	world_ground = dCreatePlane (world_space,0,0,1,0);	// 平面のジオメトリ作成 Create plane geometry
	createBody(&biped);							// ロボット生成 Create robot

	// run simulation
	dsSimulationLoop (argc,argv,640,640,&fn);

	// 後始末 clean up
	destroyBot ();
	dSpaceDestroy (world_space);
	dWorldDestroy (world);
	return 0;
}