// original code by Dr Guero
// translation and cleanup by Dmitry Shalkhakov
// coordinate system conventions: Z is up/down, X is forward/backward, Y is left/right
/*
Apply UVC to KHR.  May 8,2022 : Vre 1.0
*/

#include "stdio.h"
#include "kcb5.h"
#include "i2c.h"
#include "uart.h"
#include "pio.h"
#include "rom.h"
#include "timer.h"
#include "ics.h"
#include "ad.h"
#include "dac.h"
#include "math.h"

#include "main.h"

#define OFS_L 129.0			// 初期の股関節までの距離（膝カク防止） Initial distance to hip joint (to prevent knee jerk)
#define OFS_S 213			// 初期の股関節サーボ角 Initial hip servo angle
#define CHG_SVA 1718.9		// サーボ角に変換 Convert to servo angle (60 deg)
#define HEIGHT 185			// 185脚長 185 Leg length
#define RET_F_LEG 3			// 遊脚追従率 Idle leg tracking rate
#define RST_F_LEG 3			// 軸足垂直戻率 Pivot vertical return rate

//// グローバル変数 global variables
state_t state;

core_t core;

input_t input;

globals_t globals;

//// 汎用 General purpose ////
int32_t	ii;

vec2_t vec2_zero = { 0.0f, 0.0f };

float clamp(float value, float min, float max) {
	if (value < min) {
		return min;
	}
	if (value > max) {
		return max;
	}
	return value;
}

////////////////////////
//// ターミナル表示 terminal display ////
////////////////////////
void printS(char *StringData){
	unsigned char count = 0;
	while(StringData[count] != '\0')count++;
	uart_tx(UART_COM, (unsigned char*)StringData, 0, count);
}



////////////////////////
//// 遅延時間の設定 Setting the delay time ////
////////////////////////
void delay(int32_t t)
{
	int32_t tb=0;
	t*=1000;
	timer_write(TIMER,t);
	timer_start(TIMER);
	t=0;
	while(t >= tb){
		tb=t;
		t=timer_read(TIMER);
	}
}



/////////////////////
//// I2Cアクセス I2C access ////
/////////////////////
uint8_t read8(input_t* input, uint8_t reg )
{
	input->cmd[0]=reg;
	//アドレスを0x28から左に1bitシフトし0x50、KONDOのI2Cライブラリバグ対応 Shift address 1 bit to the left from 0x28 to 0x50, KONDO I2C library bug resolved
	i2c_read(0x50, input->cmd, 1, input->ff, 1);  //BNO055_CHIP_ID_ADDR(0x28)
	return input->ff[0];
}
bool readLen(input_t* input, uint8_t reg, uint8_t len)
{
	input->cmd[0]=reg;
	//アドレスを0x28から左に1bitシフトし0x50、KONDOのI2Cライブラリバグ対応 Shift address 1 bit to the left from 0x28 to 0x50, KONDO I2C library bug resolved
	i2c_read(0x50, input->cmd, 1, input->ff, len);  //BNO055_CHIP_ID_ADDR(0x28)
	return true;
}
bool write8(input_t* input, uint8_t reg, uint8_t dat)
{
	input->cmd[0]=dat;
	//アドレスを0x28から左に1bitシフトし0x50、KONDOのI2Cライブラリバグ対応 Shift address 1 bit to the left from 0x28 to 0x50, KONDO I2C library bug resolved
	i2c_write(0x50, reg, input->cmd, 1);  //BNO055_CHIP_ID_ADDR(0x28)
	return true;
}



/////////////////////////////////////
//// 目標値まで徐々に間接を動かす Gradually move the indirection to the target value ////
/////////////////////////////////////
void movSv(core_t* core, short *s,int d){
//引数 s:現サーボ位置 d:目標サーボ位置 Argument s: Current servo position d: Target servo position
	if(core->motCt<1)	*s = d;
	else				*s += (d-*s)/core->motCt;
}



///////////////////////////
//// 検出角度を校正する Calibrate the detection angle ////
///////////////////////////
void angAdj(core_t* core){

// **** pitch & roll ****
	if( core->pitchs<=core->ipb+1 && core->pitchs>=core->ipb-1 &&
		core->rolls <=core->irb+1 && core->rolls >=core->irb-1 	){
		++core->ip;
		core->ipa+=core->pitchs;
		core->ira+=core->rolls;
	}
	else {
		core->ip=0;
		core->ipa=0;
		core->ira=0;
	}
	core->ipb=core->pitchs;
	core->irb=core->rolls;
	sprintf( (char *)globals.dsp,"P:%4d R:%4d C:%4d\r",core->pitchs,core->rolls,core->ip );
	printS((char *)globals.dsp);
}



///////////////////////
//// 全方向転倒検知 Omnidirectional fall detection ////
///////////////////////
void detAng(core_t* core){
	if( 0.35>fabs(core->pitch) && 0.35>fabs(core->roll) )return;
	sprintf( (char *)globals.dsp," PRA:%4d %4d PRG:%4d %4d\r\n",core->pitchs,core->rolls,core->pitch_gyr,core->roll_gyr );
	printS((char *)globals.dsp);

	ics_set_pos	( UART_SIO2, 1, 4735 +1350 );	//U0R
	ics_set_pos	( UART_SIO4, 1, 9320 -1350 );	//U0L
	ics_set_pos	( UART_SIO2, 4, 4800 +2700 );	//ER
	ics_set_pos	( UART_SIO4, 4,10150 -2700 );	//EL
	ics_set_pos	( UART_SIO1, 7, 7500 -2100 );	//K0R
	ics_set_pos	( UART_SIO3, 7, 7500 +2100 );	//K0L
	ics_set_pos ( UART_SIO1, 8, 9260 -4200 );	//HR
	ics_set_pos	( UART_SIO3, 8, 5740 +4200 );	//HL
	ics_set_pos	( UART_SIO1, 9, 7910 +2100 );	//A0R
	ics_set_pos	( UART_SIO3, 9, 7100 -2100 );	//A0L
	delay(30);
	ics_set_pos		( UART_SIO1, 5, 0 );	//K2R
	ics_set_pos		( UART_SIO2, 1, 0 );	//U0R
	ics_set_pos		( UART_SIO3, 5, 0 );	//K2L
	ics_set_pos		( UART_SIO4, 1, 0 );	//U0L
	ics_set_pos		( UART_SIO1, 6, 0 );	//K1R
	ics_set_pos		( UART_SIO2, 2, 0 );	//U1R
	ics_set_pos		( UART_SIO3, 6, 0 );	//K1L
	ics_set_pos		( UART_SIO4, 2, 0 );	//U1L
	ics_set_pos		( UART_SIO1, 7, 0 );	//K0R
	ics_set_pos		( UART_SIO3, 7, 0 );	//K0L
	ics_set_pos		( UART_SIO1, 8, 0 );	//HR
	ics_set_pos		( UART_SIO2, 4, 0 );	//ER
	ics_set_pos		( UART_SIO3, 8, 0 );	//HL
	ics_set_pos		( UART_SIO4, 4, 0 );	//EL
	ics_set_pos		( UART_SIO1, 9, 0 );	//A0R
	ics_set_pos		( UART_SIO3, 9, 0 );	//A0L
	sprintf( (char *)globals.dsp,"turn over :%4d %4d\r\n",core->mode,core->pitchs );
	printS ( (char *)globals.dsp );
	// FIXME DS: needed? maybe a debug aid?
	while(1){}
}



/////////////////////
//// UVC補助制御 UVC auxiliary control ////
/////////////////////
// inputs: fwct, landF, dyi, fwctEnd, landB, rollt
// outputs: dxi, dxis, dyi, dyis, autoH
void uvcSub(core_t* core){

	// ************ UVC終了時支持脚を垂直に戻す At the end of UVC, return the support leg to vertical position ************
	if( core->fwct<=core->landF ){

		// **** 左右方向 left and right direction ****
		float k = core->dyi/(11-core->fwct);
		core->dyi -= k;
		core->dyis += k;

		// **** 前後方向 Longitudinal direction ****
		if( core->dxi>RST_F_LEG ){
			core->dxi  -= RST_F_LEG;
			core->dxis -= RST_F_LEG;
		}
		else if( core->dxi<-RST_F_LEG ){
			core->dxi  += RST_F_LEG;
			core->dxis += RST_F_LEG;
		}
		else{
			core->dxis -= core->dxi;
			core->dxi   = 0;
		}
	}
	if(core->dyis> 70)	core->dyis=  70;
	core->dxis = clamp(core->dxis, -70, 70);

	// ************ 脚長制御 leg length control ************
	if(HEIGHT>core->autoH){						// 脚長を徐々に復帰させる Gradually restore leg length
		core->autoH +=(HEIGHT- core->autoH)*0.07;//0.07
	}
	else core->autoH = HEIGHT;

	if( core->fwct>core->fwctEnd-core->landB && core->rollt>0){	// 着地時姿勢を落し衝撃吸収 Lower your posture when landing to absorb shock
		core->autoH -= ( fabs(core->dyi-core->dyib)+fabs(core->dxi-core->dxib) ) * 0.02;
	}
	if(140>core->autoH)core->autoH=140;					// 最低脚長補償 Minimum leg length compensation
}

// inputs: fwctEnd, fwct, dyi, dxi, landF, autoH
// outputs:  dyi, dxi, wk, state WESTW, K2W[0], K2W[1], diys, dxis, autoH
void uvcSub2(core_t* core, state_t* state){
	float k0,k1;

	// ************ UVC終了時支持脚を垂直に戻す At the end of UVC, return the support leg to vertical position ************

	// **** 左右方向 left and right direction ****
	k1 = core->dyi/(core->fwctEnd-core->fwct+1);
	core->dyi -= k1;

	// **** 前後方向 Longitudinal direction ****
	k0 = core->dxi/(core->fwctEnd-core->fwct+1);
	core->dxi -= k0;

	// **** 腰回転 hip rotation ****
	core->wk -= core->wk/(core->fwctEnd-core->fwct+1);
	state->WESTW  -= state->WESTW/(core->fwctEnd-core->fwct+1);
	state->K2W[0]  = state->WESTW;
	state->K2W[1]  =-state->WESTW;

	if( core->fwct<=core->landF ){
		// **** 左右方向 left and right direction ****
		core->dyis += k1;

		// **** 前後方向 Longitudinal direction ****
		core->dxis -= k0;
	}
	else{
		core->dyis -= core->dyis/(core->fwctEnd-core->fwct+1);
		core->dxis -= core->dxis/(core->fwctEnd-core->fwct+1);
	}

	// **** 脚長制御 leg length control ****
	core->autoH += (HEIGHT-core->autoH)/(core->fwctEnd-core->fwct+1);

	if(core->dyis> 70)	core->dyis=  70;
	core->dxis = clamp(core->dxis, -70, 70);
}



/////////////////
//// UVC制御 UVC control ////
/////////////////
// inputs: pitch, roll, jikuasi, landF, landB, fwct, fwctEnd, sw
// outputs: rollt, pitcht, dyi, dxi, dxis, dyis, autoH
void uvc(core_t* core){

	float pitch = core->pitch, roll = core->roll;

	// ************ 傾斜角へのオフセット適用 Apply offset to slope angle ************
	{
		float k= Vec2Length(pitch,roll);	// 合成傾斜角 Resultant tilt angle
		if( k>0.033 ){
			float k1=(k-0.033)/k;
			pitch *=k1;
			roll  *=k1;
		}
		else{
			pitch =0;
			roll  =0;
		}
	}


	// ************ 傾斜角に係数適用 Apply coefficient to slope angle ************
	core->rollt =0.25*roll;
	if(core->jikuasi==0)	core->rollt = -core->rollt;		// 横方向符号調整 Horizontal sign adjustment
	core->pitcht=0.25*pitch;

	if(core->fwct>core->landF && core->fwct<=core->fwctEnd-core->landB ){

		// ************ UVC主計算 UVC main calculation ************
		{
			float k	= atan ((core->dyi-core->sw)/core->autoH );	// 片脚の鉛直に対する開脚角 Leg angle relative to the vertical of one leg
			float kl = core->autoH/cos(k);			// 前から見た脚投影長 Leg projection length seen from the front
			float ks = k+core->rollt;					// 開脚角に横傾き角を加算 Add side tilt angle to leg angle
			float kk	= kl*sin(ks);					// 中点から横接地点までの左右距離 Left-right distance from midpoint to lateral contact point
			core->dyi	= kk+core->sw;					// 横方向UVC補正距離 Horizontal direction UVC correction distance
			core->autoH = kl*cos(ks);				// K1までの高さ更新 Height update up to K1
		}

		// **** UVC（前後） UVC (front and back) *****
		{
			float k = atan( core->dxi/core->autoH );	// 片脚のX駆動面鉛直から見た現時点の前後開脚角 Current anteroposterior leg angle as seen from the vertical direction of the X drive plane of one leg
			float kl = core->autoH/cos(k);			// 片脚のX駆動面鉛直から見た脚長 Leg length as seen from the vertical direction of the X drive surface of one leg
			float ks = k+core->pitcht;				// 振出角に前後傾き角を加算 Add the forward and backward tilt angle to the swing angle
			float kk = kl*sin(ks);					// 前後方向UVC補正距離 UVC correction distance in front and rear directions
			core->dxi = kk;								// 前後方向UVC補正距離 UVC correction distance in front and rear directions
			core->autoH = kl*cos(ks);				// K1までの高さ更新 Height update up to K1
		}

		// ************ UVC積分値リミット設定 UVC integral value limit setting ************
		core->dyi = clamp(core->dyi, 0, 45);
		core->dxi = clamp(core->dxi, -45, 45);

		// ************ 遊脚側を追従させる Make the free leg follow ************

		// **** 左右方向 left and right direction *****
		core->dyis = core->dyi;					// 遊脚Y目標値 Idle leg Y target value

		// **** 前後方向 Longitudinal direction *****
		core->dxis = -core->dxi;				// 遊脚X目標値 Idle leg X target value

		// ************ 両脚内側並行補正 Medial parallel correction for both legs ************
		{
			float k, ks;

			if(core->jikuasi==0){			// 両脚を並行以下にしない Do not let your legs be less than parallel
				k = -core->sw+core->dyi;	// 右足の外側開き具合 Outside opening of right foot
				ks=  core->sw+core->dyis;	// 左足の外側開き具合 Outside opening of left foot
			}
			else{
				ks= -core->sw+core->dyi;	// 左足の外側開き具合 Outside opening of left foot
				k =  core->sw+core->dyis;	// 右足の外側開き具合 Outside opening of right foot
			}
			if(k+ks<0)core->dyis-=k+ks;		// 遊脚を平衡に補正 Correct the swing leg to balance
		}
	}
}



////////////////////
//// 脚上げ操作 Leg raising operation ////
////////////////////
// inputs: fwct, fwctEnd, landF, landB, fhMax
// outputs: fh
void footUp(core_t* core){

	if( core->fwct>core->landF && core->fwct<=(core->fwctEnd- core->landB) ) core->fh = core->fhMax * sin( M_PI*(core->fwct-core->landF)/(core->fwctEnd-(core->landF+core->landB)) );
	else																	core->fh = 0;
}



////////////////////
//// 横振り制御 Horizontal swing control ////
////////////////////
// inputs: fwct, fwctEnd, swMax, dxi, dyi, wt
// outputs: swx, swy
void swCont(core_t* core){
	float k,t;

	k=core->swMax*sinf(M_PI*core->fwct/core->fwctEnd);//sinカーブ
	t=atan( (fabs(core->dxi)+fabs(21.5*sin(core->wt)))/(core->dyi+21.5*cos(core->wt)-core->wt) );
	if(core->dxi>0)	core->swx =  k*sin(t);
	else			core->swx = -k*sin(t);
	core->swy=k*cos(t);
}



////////////////
//// 腕制御 arm control ////
////////////////
// inputs: dyis
// outputs: state U1W[0], state U1W[1]
void armCont(core_t* core, state_t* state){
	state->U1W[0]=510*core->dyis/70; // 股幅に応じ腕を広げる Spread your arms according to the width of your thighs
	if(state->U1W[0]<0) state->U1W[0]=0;
	state->U1W[1]= state->U1W[0];
}



////////////////////
//// 最終脚駆動 Last leg drive ////
////////////////////
// inputs: HW[s], p0 : vec2_t, h, s
// outputs: K0W[s], A0W[s], autoH, K1W[s], A1W[s]
void footCont(core_t* core, state_t* state, vec2_t p0,float h,int s){
// p0 x:中点を0とする設置点前後方向距離（前+） Distance in the longitudinal direction of the installation point with the midpoint as 0 (front +)
// p0 y:中点を0とする設置点左右方向距離（右+） Distance in the horizontal direction of the installation point with the midpoint as 0 (right +)
// h:足首ロール軸を基準に股関節ロール軸までの地上高(Max194.5) Ground clearance from the ankle roll axis to the hip roll axis (Max 194.5)
// s:軸足0/遊脚1、指定 Pivot leg 0/swing leg 1, specified
// 真正面からみたK1-A1間距離 	k = sqrt( x*x + h*h ); Distance between K1 and A1 when viewed directly from the front
// K1-K0間40mm A0-A1間24.5mm 合計64.5mm -- 40mm between K1 and K0 24.5mm between A0 and A1 Total 64.5mm

// 骨格垂直真横からみたK0-A0間直角高さ k = k - 64.5 -- Perpendicular height between K0 and A0 when viewed from the vertical side of the skeleton k = k - 64.5
// K0-A0間直線距離 k = sqrt( x*x + k*k ); まとめると↓ -- Straight line distance between K0 and A0 k = sqrt( x*x + k*k ); To summarize, ↓
// 高さhの最大は40+65+65+24.5=194.5mm -- Maximum height h is 40+65+65+24.5=194.5mm

	float k = Vec2Length(p0[0], Vec2Length(p0[1], h) - 64.5);	// K0-A0間直線距離 Straight line distance between K0-A0
	if(k>129){
		float temp1 = sqrt(129*129-p0[0]*p0[0]) + 64.5;
		core->autoH = sqrt(temp1*temp1-p0[1]*p0[1]);// 高さ補正 height correction
		k=129;
	}

	float x = CHG_SVA*asin(p0[0]/k);						// K0脚振り角度 K0 leg swing angle
	float k0 = CHG_SVA*acos(k/130);					// 膝曲げ角度 knee bending angle
	if(k0>1800)k0=1800;							// 60°Max

	if		(2*k0-state->HW[s]> 100) k0=(state->HW[s]+100)/2;	// 回転速度最大 0.13s/60deg = 138count -- Maximum rotation speed
	else if	(2*k0-state->HW[s]<-100) k0=(state->HW[s]-100)/2;
//	if(core->mode!=0 && core->jikuasi==s)state->HW[s]	= k0*1.98;	// 軸足のたわみを考慮 Consider the deflection of the pivot foot
	if(core->mode!=0 && core->jikuasi==s)state->HW[s]	= k0*2;
	else							 state->HW[s]	= k0*2;
	state->K0W[s]	= k0+x;
	state->A0W[s]	= k0-x;

	float k1 = CHG_SVA*atan(p0[1]/h);						// K1角度 K1 angle

	if		(k1-state->K1W[s]> 100) k1=state->K1W[s]+100;		// 回転速度最大 0.13s/60deg = 138count -- Maximum rotation speed
	else if	(k1-state->K1W[s]<-100) k1=state->K1W[s]-100;

	if(core->mode!=0 && core->jikuasi==s)state->K1W[s] = k1;		// 軸足のたわみを考慮 Consider the deflection of the pivot foot
	else					          state->K1W[s] = k1;
	state->A1W[s] = -k1;
}



////////////////////
//// 統合脚駆動 integrated leg drive ////
////////////////////
// inputs: jikuasi, p0, p1, s
// outputs: wt, wk, state WESTW, state K2W[0], state K2W[1]
void feetCont1(core_t* core, state_t* state, vec2_t p0, vec2_t p1, int s){

	if(s==1){
		if(p0[1]+21.5==0)		core->wt = 0;			// 0除算回避 avoid divide by 0
		else if(core->jikuasi==0){
			core->wt = 0.5*atan( p0[0]/(p0[1]+21.5) );	// 腰回転角度 waist rotation angle
			core->wk=fabs(15.0*p0[0]/45);			// 45:最大歩幅 15:足内側最大移動量 45: Maximum stride length 15: Maximum amount of movement on the inside of the foot
		}
		else{
			core->wt = 0.5*atan( -p1[0]/(p1[1]+21.5) );	// 腰回転角度 waist rotation angle
			core->wk=fabs(15.0*p1[0]/45);
		}

		state->WESTW = core->wt*CHG_SVA;					// 腰回転（+で上体右回転） Hip rotation (+: upper body rotation to the right)
		state->K2W[0]= state->WESTW;
		state->K2W[1]=-state->WESTW;
	}

	if(core->jikuasi==0){
		vec2_t v1, v2;
		Vec2Set(v1, p0[0],	p0[1]-core->wk);
		Vec2Set(v2, p1[0],	p1[1]-core->wk);
		footCont(core, state, v1,  core->autoH   ,	0 );
		footCont(core, state, v2,  core->autoH-core->fh,	1 );
	}
	else{
		vec2_t u1, u2;
		Vec2Set(u1, p0[0],	p0[1]-core->wk);
		Vec2Set(u2, p1[0],	p1[1]-core->wk);
		footCont(core, state, u1,  core->autoH-core->fh,	0 );
		footCont(core, state, u2,  core->autoH   ,	1 );
	}
}


// inputs: jikuasi, dxi, swx, dyi, swy
// outputs: feetCont1() outputs
void feetCont2(core_t* core, state_t* state, int s){
	vec2_t v1, v2;

	if (core->jikuasi == 0) {
		Vec2Set(v1, core->dxi -core->swx, core->dyi -core->swy);
		Vec2Set(v2, core->dxis-core->swx, core->dyis+core->swy);
		feetCont1(core, state, v1, v2, s);
	}
	else {
		Vec2Set(v1, core->dxis-core->swx, core->dyis+core->swy);
		Vec2Set(v2, core->dxi -core->swx, core->dyi -core->swy);
		feetCont1(core, state, v1, v2, s);
	}
}



///////////////////////////
//// 周期カウンタの制御 Period counter control ////
///////////////////////////
// inputs: fwct, fwctEnd
// outputs: jikuasi, fwct, fwctUp, fh, dyis, dyi, dyib, dxis, dxi, dxib
void counterCont(core_t* core){
	if(core->fwct>=core->fwctEnd){	// 簡易仕様（固定周期） Simple specifications (fixed cycle)
		core->jikuasi^=1;
		core->fwct=0;

		core->fh=0;
		float k=core->dyis;
		core->dyis=core->dyi;
		core->dyib=core->dyi;
		core->dyi=k;

		float k1=core->dxis;
		core->dxis=core->dxi;
		core->dxib=core->dxi;
		core->dxi=k1;
	}
	else{
		core->fwct+=core->fwctUp;
		if(core->fwct>core->fwctEnd)core->fwct=core->fwctEnd;	// 最大値補償 Maximum compensation
	}
}



//##################
//#### 歩行制御 walking control ####
//##################
void walk(core_t* core, state_t* state){

	switch(core->mode){

//**** ① 開始、初期姿勢に以降 -- ① After the start and initial posture ****
case 710:
	movSv(core, &state->K0W[0],  661);
	movSv(core, &state->K1W[0],    0);
	movSv(core, &state->K2W[0],    0);
	movSv(core, &state-> HW[0], 1322);
	movSv(core, &state->A0W[0],  661);
	movSv(core, &state->A1W[0],    0);//+30
	movSv(core, &state->U0W[0],-2700);
	movSv(core, &state->U1W[0],    0);
	movSv(core, &state->U2W[0],    0);
	movSv(core, &state-> EW[0],    0);
	movSv(core, &state-> WESTW,    0);

	movSv(core, &state->K0W[1],  661);
	movSv(core, &state->K1W[1],    0);
	movSv(core, &state->K2W[1],    0);
	movSv(core, &state-> HW[1], 1322);
	movSv(core, &state->A0W[1],  661);
	movSv(core, &state->A1W[1],    0);//-30
	movSv(core, &state->U0W[1],-2760);
	movSv(core, &state->U1W[1],    0);
	movSv(core, &state->U2W[1],    0);
	movSv(core, &state-> EW[1],    0);
	movSv(core, &state->HEADW,    0);

	if(core->motCt>0)--core->motCt;
	else{
		//// 角度補正用 For angle correction ////
		core->p_ofs	=0;
		core->r_ofs	=0;
		core->ip	=0;
		core->ir	=0;
		core->ipb	=0;
		core->irb	=0;
		core->ipa	=0;
		core->ira	=0;

		//// UVC積分用 For UVC points ////
		core->dxi	=0;
		core->dyi	=0;
		core->dxis	=0;
		core->dyis	=0;
		core->dxib	=0;
		core->dyib	=0;

		core->landF	=0;
		core->landB	=0;
		core->fwctEnd	=18;
		core->walkCt	=0;
		core->fwctUp	=1;
		core->fwct=1;
		core->autoH=HEIGHT;
		core->sw=0;
		core->swx=0;
		core->swy=0;
		core->jikuasi=1;

		//// 初期姿勢 Initial posture ////
		footCont(core, state, vec2_zero,HEIGHT,0);
		core->jikuasi=0;
		footCont(core, state, vec2_zero,HEIGHT,1);
		core->mode=720;					// 状態遷移 state transition
		sprintf( (char *)globals.dsp,"mode=720\r\n" );
		printS((char *)globals.dsp);
	}
	break;


//**** ② 開始時の傾斜角補正 ② Tilt angle correction at start ****
case 720:
	angAdj(core);
	if( core->ip==100 ){
		core->p_ofs=core->ipa/100;
		core->r_ofs=core->ira/100;
		core->mode=730;		// 状態遷移 state transition
		sprintf( (char *)globals.dsp,"\r\nmode=730 ** Ready ** Pa:%4d Po:%4d  Ra:%4d Ro:%4d\r\n",core->pitchs,core->p_ofs,core->rolls,core->r_ofs );
		printS((char *)globals.dsp);
	}
	detAng(core);			// 転倒検知 Fall detection
	break;


//**** ③ 初期姿勢における傾斜検知 ③ Tilt detection in initial posture ****
case 730:

	// **** offset値補正 offset value correction ****
	if( core->ip>=50 ){
		core->ip=0;
		if(core->rolls >0) ++core->r_ofs;
		if(core->rolls <0) --core->r_ofs;
		if(core->pitchs>0) ++core->p_ofs;
		if(core->pitchs<0) --core->p_ofs;
	}
	else ++core->ip;

	feetCont2(core, state, 1);

	if(	fabs(core->roll)>0.033 || fabs(core->pitch)>0.044 ){
		// DS: if we detect roll or pitch in initial posture, we should start walking to prevent fall
		if(core->roll>0)	core->jikuasi=1;
		else				core->jikuasi=0;
		core->fwct=1;
		core->mode=740;					// 状態遷移 state transition
		sprintf( (char *)globals.dsp,"mode=740\r\n" );
		printS((char *)globals.dsp);
		break;
	}
	sprintf( (char *)globals.dsp,"P:%4d R:%4d C:%4d\r",core->pitchs,core->rolls,core->ip );
	printS((char *)globals.dsp);
	break;


//**** ④ UVC動作開始 ④ UVC action starts ****
case 740:
	uvc(core);				// UVCメイン制御 UVC main control
	uvcSub(core);			// UVCサブ制御 UVC sub control
	footUp(core);			// 脚上げによる股関節角算出 Calculating hip joint angle by raising legs
	feetCont2(core, state, 1);	// 脚駆動 leg drive
	armCont(core, state);
	counterCont(core);			// 周期カウンタの制御 Period counter control
	if(core->fwct==0){
		core->mode=750;			// 状態遷移 state transition
		sprintf( (char *)globals.dsp,"mode=750\r\n" );
		printS((char *)globals.dsp);
	}
	detAng(core);			// 転倒検知 Fall detection
	break;


//**** ⑤ UVC後、振動減衰待ち ⑤ After UVC, wait for vibration damping ****
case 750:
	// initial values in mode 750: what they were in mode 750 when fwct was set to 0
	feetCont2(core, state, 1);
	if(	core->fwct>30 ){
		core->fwct=1;

		float k=sqrt(0.5* Vec2LengthSquared(core->dxis, core->dyis));	// 移動量、前後方向は減少させる Reduce the amount of movement and the forward and backward directions
		core->swMax=17+17*k/45;

		core->mode=760;		// 状態遷移 state transition
		sprintf( (char *)globals.dsp,"mode=760 %2d \r\n",(int)core->swMax );
		printS((char *)globals.dsp);
		break;
	}
	else{
		sprintf( (char *)globals.dsp,"C:%4d\r",(int)core->fwct );
		printS((char *)globals.dsp);
		++core->fwct;
	}
	detAng(core);			// 転倒検知 Fall detection
	break;


//**** ⑥ 回復動作 ⑥ Recovery operation ****
case 760:
	// initial values in mode 760: landF = 25, fwctEnd = landF+25 (50), fwct = 1
	core->landF=25;
	core->fwctEnd=core->landF+25; // 15
	uvcSub2(core, state);	// UVCサブ制御 UVC sub control
	footUp(core);			// 脚上げによる股関節角算出 Calculating hip joint angle by raising legs
	swCont(core);			// 横振り制御 Horizontal swing control
	feetCont2(core, state, 0);	// 脚駆動 leg drive
	armCont(core, state);
	counterCont(core);		// 周期カウンタの制御5 Period counter control 5
	if(core->fwct==0){
		core->landF=0;
		core->dxi		=0;
		core->dyi		=0;
		core->dxis		=0;
		core->dyis		=0;
		core->dxib		=0;
		core->dyib		=0;
		core->landF		=0;
		core->landB		=0;
		core->fwctEnd	=18;
		core->walkCt	=0;
		core->fwctUp	=1;
		core->fwct=1;
		core->autoH=HEIGHT;
		core->sw=0;
		core->swx=0;
		core->swy=0;
		core->jikuasi=1;
		core->mode=770;			// 状態遷移 state transition
		sprintf( (char *)globals.dsp,"mode=770\r\n" );
		printS((char *)globals.dsp);
	}
	detAng(core);			// 転倒検知 Fall detection
	break;


//**** ⑦ 回復後、振動減衰待ち ⑦ After recovery, wait for vibration to dampen ****
case 770:
	// initial values in mode 770: landF = 0, fwctEnd = 18, fwctUp = 1, fwct = 1
	feetCont2(core, state, 0);
	if( core->fwct>50 ){ // 50
		core->fwct=1;
		core->mode=730;		// 状態遷移 state transition
		sprintf( (char *)globals.dsp,"mode=730\r\n" );
		printS((char *)globals.dsp);
		break;
	}
	else{
		sprintf( (char *)globals.dsp,"C:%4d\r",(int)core->fwct );
		printS((char *)globals.dsp);
		++core->fwct;
	}
	detAng(core);			// 転倒検知 Fall detection
	break;


case 780:
	feetCont2(core, state, 1);
	break;


case 790:
	break;

case 791:

	core->dxis=-core->dxi;
	core->dyis= core->dyi;

	feetCont2(core, state, 1);
	break;


case 700:				// モニター monitor
	sprintf( (char *)globals.dsp,"R:%4d  %4d  RG:%4d  %4d\r\n", (int)(core->roll*1000),(int)(core->pitcht*1000), (int)core->roll_gyr,(int)core->pitch_gyr);
	printS((char *)globals.dsp);
	break;
	}
}



//######################
//#### キー読込制御 Key reading control ####
//######################
void keyCont(input_t* input, core_t* core, state_t* state){

	//////////////////////////
	////// TTYコマンド処理 TTY command processing //////
	//////////////////////////
	input->ff[0]=0;
	input->ff[1]=0;
	uart_rx (UART_COM, input->ff, 1,1);
	if(input->ff[0]!=0){
		sprintf( (char *)globals.dsp,"%c \r\n",input->ff[0] );
		printS((char *)globals.dsp);

		if(input->ff[0]==' ')input->keyMode=0; // キーモードリセット key mode reset

		///////////////////////
		//// 基本入力モード Basic input mode ////
		///////////////////////
		if(input->keyMode==0){
			switch(input->ff[0]){
				case 'r':		// リセット reset
					core->motCt=100;
					core->mode=710;
					sprintf( (char *)globals.dsp,"**** Reset ****\r\n" );
					printS((char *)globals.dsp);
					break;

				case 'g':		// 開始 start
					sprintf( (char *)globals.dsp,"**** Go ****\r\n" );
					printS((char *)globals.dsp);
					break;

				case 't':		// 試験 test
					core->mode=790;
					sprintf( (char *)globals.dsp,"**** angle Disp ****\r\n" );
					printS((char *)globals.dsp);
					break;

				case 'y':		// 試験 test
					core->mode=791;
					sprintf( (char *)globals.dsp,"**** debug ****\r\n" );
					printS((char *)globals.dsp);
					break;

//				case 'm':		// パラメタモード Parameter mode
//					core->keyMode=1;
//					break;
				case 'p':		// 特定変数設定モード Specific variable setting mode
					input->keyMode=5;
					break;
				case 'k':		// サーボ Kグループ Servo K group
					input->keyMode=2;
					break;
				case 'u':		// サーボ Uグループ Servo U group
					input->keyMode=3;
					break;
				case 'a':		// サーボ Aグループ Servo A group
					input->keyMode=4;
					break;
				case 'h':		// サーボ Hグループ Servo H group
					input->keyMode=50;
					break;
				case 'e':		// サーボ Eグループ Servo E group
					input->keyMode=60;
					break;
				case 'z':		// サーボ HEAD Servo HEAD
					input->keyMode=700;
					break;
				case 'w':		// サーボ WEST Servo WEST
					input->keyMode=800;
					break;
			}
		}


		///////////////////////////
		//// 特定変数設定モード Specific variable setting mode ////
		///////////////////////////
		if(input->keyMode==5){
			switch(input->ff[0]){
				case '0':
					input->kn=0;
					goto dd2;
				case '1':
					input->kn=1;
					goto dd2;
				case '2':
					input->kn=2;
					goto dd2;
				case '3':
					input->kn=3;
					goto dd2;
				case '4':
					input->kn=4;
					goto dd2;
				case '5':
					input->kn=5;
					goto dd2;
				case '6':
					input->kn=6;
					goto dd2;
				case '7':
					input->kn=7;
					goto dd2;
				case '8':
					input->kn=8;
					goto dd2;
				case '9':
					input->kn=9;
					goto dd2;
				case '+':
				case '-':
					switch(input->kn){
					case 0:core->dxi		+= input->ff[0]=='+'?	1:	-1;		break;
					case 1:core->dyi		+= input->ff[0]=='+'?	1:	-1;		break;
					case 2:core->swMax		+= input->ff[0]=='+'?	1:	-1;		break;
					case 3:core->pitch_gyrg	+= input->ff[0]=='+'?	0.01:-0.01;	break;
					case 4:core->roll_gyrg	+= input->ff[0]=='+'?	0.01:-0.01;	break;
					case 5:core->fh			+= input->ff[0]=='+'?  1:  -1;		break;
					case 6:core->fhMax		+= input->ff[0]=='+'?	1:  -1;		break;
					case 7:core->walkCtLim	+= input->ff[0]=='+'?  1:  -1;		break;
					case 8:core->autoH		+= input->ff[0]=='+'?  1:  -1;		break;
					}
dd2:
					sprintf( (char *)globals.dsp, "No:%d\r\n0 dx:%d\r\n1 dy:%d\r\n2 sw:%d\r\n3 pg:%d\r\n4 rg:%d\r\n5 fh:%d\r\n6 fh%d\r\n7  wc%d\r\n8 aH:%d\r\n"
						,input->kn 	,(int)core->dxi,(int)core->dyi,(int)core->swMax,(int)(core->pitch_gyrg*100),(int)(core->roll_gyrg*100),(int)core->fh,(int)core->fhMax,(int)core->walkCtLim,(int)core->autoH);
					printS((char *)globals.dsp);
					break;
			}
		}


		/////////////////////////
		//// サーボ設定モード Servo setting mode ////
		/////////////////////////
		if(input->keyMode==2){			// サーボ Kグループ Servo K group
			switch(input->ff[0]){
				case '0':		// K0選択 K0 selection
					input->keyMode=20;
					break;
				case '1':		// K1選択 K1 selection
					input->keyMode=21;
					break;
				case '2':		// K2選択 K2 selection
					input->keyMode=22;
					break;
			}
		}
		if(input->keyMode==3){			// サーボ Uグループ Servo U group
			switch(input->ff[0]){
				case '0':		// U0選択 U0 selection
					input->keyMode=30;
					break;
				case '1':		// U1選択 U1 selection
					input->keyMode=31;
					break;
				case '2':		// U2選択 U2 selection
					input->keyMode=32;
					break;
			}
		}
		if(input->keyMode==4){			// サーボ Aグループ Servo A group
			switch(input->ff[0]){
				case '0':		// A0選択 A0 selection
					input->keyMode=40;
					break;
				case '1':		// A1選択 A1 selection
					input->keyMode=41;
					break;
			}
		}

		if(input->keyMode>=20&& input->keyMode<=60){		// サーボ K,U,A,N,Eグループ Servo K,U,A,N,E group
			switch(input->ff[0]){
				case 'r':		// K0選択 K0 selection
					input->keyMode= input->keyMode*10;
					break;
				case 'l':		// K1選択 K1 selection
					input->keyMode= input->keyMode*10+1;
					break;
				case 'b':		// K2選択 K2 selection
					input->keyMode= input->keyMode*10+2;
					break;
			}
		}

		if(input->keyMode>=200&& input->keyMode<=800){		// サーボ K,U,A,N,Eグループ Servo K,U,A,N,E group
			int16_t incr=0;
			if(input->ff[0]=='+')incr= 30;
			if(input->ff[0]=='-')incr=-30;
			if(input->ff[0]=='+'||input->ff[0]=='-'){
				switch(input->keyMode){
				case 200:	state->K0W[0]+=incr;	break;
				case 201:	state->K0W[1]+=incr;	break;
				case 202:	state->K0W[0]+=incr; state->K0W[1]+=incr;break;
				case 210:	state->K1W[0]+=incr;	break;
				case 211:	state->K1W[1]+=incr;	break;
				case 212:	state->K1W[0]+=incr; state->K1W[1]+=incr;break;
				case 220:	state->K2W[0]+=incr;	break;
				case 221:	state->K2W[1]+=incr;	break;
				case 222:	state->K2W[0]+=incr; state->K2W[1]+=incr;break;

				case 300:	state->U0W[0]+=incr;	break;
				case 301:	state->U0W[1]+=incr;	break;
				case 302:	state->U0W[0]+=incr; state->U0W[1]+=incr;break;
				case 310:	state->U1W[0]+=incr;	break;
				case 311:	state->U1W[1]+=incr;	break;
				case 312:	state->U1W[0]+=incr; state->U1W[1]+=incr;break;
				case 320:	state->U2W[0]+=incr;	break;
				case 321:	state->U2W[1]+=incr;	break;
				case 322:	state->U2W[0]+=incr; state->U2W[1]+=incr;break;

				case 400:	state->A0W[0]+=incr;	break;
				case 401:	state->A0W[1]+=incr;	break;
				case 402:	state->A0W[0]+=incr; state->A0W[1]+=incr;break;
				case 410:	state->A1W[0]+=incr;	break;
				case 411:	state->A1W[1]+=incr;	break;
				case 412:	state->A1W[0]+=incr; state->A1W[1]+=incr;break;

				case 500:	state->HW[0]+=incr;	break;
				case 501:	state->HW[1]+=incr;	break;
				case 502:	state->HW[0]+=incr; state->HW[1]+=incr;break;

				case 600:	state->EW[0]+=incr;	break;
				case 601:	state->EW[1]+=incr;	break;
				case 602:	state->EW[0]+=incr; state->EW[1]+=incr;break;

				case 700:	state->HEADW+=incr;	break;
				case 701:	state->HEADW+=incr;	break;

				case 800:	state->WESTW+=incr;	break;
				case 801:	state->WESTW+=incr;	break;
				}
				sprintf( (char *)globals.dsp,    "Mode=%d\r\n", core->modeNxt );
				printS((char *)globals.dsp);
				sprintf( (char *)globals.dsp,    "K0:%7d %7d K1:%7d %7d K2:%7d %7d \r\n", state->K0W[0], state->K0W[1], state->K1W[0], state->K1W[1], state->K2W[0], state->K2W[1] );
				printS((char *)globals.dsp);
				sprintf( (char *)globals.dsp,    "H:%7d %7d \r\n", state->HW[0] , state->HW[1]  );
				printS((char *)globals.dsp);
				sprintf( (char *)globals.dsp,    "A0:%7d %7d A1:%7d %7d \r\n\r\n", state->A0W[0], state->A0W[1], state->A1W[0], state->A1W[1] );
				printS((char *)globals.dsp);
				sprintf( (char *)globals.dsp,    "U0:%7d %7d U1:%7d %7d U2:%7d %7d \r\n", state->U0W[0], state->U0W[1], state->U1W[0], state->U1W[1], state->U2W[0], state->U2W[1] );
				printS((char *)globals.dsp);
				sprintf( (char *)globals.dsp,    "E:%7d %7d \r\n\r\n", state->EW[0] , state->EW[1]  );
				printS((char *)globals.dsp);
				sprintf( (char *)globals.dsp,    "HD:%7d WT:%7d \r\n", state->HEADW , state->WESTW  );
				printS((char *)globals.dsp);
			}
		}
	}
}

void state_init(state_t* state) {
	state->K0W[0] = 0;			// 股関節前後方向右書込用 For writing right hip joint anteroposterior
	state->K1W[0] = 0;			// 股関節横方向右書込用 For hip joint lateral right writing
	state->K2W[0] = 0;			// 股関節横方向右書込用 For hip joint lateral right writing
	state->HW[0] = 0;			// 膝関節右書込用 For knee joint right writing
	state->A0W[0] = 0;			// 足首上下方向右書込用 For ankle upper and lower direction right writing
	state->A1W[0] = 0;			// 足首横方向右書込用 For ankle lateral right writing 
	state->U0W[0] = -5400;		// 肩前後方向右書込用 For shoulder front-back direction right writing
	state->U1W[0] = 0;			// 肩横後方向右書込用 For shoulder horizontal backward right writing
	state->U2W[0] = 0;			// 肩ヨー向右書込用 For shoulder yaw direction right writing
	state->EW[0] = 0;			// 肘右書込用 For writing on the right elbow
	state->WESTW = 0;			// 腰回転書込用 For writing waist rotation

	state->K0W[1] = 0;			// 股関節前後方向左書込用 For writing left hip joint anteroposterior direction
	state->K1W[1] = 0;			// 股関節横方向左書込用 For hip joint lateral left writing
	state->K2W[1] = 0;			// 股関節横方向左書込用 For hip joint lateral left writing
	state->HW[1] = 0;			// 膝関節左書込用 For knee joint left writing
	state->A0W[1] = 0;			// 足首上下方向左書込用 For writing in the upper and lower direction of the left ankle
	state->A1W[1] = 0;			// 足首横方向左書込用 For ankle lateral left writing
	state->U0W[1] = -5400;		// 肩前後方向左書込用 For writing on the left in the shoulder anteroposterior direction
	state->U1W[1] = 0;			// 肩横後方向左書込用 Shoulder horizontal backward direction left writing
	state->U2W[1] = 0;			// 肩ヨー向左書込用 For shoulder yaw direction left writing
	state->EW[1] = 0;			// 肘左書込用  For elbow left writing
	state->HEADW = 0;			// 頭回転書込用 For head rotation writing
}

void core_init(core_t* core) {
	core->LEDct = 0;	// LED点灯カウンタ LED lighting counter

	core->tBak = 0;
	core->pitchi = 0;
	core->tNow = 0;

	core->p_ofs = 0;
	core->r_ofs = 0;
	core->ir = 0;
	core->ip = 0;
	core->irb = 0;
	core->ipb = 0;

	core->motCt = 100;
	core->cycle = 10000;
	core->mode = 710;
	core->pitch_gyrg = 0.08;
	core->roll_gyrg = 0.1;

	core->swMax = 25;//22
	core->fhMax = 35;
	core->walkCtLim = 3;
}

void input_init(input_t* input) {
	input->kn = 0;
	input->keyMode = 0;
}

// **************************************************************************
// *************** Main routine *********************************************
// **************************************************************************
int32_t main_init(state_t* state, core_t* core, input_t* input) {

	state_init(state);

	///////////////////////
	//// タイマの初期化 Initializing the timer ////
	///////////////////////
	timer_init(TIMER, TIMER_MODE_TIMER32, 1000000);// 取り敢えず周期１秒に設定 For now, set the cycle to 1 second.
	delay(500);//wait 500ms この時間待たないとダメ I have to wait this long


	///////////////////
	//// PIO初期化 -- PIO initialization ////
	///////////////////
	pio_init(PIO_LED1, PIO_SET_OUT);// PIO(LED1)を出力に設定
	pio_init(PIO_LED2, PIO_SET_OUT);// PIO(LED2)を出力に設定
	pio_init(PIO_T1, PIO_SET_IN);	// PIO(T1)を入力に設定
	pio_init(PIO_T2, PIO_SET_OUT);	// PIO(T2)を出力に設定
	pio_init(PIO_T3, PIO_SET_IN);	// PIO(T3)を入力に設定
	pio_init(PIO_T4, PIO_SET_IN);	// PIO(T4)を入力に設定
	pio_init(PIO_T5, PIO_SET_OUT);	// PIO(T5)を出力に設定
	pio_init(PIO_T6, PIO_SET_OUT);	// PIO(T6)を出力に設定
	pio_init(PIO_SW1, PIO_SET_IN);	// PIO(SW1)を入力に設定
	pio_init(PIO_SW2, PIO_SET_IN);	// PIO(SW2)を入力に設定
	pio_write (PIO_LED2, HIGH);		// 緑 OFF


	/////////////////////////////
	//// シリアルポート初期化 Serial port initialization ////
	/////////////////////////////
	uart_init(UART_COM, UART, BR115200, 8, PARITY_NONE);
	i2c_init ( 400000, I2C_MASTER );

	sio_init (UART_SIO1, BR1250000);	// SIOの初期化 Initializing SIO
	sio_init (UART_SIO2, BR1250000);	// SIOの初期化 Initializing SIO
	sio_init (UART_SIO3, BR1250000);	// SIOの初期化 Initializing SIO
	sio_init (UART_SIO4, BR1250000);	// SIOの初期化 Initializing SIO


	/////////////////////////////
	//// アナログポート初期化 Analog port initialization ////
	/////////////////////////////
	ad_init(PIO_AD1, SWEEP);		// アナログポート１設定 Analog port 1 setting
	ad_init(PIO_AD2, SWEEP);		// アナログポート２設定 Analog port 2 settings
	dac_init();						// アナログ出力設定 Analog output settings


	////////////////////////
	//// 更新日時を表示 Display update date and time ////
	////////////////////////
	sprintf((char *)globals.dsp,"Version: %s %s\r\n", __DATE__, __TIME__);
	printS((char *)globals.dsp);


	/////////////////////////
	//// BMO055 開始処理 BMO055 Start processing ////
	/////////////////////////
	//// Make sure we have the right device ////
	if(read8(input, 0) !=0xA0 ){		// BNO055_ID
		delay(1000);			// hold on for boot
		if(read8(input, 0) !=0xA0 ){	// BNO055_ID
			printS("*** NG1 ***\r\n");
			return false;		// still not? ok bail
		}
	}
	//// Switch to config mode (just in case since this is the default) ////
	write8(input, 0X3D, 0);			//BNO055_OPR_MODE_ADDR  //OPERATION_MODE_CONFIG
	delay(30);
	//// Reset ////
	write8(input, 0X3F, 0x20);			//BNO055_SYS_TRIGGER_ADDR
	delay(500);
	while (read8(input, 0) != 0xA0){	//BNO055_CHIP_ID_ADDR  //BNO055_ID
		delay(1000);			// hold on for boot
		if(read8(input, 0) != 0xA0 ){			// BNO055_ID
			printS("*** NG2 ***\r\n");
			return false;		// still not? ok bail
		}
	}
	delay(50);
	//// Set to normal power mode ////
	write8(input, 0X3E, 0X00);			//BNO055_PWR_MODE_ADDR  //POWER_MODE_NORMAL
	delay(10);
	write8(input, 0X07, 0);			//BNO055_PAGE_ID_ADDR
	write8(input, 0X3F, 0);			//BNO055_SYS_TRIGGER_ADDR
	delay(10);
	//// Set the requested operating mode (see section 3.3) ////
	//  _mode = 0X0C;			//OPERATION_MODE_NDOF
	write8(input, 0X3D, 0X0C);			//BNO055_OPR_MODE_ADDR  //mode
	delay(1000);
	//// Use external crystal for better accuracy ////
	write8(input, 0X3D, 0);			//BNO055_OPR_MODE_ADDR  //OPERATION_MODE_CONFIG
	delay(50);
	write8(input, 0X07, 0);			//BNO055_PAGE_ID_ADDR
	write8(input, 0x0, 0x80);			//BNO055_SYS_TRIGGER_ADDR
	delay(10);
	//// Set the requested operating mode (see section 3.3) ////
	write8(input, 0X3D, 0X0C);			//BNO055_OPR_MODE_ADDR  //modeback
	delay(50);
	printS("*** BNO055 INIT OK ***\r\n");


	/////////////////////////////
	//// サーボ現在角度 Servo current angle ////
	/////////////////////////////
	int16_t i=ics_set_pos ( UART_SIO2, 1, 0 );	// U0Rバンザイ位置 U0R Banzai position
	state->U0W[0]=-i+4735;
	i=ics_set_pos ( UART_SIO2, 2, 0 );	// U1R +2700
	state->U1W[0]=-i+10110;
	i=ics_set_pos ( UART_SIO2, 3, 0 );	// U2R
	state->U2W[0]=-i+7500;
	i=ics_set_pos ( UART_SIO2, 4, 0 );	// ER
	state->EW [0]=i-4800;
	i=ics_set_pos ( UART_SIO4, 1, 0 );	// U0Lバンザイ位置 Banzai position
	state->U0W[1]=i-9320;
	i=ics_set_pos ( UART_SIO4, 2, 0 );	// U1L -2700
	state->U1W[1]=i-4850;
	i=ics_set_pos ( UART_SIO4, 3, 0 );	// U2L
	state->U2W[1]=i-7500;
	i=ics_set_pos ( UART_SIO4, 4, 0 );	// EL
	state->EW [1]=-i+10150;
	i=ics_set_pos ( UART_SIO1, 5, 0 );	// K2R
	state->K2W[0]=-i+7500;
	i=ics_set_pos ( UART_SIO1, 6, 0 );	// K1R
	state->K1W[0]=-i+7500; // was +7470
	i=ics_set_pos ( UART_SIO1, 7, 0 );	// K0R
	state->K0W[0]=-i+7500;
	i=ics_set_pos ( UART_SIO1, 8, 0 );	// HR +1760
	state->HW [0]=-i+7500; // was +9260
	i=ics_set_pos ( UART_SIO1, 9, 0 );	// A0R +350
	state->A0W[0]=i-7500; // was -7910
	i=ics_set_pos ( UART_SIO1,10, 0 );	// A1R
	state->A1W[0]=i-7500; // was -7585
	i=ics_set_pos ( UART_SIO3, 5, 0 );	// K2L
	state->K2W[1]=i-7500;
	i=ics_set_pos ( UART_SIO3, 6, 0 );	// K1L
	state->K1W[1]=i-7500;
	i=ics_set_pos ( UART_SIO3, 7, 0 );	// K0L
	state->K0W[1]=i-7500;
	i=ics_set_pos ( UART_SIO3, 8, 0 );	// HL -1760
	state->HW [1]=i-7500; // was -5740
	i=ics_set_pos ( UART_SIO3, 9, 0 );	// A0L -350
	state->A0W[1]=-i+7500; // was +7100
	i=ics_set_pos ( UART_SIO3,10, 0 );	// A1L
	state->A1W[1]=-i+7500; // was +7530
	i=ics_set_pos ( UART_SIO4, 0, 0 );	// HEADL
	state->HEADW=i-7500;
	i=ics_set_pos ( UART_SIO2, 0, 0 );	// WESTR
	state->WESTW=i-7500;


	/////////////////////////////
	//// サーボストレッチ設定 Servo stretch setting (Soft)1 - 127(Hard) ////
	/// From ICS3.5 Manual: Changes the retention properties of the servo motor. ///
	/// Reducing this value reduces the retention power of the motor, making it softer like a spring ///
	/////////////////////////////
//	ics_set_param ( UART_SIO1, 7,ICS_STRC_SC,250);	// K0R
//	ics_set_param ( UART_SIO3, 7,ICS_STRC_SC,250);	// K0L

	ics_set_param ( UART_SIO2, 1,ICS_STRC_SC,20);	// U0R
	ics_set_param ( UART_SIO4, 1,ICS_STRC_SC,20);	// U0L

	ics_set_param ( UART_SIO2, 2,ICS_STRC_SC,20);	// U1R
	ics_set_param ( UART_SIO4, 2,ICS_STRC_SC,20);	// U1L

	ics_set_param ( UART_SIO2, 4,ICS_STRC_SC,20);	// ER
	ics_set_param ( UART_SIO4, 4,ICS_STRC_SC,20);	// EL


	////////////////////////
	//// 10msタイマ開始 Start 10ms timer ////
	////////////////////////
	timer_write(TIMER,1000000);
	timer_start(TIMER);


	////////////////////
	//// 変数初期化 Variable initialization ////
	////////////////////

	core_init(core);

	input_init(input);

	return i;
}

void main_loop(state_t* state, core_t* core, input_t* input, int initialI) {
	int32_t i = initialI;

//----------------------------------------------------------------------------------
		////////////////////////////////////////////////
		//////////////////  MAIN LOOP  /////////////////
		////////////////////////////////////////////////
top:
	i = main_step(state, core, input, i);
	goto top;
}

int32_t main_step(state_t* state, core_t* core, input_t* input, int initialI) {
	int32_t i = initialI;

	//////////////////////
	//// 10ms待ち処理 10ms wait processing ////
	//////////////////////
pio_write (PIO_T2, HIGH);	// OFF(wait時間確認) -- OFF (check wait time)

	do{
		core->tNow=timer_read(TIMER);
	}while(core->tNow<core->cycle);

	if(core->tNow>core->cycle+10){
		sprintf( (char *)globals.dsp,"************** %d \r\n",(int)core->tNow);
		// printS ( (char *)globals.dsp ); // DS: disabled to decrease chatter
	}
	timer_start(TIMER);


	////////////////////
	//// サーボ設定 Servo settings ////
	////////////////////

	//// 関節リミット joint limit ////
	if(state->K1W[0]> 800)state->K1W[0]	= 800; // +27 deg
	if(state->K1W[0]<-450)state->K1W[0]	=-450; // -15 deg
	if(state->K1W[1]> 800)state->K1W[1]	= 800;
	if(state->K1W[1]<-450)state->K1W[1]	=-450;
	if(state->A0W[0]> 3500)state->A0W[0]	= 3500; // +118 deg
	if(state->A0W[0]<-3500)state->A0W[0]	=-3500; // +118 deg
	if(state->A0W[1]> 3500)state->A0W[1]	= 3500;
	if(state->A0W[1]<-3500)state->A0W[1]	=-3500;
	if(state->A1W[0]> 420)state->A1W[0]	= 420;	// 添付品曲加工+アルミソールでの実験結果 Experimental results with attached product curved processing + aluminum sole +14 deg
	if(state->A1W[0]<-900)state->A1W[0]	=-900;	// 添付品曲加工+アルミソールでの実験結果 Experimental results with attached product curved processing + aluminum sole -30.3 deg
	if(state->A1W[1]> 420)state->A1W[1]	= 420;	// 添付品曲加工+アルミソールでの実験結果 Experimental results with attached product curved processing + aluminum sole
	if(state->A1W[1]<-900)state->A1W[1]	=-900;	// 添付品曲加工+アルミソールでの実験結果 Experimental results with attached product curved processing + aluminum sole

	//// ICSデータ送受信 ICS data transmission/reception ////
	//// 注意ポジションデータが10500を超えるとサーボが応答しない（間欠反応） Caution: If the position data exceeds 10500, the servo will not respond (intermittent response)
	//// 注意ポジションデータが 3600以下では反応はするが動かない Caution If the position data is less than 3600, it will react but will not move.

	pio_write (PIO_T2, LOW );	// ON
	ics_set_pos    ( UART_SIO1, 5, 7560 -(state->K2W[0])-60 );	// K2R
	core->tNow=timer_read(TIMER);if(core->tNow>10010){sprintf( (char *)globals.dsp,"K2R**** %d \r\n",(int)core->tNow);printS ( (char *)globals.dsp );}

	pio_write (PIO_T2, HIGH);	// OFF
	ii=ics_set_pos ( UART_SIO2, 1, 4735 -(state->U0W[0]) );	// U0Rバンザイ位置 U0R Banzai position
	core->tNow=timer_read(TIMER);if(core->tNow>10010){sprintf( (char *)globals.dsp,"U0R**** %d \r\n",(int)core->tNow);printS ( (char *)globals.dsp );}
	state->U0R=-(ii-4735);

	pio_write (PIO_T2, LOW );	// ON
	ics_set_pos    ( UART_SIO3, 5, 7500 +(state->K2W[1])-90 );	// K2L
	core->tNow=timer_read(TIMER);if(core->tNow>10010){sprintf( (char *)globals.dsp,"K2L**** %d \r\n",(int)core->tNow);printS ( (char *)globals.dsp );}

	pio_write (PIO_T2, HIGH);	// OFF
	ii=ics_set_pos ( UART_SIO4, 1, 9320 +(state->U0W[1])-60 );	// U0Lバンザイ位置 U0L Banzai position
	core->tNow=timer_read(TIMER);if(core->tNow>10010){sprintf( (char *)globals.dsp,"U0L**** %d \r\n",(int)core->tNow);printS ( (char *)globals.dsp );}
	state->U0L=(ii-9230);

	pio_write (PIO_T2, LOW );	// ON
	ics_set_pos    ( UART_SIO1, 6, 7470 -(state->K1W[0])-30 );	// K1R
	core->tNow=timer_read(TIMER);if(core->tNow>10010){sprintf( (char *)globals.dsp,"K1R**** %d \r\n",(int)core->tNow);printS ( (char *)globals.dsp );}

	pio_write (PIO_T2, HIGH);	// OFF
	ii=ics_set_pos ( UART_SIO2, 2,10110 -(state->U1W[0]) );	// U1R +2700
	core->tNow=timer_read(TIMER);if(core->tNow>10010){sprintf( (char *)globals.dsp,"U1R**** %d \r\n",(int)core->tNow);printS ( (char *)globals.dsp );}
	state->U1R=-(ii-10110);

	pio_write (PIO_T2, LOW );	// ON
	ics_set_pos    ( UART_SIO3, 6, 7650 +(state->K1W[1])-90 );// K1L
	core->tNow=timer_read(TIMER);if(core->tNow>10010){sprintf( (char *)globals.dsp,"K1L**** %d \r\n",(int)core->tNow);printS ( (char *)globals.dsp );}

	pio_write (PIO_T2, HIGH);	// OFF
	ii=ics_set_pos ( UART_SIO4, 2, 4850 +(state->U1W[1]) );	// U1L -2700
	core->tNow=timer_read(TIMER);if(core->tNow>10010){sprintf( (char *)globals.dsp,"U1L**** %d \r\n",(int)core->tNow);printS ( (char *)globals.dsp );}
	state->U1L=ii-4850;


	pio_write (PIO_T2, LOW );	// ON
	ii=ics_set_pos ( UART_SIO1, 7, 7480 -(state->K0W[0])-30 );	// K0R
	core->tNow=timer_read(TIMER);if(core->tNow>10010){sprintf( (char *)globals.dsp,"K0R**** %d \r\n",(int)core->tNow);printS ( (char *)globals.dsp );}
	state->K0R=7510-ii;

	pio_write (PIO_T2, HIGH);	// OFF
	ics_set_pos    ( UART_SIO2, 3, 7500 -(state->U2W[0]) );	// U2R
	core->tNow=timer_read(TIMER);if(core->tNow>10010){sprintf( (char *)globals.dsp,"U2R**** %d \r\n",(int)core->tNow);printS ( (char *)globals.dsp );}

	pio_write (PIO_T2, LOW );	// ON
	ii=ics_set_pos ( UART_SIO3, 7, 7500 +(state->K0W[1]) );	// K0L
	core->tNow=timer_read(TIMER);if(core->tNow>10010){sprintf( (char *)globals.dsp,"K0L**** %d \r\n",(int)core->tNow);printS ( (char *)globals.dsp );}
	state->K0L=ii-7500;

	pio_write (PIO_T2, HIGH);	// OFF
	ics_set_pos    ( UART_SIO4, 3, 7500 +(state->U2W[1]) );	// U2L
	core->tNow=timer_read(TIMER);if(core->tNow>10010){sprintf( (char *)globals.dsp,"U2L**** %d \r\n",(int)core->tNow);printS ( (char *)globals.dsp );}

	pio_write (PIO_T2, LOW );	//ON
	ics_set_pos    ( UART_SIO1, 8, 9320 -(state->HW [0]) );	// HR +1760
	core->tNow=timer_read(TIMER);if(core->tNow>10010){sprintf( (char *)globals.dsp,"HR**** %d \r\n",(int)core->tNow);printS ( (char *)globals.dsp );}

	pio_write (PIO_T2, HIGH);	//OFF
	ics_set_pos    ( UART_SIO2, 4, 4800 +(state->EW [0]+i) );	// ER
	core->tNow=timer_read(TIMER);if(core->tNow>10010){sprintf( (char *)globals.dsp,"ER**** %d \r\n",(int)core->tNow);printS ( (char *)globals.dsp );}

	pio_write (PIO_T2, LOW );	//ON
	ics_set_pos    ( UART_SIO3, 8, 5770 +(state->HW [1])-120 );	// HL -1760
	core->tNow=timer_read(TIMER);if(core->tNow>10010){sprintf( (char *)globals.dsp,"HL**** %d \r\n",(int)core->tNow);printS ( (char *)globals.dsp );}

	pio_write (PIO_T2, HIGH);	//OFF
	ics_set_pos    ( UART_SIO4, 4,10150 -(state->EW [1]+i) );	// EL
	core->tNow=timer_read(TIMER);if(core->tNow>10010){sprintf( (char *)globals.dsp,"EL**** %d \r\n",(int)core->tNow);printS ( (char *)globals.dsp );}


	i=core->pitch_gyrg*core->pitch_gyr;
	pio_write (PIO_T2, LOW );	//ON
	ics_set_pos    ( UART_SIO1, 9, 7870-10 +(state->A0W[0]) + i+60 );	// A0R

	pio_write (PIO_T2, HIGH);	//OFF
	ics_set_pos    ( UART_SIO2, 0, 7500 +(state->WESTW ) );			// WESTR

	pio_write (PIO_T2, LOW );	//ON
	ics_set_pos    ( UART_SIO3, 9, 7100    -(state->A0W[1]) - i );	// A0L

	pio_write (PIO_T2, HIGH);	//OFF
	ics_set_pos    ( UART_SIO4, 0, 7500 +(state->HEADW ) );			// HEADL

	i=core->roll_gyrg*core->roll_gyr;
	pio_write (PIO_T2, LOW );	//ON
	ics_set_pos    ( UART_SIO1,10, 7470 +(state->A1W[0]) - i-30 );		// A1R

	pio_write (PIO_T2, HIGH);	//OFF
	ics_set_pos    ( UART_SIO3,10, 7560 -(state->A1W[1]) - i-60 );		// A1L


	/////////////////////
	//// IMU読取処理 IMU reading process ////
	/////////////////////
	//      +----------+
	//      |         *| RST   PITCH  ROLL  HEADING
	//  ADR |*        *| SCL
	//  INT |*        *| SDA     ^            /->
	//  PS1 |*        *| GND     |            |
	//  PS0 |*        *| 3VO     Y    Z-->    \-X
	//      |         *| VIN
	//      +----------+

	//現状　2020 2/6    roll : 右側傾き（2879） 左側傾き（-2879）閾値2879 -- Current status 2020 2/6      roll: Right tilt(2879) Left tilt(-2879) Threshold 2879
	//                  pitch: 前側傾き（+）    後側傾き（-）    補正値20                                pitch: Front tilt (+) Back tilt (-) Correction value 20

	readLen(input, 0X1A, 6);	// 絶対角度読込(degの16倍表示) Absolute angle reading (displayed at 16 times deg)
	core->yaw	= ((int16_t)input->ff[0]) | (((int16_t)input->ff[1]) << 8); // 直立右回転で + Standing upright and turning clockwise +
	core->pitchs = ((int16_t)input->ff[2]) | (((int16_t)input->ff[3]) << 8); // 直立前傾で   - Standing upright and leaning forward -
	core->rolls  = ((int16_t)input->ff[4]) | (((int16_t)input->ff[5]) << 8); // 直立右傾斜で + Upright with right tilt +
	if(core->rolls>0)	core->rolls= 2879-core->rolls;
	else				core->rolls=-2879-core->rolls;

	core->pitchs -= core->p_ofs;	// 補正 correction
	core->rolls  -= core->r_ofs;	// 補正 correction

	core->pitch = (float)core->pitchs*(M_PI/(180.0*16.0));	// radに変換 convert to rad
	core->roll  = (float)core->rolls *(M_PI/(180.0*16.0));	// radに変換 convert to rad


	readLen(input, 0X14, 6);	// 角速度読込 ※rollとyawが取説と実際が逆 Angular velocity reading *Roll and yaw are opposite from the instruction manual.
	core->roll_gyr = ((int16_t)input->ff[0]) | (((int16_t)input->ff[1]) << 8);	// 直立右傾斜で ＋ Standing upright and leaning to the right +
	core->pitch_gyr	= ((int16_t)input->ff[2]) | (((int16_t)input->ff[3]) << 8);	// 直立前傾で   － Stand upright and lean forward -
	core->yaw_gyr	= ((int16_t)input->ff[4]) | (((int16_t)input->ff[5]) << 8);	// 直立右回転で ＋ Stand upright and turn clockwise +


	pio_write (PIO_T2, LOW );	// ON


	keyCont(input, core, state);


	walk(core, state);


	///////////////
	//// 頭LED Head LED ////
	///////////////
	++core->LEDct;
	if( core->LEDct > 100 )core->LEDct = -100;				// IMU Ready

	if(core->mode<=720 && core->LEDct > 10 )core->LEDct = -10;	// IMU not Ready

	if( core->LEDct > 0   ){
		dac_write (0xffff);
		pio_write (PIO_LED1, LOW );	// ON
	}
	else{
		dac_write (0);
		pio_write (PIO_LED1, HIGH);	// OFF
	}

	return i;
}

int controllerMain()
{
	int32_t i = main_init(&state, &core, &input);
	main_loop(&state, &core, &input, i);

	return 0;
}
