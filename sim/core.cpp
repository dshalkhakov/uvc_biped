/*
June 1,2021
Corrected setting errors of LEG, autoH, and autoHs.
Corrected (K0脚振り角度) (K0 leg swing angle)
*/

#include < ode/ode.h >
#include < drawstuff/drawstuff.h >
#include < stdio.h >
#include < stdlib.h >
#include < windows.h >
#include < fstream >
#include < iostream >
#include < string >
#include < sstream >
#include < iomanip >
#include < math.h >
#include < conio.h >  
#include  "biped.h"
#include  "core.h"

#define LEG 180.0	// Update in June 1,2021 :Before revision(#define LEG 190.0)
#define MAX_SWING_WIDTH	(100.0)
#define MAX_DYI			(0.0)
#define MIN_DYI			(-30.0)
#define DYI_DAMPING_FACTOR (0.90)
#define ROLL_DISPLACEMENT	(193)
#define PITCH_DISPLACEMENT	(130)

using namespace std;

core::core(void){
	adjFR	=2.04;
	autoH	=170;	// Update in June 1,2021 :Before revision(autoH=180)
	autoHs	=180;	// Update in June 1,2021 :Before revision(autoHs=190)
	mode	=0;
	pitch	=0;
	roll	=0;
}

core::~core(void){}

// *****************
// **  脚位置制御 leg position control  **
// *****************
void core::footCont(simstate_t* state, float x,float y,float h,int s){
//x:中点を0とする足前後方向距離（前+）Distance in the front and rear direction of the foot with the midpoint as 0 (front +)
//y:中点を0とする足左右方向距離（右+）Distance in the left and right direction of the foot with the midpoint as 0 (right +)
//h:足首ロール軸から股関節ロール軸までの距離 Distance from ankle roll axis to hip roll axis
//s:支持脚(0)遊脚(1)を指定 Specify support leg (0) and swing leg (1)
	float k;

	k = sqrt(x*x+(y*y+h*h));	// A0からK0までの距離 Distance from A0 to K0
	if(k>LEG)k=LEG;				// 計算エラー回避 Avoid calculation errors

	float x0 = asin(x/k);			// K0脚振り角度 Update in June 1,2021 :Before revision( x=asin(x/LEG) )
							// K0 leg swing angle Update in June 1,2021 :Before revision( x=asin(x/LEG) )

	float k0 = acos(k/LEG);			// K0膝曲げ角度 K0 knee bending angle

	state->fbAV=0;						// UVC評価の為、ジャイロは無効にする Gyro is disabled for UVC evaluation
	state->lrAV=0;
	state->K0W[s]	= k0+x0+dvi+dvo;
	state->HW[s]	= k0*2;
	state->A0W[s]	= k0-x0-0.003*state->fbAV;
	float k1 = atan(y/h);				// K1角度 K1 angle
	state->K1W[s] = k1;
	if(s==0)	state->A1W[s] = -k1-0.002*state->lrAV;
	else		state->A1W[s] = -k1+0.002*state->lrAV;
}

// *********************
// **  歩行制御メイン Walking control main  **
// *********************
void core::walk(simstate_t* state, siminput_t* input){
	switch(mode){

	////////////////////////
	//// 初期姿勢に移行 Transition to initial posture ////
	////////////////////////
	case 0:
		if(autoHs>autoH)	autoHs-=1;	
		else				mode=710;
		footCont( state, -adjFR, 0, autoHs,  0 );
		footCont( state, -adjFR, 0, autoHs,  1 );
		break;

	//////////////////////
	//// アイドル状態 idle state ////
	//////////////////////
	case 710:
		state->K0W[0]	=  0;
		state->K0W[1]	=  0;

		//// パラメータ初期化 Parameter initialization ////
		dx[0]	=0;
		dx[1]	=0;
		fwr0	=0;
		fwr1	=0;
		fwct	=0;
		dxi		=0;
		dyi		=0;
		dy		=0;
		dvi		=0;
		dvo		=0;
		fw		=0;
		jikuasi	=0;
		fwctEnd	=48;
		swf		=12;
		fhMax	=20;
		landRate=0.2;
		fhOfs	=0;
		footCont( state, -adjFR, 0, autoH, 0 );
		footCont( state, -adjFR, 0, autoH, 1 );
		if(input->walkF&0x01)	mode=720;
		break;


	/////////////////////////////////////////////////////////////////
	//////////////////////// 　歩行制御 walking control   ///////////////////////////
	/////////////////////////////////////////////////////////////////
	case 720:
	case 725:
	case 730:

		//###########################################################
		//###################  UVC(上体垂直制御) UVC (upper body vertical control) ####################
		//###########################################################
		if(fw<input->fwMax-1)dvo   = input->frRatA;			//// 前傾前進 Lean forward ////
		else dvo=0;
		dvi   += input->frRatI*(state->fbRad+dvo);			//// 上体角積分 upper body angle integral ////

		if((jikuasi==0 && state->asiPress_r<-0.1 && state->asiPress_l>-0.1) ||
			 (jikuasi==1 && state->asiPress_r>-0.1 && state->asiPress_l<-0.1)){
			float rollDispl = 1.5 * ROLL_DISPLACEMENT * sin(state->lrRad);	//// 左右方向変位 Lateral displacement (roll) ////
			if(jikuasi==0)	dyi += rollDispl;
			else			dyi -= rollDispl;
			if(dyi>MAX_DYI)	dyi=MAX_DYI;
			if(dyi<MIN_DYI)	dyi=MIN_DYI;
			float pitchDispl = 1.5 * PITCH_DISPLACEMENT * sin(state->fbRad);	//// 前後方向変位 Anteroposterior displacement (pitch) ////
			dxi += pitchDispl;
		}
		dyi*=DYI_DAMPING_FACTOR;						// 減衰 damping
		if(input->uvcOff==1){
			dxi=0;
			dyi=0;
		}


		//###########################################################
		//########################  基本歩容 Basic steps  #######################
		//###########################################################

		//// 横振り horizontal swing ////
		float horizSwing=swf*sinf(M_PI*(fwct)/fwctEnd); // sinカーブ sine curve
		if(jikuasi==0)	dy=  horizSwing; // 右振り right swing
		else			dy= -horizSwing; // 左振り left swing

		//// 軸足側前振り制御 Forward swing control on the pivot foot side ////
		if(fwct<fwctEnd/2)	dx[jikuasi] =      fwr0*(1-2.0*fwct/fwctEnd  );	// 立脚中期まで Until mid-stance
		else				dx[jikuasi] = -(fw-dxi)*(  2.0*fwct/fwctEnd-1);	// 立脚中期移行UVC適用 Mid-stance transition UVC applied

		//// 遊脚側前振り制御 Idle leg forward swing control ////
		if(mode==720||mode==725){								// 両脚シフト期間 Double leg shift period
			if( fwct<(landRate*fwctEnd) ){
				dx[jikuasi^1] = fwr1-(fwr0-dx[jikuasi]);
				if(mode==720&&fwct>2){
					if( (jikuasi==0&&state->asiPress_r<-0.1) || (jikuasi==1&&state->asiPress_l<-0.1) ) mode=725;
					else fhOfs+=input->fhRat;
				}
			}
			else{
				fwr1=dx[jikuasi^1];
				mode=730;
			}
		}

		if(mode==730){								// 前振出 forward swing
			float forwardSwing=(
				-cosf(
					M_PI*( fwct-landRate*fwctEnd )/
					( (1-landRate)*fwctEnd )			// 前振り頂点までの残りクロック数 Number of clocks remaining until the top of the forward swing
				)+1
			)/2;										// 0-1の∫的カーブ 0-1 ∫ curve
			dx[jikuasi^1] = fwr1+ forwardSwing * ( fw-dxi-fwr1 );
		}
		if(dx[jikuasi]> MAX_SWING_WIDTH){							// 振り出し幅リミット swing width limit
			dxi		   -= dx[jikuasi]-MAX_SWING_WIDTH;
			dx[jikuasi] = MAX_SWING_WIDTH;
		}
		if(dx[jikuasi]<-MAX_SWING_WIDTH){
			dxi		   -= dx[jikuasi]+MAX_SWING_WIDTH;
			dx[jikuasi] =-MAX_SWING_WIDTH;
		}
		if(dx[jikuasi^1]> MAX_SWING_WIDTH) dx[jikuasi^1] = MAX_SWING_WIDTH;	// 振り出し幅リミット抑制 Suppression of swing width limit
		if(dx[jikuasi^1]<-MAX_SWING_WIDTH) dx[jikuasi^1] =-MAX_SWING_WIDTH;

		//// 足上制御 foot control ////
		short i=landRate*fwctEnd;
		if( fwct>i ){
			if( fwct<(fwctEnd-i)/2 ) fh = fhOfs + fhMax * sinf( M_PI*(fwct-i)/(fwctEnd-i) );
			else					 fh = (fhMax+fhOfs) * sinf( M_PI*(fwct-i)/(fwctEnd-i) );
		}
		else fh = fhOfs;

		//// 脚制御関数呼び出し Leg control function call ////
		if(jikuasi==0){
			footCont( state, dx[0]-adjFR	, -dy-dyi+1, autoH,	0 );
			footCont( state, dx[1]-adjFR	,  dy-dyi+1, autoH-fh,	1 );
		}
		else{
			footCont( state, dx[0]-adjFR	, -dy-dyi+1, autoH-fh,	0 );
			footCont( state, dx[1]-adjFR	,  dy-dyi+1, autoH,	1 );
		}


		//###########################################################
		//###########  CPG（歩周期生成、今回は簡易仕様） ############
		//## CPG (step cycle generation, simple specification this time) ##
		//###########################################################
		if( fwct==fwctEnd ){	// 簡易仕様（固定周期） Simple specifications (fixed cycle)
			jikuasi^=1;
			fwct=1;
			dxi=0;
			fwr0 = dx[jikuasi];
			fwr1 = dx[jikuasi^1];
			fh=0;
			fhOfs=0;
			mode=720;
			if(fw<fwr0)fw=fwr0;
			if(fw>input->fwMax)fw=input->fwMax;			//// 上体前後角積分 Upper body anteroposterior angle integral ////
		}
		else  ++fwct;
		break;
	}
}
