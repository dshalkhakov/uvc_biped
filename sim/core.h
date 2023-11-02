/*
June 1,2021
Delete variable K2Ｗ[2]
*/

#pragma once

class core{
	public:
	float dx[2];			// 前後方向脚位置 Anteroposterior leg position
	unsigned char jikuasi;
	float adjFR;			// 前後調整値 Front and back adjustment value
	float autoH;			// 高さ自動調整 height automatic adjustment
	float autoHs;			// 高さ自動調整 height auto adjustment
	float fw;				// 中心からの振り出し股幅 Swing crotch width from center
	short mode;
	unsigned char sw;		// 横振り距離 lateral swing distance
	unsigned char swf;		// 横振り最大距離 Maximum horizontal swing distance
	float fwr0;				// 中心からの戻り股幅(軸足) Return crotch width from center (axis foot)
	float fwr1;				// 中心からの戻り股幅(遊脚) Return crotch width from center (swing leg)
	float landRate;			// 0.1一歩の内、両足接地期間の割合 Ratio of both foot contact period in 0.1 step
	short fwctEnd;			// 一周期最大カウント数 Maximum count in one cycle
	short fwct;				// 一周期カウンタ One cycle counter
	float fh;				// 足上げ高さ指示 Instruction of foot raising height
	short fhMax;			// 足上げ最大高さ Maximum height leg is raised
	float fhOfs;			// 足上げオフセット値 Foot raise offset value
	float dy;				// 横振り距離 lateral swing distance
	float dxi;				// 縦距離積分値 Vertical distance integral value
	float dyi;				// 横距離積分値 lateral distance integral value
	float dvi;				// 上体角積分値 Upper body angle integral value
	float dvo;				// 上体角オフセット値 Body angle offset value
	float pitch;
	float roll;

	void footCont(bipedstate_t* state, float x,float y,float h,int s);
	void walk(bipedstate_t* state, bipedinput_t* input);
	core(void);
	~core(void);
};