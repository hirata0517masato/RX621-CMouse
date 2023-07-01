#ifndef PARAM_H
#define PARAM_H

/*
中央に置いたとき
横壁は60　前壁は50 くらい
*/

#define maze_d_max	9999

#define W	16 //X
#define H	16 //Y

#define Start_x  0
#define Start_y  0
#define Start_angle 1 // スタートする時の向き　

#define Goal_x  0
#define Goal_y  1
#define Goal_angle 0 //ゴールした直後の向き

#define Pickup_x  2
#define Pickup_y  0

//ジャイロ関連
#define r45  (11600)	//45度 右回転 11500 10500
#define l45  (11600)	//45度 左回転 11500
#define r90  (21300) 	//90度 右回転
#define l90  (21200)	//90度 左回転
#define r180  (-43500)	//180度 右回転
#define sr90  (24000)	//スラローム 23000
#define sl90  (25000)	//スラローム 23000

#define sr90BIG  (25500)	//スラローム 
#define sl90BIG  (24500)	//スラローム 

//エンコーダ関連　memo : 1mm = 5.56 //壁切れ170 //2ndの1.34倍くらい
#define s1 (995)		//1マス直進 714
#define s45 (700)		//45度１マス直進 500
#define h1 (500)		//半マス直進 350
#define rslsl90 (942)	//スラローム 664
#define rslsr90 (942)	//スラローム 664

#define rslsl90_BIG (980)	//スラローム 大700
#define rslsr90_BIG (980)	//スラローム 大 680

#define r_cost 4		//迷路の重み　90度回転の場合


//前壁補正のパラメータ
#define F_max 275
#define F_min 260
#define F_pow 10
#define F_cnt 600


#endif