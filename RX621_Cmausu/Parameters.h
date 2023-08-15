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
#define Goal_angle 0 //ゴールした直後の向き スタートの向きが1

#define Pickup_x  2
#define Pickup_y  0

//ジャイロ関連
#define r45  (11500)	//45度 右回転 11500 10500
#define l45  (11500)	//45度 左回転 11500
#define r90  (21470) 	//90度 右回転
#define l90  (21300)	//90度 左回転
#define r180  (-43000)	//180度 右回転
#define sr90  (25000)	//スラローム 23000 //左右で同じ値でないと連続時にずれていく
#define sl90  (25000)	//スラローム 23000

#define sr90BIG  (25000)	//スラローム 
#define sl90BIG  (25000)	//スラローム 

//エンコーダ関連　memo : 1mm = 5.56 //壁切れ170 //2ndの1.34倍くらい
#define s1 (1025)		//1マス直進 714
#define s45 (780)		//45度１マス直進 500
#define h1 (525)		//半マス直進 350
#define rslsl90 (970)	//スラローム 664
#define rslsr90 (970)	//スラローム 664

#define rslsl90_BIG (985)	//スラローム 大700
#define rslsr90_BIG (985)	//スラローム 大 680

#define r_cost 4		//迷路の重み　90度回転の場合


//前壁補正のパラメータ
#define F_max 255
#define F_min 240
#define F_pow 8
#define F_cnt 600


#endif