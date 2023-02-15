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

#define Goal_x  1
#define Goal_y  1
#define Goal_angle 1 //ゴールした直後の向き

#define Pickup_x  0
#define Pickup_y  1

//ジャイロ関連
#define r45  (11500)	//45度 右回転 11500 10500
#define l45  (11500)	//45度 左回転 11500
#define r90  (22000) 	//90度 右回転
#define l90  (22000)	//90度 左回転
#define r180  (-43500)	//180度 右回転
#define sr90  (23000)	//スラローム 23000
#define sl90  (23000)	//スラローム 23000

#define sr90BIG  (24000)	//スラローム 
#define sl90BIG  (23800)	//スラローム 

//エンコーダ関連　memo : 1mm = 4 //壁切れ170
#define s1 (715)		//1マス直進 700  　735
#define s45 (500)		//45度１マス直進 500 520
#define h1 (350)		//半マス直進 350 340
#define rslsl90 (664)	//スラローム 664
#define rslsr90 (664)	//スラローム 660

#define rslsl90_BIG (700)	//スラローム 大
#define rslsr90_BIG (680)	//スラローム 大

#define r_cost 4		//迷路の重み　90度回転の場合

#endif