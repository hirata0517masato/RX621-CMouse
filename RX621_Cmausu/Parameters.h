#ifndef PARAM_H
#define PARAM_H

/*
中央に置いたとき
横壁は60　前壁は50 くらい
*/

#define maze_d_max	9999
#define H	16
#define W	16

#define Start_x  0
#define Start_y  0
#define Start_angle 1 // スタートする時の向き　

#define Goal_x  2
#define Goal_y  2
#define Goal_angle 0 //ゴールした直後の向き

#define Pickup_x  2
#define Pickup_y  0

//ジャイロ関連
#define r45  (11000)	//45度 右回転
#define l45  (11500)	//45度 左回転
#define r90  (21000) 	//90度 右回転
#define l90  (21500)	//90度 左回転
#define r180  (-44000)	//180度 右回転
#define sr90  (24400)	//スラローム
#define sl90  (24400)	//スラローム

//エンコーダ関連　memo : 1mm = 4 //壁切れ170
#define s1 (700)		//1マス直進 700  　735
#define s45 (500)		//45度１マス直進 500 520
#define h1 (350)		//半マス直進 350 340
#define rslsl90 (645)	//スラローム
#define rslsr90 (647)	//スラローム

#define r_cost 4		//迷路の重み　90度回転の場合

#endif