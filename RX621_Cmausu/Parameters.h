#ifndef PARAM_H
#define PARAM_H


#define maze_d_max	9999
#define H	16
#define W	16

#define Start_x  0
#define Start_y  0
#define Start_angle 1

#define Goal_x  6
#define Goal_y  0
#define Goal_angle 0

#define Pickup_x  5
#define Pickup_y  0

//ジャイロ関連
#define r45  (12000)	//45度 右回転
#define l45  (12500)	//45度 左回転
#define r90  (20500) 	//90度 右回転
#define l90  (21000)	//90度 左回転
#define r180  (-43000)	//180度 右回転
#define sr90  (23000)	//スラローム
#define sl90  (23500)	//スラローム

//エンコーダ関連　memo : 1mm = 4 //壁切れ170
#define s1 (700)		//1マス直進 700  　735
#define s45 (500)		//45度１マス直進 500 520
#define h1 (350)		//半マス直進 350 340
#define rsls90 (647)	//スラローム

#define r_cost 4		//迷路の重み　90度回転の場合

#endif