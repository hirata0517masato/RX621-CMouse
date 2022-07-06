#ifndef PARAM_H
#define PARAM_H


#define maze_d_max	9999
#define H	16
#define W	16
#define Start_x  0
#define Start_y  0
#define Goal_x  0
#define Goal_y  2

//ジャイロ関連
#define r45  (10300)	//45度 右回転
#define l45  (10300)	//45度 左回転
#define r90  (21000) 	//90度 右回転
#define l90  (21500)	//90度 左回転
#define r180  (-43000)	//180度 右回転
#define sr90  (23000)	//スラローム
#define sl90  (23000)	//スラローム

//エンコーダ関連　memo : 1mm = 4
#define s1 (735)		//1マス直進
#define s45 (520)		//45度１マス直進
#define h1 (340)		//半マス直進
#define rsls90 (647)	//スラローム

#define r_cost 4		//迷路の重み　90度回転の場合

#endif