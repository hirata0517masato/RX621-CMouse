#ifndef PARAM_H
#define PARAM_H

#define maze_d_max	9999

#define W	16 //X  変更すると迷路情報の保存ができなくなるバグあり
#define H	16 //Y

#define Start_x  0
#define Start_y  0
#define Start_angle 1 // スタートする時の向き　


////////////////////////////////////////////////////////////////////////////////////////////////////

#define Goal_x  7
#define Goal_y  7
#define Goal_angle 3 //ゴールした直後の向き スタートの向きが1

//最短走行後、取り上げやすい位置に移動　固定値版
//#define Pickup_x  1	
//#define Pickup_y  1

//最短走行後、取り上げやすい位置に移動　範囲指定版
#define Not_Pickup_x_min  3		//0〜15 の範囲で設定　　( j < Not_Pickup_x_min ||  Not_Pickup_x_max < j)が対象範囲
#define Not_Pickup_y_min  3

#define Not_Pickup_x_max  17
#define Not_Pickup_y_max  17

//ジャイロ関連
#define r45  (11500)	//45度 右回転 11500 10500
#define l45  (11500)	//45度 左回転 11500
#define r90  (21300) 	//90度 右回転
#define l90  (21200)	//90度 左回転
#define r180  (-43000)	//180度 右回転
#define sr90  (25000)	//スラローム 23000 //左右で同じ値でないと連続時にずれていく
#define sl90  (25000)	//スラローム 23000

#define sr90BIG  (25500)	//スラローム 大曲
#define sl90BIG  (25500)	//スラローム 大曲

#define ur180  (47500)	//Uターン //左右で同じ値でないと連続時にずれていく
#define ul180  (47500)	//Uターン

//エンコーダ関連　memo : 1mm = 5.56 //壁切れ170 //2ndの1.34倍くらい
#define s1 (1026)		//1マス直進      タイヤ削れた時：1025
#define s45 (800)		//45度１マス直進 タイヤ削れた時：800
#define h1 (530)		//半マス直進 	タイヤ削れた時：525　最短用
#define h1_2 (500)		//半マス直進 	タイヤ削れた時：525  探索の壁切れ用

#define s45_V2 (140)		//２マスVターンの距離　壁切れができない可能性が高いので実質距離で調整する
#define s45_V2_out (400)	//２マスVターンの距離　壁切れができない可能性が高いので実質距離で調整する
#define s45_V2_out_LR (120)	//２マスVターンの距離　壁切れができない可能性が高いので実質距離で調整する　壁切れ後にカーブ
#define s45_V2not (300)		//２マス斜め（Vターンではない）の距離　壁切れができない可能性が高いので実質距離で調整する

#define rslsl90 	(980)	//スラローム 664
#define rslsl90_offset  (0)
#define rslsr90 	(980)	//スラローム 664
#define rslsr90_offset  (-0)

#define rslsl90_BIG (1600)	//スラローム 大700
#define rslsl90_BIG_offset  (-100)
#define rslsr90_BIG (1600)	//スラローム 大 680
#define rslsr90_BIG_offset  (-100)

#define usll180     (2250)	//Uターン
#define usll180_fin (0)	//Uターン 微調整
#define uslr180     (2250)	//Uターン
#define uslr180_fin (0)	//Uターン　微調整

#define r_cost 4		//迷路の重み　90度回転の場合

#define r45_cost 1		//迷路の重み　45度回転の場合


//前壁補正のパラメータ
#define F_max 250
#define F_min 230
#define F_pow 15
#define F_cnt 400
#define F_max_time 1000


#endif