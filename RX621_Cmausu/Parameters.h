#ifndef PARAM_H
#define PARAM_H

#define maze_d_max	9999

#define W	16 //X  変更すると迷路情報の保存ができなくなるバグあり
#define H	16 //Y

#define Start_x  0
#define Start_y  0
#define Start_angle 1 // スタートする時の向き　


////////////////////////////////////////////////////////////////////////////////////////////////////

#define Goal_x  0
#define Goal_y  1
#define Goal_angle 0 //ゴールした直後の向き スタートの向きが1

//最短走行後、取り上げやすい位置に移動　固定値版 //コメントアウトすると範囲指定版に切り替わる
#define Pickup_x  1	
#define Pickup_y  1

//最短走行後、取り上げやすい位置に移動　範囲指定版
#define Not_Pickup_x_min  4		//0～15 の範囲で設定　　( j < Not_Pickup_x_min ||  Not_Pickup_x_max < j)が対象範囲
#define Not_Pickup_y_min  4

#define Not_Pickup_x_max  16
#define Not_Pickup_y_max  16

//タイヤ交換したときに変更するパラメータ
#define W_D	(220) //ホイールの直径(0.1mm)
#define W_P	(400) //エンコーダ1週のパルス数

//ジャイロ関連
#define l45  (11500)	//45度 左回転 11500
#define r45  (11500)	//45度 右回転 11500 10500
#define l90  (21500)	//90度 左回転
#define r90  (21500) 	//90度 右回転
#define r180  (-43000)	//180度 右回転
#define sl90  (24500)	//スラローム 23000 25000
#define sr90  (24500)	//スラローム 23000 25000//左右で同じ値でないと連続時にずれていく

#define sl90BIG  (26000)	//スラローム 大曲
#define sr90BIG  (26000)	//スラローム 大曲

#define ul180  (47500)	//Uターン
#define ur180  (47500)	//Uターン //左右で同じ値でないと連続時にずれていく


//エンコーダ関連　memo : 1mm = 5.56 //壁切れ170 //2ndの1.34倍くらい
#define s1 (1015)		//1マス直進      タイヤ削れた時：1025
#define s45 (785)		//45度１マス直進 タイヤ削れた時：800
#define h1 (525)		//半マス直進 	タイヤ削れた時：525　最短用
#define h1_2 (520)		//半マス直進 	タイヤ削れた時：525  探索の壁切れ用

#define s45_V2 (100)	//140	//２マスVターンの距離　壁切れができない可能性が高いので実質距離で調整する
#define s45_V2_out (300)	//２マスVターンの距離　壁切れができない可能性が高いので実質距離で調整する
#define s45_V2_out_LR (120)	//２マスVターンの距離　壁切れができない可能性が高いので実質距離で調整する　壁切れ後にカーブ
#define s45_V2not (300)		//２マス斜め（Vターンではない）の距離　壁切れができない可能性が高いので実質距離で調整する
#define s45_V2notFirst (350)	//初手斜め用　２マス斜め（Vターンではない）の距離　壁切れができない可能性が高いので実質距離で調整する

#define rslsl90 	(980)	//スラローム 664
#define rslsl90_offset  (0)
#define rslsr90 	(980)	//スラローム 664
#define rslsr90_offset  (-0)

#define rslsl90_BIG (1610)	//スラローム 大 タイヤ削れた時：1600  1700
#define rslsl90_BIG_offset  (-100)	      //タイヤ削れた時：-100	
#define rslsr90_BIG (1610)	//スラローム 大 タイヤ削れた時：1600  1700
#define rslsr90_BIG_offset  (-100)	      //タイヤ削れた時：-100

#define usll180     (2250)	//Uターン
#define usll180_fin (0)	//Uターン 微調整
#define uslr180     (2250)	//Uターン
#define uslr180_fin (0)	//Uターン　微調整

#define r_cost 4		//迷路の重み　90度回転の場合

#define r45_cost 1		//迷路の重み　45度回転の場合

//ダイクストラ用
#define cost_centor_wall  10 //直線用の重み
#define cost_wall_wall    12 //斜め用の重み
    

//前壁補正のパラメータ
#define F_max 245
#define F_min 235
#define F_pow 10
#define F_cnt 4000
#define F_max_time 500

#define MAKE_KABE_tikai (135 + 60)
#define MAKE_KABE_tooi (135 - 60)


#endif