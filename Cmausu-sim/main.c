#include <stdio.h>
#include <windows.h>

#include "Parameters.h"
#include "Queue.h"
#include "dijkstra.h"

//------------------------------------------------------------------------------------------------------------------------------------
#define max(a,b) (((a) > (b)) ? (a) : (b))
#define min(a,b) (((a) < (b)) ? (a) : (b))
#define true 1
#define false 0

char Get_Goal_x(void);
char Get_Goal_y(void);
short get_r_cost(void);
short get_r45_cost(void);
char shortest_path_search(short target_x,short target_y);
void run_shortest_path(void);
void make_shortest_path_list(short target_x,short target_y);
void maze_search_adachi(short,short);
void maze_update(char,char,char,char);
void maze_update2(char ,char );
void shortest_path_search_perfect_unknown(short* ,short* );
void shortest_path_search_dijkstra_unknown(short* target_x,short* target_y);
void search_pickup(int*,int*);
void maze_search_all(void);
int get_node_num(int x,int y,int a);
void remake_shortest_path_list_naname2(void);
void path_compression(void);

//空関数
void GyroSum_reset(void){
    return;
}
void mae_kabe(void){
    return;
}
void  led_down(void){
    return;
}
void  led_up(void ){
    return;
}
void delay(int _time){
    return;
}
void Tmotor(long long A){
    return ;
}
void motor(int _L,int _R){
    return;
}
void led(int _r){
    return;
}

//グローバル変数
short dx[4] = {0,1,0,-1},dy[4] = {-1,0,1,0};
char my_x = Start_x,my_y = Start_y,my_angle = Start_angle;//0:up 1:right 2:down 3:left

char maze_w[H][W] = {0};	//上位4bit = 壁の確定bit 下位4bit = 壁の情報（未確定含む）
short maze_d[H][W][4] = {0};	//4方向分の重み
char maze_w_backup[H][W] = {0};	//上位4bit = 壁の確定bit 下位4bit = 壁の情報（未確定含む）
short maze_d_backup[H][W][4] = {0};	//4方向分の重み

short maze_d_perfect[H][W] = {0};
short maze_d_dijkstra[H][W] = {0};

short status_log = 99;
long long time_limit = 9999;
int pickup_x = 1;
int pickup_y = 1;

//------------------------------------------------------------------------------------------------------------------------------------

void maze_input(void);
void print_maze(void);

char maze_w_input[H][W] = {0};	//上位4bit = 壁の確定bit 下位4bit = 壁の情報（未確定含む）
int loop_cnt = 0;

#define PRINT_WAIT 50

int main(){

    maze_input();

    //maze_search_adachi(Get_Goal_x(),Get_Goal_y());
    //maze_search_adachi(Start_x,Start_y);

    maze_search_adachi(Get_Goal_x(),Get_Goal_y());
    maze_search_all();

    print_maze();
    return 0;
}


void maze_input(){
    FILE *fp;
    fp = fopen("maze_input.txt", "r");

    if (fp == NULL) {
        printf("ファイルを開けませんでした。\n");
        return;
    }

    for (int i = 0; i < H; i++) {
        for (int j = 0; j < W; j++) {
            if (fscanf(fp, "%d", &maze_w_input[i][j]) != 1) {
                printf("データの読み込みに失敗しました。\n");
                fclose(fp);
                return;
            }
        }
    }

    fclose(fp);
}



void print_maze(){
    printf("\033[H");
    printf("loop_cnt=%d                                                                               \n",loop_cnt);
    printf("X=%2d Y=%2d A=%d                                                                               \n",my_x,my_y,my_angle);
    for(int i = 0; i < H;i++){
		for(int j = 0; j < W; j++){
			printf("+");
			if(maze_w[i][j]&0x01)printf("----");
			else if(maze_w[i][j]&0x10)printf("    ");
			else printf("....");
		}
		printf("+\n");
		
		for(int j = 0; j < W; j++){
			if(maze_w[i][j]&0x08)printf("|");
			else if(maze_w[i][j]&0x80)printf(" ");
			else printf(":");
			
            
            if((i == my_y) && (j == my_x)){
                if(my_angle == 0){
                    printf("  ↑ ");
                }else if(my_angle == 1){
                    printf("  → ");
                }else if(my_angle == 2){
                    printf("  ↓ ");
                }else if(my_angle == 3){
                    printf("  ← ");
                }
                
            }else if(min(maze_d[i][j][3],min(maze_d[i][j][2],min(maze_d[i][j][1],maze_d[i][j][0] ))) == maze_d_max){
				printf("    ");
			}else{
                printf("    ");
				//printf("%4d",min(maze_d[i][j][3],min(maze_d[i][j][2],min(maze_d[i][j][1],maze_d[i][j][0] )))  );	
			}
			
		}
		printf("|\n");
    }
    printf("+----+----+----+----+----+----+----+----+----+----+----+----+----+----+----+----+\n");
    Sleep(PRINT_WAIT);
    loop_cnt++;
}
/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */
/* 関 数 概 要：迷路情報の更新											  			            */
/* 関 数 詳 細：												                                   */
/* 引       数： 現在地のXY座標、方角 ,更新する向き（1:前 2:横 それ以外:両方)					    */
/* 戻  り   値： なし										    									*/
/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */ 
void maze_update(char x,char y,char angle, char type){
	
    if((maze_w[y][x]&0xf0) == 0xf0)return;

    for(short i = -1; i < 3;i++){
        if(type == 1){
            if(i != 0)continue;//前以外は更新しない
        }else if(type == 2){
            if(i == 0)continue;//横以外は更新しない
        }
        
        short ii = (4 + angle+i)%4;
        int nx = x+dx[ii], ny = y+dy[ii];

        
        if((maze_w_input[y][x] & (1 << (ii))) != 0){
            maze_w[y][x] |= 1 << ii;
                    
        }else{
            maze_w[y][x] &= ~(1 << ii); 
        }  
        maze_w[y][x] |= 1 << (4+ii);
         
     
        if((0 <= nx && nx < W) && (0 <= ny && ny < H)){
            maze_w[ny][nx] |= 1 << (4+(ii+2)%4);
            maze_w[ny][nx] &= ~(1 << ((ii+2)%4));
            
            if((maze_w[y][x] & (1 << ii)) != 0){	
                maze_w[ny][nx] |= 1 << ((ii+2)%4);
            }
        }
    }
    
    if(x < Get_Goal_x()-1 || Get_Goal_x()+1 < x || y < Get_Goal_y()-1 || Get_Goal_y()+1 < y ){//ゴール座標周辺では実施しない
	    maze_update2(x,y);
	    maze_update2(x-1,y);
	    maze_update2(x-1,y-1);
	    maze_update2(x,y-1);
	    maze_update2(x,y);
	    maze_update2(x-1,y);
	    maze_update2(x-1,y-1);
	    maze_update2(x,y-1);
    }
}

void maze_update2(char x,char y){

	if(x == 7 && y == 7)return;
	
	char kakutei_cnt = 0;
	char kakutei_flag = 0;
	char mi_kakutei = 0;
	char kakutei_kabe_cnt = 0;
		
	if(x == 15)x--;
	if(y == 15)y--;
	
	if(0 <= x && x < 15 && 0 <= y && y < 15){//外枠ではない
		
		if((maze_w[y][x] & (1 << (4+1))) != 0 ){
			kakutei_flag |= 1;
			kakutei_cnt++;
			
			if((maze_w[y][x] & (1 << (1))) != 0){
				kakutei_kabe_cnt++;
			}
		}else{
			mi_kakutei = 0;
		}
		
		if((maze_w[y][x+1] & (1 << (4+2))) != 0 ){
			kakutei_flag |= 2;
			kakutei_cnt++;
			
			if((maze_w[y][x+1] & (1 << (2))) != 0){
				kakutei_kabe_cnt++;
			}
		}else{
			mi_kakutei = 1;
		}
		
		if((maze_w[y+1][x] & (1 << (4+1))) != 0 ){
			kakutei_flag |= 4;
			kakutei_cnt++;
			
			if((maze_w[y+1][x] & (1 << (1))) != 0){
				kakutei_kabe_cnt++;
			}
		}else{
			mi_kakutei = 2;
		}
		
		if((maze_w[y][x] & (1 << (4+2))) != 0 ){
			kakutei_flag |= 8;
			kakutei_cnt++;
			
			if((maze_w[y][x] & (1 << (2))) != 0){
				kakutei_kabe_cnt++;
			}
		}else{
			mi_kakutei = 3;
		}
		
		if(kakutei_cnt == 3){//残りの1壁も確定にできる
		
			if(kakutei_kabe_cnt == 0){//３枚　壁がない　＝残りは壁がある
			
				if(mi_kakutei == 0){
					maze_w[y][x] |= 1 << (4+1);
					maze_w[y][x] |= (1 << (1));
					
					maze_w[y][x+1] |= 1 << (4+3);
					maze_w[y][x+1] |= (1 << (3));
					
				}else if(mi_kakutei == 1){
					maze_w[y][x+1] |= 1 << (4+2);
					maze_w[y][x+1] |= (1 << (2));
					
					maze_w[y+1][x+1] |= 1 << (4+0);
					maze_w[y+1][x+1] |= (1 << (0));
					
				}else if(mi_kakutei == 2){
					maze_w[y+1][x] |= 1 << (4+1);
					maze_w[y+1][x] |= (1 << (1));
					
					maze_w[y+1][x+1] |= 1 << (4+3);
					maze_w[y+1][x+1] |= (1 << (3));
					
				}else if(mi_kakutei == 3){
					maze_w[y][x] |= 1 << (4+2);
					maze_w[y][x] |= (1 << (2));
					
					maze_w[y+1][x] |= 1 << (4+0);
					maze_w[y+1][x] |= (1 << (0));
				}
			}
		}
		
		
	}else{//外枠なので　柱につながる壁は３つ
		
	}
	
}

/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */
/* 関 数 概 要：最短経路を走行									  			            */
/* 関 数 詳 細：												                                   */
/* 引       数： なし														    */
/* 戻  り   値： なし										    									*/
/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */ 
void run_shortest_path(){
    GyroSum_reset();
    //Encoder_reset();
	
    short comand ,path_num,path_num_tmp;
    
    

    while(!queue_empty()){
        print_maze();

        comand = dequeue();path_num = dequeue();
        status_log = comand;
	
	    switch(comand){
        case -1://L
            maze_update(my_x,my_y,my_angle,3);

            my_angle = (4+my_angle-1)%4;

            maze_update(my_x,my_y,my_angle,3);

            break;
        case 0://S
            switch(my_angle){
                case 0:
                my_y -= path_num;
                break;
                case 1:
                my_x += path_num;
                break;
                case 2:
                my_y += path_num;
                break;
                case 3:
                my_x -= path_num;
                break;
            }

	        break;
		
        case 10://S 未確定の直線
 
            for(int i = 0; i < path_num; i++){
                if((maze_w_input[my_y][my_x] & (1 << (my_angle))) != 0){//目の前に壁がある
                    break;
                }else{
                    switch(my_angle){
                        case 0:
                        my_y -= 1;
                        break;
                        case 1:
                        my_x += 1;
                        break;
                        case 2:
                        my_y += 1;
                        break;
                        case 3:
                        my_x -= 1;
                        break;
                    }
                    maze_update(my_x,my_y,my_angle,3);
                    print_maze();
                }
            }
            break;

        case 1://R
            maze_update(my_x,my_y,my_angle,3);

            my_angle = (4+my_angle+1)%4;

            maze_update(my_x,my_y,my_angle,3);
            break;

        case 2://B
            maze_update(my_x,my_y,my_angle,3);

            my_angle = (4+my_angle+2)%4;

            maze_update(my_x,my_y,my_angle,3);
            
            break;
	    }
    }
    status_log = 99;
}

/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */
/* 関 数 概 要：ゴール座標を返却する		   		 			            */
/* 関 数 詳 細：										    */
/* 引       数： なし										    */
/* 戻  り   値： ゴール座標									    */
/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */ 
char Get_Goal_x(void){
	return Goal_x;
}

/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */
/* 関 数 概 要：ゴール座標を返却する		   		 			            */
/* 関 数 詳 細：										    */
/* 引       数： なし										    */
/* 戻  り   値： ゴール座標									    */
/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */ 
char Get_Goal_y(void){
	return Goal_y;
}

/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */
/* 関 数 概 要：90度回転の重みを取得										  			            */
/* 関 数 詳 細：												                                   */
/* 引       数： 													    */
/* 戻  り   値：なし										    									*/
/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */ 
short get_r_cost(void){
    return r_cost;
}

/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */
/* 関 数 概 要：45度回転の重みを取得										  			            */
/* 関 数 詳 細：												                                   */
/* 引       数： 													    */
/* 戻  り   値：なし										    									*/
/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */ 
short get_r45_cost(void){
    return r45_cost ;
}



//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//以下、実機と同じコード
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */
/* 関 数 概 要：					*/
/* 関 数 詳 細：												                                   */
/* 引       数： なし														    */
/* 戻  り   値： なし										    									*/
/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */ 
int get_node_num(int x,int y,int a){
	//aの変換 (上、右、下、左、中央) →(中央、上、左、右、下）
	switch(a){
		case 0:
			a = 1;
			break;
		case 1:
			a = 3;
			break;
		case 2:
			a = 4;
			break;
		case 3:
			a = 2;
			break;
		case 4:
			a = 0;
			break;
	}
/*
  ノード番号の考え方
  
  １マスにつき3つのノードを作成する　中央、上、左
  
   入力
   	x:0~15
	y:0~15
	a:0~4 (中央、上、左、右、下）
	
		右の場合
			x+1 の 左　に置き換える
		下の場合
			y+1 の 上に置き換える
	
	ノード番号= ((y * 16) + x ) + (a * 256)
   
 */
 	
	
 	if(a == 3){//右の場合
		if(x < 15){ //一番したのマスの場合でなければ
			//x+1 の 左　に置き換える
			x++;
			a = 2;
		}else{
			return (3 * 256)+ y ;
		}
	}else if(a == 4){//下の場合
		if(y < 15){ //一番したのマスの場合でなければ
			//y+1 の 上に置き換える
			y++;
			a = 1;
		}else{
			return (3 * 256)+ 16 + x ;
		}
	}
	
	return ((y * 16) + x ) + (a * 256);
}

/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */
/* 関 数 概 要：最短経路作成											  			            */
/* 関 数 詳 細：												                                   */
/* 引       数： 目的地のXY座標															    */
/* 戻  り   値： なし										    									*/
/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */ 
void make_shortest_path_list_pickup(short target_x,short target_y){
    queue_reset();
    short s_path = 0;
    short x = my_x,y = my_y,angle = my_angle;
    short unknown_flag = 1; // 0:確定壁　1:未確定壁
	
    int mas_cnt = 0;//直線に進むマスの数
    int last = 0;
	
    while(x != target_x || y != target_y){
    	short num = maze_d[y][x][(angle+2)%4];
    	short n_num = angle;
    	char s_flag = 0;
    	short nx = x+dx[angle],ny = y+dy[angle];
    	if((0 <= nx && nx < W) && (0 <= ny && ny < H) && ((maze_w[y][x] & (1<<angle)) == 0 ) && ((maze_w[y][x] & (1<<(angle+4))) != 0 ) ){//目の前が迷路内　&& 壁がない && 確定マス
            short next = maze_d[ny][nx][(angle+2)%4];
            if(num == next+1){//目の前のマスがゴールに近い
                n_num = (angle+2)%4;
                num = next;
                s_flag = true;
            }
    	}

    	if(s_flag == false){//回転する必要がる
            for(int i = 0;i < 4;i++){
                if(i == (angle+2)%4){//B
                    //初期値なので更新不要
                }else{
                    short next = maze_d[y][x][i];
                    if(num > next){
                        n_num = i;
                        num = next;
                                
                    }else if(num == next){// LとRが同じ重み　斜めを優先したい
                        if(last == -1){//前回がLなら今回はR
                            n_num = (angle-1+4)%4;
                            
                        }else if(last == 1){//前回がRなら今回はL
                            n_num = (angle+1+4)%4;
                            
                        }else{//前回がSなら今回は?
                            //わからんから先に見つかった方にする
                        }
                    }
                }
            }
    	}
    
    	n_num = (n_num+2)%4;// 0 ~ 4　進みたい方角
    	short ni = ((4 + n_num - ((4+angle-1)%4))%4) -1;// -1 ~ 2　マシンから見た方角

    	if((maze_w[y][x] & (1 << (4+n_num))) == 0){//未確定の壁
            //break;
                
            if( unknown_flag == 1){//初めから未確定の直線の場合は有効　一度でも確定のマスを進むと無効
                if(ni != 0)break;//直進方向でなければ打ち切り//////////////////////////////////////////////////////////////////////////////////////////////////////////////
                ni = 10;//未確定の直線
            }else{
                break;//未確定の壁を確認する必要があるので打ち切り
            }
                
	    }else{//確定
	        if(ni == 0){//直線
		        if(mas_cnt == 0){//はじめての直進の１マス目
		            ni = 10;//直進方向なら未確定の直線の可能性あり メモ：目の前の壁は確定してるから
		        }else{
					
		        }
	        }
	    }
	
    	switch(ni){
        case -1://L
            if(s_path > 0){
                if(unknown_flag == 1 && s_path == 1){//1マスだけなら確定の直線
                    enqueue(0);
                    enqueue(s_path);
                    s_path = 0;
                    unknown_flag = 0;
                }else{
                    enqueue(10 * unknown_flag );
                    //enqueue(0);
                    enqueue(s_path);
                    s_path = 0;
                                
                    if(unknown_flag == 1)return;//未確定の直線のあとはルートを作成してはいけない
                }
            }

            enqueue(-1);
            enqueue(1);

            //s_path += 1;
            
            angle = (4+angle-1)%4;
            // x += dx[n_num];
            // y += dy[n_num];
                    
            //unknown_flag = 0;
            last = -1;
            break;
        case 0://S
    
            if(unknown_flag == 1 && s_path > 0 && mas_cnt > 1){
                enqueue(10);
                enqueue(s_path);
                s_path = 0;
                            
                return;//未確定の直線のあとはルートを作成してはいけない
            }
                    
            s_path +=1;
            x += dx[n_num];
            y += dy[n_num];
                    
            unknown_flag = 0;
            mas_cnt++;
                    
            //last = 0; 
            break;
                    
        case 10://S 未確定の直線
                    
            s_path +=1;
            x += dx[n_num];
            y += dy[n_num];
                    
            mas_cnt++;
            //last = 0;
            break;
                    
        case 1://R
            if(s_path > 0){
                if(unknown_flag == 1 && s_path == 1){//1マスだけなら確定の直線
                    enqueue(0);
                    enqueue(s_path);
                    s_path = 0;
                    unknown_flag = 0;
                }else{
                    enqueue(10 * unknown_flag);
                    //enqueue(0);
                    enqueue(s_path);
                    s_path = 0;
                                
                    if(unknown_flag == 1)return;//未確定の直線のあとはルートを作成してはいけない
                }
            }
            
            enqueue(1);
            enqueue(1);

            // s_path += 1;
                    
            angle = (4+angle+1)%4;
            // x += dx[n_num];
            // y += dy[n_num];
                    
            //unknown_flag = 0;
            last = 1;
            break;
        case 2://B
            if(s_path > 0){
                if(unknown_flag == 1 && s_path == 1){//1マスだけなら確定の直線
                    enqueue(0);
                    enqueue(s_path);
                    s_path = 0;
                    unknown_flag = 0;
                }else{
                    enqueue(10 * unknown_flag);
                    //enqueue(0);
                    enqueue(s_path);
                    s_path = 0;
                                
                    if(unknown_flag == 1)return;//未確定の直線のあとはルートを作成してはいけない
                }
            }
            
            enqueue(2);
            enqueue(1);

            //s_path += 1;
                    
            angle = (4+angle+2)%4;
            //x += dx[n_num];
            //y += dy[n_num];
                    
            unknown_flag = 0;
            //last = 2;
            break;
        }
    }
 
    if(s_path > 0){
    	enqueue(10 * unknown_flag);
	//enqueue(0);
    	enqueue(s_path);
    	s_path = 0;
    }
}

/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */
/* 関 数 概 要：最短経路探索											  			            */
/* 関 数 詳 細：												                                   */
/* 引       数： 目的地のXY座標															    */
/* 戻  り   値： なし										    									*/
/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */ 
char shortest_path_search_kichikukan(short target_x,short target_y){
    char target_flag = 0;
    
    queue_reset();
    for(int i = 0; i < H;i++){
        for(int j = 0;j < W; j++){
            for(int k = 0; k < 4; k++){
            maze_d[i][j][k] = maze_d_max;
            }
        }
    }
    for(int k = 0; k < 4; k++){
	    if((maze_w[target_y][target_x] & (1<<k)) == 0 )maze_d[target_y][target_x][k] = 0;//ゴール向きが無関係なので０すべて０を設定
    }
    enqueue(target_x*100 + target_y);
  
    while(!queue_empty()){
        short x = dequeue(),y;
        y = x%100;
        x /=100;

        do{
            for(char i =0;i<4;i++){
                char update_flag = 0;
                short nx = x+dx[i],ny = y+dy[i];
            
                if((0 <= nx && nx < W) && (0 <= ny && ny < H) && ((maze_w[y][x] & (1<<i)) == 0 ) && ((maze_w[y][x] & (1<<(i+4))) != 0  || (target_flag == 2 && target_x == x && target_y == y )) ){//確定マスのみ ターゲットマスの時は未確定でもOK

                    short num = maze_d[y][x][i];
                    for(int k = 0; k < 4; k++){
                    
                        if(i == k){//S
                            if(maze_d[ny][nx][k] > num + 1){
                                update_flag = true;
                                maze_d[ny][nx][k] = num + 1;
                            }
                        }else if((i+2+4)%4 == k){//B
                            if(maze_d[ny][nx][k] > num+1 + get_r_cost()*2){
                                update_flag = true;
                                maze_d[ny][nx][k] = num+1 + get_r_cost()*2;
                            }
                        }else{// L or R
                            if(maze_d[ny][nx][k] > num+1 + get_r_cost()){
                                update_flag = true;
                                maze_d[ny][nx][k] = num+1 + get_r_cost();
                            }
                        }
                    }
                    if(update_flag){
                        enqueue(nx*100 + ny);
                        
                        if(target_x == x && target_y == y){//目的地の隣のマスに進むことができる
                            target_flag = 1;	
                        }
                    }
                }
            }
            
            if(target_flag == 0 && target_x == x && target_y == y){//既知区間だけでは目的地の隣のマスに進むことができない
                target_flag = 2;
            }
        }while(target_flag == 2 && target_x == x && target_y == y);//目的地の隣のみ未知区間の壁でも通過可能として再取得する
    }
    
    char ng_flag = 1;
    for(int k = 0; k < 4; k++){
    	if(maze_d[Start_y][Start_x][k] != maze_d_max ){//スタート位置の重みが更新されてなかったら＝最短経路が存在しない
    		ng_flag = 0;
    	}
    }
    if(ng_flag == 1){
	    return 1;    
    }
    
    return 0;
}

/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */
/* 関 数 概 要：最短経路探索											  			            */
/* 関 数 詳 細：												                                   */
/* 引       数： 目的地のXY座標															    */
/* 戻  り   値： なし										    									*/
/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */ 
char shortest_path_search_pickup(short target_x,short target_y){
    queue_reset();
    for(int i = 0; i < H;i++){
        for(int j = 0;j < W; j++){
            for(int k = 0; k < 4; k++){
            maze_d[i][j][k] = maze_d_max;
            }
        }
    }
    for(int k = 0; k < 4; k++){
	    if((maze_w[target_y][target_x] & (1<<k)) == 0 )maze_d[target_y][target_x][k] = 0;//ゴール向きが無関係なので０すべて０を設定
    }
    enqueue(target_x*100 + target_y);
  
    while(!queue_empty()){
        short x = dequeue(),y;
        y = x%100;
        x /=100;

	    for(char i =0;i<4;i++){
            char update_flag = 0;
            short nx = x+dx[i],ny = y+dy[i];
            if((0 <= nx && nx < W) && (0 <= ny && ny < H) && ((maze_w[y][x] & (1<<i)) == 0 ) && ((maze_w[y][x] & (1<<(i+4))) != 0 ) ){//確定マスのみ
                short num = maze_d[y][x][i];
                for(int k = 0; k < 4; k++){
                
                    if(i == k){//S
                        if(maze_d[ny][nx][k] > num + 1){
                            update_flag = true;
                            maze_d[ny][nx][k] = num + 1;
                        }
                    }else if((i+2+4)%4 == k){//B
                        if(maze_d[ny][nx][k] > num+1 + get_r_cost()*2){
                            update_flag = true;
                            maze_d[ny][nx][k] = num+1 + get_r_cost()*2;
                        }
                    }else{// L or R
                        if(maze_d[ny][nx][k] > num+1 + get_r_cost()){
                            update_flag = true;
                            maze_d[ny][nx][k] = num+1 + get_r_cost();
                        }
                    }
                }
                if(update_flag)enqueue(nx*100 + ny);
            }
        }
    }
    
    char ng_flag = 1;
    for(int k = 0; k < 4; k++){
    	if(maze_d[Start_y][Start_x][k] != maze_d_max ){//スタート位置の重みが更新されてなかったら＝最短経路が存在しない
    		ng_flag = 0;
    	}
    }
    if(ng_flag == 1){
	    return 1;    
    }
    
    return 0;
}

/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */
/* 関 数 概 要：ピックアップ位置まで走行 探索走行と同等の設定											  			            */
/* 関 数 詳 細：												                                   */
/* 引       数： 目的地のXY座標															    */
/* 戻  り   値： なし										    									*/
/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */ 
void run_pickup(short target_x,short target_y){
   

    GyroSum_reset();
   // Encoder_reset();

    led_down();
		
    while(1){
    	shortest_path_search_pickup(target_x,target_y);
    
    	make_shortest_path_list_pickup(target_x,target_y);
    
    	run_shortest_path();

	    motor(0,0);
	
    	if(target_x == my_x && target_y == my_y){//ゴール
            motor(0,0);
            led_up();
			
            if((target_x == Start_x && target_y == Start_y) || (target_x == Get_Goal_x() && target_y == Get_Goal_y())){
                    
                GyroSum_reset();
                
                mae_kabe();//前壁距離補正
                
                Tmotor(r180);
                motor(0,0);
                delay(100);
                
                my_angle = (4+my_angle+2)%4;
            }
	        break;
	    }
    }
}
/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */
/* 関 数 概 要：現在位置から近い取り上げやすい位置を探す   		 			            */
/* 関 数 詳 細：												                                   */
/* 引       数： 取り上げやすいXY座標、								    */
/* 戻  り   値： なし										    									*/
/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */ 
void search_pickup(int* pickuup_x,int* pickuup_y){
	
    int x = 0,y = 0;
    int cost = maze_d_max;
	
    //shortest_path_search(Get_Goal_x(),Get_Goal_y());
   // shortest_path_search_fin();//ゴールからの距離を算出
    shortest_path_search_kichikukan(my_x,my_y);//現在位置からの距離を算出
	
    for(int i = 0; i < H;i++){
        for(int j = 0;j < W; j++){
                    
            if( !( (Not_Pickup_y_min <= i  && i <=  Not_Pickup_y_max) && (Not_Pickup_x_min <= j &&  j <= Not_Pickup_x_max) )){//拾いにくいところ　ではない
                    
                //if(j != Get_Goal_x() || i != Get_Goal_y()){//ゴール座標は対象外
                if(j != my_x || i != my_y){//現在位置は対象外
                    for(int k = 0; k < 4; k++){
                                
                        if(maze_d[i][j][k] < cost){
                            cost = 	maze_d[i][j][k];
                            x = j;
                            y = i;
                        }
                    }
                }
            }
        }
    }
	
    *pickuup_x = x;
    *pickuup_y = y;
}


/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */
/* 関 数 概 要：					*/
/* 関 数 詳 細：												                                   */
/* 引       数： なし														    */
/* 戻  り   値： なし										    									*/
/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */ 
void shortest_path_search_dijkstra_unknown(short* target_x,short* target_y){

    char my_x_tmp = my_x,my_y_tmp = my_y,my_angle_tmp = my_angle;//現在位置のバックアップ
    
    led(0);
 
    int node_num_centor;
    
    init_dijkstra();
    
    //壁情報からパス情報を生成
    		//add_edge(int, int, int );
    for(int i = 0; i < H; i++){
	    for(int j = 0; j < W;j++){
		    
		    node_num_centor = get_node_num(j,i,4);
		    
		    for(int k = 0; k < 4;k++){
			    
			   //マスの中央と壁
			   if((maze_w[i][j] & (1 << k)) == 0) {//壁が無ければマスの中央とつながってる  未確定の壁は壁無しとする
				   add_edge(get_node_num(j,i,k), node_num_centor, cost_centor_wall );
				   
				   //printf2("x=%d y=%d a=%d\n",j,i,k);
				   //斜めの壁と壁
				   if((maze_w[i][j] & (1 << ((k+1)%4))) == 0){ //未確定の壁は壁無しとする
					   add_edge(get_node_num(j,i,k), get_node_num(j,i,((k+1)%4))  ,cost_wall_wall );
				   }
				   if(((maze_w[i][j] & (1 << ((k+4-1)%4)))) == 0){ //未確定の壁は壁無しとする
					   add_edge(get_node_num(j,i,k), get_node_num(j,i,((k+4-1)%4))  , cost_wall_wall );
				   }
			   }   
		    }
	    }
    }
    
    //ゴール地点からダイクストラを実行
    run_dijkstra(get_node_num(Get_Goal_x(),Get_Goal_y(),4)); //ゴールのマス中央を設定する

    //距離情報から歩数マップを生成
    		//long get_dist(int);
     for(int i = 0; i < H; i++){
	    for(int j = 0; j < W;j++){
		    maze_d_dijkstra[i][j] = get_dist(get_node_num(j,i,4));//ゴールからマスの中央の距離を設定する
		    
	    }
     }
    
    //歩数マップから走行リストを生成
    //run_list
    queue_reset();
    short h_path = 0;
    char maze_flag[H][W] = {0};
     
    my_x = Start_x;my_y = Start_y;my_angle = Start_angle;
 
    int last = 0;
	
    while(my_x != Get_Goal_x() || my_y != Get_Goal_y()){

        short num = maze_d_max -100; //maze_d_dijkstra[my_y][my_x];  周囲のマスが現在地より小さいとは言えないため最大値-100に変更
        short n_num = 0;
        char first_flag = 0;
        
        maze_flag[my_y][my_x] = 1;//一度到達したマスには戻らないようにする
	
        for(int i = 0;i < 4;i++){//ゴールに近いマスを探す
            int nx = my_x+dx[i],ny = my_y+dy[i];
            
            //迷路の範囲内　＆＆　壁が無いことが確定している
            if((0 <= nx && nx < W) && (0 <= ny && ny < H) && ((maze_w[my_y][my_x] & (1<<i)) == 0 )){//  && ((maze_w[my_y][my_x] & (1<<(4+i))) != 0 ) ){
                
                if(maze_flag[ny][nx] != 1){//まだ到達してなければ
                    if(first_flag == 0 || num > maze_d_dijkstra[ny][nx]){//初めのマスは無条件で移動す候補にする || ゴールに近いマスを見つけた
                        num = maze_d_dijkstra[ny][nx];
                        n_num = i;
                        
                        first_flag = 1;
                        
                    }else if(num == maze_d_dijkstra[ny][nx]){// LとRが同じ重み　斜めを優先したい
                    
                        /*if( maze_d[ny][nx][i] < maze_d[my_y+dy[n_num]][my_x+dx[n_num]][n_num] ){//斜めを考慮しない重みの小さいほうを優先する
                            n_num = i;
                            
                        }else if( maze_d[ny][nx][i] == maze_d[my_y+dy[n_num]][my_x+dx[n_num]][n_num]) {
                        */	if(last == -1 && (i - my_angle + 4)%4 == 1   ){//前回がL かつ　今回はR  
                                n_num = i;
                                    
                            }else if(last == 1 && (i - my_angle + 4)%4 == -1   ){//前回がR　かつ　今回はL
                                n_num = i;
                                    
                            }else{//前回がSなら今回は?
                                if( maze_d[ny][nx][i] < maze_d[my_y+dy[n_num]][my_x+dx[n_num]][n_num] ){//斜めを考慮しない重みの小さいほうを優先する
                                    n_num = i;
                                }
                            }
                        //}

                        first_flag = 1;
                    }
                }
            }
        }
        
        if((maze_w[my_y][my_x] & (1<<(4+n_num))) == 0){//移動する向きが未確定の壁だった場合
            *target_x = my_x + dx[n_num];
            *target_y = my_y + dy[n_num];
            
            //現在位置をバックアップから復元
            my_x = my_x_tmp;
            my_y = my_y_tmp;
            my_angle = my_angle_tmp;
            return;
        }
        
        //移動する
        short ni = ((4 + n_num - ((4+my_angle-1)%4))%4) -1;// -1 ~ 2

        switch(ni){
        case -1://L
            if(h_path > 0){
                if(queue_empty())h_path--;
                enqueue(0);
                enqueue(h_path);
                h_path = 0;
            }

            enqueue(-1);
            enqueue(1);
            my_angle = (4+my_angle-1)%4;
            
            my_x += dx[n_num];
            my_y += dy[n_num];
            
            last = -1;
            break;
        case 0://S
    
            h_path +=2;
            my_x += dx[n_num];
            my_y += dy[n_num];
            
            //last = 0;
            break;
        case 1://R
            if(h_path > 0){
                if(queue_empty())h_path--;
                enqueue(0);
                enqueue(h_path);
                h_path = 0;
            }
            
            enqueue(1);
            enqueue(1);
        
            my_angle = (4+my_angle+1)%4;

            my_x += dx[n_num];
            my_y += dy[n_num];
            
            last = 1;
            break;
        
        case 2://B
            if(h_path > 0){
                if(queue_empty())h_path--;
                enqueue(0);
                enqueue(h_path);
                h_path = 0;
            }
            
            enqueue(2);
            enqueue(1);
            my_angle = (4+my_angle+2)%4;
            break;
        }
    }
 
    if(h_path > 0){
        if(queue_empty())h_path--;
        enqueue(0);
        enqueue(h_path+1);
        h_path = 0;
    }else{
        enqueue(0);
        enqueue(1);
    }
    
   //未確定マスを通らずにゴールまで経路を確認できた。
    *target_x = Get_Goal_x();
    *target_y = Get_Goal_y(); 
    
    
    
    //現在位置をバックアップから復元
    my_x = my_x_tmp;
    my_y = my_y_tmp;
    my_angle = my_angle_tmp;
  
}

/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */
/* 関 数 概 要：最短経路に斜め有効化										  			            */
/* 関 数 詳 細：												                                   */
/* 引       数： なし													    */
/* 戻  り   値： なし										    									*/
/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */ 
void remake_shortest_path_list_naname2(){
    enqueue(99);//目印
    enqueue(99);
    int naname_cnt = 0,lr = 0,first_lr = 0;
  
    while(1){
        short mode = dequeue(),num = dequeue();
        if(mode == 99)break;

        switch(mode){
        case -1://L
            if(naname_cnt == 0){
                naname_cnt = 1;
                lr = -1;
                first_lr = -1;
            }else if(naname_cnt == 1 && lr == -1 && queue_next(1) == 0){//Uターン
                enqueue(-1);
                enqueue(1);
                        
                enqueue(-1);
                enqueue(1);
                        
                naname_cnt = 0;
                lr = 0;
            }else if(naname_cnt == 1 && lr == -1 && queue_next(1) == 1){//L L R //１つ目はスラロームにする
                enqueue(-1);
                enqueue(1);
                                    
            }else{
                if(lr == 1){
                        lr = -1;
                        naname_cnt++;
                }else{//斜め中のVターン
                    if(first_lr == -1){
                        enqueue(-11);
                        enqueue(1);
                    }else if(first_lr == 1){
                        enqueue(11);
                        enqueue(1);
                    }
                    first_lr = 0;
                        
                    enqueue(10);
                    enqueue(naname_cnt);
                    
                    enqueue(-11);
                    enqueue(1);
                        
                    if(queue_next(1) == 0){//L L (S) //スラロームにする
                        naname_cnt = 0;
                        enqueue(-1);
                        enqueue(1);
                    }else{
                        naname_cnt = 1;
                        lr = -1;
                        first_lr = -1;
                    }
                }
            }
            break;
        case 0://S
            if(naname_cnt == 1){//斜めが1マスのときはスラロームにする
                if(lr == 1){
                    enqueue(1);
                    enqueue(1);
                }else{
                    enqueue(-1);
                    enqueue(1);
                }
                naname_cnt = 0;
                lr = 0;
                                    
            }else if(naname_cnt > 1){
                if(first_lr == 1){
                    enqueue(11);
                    enqueue(1);
                }else if(first_lr == -1){
                    enqueue(-11);
                    enqueue(1);
                }
                first_lr = 0;
                
                if(lr == 1){
                    enqueue(10);
                    enqueue(naname_cnt);
                        
                    enqueue(11);
                    enqueue(1);
                    naname_cnt = 0;
                    lr = 0;
                }else{
                    enqueue(10);
                    enqueue(naname_cnt);
                        
                    enqueue(-11);
                    enqueue(1);
                    naname_cnt = 0;
                    lr = 0;
                }
            }
            enqueue(mode);
            enqueue(num);
            break;
        case 1://R
            if(naname_cnt == 0){
                naname_cnt = 1;
                lr = 1;
                first_lr = 1;
            }else if(naname_cnt == 1 && lr == 1 && queue_next(1) == 0){//Uターン
                enqueue(1);
                enqueue(1);
                        
                enqueue(1);
                enqueue(1);
                        
                naname_cnt = 0;
                lr = 0;
            }else if(naname_cnt == 1 && lr == 1 && queue_next(1) == -1){//R R L //１つ目はスラロームにする
                enqueue(1);
                enqueue(1);
                        
                
            }else{
                if(lr == -1){
                        lr = 1;
                        naname_cnt++;
                }else{//斜め中のVターン
                    if(first_lr == 1){
                        enqueue(11);
                        enqueue(1);
                    }else if(first_lr == -1){
                        enqueue(-11);
                        enqueue(1);
                    }
                    first_lr = 0;
                        
                    enqueue(10);
                    enqueue(naname_cnt);
                        
                    enqueue(11);
                    enqueue(1);
                        
                    if(queue_next(1) == 0){//R R (S) //スラロームにする
                        naname_cnt = 0;
                        enqueue(1);
                        enqueue(1);
                    }else{
                        naname_cnt = 1;
                        lr = 1;
                        first_lr = 1;
                    }
                }
            }
            break;
        }
    }
}

/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */
/* 関 数 概 要：最短経路の圧縮											  			            */
/* 関 数 詳 細：												                                   */
/* 引       数： なし														    */
/* 戻  り   値： なし										    									*/
/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */ 
void path_compression(){
    enqueue(99);//目印
    enqueue(99);
    
    short next_num_add = 0;
    
    while(1){
        short mode = dequeue(),num = dequeue();
        if(mode == 99)break;

        num += next_num_add;
        next_num_add = 0;
        
        switch(mode){
        
            case 0://S
                if(queue_next(1) == -1 && queue_next(3) == 0){//L大曲
                    dequeue();
                    dequeue();
                    
                    enqueue(mode);
                    enqueue(num -1);
                    
                    enqueue(-12);
                    enqueue(1);
                    
                    next_num_add = -1;
                }else if(queue_next(1) == 1 && queue_next(3) == 0){//R大曲
                    dequeue();
                    dequeue();
                    
                    enqueue(mode);
                    enqueue(num -1);
                    
                    enqueue(12);
                    enqueue(1);
                    
                    next_num_add = -1;
                
                }else if(queue_next(1) == 11){//直線からのR45
                    dequeue();
                    dequeue();
                    
                    enqueue(mode);
                    enqueue(num -1);
                    
                    enqueue(14);
                    enqueue(1);
                    
                    //next_num_add = -1;
                    
                }else if(queue_next(1) == -11){//直線からのL45
                    dequeue();
                    dequeue();
                    
                    enqueue(mode);
                    enqueue(num -1);
                    
                    enqueue(-14);
                    enqueue(1);
                    
                    //next_num_add = -1;
                    
                }else{//変更なし
                    enqueue(mode);
                    enqueue(num);
                }
                break;
            
            case 11://R45
                if(queue_next(1) == 0 || queue_next(1) == 1 || queue_next(1) == 11){//斜め終わり
                    enqueue(13);
                    enqueue(num);	
                }else{
                    enqueue(mode);
                    enqueue(num);
                }
                break;
            
            case -11://R45
                if(queue_next(1) == 0 || queue_next(1) == -1 || queue_next(1) == -11){//斜め終わり
                    enqueue(-13);
                    enqueue(num);	
                }else{
                    enqueue(mode);
                    enqueue(num);
                }
                break;
                
            default:
                enqueue(mode);
                enqueue(num);
                break;
        }
	
    }
    
    //Uターン
    enqueue(99);//目印
    enqueue(99);
    
    next_num_add = 0;
    
    while(1){
        short mode = dequeue(),num = dequeue();
        if(mode == 99)break;

        num += next_num_add;
        next_num_add = 0;
        
        switch(mode){
            case 0://S
                if(queue_next(1) == -1 && queue_next(3) == -1 && queue_next(5) == 0){//L Uターン
                    dequeue();
                    dequeue();
                    
                    dequeue();
                    dequeue();
                    
                    enqueue(mode);
                    enqueue(num -1);
                    
                    enqueue(-15);
                    enqueue(1);
                    
                    //next_num_add = -1;
                    
                }else if(queue_next(1) == 1 && queue_next(3) == 1 && queue_next(5) == 0){//R Uターン
                    dequeue();
                    dequeue();
                    
                    dequeue();
                    dequeue();
                    
                    enqueue(mode);
                    enqueue(num -1);
                    
                    enqueue(15);
                    enqueue(1);
                    
                    //next_num_add = -1;
                    
                }else{//変更なし
                    enqueue(mode);
                    enqueue(num);
                }
                break;
            
            default://変更なし
                enqueue(mode);
                enqueue(num);
                break;
        }
	
    }
	
}
/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */
/* 関 数 概 要：斜めも考慮して最短経路上の未確定マスを探す	未確定マスも通過する					*/
/* 関 数 詳 細：												                                   */
/* 引       数： なし														    */
/* 戻  り   値： なし										    									*/
/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */ 
void shortest_path_search_perfect_unknown(short* target_x,short* target_y){

    int comand ,path_num;

    char my_x_tmp = my_x,my_y_tmp = my_y,my_angle_tmp = my_angle;//現在位置のバックアップ
    
    led(0);
    ////////////// ゴールからの距離を計算する
    shortest_path_search(Get_Goal_x(),Get_Goal_y());
   
    //////////////////////////////////////////////////
    for(int i = 0; i < H;i++){
        for(int j = 0;j < W; j++){
            maze_d_perfect[i][j] = maze_d_max;
        }
    }
    
   
    for(int i = 0; i < H;i++){//全マスからゴールまでの走行経路を算出する
        for(int j = 0;j < W; j++){
            // led(i);
            // printf2("%d %d ",i,j);
                
            if(i == Get_Goal_y() && j == Get_Goal_x()){
                maze_d_perfect[Get_Goal_y()][Get_Goal_x()] = 0; 
                
            }else if (maze_d[i][j][0] >= maze_d_max){//到達不能マス
                maze_d_perfect[i][j] = maze_d_max;
                
            }else{//走行経路を算出する
                
                //run_list		
                maze_d_perfect[i][j] = maze_d_max;
                
                for(int k = 0;k < 4;k++){//スタート向きの設定
                    //printf2("x=%d y=%d a=%d\n",j,i,k);
                    
                    queue_reset();
                    short h_path = 0;
                    my_x = j;my_y = i;my_angle = 0;//スタート位置の設定
                    
                    /*if(maze_d[i][j][my_angle] > maze_d[i][j][k]){
                        my_angle = k;
                    }*/
                    if((maze_w[my_y][my_x] & (1 << ((k+2)%4))) != 0){//目の前に壁がある場合スタートの向きにはならない
                        //printf2("NG \n");
                        continue;
                    }
                    my_angle = k;
                    
                    my_angle = (my_angle+2)%4;
                    //printf2("%d ",my_angle);
                    int last = 0;
            
                    while(my_x != Get_Goal_x() || my_y != Get_Goal_y()){

                        short num = maze_d[my_y][my_x][(my_angle+2)%4];
                        short n_num = my_angle;
                        char s_flag = 0;
                        int nx = my_x+dx[my_angle],ny = my_y+dy[my_angle];
                    
                    
                        if((0 <= nx && nx < W) && (0 <= ny && ny < H) && ((maze_w[my_y][my_x] & (1<<my_angle)) == 0 ) ){
                                short next = maze_d[ny][nx][(my_angle+2)%4];
                                if(num == next+1){
                                    n_num = (my_angle+2)%4;
                                    num = next;
                                    s_flag = true;
                                }
                        }

                        if(s_flag == false){
                            for(int ii = 0;ii < 4;ii++){
                                if(ii == (my_angle+2)%4){//逆走はありえない
                                }else{// L or R
                                    short next = maze_d[my_y][my_x][ii];
                                    if(num > next){
                                        n_num = ii;
                                        num = next;
                                    }else if(num == next){// LとRが同じ重み　斜めを優先したい
                                        if(last == -1){//前回がLなら今回はR
                                            n_num = (my_angle-1+4)%4;
                                
                                        }else if(last == 1){//前回がRなら今回はL
                                            n_num = (my_angle+1+4)%4;
                                
                                        }else{//前回がSなら今回は?
                                            //わからんから先に見つかった方にする
                                        }
                                    }
                                }
                            }
                        }
                // printf2("1-OK %d %d num=%d n_num=%d\n",my_x,my_y,num,n_num);
                        n_num = (n_num+2)%4;// 0 ~ 4
                        short ni = ((4 + n_num - ((4+my_angle-1)%4))%4) -1;// -1 ~ 2

                        switch(ni){
                            case -1://L
                                if(h_path > 0){
                                    if(queue_empty())h_path--;
                                    enqueue(0);
                                    enqueue(h_path);
                                    h_path = 0;
                                }

                                enqueue(-1);
                                enqueue(1);
                                my_angle = (4+my_angle-1)%4;
                            
                                my_x += dx[n_num];
                                my_y += dy[n_num];
                                        
                                last = -1;
                                break;
                            case 0://S
                
                                h_path +=2;
                                my_x += dx[n_num];
                                my_y += dy[n_num];
                                    
                                //last = 0;
                                break;
                            case 1://R
                                if(h_path > 0){
                                    if(queue_empty())h_path--;
                                    enqueue(0);
                                    enqueue(h_path);
                                    h_path = 0;
                                }
                                
                                enqueue(1);
                                enqueue(1);
                            
                                my_angle = (4+my_angle+1)%4;

                                my_x += dx[n_num];
                                my_y += dy[n_num];
                                
                                last = 1;
                                break;
                            
                            case 2://B  最短走行で逆走はありえない
                                
                                my_angle = (4+my_angle+2)%4;//メモ　無限ループ回避のため、仕方なく実装
                                break;
                        }
                    }
                    
                    if(h_path > 0){
                        if(queue_empty())h_path--;
                        enqueue(0);
                        enqueue(h_path+1);
                        h_path = 0;
                    }else{
                        enqueue(0);
                        enqueue(1);
                    }
                    
                    //printf2("2-OK ");
                    //ここまでで走行経路が算出完了 
                    remake_shortest_path_list_naname2(); //２マスも斜めにするモード
                    path_compression();//大曲など  
                    
                    //走行経路から距離に変換
                    short maze_d_perfect_tmp = 0;
                    
                    while(!queue_empty()){
                        comand = dequeue();path_num = dequeue();
                        //printf2("%d %d\n",comand,path_num);
                        //delay(1)
                        
                        if(comand == 0){//直線
                            maze_d_perfect_tmp += max(1,(path_num+1) / 2) * 2; //メモ　斜め1マス＝１としたときの直線の重み
                            
                        }else if(comand == 10){//斜め直線
                            maze_d_perfect_tmp += max(1,path_num);  //斜め１マス
                            
                        }else if(comand == -11 || comand == -13 || comand == -14 || comand == 11 || comand == 13 || comand == 14){//斜め45
                            maze_d_perfect_tmp += get_r45_cost();  //１マス
                            
                        }else{//カーブ
                            maze_d_perfect_tmp += path_num * get_r_cost();
                        }
                        
                    } 
                    
                    //printf2("\td=%d\n",maze_d_perfect_tmp);
                    
                    //距離情報の更新確認
                    maze_d_perfect[i][j] = min(maze_d_perfect_tmp, maze_d_perfect[i][j]);
                }
            }
            // printf2("OK \n");
        }
    }
    /*
     for(int i = 0; i < H;i++){
	for(int j = 0;j < W; j++){
		if(maze_d_perfect[i][j] == maze_d_max)printf2("xxx");
		else printf2("%3d",maze_d_perfect[i][j]);
		delay(1);
	}
	printf2("\n");
     }
     while(1);
     */
    ///////////////////////////////////////////////
    
    //run_list
    queue_reset();
    short h_path = 0;
    char maze_flag[H][W] = {0};
     
    my_x = Start_x;my_y = Start_y;my_angle = Start_angle;
 
    int last = 0;
	
    while(my_x != Get_Goal_x() || my_y != Get_Goal_y()){

        short num = maze_d_max -100; //maze_d_perfect[my_y][my_x];  周囲のマスが現在地より小さいとは言えないため最大値-100に変更
        short n_num = 0;
        char first_flag = 0;
        
        maze_flag[my_y][my_x] = 1;//一度到達したマスには戻らないようにする
        
        for(int i = 0;i < 4;i++){//ゴールに近いマスを探す
            int nx = my_x+dx[i],ny = my_y+dy[i];
            
            //迷路の範囲内　＆＆　壁が無いことが確定している
            if((0 <= nx && nx < W) && (0 <= ny && ny < H) && ((maze_w[my_y][my_x] & (1<<i)) == 0 )){//  && ((maze_w[my_y][my_x] & (1<<(4+i))) != 0 ) ){
                
                if(maze_flag[ny][nx] != 1){//まだ到達してなければ
                    if(first_flag == 0 || num > maze_d_perfect[ny][nx]){//初めのマスは無条件で移動す候補にする || ゴールに近いマスを見つけた
                        num = maze_d_perfect[ny][nx];
                        n_num = i;
                        
                        first_flag = 1;
                        
                    }else if(num == maze_d_perfect[ny][nx]){// LとRが同じ重み　斜めを優先したい
                    
                        /*if( maze_d[ny][nx][i] < maze_d[my_y+dy[n_num]][my_x+dx[n_num]][n_num] ){//斜めを考慮しない重みの小さいほうを優先する
                            n_num = i;
                            
                        }else if( maze_d[ny][nx][i] == maze_d[my_y+dy[n_num]][my_x+dx[n_num]][n_num]) {
                        */	if(last == -1 && (i - my_angle + 4)%4 == 1   ){//前回がL かつ　今回はR  
                                n_num = i;
                                    
                            }else if(last == 1 && (i - my_angle + 4)%4 == -1   ){//前回がR　かつ　今回はL
                                n_num = i;
                                    
                            }else{//前回がSなら今回は?
                                if( maze_d[ny][nx][i] < maze_d[my_y+dy[n_num]][my_x+dx[n_num]][n_num] ){//斜めを考慮しない重みの小さいほうを優先する
                                    n_num = i;
                                }
                            }
                        //}

                        first_flag = 1;
                    }
                }
            }
        }
        
        if((maze_w[my_y][my_x] & (1<<(4+n_num))) == 0){//移動する向きが未確定の壁だった場合
            *target_x = my_x + dx[n_num];
            *target_y = my_y + dy[n_num];
            
            //現在位置をバックアップから復元
            my_x = my_x_tmp;
            my_y = my_y_tmp;
            my_angle = my_angle_tmp;
            return;
        }
        
        //移動する
        short ni = ((4 + n_num - ((4+my_angle-1)%4))%4) -1;// -1 ~ 2

        switch(ni){
        case -1://L
            if(h_path > 0){
                if(queue_empty())h_path--;
                enqueue(0);
                enqueue(h_path);
                h_path = 0;
            }

            enqueue(-1);
            enqueue(1);
            my_angle = (4+my_angle-1)%4;
            
            my_x += dx[n_num];
            my_y += dy[n_num];
            
            last = -1;
            break;
        case 0://S
    
            h_path +=2;
            my_x += dx[n_num];
            my_y += dy[n_num];
            
            //last = 0;
            break;
        case 1://R
            if(h_path > 0){
                if(queue_empty())h_path--;
                enqueue(0);
                enqueue(h_path);
                h_path = 0;
            }
            
            enqueue(1);
            enqueue(1);
        
            my_angle = (4+my_angle+1)%4;

            my_x += dx[n_num];
            my_y += dy[n_num];
            
            last = 1;
            break;
        
        case 2://B
            if(h_path > 0){
                if(queue_empty())h_path--;
                enqueue(0);
                enqueue(h_path);
                h_path = 0;
            }
            
            enqueue(2);
            enqueue(1);
            my_angle = (4+my_angle+2)%4;
            break;
        }
    }
 
    if(h_path > 0){
	    if(queue_empty())h_path--;
        enqueue(0);
        enqueue(h_path+1);
        h_path = 0;
    }else{
        enqueue(0);
        enqueue(1);
    }
    
   //未確定マスを通らずにゴールまで経路を確認できた。
    *target_x = Get_Goal_x();
    *target_y = Get_Goal_y(); 
    
    //現在位置をバックアップから復元
    my_x = my_x_tmp;
    my_y = my_y_tmp;
    my_angle = my_angle_tmp;
  
}

/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */
/* 関 数 概 要：最短経路上の未確定マスを探す											            */
/* 関 数 詳 細：												                                   */
/* 引       数： 																				    */
/* 戻  り   値： なし										    									*/
/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */ 
void maze_search_unknown(short* target_x,short* target_y){
 
    short x = Start_x,y = Start_y,angle = Start_angle;
    int last = 0;
    
    short Goal_x_tmp,Goal_y_tmp;
    

    Goal_x_tmp = Get_Goal_x();
    Goal_y_tmp = Get_Goal_y();
	    
    while(x != Goal_x_tmp || y != Goal_y_tmp){
    	short num = maze_d[y][x][(angle+2)%4];
    	short n_num = angle;
    	char s_flag = 0;
    	short nx = x+dx[angle],ny = y+dy[angle];
    	if((0 <= nx && nx < W) && (0 <= ny && ny < H) && ((maze_w[y][x] & (1<<angle)) == 0 ) ){//目の前が迷路内　&& 壁がない 
	        short next = maze_d[ny][nx][(angle+2)%4];
            if(num == next+1){//目の前のマスがゴールに近い
                n_num = (angle+2)%4;
                num = next;
                s_flag = true;
            }
    	}

    	if(s_flag == false){//回転する必要がる
	        for(int i = 0;i < 4;i++){
                if(i == (angle+2)%4){//B
                    //初期値なので更新不要
                }else{
                    short next = maze_d[y][x][i];
                    if(num > next){
                        n_num = i;
                        num = next;
                    }else if(num == next){// LとRが同じ重み　斜めを優先したい
                        if(last == -1){//前回がLなら今回はR
                            n_num = (angle-1+4)%4;
                                        
                        }else if(last == 1){//前回がRなら今回はL
                            n_num = (angle+1+4)%4;
                                        
                        }else{//前回がSなら今回は?
                            //わからんから先に見つかった方にする
                        }
                    }
                }
            }
    	}
    
    	n_num = (n_num+2)%4;// 0 ~ 4　進みたい方角
    	short ni = ((4 + n_num - ((4+angle-1)%4))%4) -1;// -1 ~ 2　マシンから見た方角

    	if((maze_w[y][x] & (1 << (4+n_num))) == 0){//未確定の壁
            *target_x = x;
            *target_y = y;
            
            return;
	    	
	    }
	
    	switch(ni){
        case -1://L
            angle = (4+angle-1)%4;
                    
            last = -1;
            break;
        case 0://S
    
            x += dx[n_num];
            y += dy[n_num];
                    
            //last = 0;
            break;
                    
        case 1://R
            angle = (4+angle+1)%4;
                
            last = 1;
            break;
        case 2://B 不要のはず
                
            angle = (4+angle+2)%4;
                    
            //last = 2;
            break;
    	}
    }
	
    //未確定マスを通らずにゴールまで経路を確認できた。
    *target_x = Get_Goal_x();
    *target_y = Get_Goal_y();

}

/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */
/* 関 数 概 要：最短経路はすべて確定マスにする  										            */
/* 関 数 詳 細：												                                   */
/* 引       数： 																				    */
/* 戻  り   値： なし										    									*/
/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */ 
void maze_search_all(){
    GyroSum_reset();
   // Encoder_reset();

    led_down();
	
    short target_x,target_y;
    //short target_x_tmp,target_y_tmp;
    char path_ng = 0;
    
    static char phese_flag = 0;
    
    short mazed_miti,mazed_kiti,cost_tmp;
    
    
    //time_limit = xxxx;//60秒  別のところで設定するように変更
	
    while(time_limit > 0){//制限時間の間走行可能
	path_ng = 0;
	
	maze_update(my_x,my_y,my_angle,3);
	
	path_ng = shortest_path_search(Get_Goal_x(),Get_Goal_y());//重みマップの作成　未確定の壁は無いと考える
	
	if(path_ng == 1){//最短経路が存在しない→迷路情報を元に戻す
		for(int i = 0; i < H;i++){
			for(int j = 0; j < W; j++){
				maze_w[i][j] = maze_w_backup[i][j];
				for(int k = 0; k < 4;k++)maze_d[i][j][k] = maze_d_backup[i][j][k];
			}
		
	    	}
	}else{//迷路情報をバックアップする
		for(int i = 0; i < H;i++){
			for(int j = 0; j < W; j++){
				maze_w_backup[i][j] = maze_w[i][j];
				for(int k = 0; k < 4;k++)maze_d_backup[i][j][k] = maze_d[i][j][k];
			}
		
	    	}	
	}
	
	//これ以降は確実に最短経路が存在する迷路情報を持っている必要がある
	
	if(phese_flag == 0){
		maze_search_unknown(&target_x,&target_y);//最短経路上の未確定マスの座標を取得	
		
		if(target_x == Get_Goal_x() && target_y == Get_Goal_y()){//最短経路上に未確定マスがなければ
			phese_flag = 1;
		}
		
	}
	
	if(phese_flag == 1){
		//phese_flag = 2;
		
		shortest_path_search_perfect_unknown(&target_x,&target_y);//斜めも考慮した最短経路上の未確定マスの座標を取得
		
		if(target_x == Get_Goal_x() && target_y == Get_Goal_y()){//最短経路上に未確定マスがなければ
			phese_flag = 2;
		}
	}
	
	if(phese_flag == 2){
		shortest_path_search_dijkstra_unknown(&target_x,&target_y);//ダイクストラ　最短経路上の未確定マスの座標を取得
		
		if(target_x == Get_Goal_x() && target_y == Get_Goal_y()){//最短経路上に未確定マスがなければ
			motor(0,0);

#ifdef Pickup_x
			pickup_x = Pickup_x;
			pickup_y = Pickup_y;
			run_pickup(pickup_x,pickup_y);//拾いやすいところまで移動する
#else
			if( (Not_Pickup_y_min <= my_y  && my_y <=  Not_Pickup_y_max) && (Not_Pickup_x_min <= my_x &&  my_x <= Not_Pickup_x_max) ){//拾いにくいところにいる
				search_pickup(&pickup_x,&pickup_y);
				run_pickup(pickup_x,pickup_y);//拾いやすいところまで移動する
			}
#endif			
//			maze_search_adachi(Start_x,Start_y);//ピックアップ位置ではなくスタート位置に戻したいときに上の処理と切り替える
			
			led_down();
			led_up();
			led_down();
			led_up();
						
			motor(0,0);
			led(7);
			return;
		}
	}
	
	
		
		/*
			//確実に最短経路にならないマスも探索することになる　無効化する
			
			if(phese_flag == 0){//大まかに探索
	
				shortest_path_search(my_x,my_y);//ゴールに近いマスではなく、現在位置に近いマスから探索する場合
				
				maze_search_unknown_wall(&target_x_tmp,&target_y_tmp);//4方向すべての壁が確定していないマスからゴールに近いマスの座標を取得
				
				if(target_x_tmp == Get_Goal_x() && target_y_tmp == Get_Goal_y()){//ゴール座標が設定されているときは対象のマスがないとき
					phese_flag = 1;
					
				}else{//目標地点を大まかに探索した結果に置き換える
					target_x = target_x_tmp;
					target_y = target_y_tmp;
				}
				
			}
		*/	
		
	
		
	//走行経路の選択のため、重みマップの作成
    shortest_path_search(target_x,target_y);//重みマップの作成　未確定の壁は無いと考える
	mazed_miti = maze_d[my_y][my_x][my_angle];
	
	shortest_path_search_kichikukan(target_x,target_y);//未確定の壁は　ある　として考える　＝　未探索のマスを走行しない　＝　ほこりが少ない経路を選択する
	mazed_kiti = maze_d[my_y][my_x][my_angle];

	cost_tmp = (get_r_cost() * Search_all_r_num) + Search_all_s_num;
	if((mazed_kiti == maze_d_max) || (mazed_miti <= cost_tmp && cost_tmp < mazed_kiti) ){//既知区間では到達できない場合　|| 未知区間だと近いのに既知区間だと遠い場合,振り回されるので未知区間で移動距離優先にする
		shortest_path_search(target_x,target_y);//重みマップの作成　未確定の壁は無いと考える
	}
	
	
	//走行経路作成
	make_shortest_path_list(target_x,target_y); //未確定マスでも連続する直線なら進む
    	//make_shortest_path_list_simple(target_x,target_y); //未確定マスでとまる
		
	run_shortest_path();
  	
	motor(0,0);

    }
	
    
    
    if(time_limit <= 0){//　制限時間内に探索できなかった　

#ifdef Pickup_x
	pickup_x = Pickup_x;
	pickup_y = Pickup_y;
	run_pickup(pickup_x,pickup_y);//拾いやすいところまで移動する
#else
	if( (Not_Pickup_y_min <= my_y  && my_y <=  Not_Pickup_y_max) && (Not_Pickup_x_min <= my_x &&  my_x <= Not_Pickup_x_max) ){//拾いにくいところにいる
		search_pickup(&pickup_x,&pickup_y);
		run_pickup(pickup_x,pickup_y);//拾いやすいところまで移動する
	}
#endif
//	maze_search_adachi(Start_x,Start_y);//ピックアップ位置ではなくスタート位置に戻したいときに上の処理と切り替える
	
	//以下は最短経路を確定できたかどうかの確認用
	shortest_path_search(Get_Goal_x(),Get_Goal_y());
	
	maze_search_unknown(&target_x,&target_y);//最短経路上の未確定マスの座標を取得 
	
	
/*	while(1){
		motor(0,0);
		if(get_sw() == 1){
			printf2("%d : %d \n",target_x,target_y);	
		}
	}
*/
	if(target_x == Get_Goal_x() && target_y == Get_Goal_y()){//最短経路上に未確定マスがなければ
		
		shortest_path_search_perfect_unknown(&target_x,&target_y);//斜めも考慮した最短経路上の未確定マスの座標を取得 
	
		if(target_x == Get_Goal_x() && target_y == Get_Goal_y()){//最短経路上に未確定マスがなければ
		
			shortest_path_search_dijkstra_unknown(&target_x,&target_y);//ダイクストラ　最短経路上の未確定マスの座標を取得
			
			if(target_x == Get_Goal_x() && target_y == Get_Goal_y()){//最短経路上に未確定マスがなければ

		    		led_down();
				led_up();
				led_down();
				led_up();
				
				led(7);
				motor(0,0);
	    			return;
				
			}else{//ダイクストラは未確定マスがある
				led(15);
			        delay(500);
			        led(0);
			        delay(500);
			        led(15);
			        delay(500);
		    
		        	//Tmotor(l45 /2);//45度 / 2 回転し、最後まで探索できなかったことをわかるようにする
				
				led(3);
			        motor(0,0);
	    			return;
			}
			
		}else{//斜め考慮は未確定マスがある
			led(15);
		        delay(500);
		        led(0);
		        delay(500);
		        led(15);
		        delay(500);
		    
		        //Tmotor(r45 /2);//45度 / 2 回転し、最後まで探索できなかったことをわかるようにする
			
			led(1);
			motor(0,0);
	    		return;
		}
	}else{
	    led(15);
	    delay(500);
	    led(0);
	    delay(500);
	    led(15);
	    delay(500);
	    
	    //Tmotor(r45);//45度回転し、最後まで探索できなかったことをわかるようにする
	    
	    led(8);
	    motor(0,0);
	    return;
	}
    }
}


/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */
/* 関 数 概 要：最短経路作成											  			            */
/* 関 数 詳 細：												                                   */
/* 引       数： 目的地のXY座標															    */
/* 戻  り   値： なし										    									*/
/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */ 
void make_shortest_path_list(short target_x,short target_y){
    queue_reset();
    short s_path = 0;
    short x = my_x,y = my_y,angle = my_angle;
    short unknown_flag = 1; // 0:確定壁　1:未確定壁
	
    int mas_cnt = 0;//直線に進むマスの数
    int last = 0;
	
    while(x != target_x || y != target_y){
    	short num = maze_d[y][x][(angle+2)%4];
    	short n_num = angle;//0;
    	char s_flag = 0;
    	short nx = x+dx[angle],ny = y+dy[angle];
    	if((0 <= nx && nx < W) && (0 <= ny && ny < H) && ((maze_w[y][x] & (1<<angle)) == 0 ) ){//目の前が迷路内　&& 壁がない 
            short next = maze_d[ny][nx][(angle+2)%4];
            if(num == next+1){//目の前のマスがゴールに近い
                n_num = (angle+2)%4;
                num = next;
                s_flag = true;
            }
    	}

    	if(s_flag == false){//回転する必要がる
            for(int i = 0;i < 4;i++){
                if(i == (angle+2)%4){//B
                    //初期値なので更新の必要なし
                }else{
                    short next = maze_d[y][x][i];
                    if(num > next){
                        n_num = i;
                        num = next;
                            
                    }else if(num == next){// LとRが同じ重み　斜めを優先したい
                        if(last == -1){//前回がLなら今回はR
                            n_num = (angle-1+4)%4;
                            
                        }else if(last == 1){//前回がRなら今回はL
                            n_num = (angle+1+4)%4;
                            
                        }else{//前回がSなら今回は?
                            //わからんから先に見つかった方にする
                        }
                    }
                }
            }
    	}
    
    	n_num = (n_num+2)%4;// 0 ~ 4　進みたい方角
    	short ni = ((4 + n_num - ((4+angle-1)%4))%4) -1;// -1 ~ 2　マシンから見た方角

    	if((maze_w[y][x] & (1 << (4+n_num))) == 0){//未確定の壁
            //break;
                
            if( unknown_flag == 1){//初めから未確定の直線の場合は有効　一度でも確定のマスを進むと無効
                if(ni != 0)break;//直進方向でなければ打ち切り//////////////////////////////////////////////////////////////////////////////////////////////////////////////
                ni = 10;//未確定の直線
            }else{
                break;//未確定の壁を確認する必要があるので打ち切り
            }
			
        }else{//確定
            if(ni == 0){//直線
                if(mas_cnt == 0){//はじめての直進の１マス目
                    ni = 10;//直進方向なら未確定の直線の可能性あり メモ：目の前の壁は確定してるから
                }else{
                            
                }
            }
        }
	
        switch(ni){
            case -1://L
                if(s_path > 0){
                    if(unknown_flag == 1 && s_path == 1){//1マスだけなら確定の直線
                        enqueue(0);
                        enqueue(s_path);
                        s_path = 0;
                        unknown_flag = 0;
                    }else{
                        enqueue(10 * unknown_flag );
                        //enqueue(0);
                        enqueue(s_path);
                        s_path = 0;
                                    
                        if(unknown_flag == 1)return;//未確定の直線のあとはルートを作成してはいけない
                    }
                }

                enqueue(-1);
                enqueue(1);

                //s_path += 1;
                
                angle = (4+angle-1)%4;
                // x += dx[n_num];
                // y += dy[n_num];
                        
            // unknown_flag = 0;
                last = -1;
                break;
            case 0://S
        
                if(unknown_flag == 1 && s_path > 0 && mas_cnt > 1){
                    enqueue(10);
                    enqueue(s_path);
                    s_path = 0;
                                
                    return;//未確定の直線のあとはルートを作成してはいけない
                }
                        
                s_path +=1;
                x += dx[n_num];
                y += dy[n_num];
                        
                unknown_flag = 0;
                mas_cnt++;
                        
                //last = 0; 
                break;
                        
            case 10://S 未確定の直線
                        
                s_path +=1;
                x += dx[n_num];
                y += dy[n_num];
                        
                mas_cnt++;
                //last = 0;
                break;
                        
            case 1://R
                if(s_path > 0){
                    if(unknown_flag == 1 && s_path == 1){//1マスだけなら確定の直線
                        enqueue(0);
                        enqueue(s_path);
                        s_path = 0;
                        unknown_flag = 0;
                    }else{
                        enqueue(10 * unknown_flag);
                        //enqueue(0);
                        enqueue(s_path);
                        s_path = 0;
                                    
                        if(unknown_flag == 1)return;//未確定の直線のあとはルートを作成してはいけない
                    }
                }
                
                enqueue(1);
                enqueue(1);

                // s_path += 1;
                        
                angle = (4+angle+1)%4;
                // x += dx[n_num];
                // y += dy[n_num];
                        
                //unknown_flag = 0;
                last = 1;
                break;
            case 2://B
                if(s_path > 0){
                    if(unknown_flag == 1 && s_path == 1){//1マスだけなら確定の直線
                        enqueue(0);
                        enqueue(s_path);
                        s_path = 0;
                        unknown_flag = 0;
                    }else{
                        enqueue(10 * unknown_flag);
                        //enqueue(0);
                        enqueue(s_path);
                        s_path = 0;
                                    
                        if(unknown_flag == 1)return;//未確定の直線のあとはルートを作成してはいけない
                    }
                }
                
                enqueue(2);
                enqueue(1);

                //s_path += 1;
                        
                angle = (4+angle+2)%4;
                //x += dx[n_num];
                //y += dy[n_num];
                        
                //unknown_flag = 0;
                //last = 2;
                break;
        }
    }
 
    if(s_path > 0){
    	enqueue(10 * unknown_flag);
	//enqueue(0);
    	enqueue(s_path);
    	s_path = 0;
    }
}

/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */
/* 関 数 概 要：最短経路探索											  			            */
/* 関 数 詳 細：												                                   */
/* 引       数： 目的地のXY座標															    */
/* 戻  り   値： なし										    									*/
/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */ 
char shortest_path_search(short target_x,short target_y){
    queue_reset();
    for(int i = 0; i < H;i++){
        for(int j = 0;j < W; j++){
            for(int k = 0; k < 4; k++){
            maze_d[i][j][k] = maze_d_max;
            }
        }
    }
    for(int k = 0; k < 4; k++){
	    if((maze_w[target_y][target_x] & (1<<k)) == 0 )maze_d[target_y][target_x][k] = 0;//ゴール向きが無関係なので０すべて０を設定
    }
    enqueue(target_x*100 + target_y);
  
    while(!queue_empty()){
        short x = dequeue(),y;
        y = x%100;
        x /=100;

        for(char i =0;i<4;i++){
            char update_flag = 0;
            short nx = x+dx[i],ny = y+dy[i];
            if((0 <= nx && nx < W) && (0 <= ny && ny < H) && ((maze_w[y][x] & (1<<i)) == 0 ) ){//未確定の場合は壁無しとして考える

            short num = maze_d[y][x][i];
            for(int k = 0; k < 4; k++){
            
                if(i == k){//S
                    if(maze_d[ny][nx][k] > num + 1){
                        update_flag = true;
                        maze_d[ny][nx][k] = num + 1;
                    }
                }else if((i+2+4)%4 == k){//B
                    if(maze_d[ny][nx][k] > num+1 + get_r_cost()*2){
                        update_flag = true;
                        maze_d[ny][nx][k] = num+1 + get_r_cost()*2;
                    }
                }else{// L or R
                    if(maze_d[ny][nx][k] > num+1 + get_r_cost()){
                        update_flag = true;
                        maze_d[ny][nx][k] = num+1 + get_r_cost();
                    }
                }
            }
            if(update_flag)enqueue(nx*100 + ny);
            }
        }
    }
    
    char ng_flag = 1;
    for(int k = 0; k < 4; k++){
    	if(maze_d[Start_y][Start_x][k] != maze_d_max ){//スタート位置の重みが更新されてなかったら＝最短経路が存在しない
    		ng_flag = 0;
    	}
    }
    if(ng_flag == 1){
	return 1;    
    }
    
    return 0;
}
/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */
/* 関 数 概 要：足立法で探索走行											  			            */
/* 関 数 詳 細：												                                   */
/* 引       数： 目的地のXY座標															    */
/* 戻  り   値： なし										    									*/
/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */ 
void maze_search_adachi(short target_x,short target_y){
   
    char path_ng = 0;
    
    GyroSum_reset();
   // Encoder_reset();

    led_down();
	
    while(1){
	maze_update(my_x,my_y,my_angle,3);
	/*if((target_x != Get_Goal_x() || target_y != Get_Goal_y()) && (target_x != Start_x || target_y != Start_y)){//スタート地点、ゴール地点以外が目標地点のとき
	  if((maze_w[target_y][target_x] & 0xf0) == 0xf0)break;    //目標地点の壁がすべて確定したら探索完了  
	  }*/
	
	path_ng = shortest_path_search(Get_Goal_x(),Get_Goal_y());
	
	if(path_ng == 1){//最短経路が存在しない→迷路情報を元に戻す
		for(int i = 0; i < H;i++){
			for(int j = 0; j < W; j++){
				maze_w[i][j] = maze_w_backup[i][j];
				for(int k = 0; k < 4;k++)maze_d[i][j][k] = maze_d_backup[i][j][k];
			}
	    }
	}else{//迷路情報をバックアップする
		for(int i = 0; i < H;i++){
			for(int j = 0; j < W; j++){
				maze_w_backup[i][j] = maze_w[i][j];
				for(int k = 0; k < 4;k++)maze_d_backup[i][j][k] = maze_d[i][j][k];
			}
	    }	
	}
	
	if(target_x == my_x && target_y == my_y){//ゴール
	    motor(0,0);
	    led_up();
			
	    if((target_x == Start_x && target_y == Start_y) || (target_x == Get_Goal_x() && target_y == Get_Goal_y())){
				
		GyroSum_reset();
		
		mae_kabe();//前壁距離補正
		
		Tmotor(r180);
		motor(0,0);
		delay(100);
		
		my_angle = (4+my_angle+2)%4;
	    }
	    break;
	}
		
	shortest_path_search(target_x,target_y);
	make_shortest_path_list(target_x,target_y);
	run_shortest_path();
	
	motor(0,0);
	GyroSum_reset();
	motor(0,0);
	
    }
    motor(0,0);
}
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
