#ifndef PARAM_H
#define PARAM_H

/*
�����ɒu�����Ƃ�
���ǂ�60�@�O�ǂ�50 ���炢
*/

#define maze_d_max	9999
#define H	16
#define W	16

#define Start_x  0
#define Start_y  0
#define Start_angle 1

#define Goal_x  8
#define Goal_y  7
#define Goal_angle 0

#define Pickup_x  4
#define Pickup_y  11

//�W���C���֘A
#define r45  (11000)	//45�x �E��]
#define l45  (11500)	//45�x ����]
#define r90  (21000) 	//90�x �E��]
#define l90  (21500)	//90�x ����]
#define r180  (-44000)	//180�x �E��]
#define sr90  (24400)	//�X�����[��
#define sl90  (24400)	//�X�����[��

//�G���R�[�_�֘A�@memo : 1mm = 4 //�ǐ؂�170
#define s1 (700)		//1�}�X���i 700  �@735
#define s45 (500)		//45�x�P�}�X���i 500 520
#define h1 (350)		//���}�X���i 350 340
#define rslsl90 (645)	//�X�����[��
#define rslsr90 (647)	//�X�����[��

#define r_cost 4		//���H�̏d�݁@90�x��]�̏ꍇ

#endif