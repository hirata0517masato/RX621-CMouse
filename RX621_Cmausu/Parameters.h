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
#define Start_angle 1 // �X�^�[�g���鎞�̌����@

#define Goal_x  1
#define Goal_y  1
#define Goal_angle 1 //�S�[����������̌���

#define Pickup_x  0
#define Pickup_y  1

//�W���C���֘A
#define r45  (11500)	//45�x �E��]
#define l45  (11500)	//45�x ����]
#define r90  (22700) 	//90�x �E��]
#define l90  (22000)	//90�x ����]
#define r180  (-44000)	//180�x �E��]
#define sr90  (22800)	//�X�����[�� 22400
#define sl90  (22800)	//�X�����[�� 22400

//�G���R�[�_�֘A�@memo : 1mm = 4 //�ǐ؂�170
#define s1 (710)		//1�}�X���i 700  �@735
#define s45 (500)		//45�x�P�}�X���i 500 520
#define h1 (350)		//���}�X���i 350 340
#define rslsl90 (660)	//�X�����[�� 655
#define rslsr90 (657)	//�X�����[�� 652

#define r_cost 4		//���H�̏d�݁@90�x��]�̏ꍇ

#endif