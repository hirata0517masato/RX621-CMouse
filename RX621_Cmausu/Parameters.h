#ifndef PARAM_H
#define PARAM_H

/*
�����ɒu�����Ƃ�
���ǂ�60�@�O�ǂ�50 ���炢
*/

#define maze_d_max	9999

#define W	16 //X
#define H	16 //Y

#define Start_x  0
#define Start_y  0
#define Start_angle 1 // �X�^�[�g���鎞�̌����@

#define Goal_x  0
#define Goal_y  1
#define Goal_angle 0 //�S�[����������̌���

#define Pickup_x  2
#define Pickup_y  0

//�W���C���֘A
#define r45  (11600)	//45�x �E��] 11500 10500
#define l45  (11600)	//45�x ����] 11500
#define r90  (21300) 	//90�x �E��]
#define l90  (21200)	//90�x ����]
#define r180  (-43500)	//180�x �E��]
#define sr90  (24000)	//�X�����[�� 23000
#define sl90  (25000)	//�X�����[�� 23000

#define sr90BIG  (25500)	//�X�����[�� 
#define sl90BIG  (24500)	//�X�����[�� 

//�G���R�[�_�֘A�@memo : 1mm = 5.56 //�ǐ؂�170 //2nd��1.34�{���炢
#define s1 (995)		//1�}�X���i 714
#define s45 (700)		//45�x�P�}�X���i 500
#define h1 (500)		//���}�X���i 350
#define rslsl90 (942)	//�X�����[�� 664
#define rslsr90 (942)	//�X�����[�� 664

#define rslsl90_BIG (980)	//�X�����[�� ��700
#define rslsr90_BIG (980)	//�X�����[�� �� 680

#define r_cost 4		//���H�̏d�݁@90�x��]�̏ꍇ


//�O�Ǖ␳�̃p�����[�^
#define F_max 275
#define F_min 260
#define F_pow 10
#define F_cnt 600


#endif