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
#define Goal_angle 0 //�S�[����������̌��� �X�^�[�g�̌�����1

//�ŒZ���s��A���グ�₷���ʒu�Ɉړ��@�Œ�l��
#define Pickup_x  1	
#define Pickup_y  1

//�ŒZ���s��A���グ�₷���ʒu�Ɉړ��@�͈͎w���
#define Not_Pickup_x_min  3		//0�`15 �͈̔͂Őݒ�@�@( j < Not_Pickup_x_min ||  Not_Pickup_x_max < j)���Ώ۔͈�
#define Not_Pickup_y_min  3

#define Not_Pickup_x_max  17
#define Not_Pickup_y_max  17

//�W���C���֘A
#define r45  (11500)	//45�x �E��] 11500 10500
#define l45  (11500)	//45�x ����] 11500
#define r90  (21470) 	//90�x �E��]
#define l90  (21300)	//90�x ����]
#define r180  (-43000)	//180�x �E��]
#define sr90  (25000)	//�X�����[�� 23000 //���E�œ����l�łȂ��ƘA�����ɂ���Ă���
#define sl90  (25000)	//�X�����[�� 23000

#define sr90BIG  (25500)	//�X�����[�� ���
#define sl90BIG  (25500)	//�X�����[�� ���

#define ur180  (47500)	//U�^�[�� //���E�œ����l�łȂ��ƘA�����ɂ���Ă���
#define ul180  (47500)	//U�^�[��

//�G���R�[�_�֘A�@memo : 1mm = 5.56 //�ǐ؂�170 //2nd��1.34�{���炢
#define s1 (1010)		//1�}�X���i      �^�C����ꂽ���F1025
#define s45 (795)		//45�x�P�}�X���i �^�C����ꂽ���F800
#define h1 (500)		//���}�X���i 	�^�C����ꂽ���F525�@�ŒZ�p
#define h1_2 (500)		//���}�X���i 	�^�C����ꂽ���F525  �T���̕ǐ؂�p

#define rslsl90 	(980)	//�X�����[�� 664
#define rslsl90_offset  (0)
#define rslsr90 	(980)	//�X�����[�� 664
#define rslsr90_offset  (-0)

#define rslsl90_BIG (1000)	//�X�����[�� ��700
#define rslsr90_BIG (1000)	//�X�����[�� �� 680

#define usll180     (2250)	//U�^�[��
#define usll180_fin (0)	//U�^�[�� ������
#define uslr180     (2250)	//U�^�[��
#define uslr180_fin (0)	//U�^�[���@������

#define r_cost 4		//���H�̏d�݁@90�x��]�̏ꍇ


//�O�Ǖ␳�̃p�����[�^
#define F_max 250
#define F_min 235
#define F_pow 5
#define F_cnt 600


#endif