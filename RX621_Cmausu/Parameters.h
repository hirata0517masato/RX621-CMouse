#ifndef PARAM_H
#define PARAM_H

#define maze_d_max	9999

#define W	16 //X  �ύX����Ɩ��H���̕ۑ����ł��Ȃ��Ȃ�o�O����
#define H	16 //Y

#define Start_x  0
#define Start_y  0
#define Start_angle 1 // �X�^�[�g���鎞�̌����@


////////////////////////////////////////////////////////////////////////////////////////////////////

#define Goal_x  0
#define Goal_y  1
#define Goal_angle 0 //�S�[����������̌��� �X�^�[�g�̌�����1

//�ŒZ���s��A���グ�₷���ʒu�Ɉړ��@�Œ�l�� //�R�����g�A�E�g����Ɣ͈͎w��łɐ؂�ւ��
#define Pickup_x  1	
#define Pickup_y  1

//�ŒZ���s��A���グ�₷���ʒu�Ɉړ��@�͈͎w���
#define Not_Pickup_x_min  4		//0�`15 �͈̔͂Őݒ�@�@( j < Not_Pickup_x_min ||  Not_Pickup_x_max < j)���Ώ۔͈�
#define Not_Pickup_y_min  4

#define Not_Pickup_x_max  16
#define Not_Pickup_y_max  16

//�^�C�����������Ƃ��ɕύX����p�����[�^
#define W_D	(220) //�z�C�[���̒��a(0.1mm)
#define W_P	(400) //�G���R�[�_1�T�̃p���X��

//�W���C���֘A
#define l45  (11500)	//45�x ����] 11500
#define r45  (11500)	//45�x �E��] 11500 10500
#define l90  (21500)	//90�x ����]
#define r90  (21500) 	//90�x �E��]
#define r180  (-43000)	//180�x �E��]
#define sl90  (24500)	//�X�����[�� 23000 25000
#define sr90  (24500)	//�X�����[�� 23000 25000//���E�œ����l�łȂ��ƘA�����ɂ���Ă���

#define sl90BIG  (26000)	//�X�����[�� ���
#define sr90BIG  (26000)	//�X�����[�� ���

#define ul180  (47500)	//U�^�[��
#define ur180  (47500)	//U�^�[�� //���E�œ����l�łȂ��ƘA�����ɂ���Ă���


//�G���R�[�_�֘A�@memo : 1mm = 5.56 //�ǐ؂�170 //2nd��1.34�{���炢
#define s1 (1015)		//1�}�X���i      �^�C����ꂽ���F1025
#define s45 (785)		//45�x�P�}�X���i �^�C����ꂽ���F800
#define h1 (525)		//���}�X���i 	�^�C����ꂽ���F525�@�ŒZ�p
#define h1_2 (520)		//���}�X���i 	�^�C����ꂽ���F525  �T���̕ǐ؂�p

#define s45_V2 (100)	//140	//�Q�}�XV�^�[���̋����@�ǐ؂ꂪ�ł��Ȃ��\���������̂Ŏ��������Œ�������
#define s45_V2_out (300)	//�Q�}�XV�^�[���̋����@�ǐ؂ꂪ�ł��Ȃ��\���������̂Ŏ��������Œ�������
#define s45_V2_out_LR (120)	//�Q�}�XV�^�[���̋����@�ǐ؂ꂪ�ł��Ȃ��\���������̂Ŏ��������Œ�������@�ǐ؂��ɃJ�[�u
#define s45_V2not (300)		//�Q�}�X�΂߁iV�^�[���ł͂Ȃ��j�̋����@�ǐ؂ꂪ�ł��Ȃ��\���������̂Ŏ��������Œ�������
#define s45_V2notFirst (350)	//����΂ߗp�@�Q�}�X�΂߁iV�^�[���ł͂Ȃ��j�̋����@�ǐ؂ꂪ�ł��Ȃ��\���������̂Ŏ��������Œ�������

#define rslsl90 	(980)	//�X�����[�� 664
#define rslsl90_offset  (0)
#define rslsr90 	(980)	//�X�����[�� 664
#define rslsr90_offset  (-0)

#define rslsl90_BIG (1610)	//�X�����[�� �� �^�C����ꂽ���F1600  1700
#define rslsl90_BIG_offset  (-100)	      //�^�C����ꂽ���F-100	
#define rslsr90_BIG (1610)	//�X�����[�� �� �^�C����ꂽ���F1600  1700
#define rslsr90_BIG_offset  (-100)	      //�^�C����ꂽ���F-100

#define usll180     (2250)	//U�^�[��
#define usll180_fin (0)	//U�^�[�� ������
#define uslr180     (2250)	//U�^�[��
#define uslr180_fin (0)	//U�^�[���@������

#define r_cost 4		//���H�̏d�݁@90�x��]�̏ꍇ

#define r45_cost 1		//���H�̏d�݁@45�x��]�̏ꍇ

//�_�C�N�X�g���p
#define cost_centor_wall  10 //�����p�̏d��
#define cost_wall_wall    12 //�΂ߗp�̏d��
    

//�O�Ǖ␳�̃p�����[�^
#define F_max 245
#define F_min 235
#define F_pow 10
#define F_cnt 4000
#define F_max_time 500

#define MAKE_KABE_tikai (135 + 60)
#define MAKE_KABE_tooi (135 - 60)


#endif