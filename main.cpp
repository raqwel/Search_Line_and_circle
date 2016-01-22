#include <opencv2/opencv.hpp>
#include <opencv2\opencv_lib.hpp>
#include <iostream>
#include <math.h>

//�z��̓Y���@�������炢���o���邩��̂�\�z���ēK�X�ύX����
#define NUM 30

void FindLines2(void);
void FindCircles(void);


cv::Mat dst_img,work_img,sort_img,fin_img,choose_img;

int main()
{
	//���s�����o
	FindLines2();

	//���S�~���o
	//FindCircles();

	cv::waitKey(0);
	return 0;
}

void FindLines2(){

	/*���m�����p�x��X���A�ؕЂ�����ϐ�*/
	double ang_box[NUM];
	double tilt_box[NUM];
	double b[NUM];

	//�J�E���g����ϐ�
	int i = 0,k = 0, l = 0;
	int count = 0;
	
	//���o�����n�_�I�_���i�[����ϐ�
	cv::Point startPoint[NUM],goalPoint[NUM];

	//���摜�̓ǂݍ���
	cv::Mat src_img = cv::imread("image/test5.png");

	//���ʂ�`�悷��摜������
	dst_img = src_img.clone();
	sort_img = src_img.clone();
	fin_img = src_img.clone();
	choose_img = src_img.clone();

	//���ʂ�`�悷�邽�߂ɐ^�����ȉ摜�̐���
	fin_img = cv::Scalar(255,255,255);

	//���摜�̏o��
	cv::namedWindow("src",CV_WINDOW_AUTOSIZE|CV_WINDOW_FREERATIO);
	cv::imshow("src",src_img);

	//�O���[�X�P�[���ɕϊ�
	cv::cvtColor(src_img,work_img,CV_BGR2GRAY);

	//�G�b�W���o
	cv::Canny(work_img,work_img,100,100,3);

	//�m���I�n�t�ϊ�
	std::vector<cv::Vec4i> lines;

	cv::HoughLinesP(work_img, lines, 1, CV_PI/180, 50, 60, 10);

	std::vector<cv::Vec4i>::iterator it = lines.begin();

	//���m������A�n�_�I�_�p�x��ϐ��ɓ����
	for(; it!=lines.end(); ++it) {
		cv::Vec4i l = *it;
		cv::line(dst_img, cv::Point(l[0], l[1]), cv::Point(l[2], l[3]), cv::Scalar(0,0,255), 1, CV_AA);

		//�p�x����W���i�[���Ă���
		double ang = atan2(double(l[3]-l[1]),double(l[2]-l[0]));

		ang = ang * 180 / 3.1415926535;

		//0�`360�ȓ��ɂ���
		if(ang < 0){
			ang = ang + 360;
		}

		double tilt = tan(ang) *-1;

		ang_box[i] = ang;

		tilt_box[i] = tilt;

		b[i] = l[1] - tilt * l[0];

		//std::cout << "�n�_���W(x,y) = " << cv::Point(l[0],l[1]) << std::endl;

		//std::cout << "�I�_���W(x,y) = " << cv::Point(l[2],l[3]) << std::endl;

		//���ꂼ��̎n�_�I�_���W��z��Ɋi�[
		startPoint[i] = cv::Point(l[0],l[1]);

		goalPoint[i] = cv::Point(l[2],l[3]);

		//std::cout << "ang = " << (int)ang << std::endl;

		//std::cout << "�X�� = " << tilt << std::endl;

		//std::cout << " b = " << l[1] - tilt * l[0] << std::endl;

		//�~�̂������o�������J�E���g����
		i++;

	}

	double length,start_len,goal_len,sg_len;
	int check[NUM] = {0};

	/*	�n�_�I�_���_�A�����_�̏I�_�Ǝ���̎n�_�A�X�����߂����̂ŋ߂��ɂ����������	*/
	for(k=0;k<i;k++){
		for(l=k;l<i;l++){
			start_len = sqrt(pow((startPoint[k].x-startPoint[l].x),2.0) + pow((startPoint[k].y-startPoint[l].y),2.0));
			goal_len = sqrt(pow((goalPoint[k].x-goalPoint[l].x),2.0) + pow((goalPoint[k].y-goalPoint[l].y),2.0));
			sg_len = sqrt(pow((goalPoint[k].x-startPoint[l].x),2.0) + pow((goalPoint[k].y-startPoint[l].y),2.0));
			//std::cout << "�n�_���m���� = "<< start_len << std::endl;
			//std::cout << "�I�_���m���� = "<< goal_len << std::endl;
			if( (((0 < start_len && start_len < 75) && (0 < goal_len && goal_len < 75)) || (sg_len < 70) && (tilt_box[l] / tilt_box[k]) < 1.2
				&& (tilt_box[l] / tilt_box[k]) > 0.8) && (b[k] - b[l] < 30 || b[l] - b[k] < 30)){
					//vote++;
					check[k] = 1;
			}
		}
	}

	count = 0;
	std::cout << "-----------������--------------" << std::endl;

	double sort_tilt[NUM]={0},sort_b[NUM]={0};
	cv::Point sort_start[NUM]={0},sort_goal[NUM]={0};

	for(int k = 0; k < i;k++){
		if(check[k] == 0){
			std::cout << "[�X�� �ؕ� �n�_�@�I�_] = [" << tilt_box[k] << "," << b[k] << "," << startPoint[k] << "," << goalPoint[k] << "]" << std::endl;
			cv::line(choose_img, startPoint[k], goalPoint[k], cv::Scalar(255,0,0), 1, CV_AA);
			sort_tilt[count] = tilt_box[k];
			sort_b[count] = b[k];
			sort_start[count] = startPoint[k];
			sort_goal[count] = goalPoint[k];
			count++;
		}
	}

	std::cout << "�c������" << std::endl;
	for(k=0;k<count;k++){
		std::cout << "[�X�� �ؕ� �n�_�@�I�_] = [" << sort_tilt[k] << "," << sort_b[k] << "," << sort_start[k] << "," << sort_goal[k] << "]" << std::endl;
	}

	/*	�������������̂ŌX�����������̂�T���Čv�Z���Đ�������	*/
	/*	������Œ肵�A���̐��ɑ΂��đ��̐��������ɍ��������Ă���	*/

	cv::Point fin_s,fin_g;

	for(k=0;k<count;k++){
		for(l=k+1;l<count;l++){
			if(sort_tilt[k] / sort_tilt[l] < 2.5 && sort_tilt[k] / sort_tilt[l] > 0.5 && (sort_b[k] * sort_b[l]) > 0){

				//�������p�X�������̑g�ݍ��킹�Ɋւ��Ďn�_�I�_�̕��ς����
				fin_s.x = (sort_start[k].x+sort_start[l].x) / 2 ;
				fin_s.y = (sort_start[k].y+sort_start[l].y) / 2 ;

				fin_g.x = ((sort_goal[k].x+sort_goal[l].x)) / 2;
				fin_g.y = ((sort_goal[k].y+sort_goal[l].y)) / 2; 

				//��������
				cv::line(fin_img, fin_s, fin_g , cv::Scalar(0,0,255), 1, CV_AA);
			}
		}
	}

	//���ʂ̕`��
	cv::namedWindow("HoughLinesP", CV_WINDOW_AUTOSIZE|CV_WINDOW_FREERATIO);
	cv::imshow("HoughLinesP", dst_img);

	cv::namedWindow("sort", CV_WINDOW_AUTOSIZE|CV_WINDOW_FREERATIO);
	cv::imshow("sort", sort_img);

	cv::namedWindow("choose", CV_WINDOW_AUTOSIZE|CV_WINDOW_FREERATIO);
	cv::imshow("choose", choose_img);

	cv::namedWindow("fin", CV_WINDOW_AUTOSIZE|CV_WINDOW_FREERATIO);
	cv::imshow("fin", fin_img);

}

void FindCircles(){

	//�摜���i�[����ϐ��̗p��
	cv::Mat src_img = cv::imread("image/test9.png",1);
	cv::Mat src_gray;
	cv::Mat dst_img, work_img, diff_img,np_img,gray_img,mask_img,result_img,final_img;
	
	mask_img = cv::Scalar(0,0,0);
	dst_img = src_img.clone();
	fin_img = src_img.clone();
	result_img = src_img.clone();
	final_img = src_img.clone();

	//���ʂ�`�悷�邽�߂ɐ^�����ȉ摜�̗p��
	dst_img = cv::Scalar(255,255,255);
	result_img = cv::Scalar(255,255,255);
	final_img = cv::Scalar(255,255,255);

	//inpaint�֐���n�t�ϊ����s�����߂ɉ摜���O���[�X�P�[���ɂ���
	cv::cvtColor(src_img,src_gray, CV_RGB2GRAY);
	cv::cvtColor(src_img, work_img, CV_BGR2GRAY);

	//�J�E���g�A���a�A���S���W���i�[����ϐ�
	int i=0,k=0,l=0,m=0;
	int rad_box[NUM] = {0},rad_box2[NUM] = {0};
	cv::Point cen_box[NUM] = {0},cen_box2[NUM] = {0};

	// Hough�ϊ��̂��߂̑O�����i�摜�̕��������s�Ȃ�Ȃ��ƌ댟�o���������₷���j
	cv::GaussianBlur(work_img, work_img, cv::Size(11,11), 3, 3);

	// Hough�ϊ��ɂ��~�̌��o�ƌ��o�����~�̕`��
	std::vector<cv::Vec3f> circles;
	cv::HoughCircles(work_img, circles, CV_HOUGH_GRADIENT, 1, 5, 40, 100, 20, 100);

	std::vector<cv::Vec3f>::iterator it = circles.begin();
	for(; it!=circles.end(); ++it) {
		cv::Point center(cv::saturate_cast<int>((*it)[0]), cv::saturate_cast<int>((*it)[1]));
		int radius = cv::saturate_cast<int>((*it)[2]);

		//cv::circle(dst_img, center, 2, cv::Scalar(255,0,0), -1, 8);

		cv::circle(dst_img, center, radius, cv::Scalar(0,0,255), 8, 8, 0);

		//���a�ƒ��S���W���i�[
		rad_box[i] = radius;
		cen_box[i].x = center.x;
		cen_box[i].y = center.y;

		//���o���̃J�E���g
		i++;
	}

	//���o�����~�̔��a�ƒ��S���W��\��
	for(k=0;k<i;k++){
		std::cout << "[���a,���S] = {" << rad_box[k] << "," << cen_box[k] << "}" << std::endl;
	}
	
	//inpaint�֐���p���邽�߂ɃO���[�X�P�[���ϊ�
	cv::cvtColor(dst_img, gray_img, CV_RGB2GRAY);

	//�}�X�N�������s�����߂Ƀl�K�|�W���]�����Ă���
	np_img = ~gray_img;
	
	//�s�v�ȉ~���폜
	cv::inpaint(src_gray,np_img,fin_img,15,cv::INPAINT_NS);

	//�~���o
	cv::HoughCircles(fin_img, circles, CV_HOUGH_GRADIENT, 1, 5, 30, 90, 1, 100);

	std::vector<cv::Vec3f>::iterator it2 = circles.begin();
	for(; it2!=circles.end(); ++it2) {
		cv::Point center2(cv::saturate_cast<int>((*it2)[0]), cv::saturate_cast<int>((*it2)[1]));
		int radius2 = cv::saturate_cast<int>((*it2)[2]);

		cv::circle(result_img, center2, radius2, cv::Scalar(0,0,255), 8, 8, 0);

		//���o�����~�̔��a�ƒ��S���W���i�[����
		rad_box2[l] = radius2;
		cen_box2[l].x = center2.x;
		cen_box2[l].y = center2.y;

		//���o���̃J�E���g
		l++;
	}

	//���₷���悤�Ɏd�؂�����
	std::cout << "------------------------------------------------" << std::endl;

	//���S�~�ł��邩�𔻒肵�A���S�~�Ȃ甼�a�ƒ��S���W��\�������S���W��`��
	for(k=0;k<i;k++){
		for(m=0;m<l;m++){
			if(cen_box[k].x - cen_box2[m].x <= 5 && cen_box[k].y - cen_box2[m].y <= 5){
				std::cout << "[���a,���S] = {" << rad_box2[k] << "," << cen_box2[k] << "}" << std::endl;
				cv::circle(final_img, cen_box2[m], 1, cv::Scalar(0,0,0), 8, 8, 0);
			}
		}
	}
	
	/*�摜�̕\��*/
	cv::namedWindow("src", CV_WINDOW_AUTOSIZE|CV_WINDOW_FREERATIO);
	cv::imshow("src", src_img);

	cv::namedWindow("HoughCircles", CV_WINDOW_AUTOSIZE|CV_WINDOW_FREERATIO);
	cv::imshow("HoughCircles", dst_img);

	cv::namedWindow("fin", CV_WINDOW_AUTOSIZE|CV_WINDOW_FREERATIO);
	cv::imshow("fin", fin_img);

	cv::namedWindow("np", CV_WINDOW_AUTOSIZE|CV_WINDOW_FREERATIO);
	cv::imshow("np", np_img);


	cv::namedWindow("result", CV_WINDOW_AUTOSIZE|CV_WINDOW_FREERATIO);
	cv::imshow("result", result_img);

	cv::namedWindow("final", CV_WINDOW_AUTOSIZE|CV_WINDOW_FREERATIO);
	cv::imshow("final", final_img);
}