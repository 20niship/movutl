#include "Effects.h"

///////////////////////////////////////////////////
//                  reference   �@�@�@�@�@�@�@�@ //
///////////////////////////////////////////////////

// http://opencv.jp/opencv-2svn/cpp/imgproc_image_filtering.html
// https://book.mynavi.jp/support/pc/opencv2/c3/opencv_img.html
// https://qiita.com/hmichu/items/0a399d9e3bbf3a2a4454


///////////////////////////////////////////////////
//                  �􉽊w�I�ϊ��@�@�@�@�@�@�@�@ //
///////////////////////////////////////////////////

bool rotate(cv::Mat *img_before, float angle = 0.0) {

	// get rotation matrix for rotating the image around its center in pixel coordinates
	cv::Point2f center(((*img_before).cols - 1) / 2.0, ((*img_before).rows - 1) / 2.0);
	cv::Mat rot = cv::getRotationMatrix2D(center, angle, 1.0);

	// determine bounding rectangle, center not relevant
	cv::Rect2f bbox = cv::RotatedRect(cv::Point2f(), (*img_before).size(), angle).boundingRect2f();

	// adjust transformation matrix
	rot.at<double>(0, 2) += bbox.width / 2.0 - (*img_before).cols / 2.0;
	rot.at<double>(1, 2) += bbox.height / 2.0 - (*img_before).rows / 2.0;

	cv::Mat dst;
	cv::warpAffine((*img_before), (*img_before), rot, bbox.size());
	return true;
}


// �摜�g��Bcenter_x,�@center_y�͊g�咆�S��\���A-1�̎��͉摜�̒��S���g�咆�S�Ƃ���
bool Zoom(cv::Mat *img_before, float zoom_x, float zoom_y, float center_x=-1, float center_y=-1) {
	if (zoom_x == 100 && zoom_y == 100) return false;
	if (zoom_x == 0 || zoom_y == 0) return false;  //TODO�F���̂܂܂��Ɖ摜���������Ȃ��܂ܕԂ��Ă��܂��̂ŁA�����x��100���ɂ��ĕ\�����Ȃ��悤�ɂ���

	if (center_x < 0 && center_y < 0) {
		// https://stackoverflow.com/questions/58216668/opencv-c-zoom-function

		cv::Mat dst;
		cv::resize((*img_before), (*img_before), cv::Size(), zoom_x / 100.0, zoom_y / 100.0);
		return true;
	}else {
		if(zoom_x > 100 && zoom_y > 100){
			int width = (*img_before).cols;
			int height = (*img_before).rows;

			//TODO: center_x > width�̂悤�ȂƂ���
			center_x = MIN(MAX(center_x, 0), width);
			center_y = MIN(MAX(center_y, 0), height);
			return false;
		}
		if (zoom_x < 100 && zoom_y < 100) {
			return false;
			//TODO: �摜�k���ɂ��A�摜�̑��΍��W��ς���
		}

	}
}

//�@�摜���S���g�咆�S�Ƃ���Y�[���G�t�F�N�g
bool Zoom_simple(cv::Mat *img_before, float zoom_x, float zoom_y) {
	// https://stackoverflow.com/questions/58216668/opencv-c-zoom-function
	cv::Mat dst;
	cv::resize((*img_before), (*img_before), cv::Size(), zoom_x / 100.0, zoom_y / 100.0);
	return true;
}

bool Clipping(cv::Mat *img_before, float left, float top, float right, float bottom) {
	//TODO: �ϊ����ꂽcv::Mat�̕\���ʒu�����炷
	int width = (*img_before).cols - left - right;
	int height = (*img_before).rows - top - bottom;
	cv::Rect roi(cv::Point(left, top), cv::Size(MAX(width, 0), MAX(height, 0)));
	(*img_before) = (*img_before)(roi); // �؂�o���摜
	return true;
}


bool ColorTragger(cv::Mat *img_before, int degree, int Rpos, int Gpos, int Bpos) {
	cv::Size size = cv::Point((*img_before).cols, (*img_before).rows);
	if (degree != 0) rotate(img_before, degree);

	int drag_pos[] = { Rpos, Gpos, Bpos };

	for (int color_num = 0; color_num < 3; color_num++) {
		if (drag_pos[color_num] > 0) {
			for (int x = 0; x < (*img_before).cols - 1; x++) {
				for (int y = 0; y < (*img_before).rows - 1; y++) {
					int x_dragged = MIN((*img_before).cols, MAX(x + drag_pos[color_num], 0));
					(*img_before).at<cv::Vec3b>(y, x)[color_num] = (*img_before).at<cv::Vec3b>(y, x_dragged)[color_num];
				}
			}
		}
		else {
			for (int x = (*img_before).cols - 1; x >= 0; x--) {
				for (int y = 0; y < (*img_before).rows - 1; y++) {
					int x_dragged = MIN((*img_before).cols, MAX(x + drag_pos[color_num], 0));
					(*img_before).at<cv::Vec3b>(y, x)[color_num] = (*img_before).at<cv::Vec3b>(y, x_dragged)[color_num];
				}
			}
		}
	}

	if (degree != 0) {
		rotate(img_before, -degree);

		int left = ((*img_before).cols - size.width) / 2;
		int top = ((*img_before).rows - size.height) / 2;
		cv::Rect roi(cv::Point(left, top), size);
		(*img_before) = (*img_before)(roi); // �؂�o���摜
	}
	return true;
}

bool ColorTragger_easy(cv::Mat *img_before, int degree, int pos) {
	ColorTragger(img_before, degree, pos, -pos, 0);
	return true;
}

bool Flip(cv::Mat *img_before, int FlipRL, int FlipTD) {
	if (FlipRL > 0 && FlipTD > 0) {
		cv::flip((*img_before), (*img_before), -1);
		return true;
	}
	if (FlipRL > 0) {
		cv::flip((*img_before), (*img_before), 1);
		return true;
	}
	if (FlipTD > 0) {
		cv::flip((*img_before), (*img_before), 0);
		return true;
	}
}



///////////////////////////////////////////////////
//                 �e��t�B���^ �@�@�@�@�@�@�@�@ //
///////////////////////////////////////////////////

bool ToneCorrection(cv::Mat *img_before, float brightness, float contrast) {
	(*img_before).convertTo((*img_before), -1, (double)contrast / 100.0, brightness - 100);
	return true;
}

bool ColorCorrection(cv::Mat *img_before, int Hue_Offset, int Saturation_Offset, int Brightness_Offset) {
	cv::Mat hsvImage;
	cvtColor((*img_before), hsvImage, CV_RGB2HSV_FULL);
	for (int x = 0; x < (*img_before).cols - 1; x++) {
		for (int y = 0; y < (*img_before).rows - 1; y++) {
			hsvImage.at<cv::Vec3b>(y, x)[0] = MIN(254, MAX(hsvImage.at<cv::Vec3b>(y, x)[0] * Hue_Offset / 100.0, 0));  //H
			hsvImage.at<cv::Vec3b>(y, x)[1] = MIN(254, MAX(hsvImage.at<cv::Vec3b>(y, x)[1] * Saturation_Offset / 100, 0));  //S
			hsvImage.at<cv::Vec3b>(y, x)[2] = MIN(254, MAX(hsvImage.at<cv::Vec3b>(y, x)[2] + Brightness_Offset - 100, 0));  //V
		}
	}
	cvtColor(hsvImage, (*img_before), CV_HSV2RGB_FULL);
	return true;
}

bool ColorCorrectionRGB(cv::Mat *img_before, int R_Offset, int G_Offset, int B_Offset, float R_Gamma, float G_Gamma, float B_Gamma) {

	// TODO: at<...>����|�C���^�[���g������ɕύX��������
	for (int x = 0; x < (*img_before).cols - 1; x++) {
		for (int y = 0; y < (*img_before).rows - 1; y++) {
			(*img_before).at<cv::Vec3b>(y, x)[0] = MIN(254, MAX((*img_before).at<cv::Vec3b>(y, x)[0] * R_Gamma / 100.0 + R_Offset - 100, 0));
			(*img_before).at<cv::Vec3b>(y, x)[1] = MIN(254, MAX((*img_before).at<cv::Vec3b>(y, x)[1] * G_Gamma / 100.0 + G_Offset - 100, 0));
			(*img_before).at<cv::Vec3b>(y, x)[2] = MIN(254, MAX((*img_before).at<cv::Vec3b>(y, x)[2] * B_Gamma / 100.0 + B_Offset - 100, 0));
		}
	}
	return true;
}

bool blur(cv::Mat *img_before, int kernel_size_x, int kernel_size_y) {
	kernel_size_x = MAX(1, kernel_size_x);
	kernel_size_y = MAX(1, kernel_size_y);

	cv::blur((*img_before), (*img_before), cv::Size(kernel_size_x, kernel_size_y));  /* ROI��ݒ肵���摜�œ��o�͂��s�� */
	return true;
}

bool blur_g(cv::Mat *img_before, int kernel_size_x, int kernel_size_y, int sigma) {
	if (!(*img_before).data) return -1;
	cv::Mat dst;
	// �K�E�V�A����p����������
	// ���͉摜�C�o�͉摜�C�J�[�l���T�C�Y�C�W���΍�x, y
	cv::GaussianBlur((*img_before), (*img_before), cv::Size(kernel_size_x * 2 + 1, kernel_size_y * 2 + 1), sigma, 0);

	return true;
}

bool toGray(cv::Mat *img_before) {
	cv::Mat img_gray;
	cvtColor((*img_before), img_gray, CV_RGB2GRAY);
	cvtColor(img_gray, (*img_before),  CV_GRAY2BGRA);
	return true;
}

bool Binarize(cv::Mat *img_before, int type) {
	// �Œ��臒l����
	cv::Mat gray_img;
	cvtColor((*img_before), gray_img, CV_RGBA2GRAY);

	cv::Mat bin_img, bininv_img, trunc_img, tozero_img, tozeroinv_img;
	// ���͉摜�C�o�͉摜�C臒l�CmaxVal�C臒l������@
	cv::threshold(gray_img, bin_img, 0, 255, cv::THRESH_BINARY | cv::THRESH_OTSU);
	cv::threshold(gray_img, bininv_img, 0, 255, cv::THRESH_BINARY_INV | cv::THRESH_OTSU);
	cv::threshold(gray_img, trunc_img, 0, 255, cv::THRESH_TRUNC | cv::THRESH_OTSU);
	cv::threshold(gray_img, tozero_img, 0, 255, cv::THRESH_TOZERO | cv::THRESH_OTSU);
	cv::threshold(gray_img, tozeroinv_img, 0, 255, cv::THRESH_TOZERO_INV | cv::THRESH_OTSU);

	// �K���I��臒l����
	cv::Mat adaptive_img;
	// ���͉摜�C�o�͉摜�CmaxVal�C臒l�����@�C臒l������@�CblockSize�CC
	cv::adaptiveThreshold(gray_img, adaptive_img, 255, cv::ADAPTIVE_THRESH_GAUSSIAN_C, cv::THRESH_BINARY, 7, 8);
	cv::imshow("test", gray_img);
	(*img_before) = adaptive_img;
	return true;
}

bool MultiSlicer(cv::Mat *img_before, float angle = 0.0) {
	return true;
}


bool Radiation_blur(cv::Mat *img_before, int center_x, int center_y, int zoom_rate, int smoothness=3) {
    // http://shira.hatenadiary.jp/entry/2018/06/30/022357
	// TODO:  center�̎���
	int width = (*img_before).cols;
	int height = (*img_before).rows;

	cv::Mat original = (*img_before).clone();

	float zoom_rate_max = float( MAX(zoom_rate - 100, 0)) / float(smoothness);

	for (int i = 1; i < smoothness; i++) {
		float zoom_rate = zoom_rate_max * i + 100;
		cv::Mat zoomed = original.clone();
		
		Zoom(&zoomed, zoom_rate, zoom_rate);
		blur(&zoomed, 3*i, 3*i);

		int left = (zoomed.cols - width) / 2;
		int top = (zoomed.rows - height) / 2;
 
		cv::Rect roi(cv::Point(left, top), cv::Size(width, height));
		zoomed = zoomed(roi); // �؂�o���摜

		double alpha = 0.9 + float(i-1) / float(smoothness*10);
		cv::addWeighted((*img_before), alpha, zoomed, 1.0f-alpha, 0, (*img_before));
	}

	return true;
}

