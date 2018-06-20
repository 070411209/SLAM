
#include <iostream>
#include <chrono>
#include <time.h>
#include <sys/time.h>


#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
//#include "ORBextractor.h"

#include <opencv2/features2d/features2d.hpp>
#include <opencv2/imgproc/imgproc.hpp>
//#include <opencv2/calib3d/calib3d.hpp>
#include "motract.h"


// #include "mm.h"
using namespace std;

//using namespace ORB_SLAM2;

using namespace cv;


int main ()
{
    // 读取argv[1]指定的图像
	// 读取灰度图像
	Mat g_grayImage;	// g1_grayImage, g2_grayImage, g11_grayImage;
	Mat leftImage, rightImage;
    //g_grayImage = imread ( "../image/a.bmp", 0 );
    //g1_grayImage = imread ( "../test_1486_rgb.bmp", 0 );
	//g2_grayImage = imread ( "../test_1487_rgb.bmp", 0 );
	
	leftImage  = imread ( "../image/1.png", 0 );
	rightImage = imread ( "../image/2.png", 0 );
	
	cout<<"输入图像宽1为"<<leftImage.cols<<",高为"<<leftImage.rows<<",通道数为"<<leftImage.channels()<<endl;
	cout<<"输入图像宽2为"<<rightImage.cols<<",高为"<<rightImage.rows<<",通道数为"<<rightImage.channels()<<endl;
	
	//cout<<"输入图像宽2为"<<g1_grayImage.cols<<",高为"<<g1_grayImage.rows<<",通道数为"<<g1_grayImage.channels()<<endl;
	
	// 特征点集合
	vector<cv::KeyPoint> cv_feats, cv1_feats;	//cv2_feats, cv11_feats;
	// 描述子集合
    Mat cv_descs, cv1_descs;	//, cv2_descs, cv11_descs;
	
	
	cout << "------------Start-------------" << endl;
	
	//Size ksize = 7;
	/*
	GaussianBlur(g1_grayImage, g11_grayImage, Size(7, 7), 1.2, 0);
	
	imshow("orb_slam_img_11_blur", g11_grayImage);	
	//waitKey(0);
	
	imwrite("../test.bmp", g11_grayImage);
	
	*/
	
	/*
	clock_t t;
	t = clock();
	// 初始化
	ORBextractor();
	// 计算特征点和描述子
	computeORB( leftImage, cv::Mat(), cv_feats, cv_descs);
	t = clock() - t;
	cout << "It takes " << t << "clock!" << endl;
	// 输出特征点个数
	cout << "cv_feats size :" << cv_feats.size() << endl;
	*/
	//------------------------------------------------------------

	cv::Ptr<cv::ORB> orb = cv::ORB::create(500,1.2f,8,19,0,2,cv::ORB::FAST_SCORE,31,30);	
	orb->detectAndCompute(leftImage, cv::Mat(), cv_feats, cv_descs);
	
	
	cv::Ptr<cv::ORB> orb2 = cv::ORB::create(500,1.2f,8,19,0,2,cv::ORB::FAST_SCORE,31,30);
	orb2->detectAndCompute(rightImage, cv::Mat(), cv1_feats, cv1_descs);
	
	/*	
	cv::Ptr<cv::ORB> orb3 = cv::ORB::create(500,1.2f,8,19,0,2,cv::ORB::FAST_SCORE,31,30);
	
	orb2->detectAndCompute(g2_grayImage, cv::Mat(), cv2_feats, cv2_descs);	
	
	cv::Ptr<cv::ORB> orb4 = cv::ORB::create(500,1.2f,8,19,0,2,cv::ORB::FAST_SCORE,31,10);
	
	orb2->detectAndCompute(g11_grayImage, cv::Mat(), cv11_feats, cv11_descs);		
	*/
	// ----------------------orb slam2----------------------------
	
	//ORB_SLAM2::ORBextractor extractor;

    // 遍历图像, 请注意以下遍历方式亦可使用于随机像素访问
	
    // extractor(g_grayImage, cv::Mat(), cv_feats, cv_descs);

	// --------------------------------------------------
	
	// Imshow 
	Mat img_rgb = leftImage.clone();
    for_each(cv_feats.begin(), cv_feats.end(), [&](cv::KeyPoint i) {
        circle(img_rgb, i.pt, 2 * (i.octave + 1), cv::Scalar(255, 255, 255), 1);
    });
	
    imshow("orb_slam_img_1", img_rgb);	
	waitKey(0);
	
	// Imshow new
	Mat img1_rgb = rightImage.clone();
    for_each(cv1_feats.begin(), cv1_feats.end(), [&](cv::KeyPoint i) {
        circle(img1_rgb, i.pt, 2 * (i.octave + 1), cv::Scalar(255, 255, 255), 1);
    });
	
    imshow("orb_slam_img_2", img1_rgb);	
	waitKey(0);

	/*
	for (int i = 0; i < cv_feats.size(); i++)
		cout << " cv_feats pos : " << cv_feats.at(i).pt << endl;

	// new add
	computeORB( g_grayImage, cv::Mat(), cv1_feats, cv1_descs);
	*/		
	
	/*



	
	// Imshow new
	Mat img2_rgb = g2_grayImage.clone();
    for_each(cv2_feats.begin(), cv2_feats.end(), [&](cv::KeyPoint i) {
        circle(img2_rgb, i.pt, 2 * (i.octave + 1), cv::Scalar(255, 255, 255), 1);
    });
	
    imshow("orb_slam_img_3", img2_rgb);	
	waitKey(0);
	
	// Imshow new
	Mat img11_rgb = g11_grayImage.clone();
    for_each(cv11_feats.begin(), cv11_feats.end(), [&](cv::KeyPoint i) {
        circle(img11_rgb, i.pt, 2 * (i.octave + 1), cv::Scalar(255, 255, 255), 1);
    });
	
    imshow("orb_slam_img_11", img11_rgb);	
	waitKey(0);	
	*/
	//---------------------------------------------------
	
	//BruteForceMatcher<<L2<float>> matcher;
	//vector<DMatch> matches;
	//Ptr<DescriptorMatcher> matcher = DescriptorMatcher::create("BRUTEFORCE");
	/*
	Ptr<DescriptorMatcher> descriptorMatcher = DescriptorMatcher::create( "BruteForce" );  
	
	vector<DMatch> matches;  
	descriptorMatcher->match(cv_descs, cv1_descs, matches);  
	// 
	
	Mat imResultOri;  
    drawMatches(g_grayImage, cv_feats, g1_grayImage, cv1_feats, matches, imResultOri, CV_RGB(0,255,0), CV_RGB(0,0,0)); 
	
	//Mat imgMatches;
	//drawMatches(g_grayImage, cv_feats, g1_grayImage, cv1_feats, matches, imgMatches);
	
	
	imshow("aa", imResultOri);
	waitKey(0);
	*/
////////////


	Ptr<DescriptorMatcher> descriptorMatcher2 = DescriptorMatcher::create( "BruteForce" );  
	
	vector<DMatch> matches2;  
	descriptorMatcher2->match(cv_descs, cv1_descs, matches2);  
	// 
	
	Mat imResultOri2;  
    drawMatches(leftImage, cv_feats, rightImage, cv1_feats, matches2, imResultOri2, CV_RGB(0,255,0), CV_RGB(0,0,0)); 
	
	//Mat imgMatches;
	//drawMatches(g_grayImage, cv_feats, g1_grayImage, cv1_feats, matches, imgMatches);
	
	
	imshow("match image", imResultOri2);
	waitKey(0);


	
    return 0;
}