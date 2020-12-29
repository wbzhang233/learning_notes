/* SceneMatching��
** ����ƥ��
** Author: wbzhang
** Email: wbzhang233@163.com
** Date: 2019.9.28 in bafs
** Enviornment: c++11 ,vs2017+opencv4.1.0 & contrib
*/

#include "../include/SceneMatching.h"
#include<iostream>
#include<string>


SceneMatching::SceneMatching(string mapPath, string picsPath)
{
	map_path = mapPath;
	pics_path = picsPath;
	img1 = imread(mapPath);
	featureDetection(img1, _ORB, false, keyPoints1, descriptor1);
	cout<<"kp1Size:"<<keyPoints1.size()<<endl;
}

SceneMatching::SceneMatching()
{

}


SceneMatching::~SceneMatching()
{
}

void SceneMatching::Run()
{
	VideoCapture sequence(pics_path);
	int count = 1;
	int frameRate = 10;
	track.clear();
	while (count < 1680)
	{
		if (count % 10 == 0) {
			sequence >> img2;
			char img2_name[20];
			sprintf(img2_name, "%d.jpg", count);
			string img_name = pics_path + img2_name;
			img2 = imread(img_name);
			cout <<img_name <<endl;
			cout << count / 10;
			featureDetection(img2, _ORB, false, keyPoints2, descriptor2);
			vector<DMatch> goodMatchs;
			featureMatching(descriptor2, _KNN, false, goodMatchs);
			ransacMatching(goodMatchs, ransacMatches,true);
			//���ƺ���ͼ
			namedWindow("FlyingTrace", WINDOW_KEEPRATIO);
			resizeWindow("FlyingTrace", img_trace.cols *3/ 4, img_trace.rows*3 / 4);
			imshow("FlyingTrace", img_trace);
		}
		else
		{
			++count;
			continue;
		}
		cout <<  endl;
		waitKey(5);
		count += frameRate;
	}
	waitKey();
}

void SceneMatching::featureDetection(const cv::Mat img, FeatureType featureType, bool ShowDetectResult,
	std::vector<cv::KeyPoint>& keyPoints, cv::Mat& descriptor)
{
	double t1 = (double)getTickCount();

	switch (featureType)
	{
	case _ORB:
	{
		//��1��ORB����
		/* ��������١��ٶȷǳ��죬��1ms���¡�*/
		const char* window_name = "��ORB������ƥ��";
		std::cout << ".ORB�������:   \t";
		Ptr<ORB> orb = ORB::create(15000);
		orb->detectAndCompute(img, noArray(), keyPoints, descriptor);
		window_name = "ORB����ƥ��";
		break;
	}
	case _FREAK:
	{

		//��2��FREAK����
		Mat imgGray;
		cvtColor(img, imgGray, COLOR_BGR2GRAY);
		const char* window_name = "FREAK����ƥ��";
		std::cout << ".FREAK�������:   \t";
		Mat homography;
		FileStorage fs("./H1to3p.xml", FileStorage::READ);
		fs.getFirstTopLevelNode() >> homography;

		FAST(imgGray, keyPoints, 30);   //ʹ��FAST��������㣬Ч�����
		//Ptr<Feature2D> detector = xfeatures2d::SURF::create();                 //Ч���Ϻ�
		//Ptr<Feature2D> detector = xfeatures2d::SIFT::create(0, 3, 0.04, 10);  //Ч���Ϻ�
		//Ptr<cv::ORB> detector = cv::ORB::create(10000);      //��̫���ˣ����ʺ�
		//Ptr<BRISK> detector = BRISK::create(80, 4, 1.0f);   //��̫���ˣ����ʺ�
		//Ptr<AKAZE> detector = AKAZE::create();          //�������ٶȷǳ���

		Ptr<xfeatures2d::FREAK> freak = xfeatures2d::FREAK::create();
		//detector->detect(img1gray, keyPoints);
		freak->compute(imgGray, keyPoints, descriptor);
		window_name = "FREAK����ƥ��";
		break;
	}
	case _SIFT:
	{
		//��3��SIFT����
		/* ������϶ࡣ���ʱ����10ms���ҡ� */
		const char* window_name = "��SIFT������ƥ��";
		std::cout << ".��SIFT���������: ";
		Ptr<Feature2D> sift = xfeatures2d::SIFT::create(0, 3, 0.04, 10);
		sift->detectAndCompute(img, noArray(), keyPoints, descriptor);
		window_name = "SIFT����ƥ��";
		break;
	}
	case _SURF:
	{
		//��4��SURF����
		/* KNNɸѡ��������϶ࡣ���ʱ����10ms���ҡ�*/
		const char* window_name = "��SURF������ƥ��";
		std::cout << ".��SURF���������: ";
		Ptr<Feature2D> surf = xfeatures2d::SURF::create();
		surf->detectAndCompute(img, noArray(), keyPoints, descriptor);
		window_name = "SURF����ƥ��";
		break;
	}
	case _BRISK:
	{
		//��5��BRISK����
		/* KNNɸѡ����������١����ʱ����4ms���ҡ�*/
		const char* window_name = "��BRISK������ƥ��";
		std::cout << ".��BRISK���������: ";
		Ptr<BRISK> brisk = BRISK::create(30, 4, 1.0f);
		brisk->detectAndCompute(img, Mat(), keyPoints, descriptor, false);
		window_name = "BRISK����ƥ��";
		break;
	}
	case _AKAZE:
	{
		//��6��AKAZE����  
		/* KNNɸѡ���������١��ٶȷǳ������������ʱ����17ms����*/
		const char* window_name = "��AKAZE������ƥ��";
		std::cout << ".��AKAZE���������: ";
		Ptr<AKAZE> akaze = AKAZE::create();
		akaze->detectAndCompute(img, noArray(), keyPoints, descriptor);
		window_name = "AKAZE����ƥ��";
		break;
	}
	default:
	{
		const char* window_name = "ORB����ƥ��";
		std::cout << ".ORB�������:   \t";
		Ptr<ORB> orb = ORB::create(10000);
		orb->detectAndCompute(img, noArray(), keyPoints, descriptor);
		window_name = "ORB����ƥ��";
		break;
	}

	}

	t1 = ((double)getTickCount() - t1) / getTickFrequency();
	std::cout << t1 / 1.0 << "s" << endl;
	std::cout << "# Keypoints:       \t" << keyPoints.size() << endl;
	float perPtTime = t1 / keyPoints.size();
	std::cout << "detect per point time:       \t" << perPtTime <<"s"<< endl;

	if (ShowDetectResult) {
		//����������
		Mat imgDrawKeypoints;
		drawKeypoints(img, keyPoints, imgDrawKeypoints);
		namedWindow("��img��������", 0);
		imshow("��img��������", imgDrawKeypoints);
	}
}

void SceneMatching::featureMatching(cv::Mat descriptor, MatchType matchType,
	bool ShowMatchResult, std::vector<DMatch>& goodMatches)
{
	//��3������������ƥ��
	Mat img1_clone = img1.clone();
	Mat img2_clone = img2.clone();
	double t2 = (double)getTickCount();
	switch (matchType)
	{
	case _KNN: {
		//��1��KNN-�����ƥ��ɸѡ
		std::cout << "-��KNN�������ƥ��:   \t";
		vector<vector<DMatch> > knn_match;

		/*
		Ptr<DescriptorMatcher> descriptor_matcher = DescriptorMatcher::create("BruteForce");   //��������ƥ������
		descriptor_matcher->knnMatch(descriptor1, descriptor, knn_match, 2);   //��������ƥ��ʹ���ƥ�䵽knn_match������Ϊimg1_clone��img2_clone
		BFMatcher matcher(NORM_L1);   //L1��L2������SIFT��SURF
		*/

		BFMatcher matcher(NORM_HAMMING);   //HAMMING������ORB��BRISK��FREAK
		matcher.knnMatch(descriptor1, descriptor, knn_match, 2);

		//��ȷƥ�䱣֤����ƥ��ʹ���ƥ��֮����нϴ��࣬�������֮�����С������Ϊ�Ǵ���ƥ��
		const float minRatio = 2.0f / 2.5f;
		for (int i = 0; i < knn_match.size(); i++)
		{
			const DMatch& bestMatch = knn_match[i][0];
			const DMatch& betterMatch = knn_match[i][1];

			float r = bestMatch.distance / betterMatch.distance;
			if (r < minRatio)
				goodMatches.push_back(bestMatch);
		}

		//������ƥ�����
		t2 = ((double)getTickCount() - t2) / getTickFrequency();
		std::cout << t2 / 1.0 << "s" << endl;

		//���KNNƥ����Ϣ
		std::cout << "# KNNmatches:       \t" << knn_match.size() << endl;
		std::cout << "# GoodMatches:     \t " << goodMatches.size() << endl;
		double MatchRatio = goodMatches.size()*1.00 / knn_match.size();
		std::cout << "# MatcheRatio:   \t " << MatchRatio*100 << "%"<<endl;

		if (ShowMatchResult)
		{
			//����ƥ���� 
			Mat img_knnMatches;
			drawMatches(img1_clone, keyPoints1, img2_clone, keyPoints1, goodMatches, img_knnMatches);
			namedWindow("��KNN�������ƥ��ͼ", WINDOW_KEEPRATIO);
			resizeWindow("��KNN�������ƥ��ͼ", img_knnMatches.cols / 2, img_knnMatches.rows / 2);
			imshow("��KNN�������ƥ��ͼ", img_knnMatches);
			imwrite("knn_match.jpg", img_knnMatches);
		}
		break;
	}
	case _BROUTFORCE: {
		//��2������ƥ�䷨
		const float good_ratio = 0.4f;         //����ƥ�䷨������ֵ�� 0.4
		std::cout << "--��BF������ƥ�䷨:   \t";
		//����������ƥ�乹����
		//BFMatcher matcher(NORM_L1);   //L1��L2������SIFT��SURF
		BFMatcher matcher(NORM_HAMMING);   //HAMMING������ORB��BRISK��FREAK
		vector<DMatch> bfmatches;     //���ڴ洢ƥ������ʸ��
		matcher.match(descriptor1, descriptor, bfmatches);

		//����ƥ���
		double max_dist = 0; double min_dist = 10000;
		//-- Quick calculation of max and min distances between keypoints
		for (int i = 0; i < descriptor1.rows; i++)
		{
			double dist = bfmatches[i].distance;
			if (dist < min_dist) min_dist = dist;
			if (dist > max_dist) max_dist = dist;
		}
		//printf("-- Max dist : %f \n", max_dist);
		//printf("-- Min dist : %f \n", min_dist);

		for (int i = 0; i < descriptor1.rows; i++)
		{
			if (bfmatches[i].distance < 0.65*max_dist)
			{
				goodMatches.push_back(bfmatches[i]);
			}
		}

		//������ƥ�����
		t2 = ((double)getTickCount() - t2) / getTickFrequency();
		std::cout << t2 / 1.0 << "s" << endl;

		//�������ƥ�䷨��ʾ��Ϣ
		std::cout << "# BFmatches:       \t" << bfmatches.size() << endl;
		std::cout << "# GoodMatches:     \t " << goodMatches.size() << endl;
		double MatchRatio = goodMatches.size()*1.00 / bfmatches.size();
		std::cout << "# MatcheRatio:   \t " << MatchRatio*100 << "%"<< endl;
		//cout << "# good_ratio:    \t" << good_ratio << endl;

		if (ShowMatchResult)
		{
			//����ƥ����
			Mat img_BFMatches;
			drawMatches(img1_clone, keyPoints1, img2_clone, keyPoints2, goodMatches, img_BFMatches);
			namedWindow("��BF������ƥ��ͼ", 1);
			resizeWindow("��BF������ƥ��ͼ", img_BFMatches.cols / 2, img_BFMatches.rows / 2);
			imshow("��BF������ƥ��ͼ", img_BFMatches);
			imwrite("MatchPic/ORB/BF�Լ�1.jpg", img_BFMatches);
		}
		break;
	}
	default:
		std::cout << "Error in choose matching method!" << endl;
		break;
	}

}

void SceneMatching::ransacMatching(std::vector<DMatch> good_matches, std::vector<DMatch>& ransac_matches,bool ShowRansacResult)
{
	ransac_matches.clear();
	//RANSAC���㵥Ӧ�任�����ų�����ƥ����
	vector<KeyPoint> alignedKps1, alignedKps2;
	for (int i = 0; i < good_matches.size(); i++) {
		alignedKps1.push_back(keyPoints1[good_matches[i].queryIdx]);
		alignedKps2.push_back(keyPoints2[good_matches[i].trainIdx]);
	}

	vector<Point2f> ps1, ps2;
	for (int i = 0; i < alignedKps1.size(); i++)
		ps1.push_back(alignedKps1[i].pt);

	for (int i = 0; i < alignedKps2.size(); i++)
		ps2.push_back(alignedKps2[i].pt);

	if (good_matches.size() < 4) {
		printf("good_matches number is not enough");
		RANSAC_FLAG = false;
	}

	if (RANSAC_FLAG)
	{
		Mat status = Mat::zeros(good_matches.size(), 1, CV_8UC1);
		Mat H = findHomography(ps2, ps1, FM_RANSAC, 3, status);

		uchar *status_p;
		vector<DMatch>::const_iterator it_match = good_matches.begin();
		for (int i = 0; i < good_matches.size(); i++) {
			status_p = status.ptr<uchar>(i);
			if (*status_p)
				ransac_matches.push_back(it_match[i]);
		}

		//��ƥ��ͼ�ڵ�ͼ�е�λ��
		Mat img_detect = img1.clone();                 //���Ƶ�ͼͼ�����ڱ�ʶ
		vector<Point2f> model_corners(4);             //�洢ģ��ͼ����ĸ��ǵ�
		vector<Point2f> scene_corners(4);             //�洢ģ��ͼ���ڵ�ͼ�е��ĸ��ǵ�

		model_corners[0] = Point(0, 0);            //���Ͻ�Ϊԭ��(0,0)��x��ָ�����y��ָ���
		model_corners[1] = Point(img2.cols, 0);
		model_corners[2] = Point(img2.cols, img2.rows);
		model_corners[3] = Point(0, img2.rows);

		perspectiveTransform(model_corners, scene_corners, H);

		line(img_detect, scene_corners[0], scene_corners[1], Scalar(0, 255, 0), 2);   //����ģ���ڳ����еķ�Χ
		line(img_detect, scene_corners[1], scene_corners[2], Scalar(0, 255, 0), 2);
		line(img_detect, scene_corners[2], scene_corners[3], Scalar(0, 255, 0), 2);
		line(img_detect, scene_corners[3], scene_corners[0], Scalar(0, 255, 0), 2);

		if (ShowRansacResult) {
			//����RANSACɸѡ���ƥ����
			drawMatches(img_detect, keyPoints1, img2, keyPoints2, ransac_matches, imgRansacMatches);   //�ڸ��Ƶĵ�ͼͼ���л���
			namedWindow(window_name, 0);
			resizeWindow(window_name, imgRansacMatches.cols / 2, imgRansacMatches.rows / 2);
			imshow(window_name, imgRansacMatches);
		}

		//����ƥ��ƥ��trace�Ļ���
		currentPosition.x = (scene_corners[0].x + scene_corners[2].x) / 2;
		currentPosition.y = (scene_corners[0].y + scene_corners[2].y) / 2;
		track.push_back(currentPosition);

		img_trace = img1.clone();
		for (int i = 0; i < track.size(); i++)
		{
			RNG rng(12345);
			Scalar RandColor = Scalar(rng.uniform(0, 255), rng.uniform(0, 255), rng.uniform(0, 255));
			//circle(img_trace,track[i],5, Scalar(177, 255, 249),2,8);
			//circle(img_trace, track[i], 2,Scalar(255,255,255), 1, 8);
			if (i >= 1)
			{
				line(img_trace, track[i - 1], track[i], Scalar(0, 0, 255), 2, 8);
			}
			//circle(img_trace,track[i],5, Scalar(177, 255, 249),2,8);
			circle(img_trace, track[i], 2, Scalar(255, 255, 255), 1, 8);
		}
		char text[40];
		sprintf(text, "currentPos:(%.3f,%.3f)", currentPosition.x, currentPosition.y);
		putText(img_trace, text, Point(150, 150), 1, 1.5, Scalar(240, 245, 240), 2,8);
		//namedWindow("FlyingTrace", WINDOW_KEEPRATIO);
		//resizeWindow("FlyingTrace", img_trace.cols / 2, img_trace.rows / 2);
		//imshow("FlyingTrace", img_trace);
	}
}

int main()
{
	string map_path = "/home/wbzhang/map.jpg";
	string pics_path = "/home/wbzhang/scenePics/";

	SceneMatching sceneMatcher = SceneMatching(map_path, pics_path);
	sceneMatcher.Run();

	return 0;
}
