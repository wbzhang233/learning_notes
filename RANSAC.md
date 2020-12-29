# RANSAC

[匹配参考：](https://blog.csdn.net/z_johnking/article/details/106022261)

```c
#include<opencv2/opencv.hpp>
#include <opencv2/xfeatures2d.hpp>        //SIFT SURF
 
#include<iostream>
#include<vector>
 
constexpr auto path0 = "F:\\workspace\\opencv\\2_xfeature2d\\pic\\0.png";
constexpr auto path1 = "F:\\workspace\\opencv\\2_xfeature2d\\pic\\1.png";
 
int main() {
	cv::Mat image0 = cv::imread(path0, 1);
	cv::Mat image1 = cv::imread(path1, 1);
 
	cv::imshow("image0", image0);
	cv::imshow("image1", image1);
	/*
	step1:特征检测器
	*/
	cv::Ptr<cv::xfeatures2d::SURF> detector;
	detector = cv::xfeatures2d::SURF::create(800);  //800为海塞矩阵阈值，越大越精准
 
	/*
	-----SURF----
	cv::Ptr<cv::xfeatures2d::SURF> detector;
	detector = cv::xfeatures2d::SURF::create(800);  //800为海塞矩阵阈值，越大越精准
	-----SIFT-----
	cv::Ptr<cv::xfeatures2d::SIFT> detector;
	detector = cv::xfeatures2d::SIFT::create(800);//800为保留的点数
	-----ORB------
	cv::Ptr<cv::ORB> detector;
	detector  = cv::ORB::create(800);//保留点数
	-----STAR-----
	cv::Ptr<cv::xfeatures2d::StarDetector> detector;
	detector = cv::xfeatures2d::StarDetector::create();
	-----MSD-----
	cv::Ptr<cv::xfeatures2d::MSDDetector> detector;
	detector = cv::xfeatures2d::MSDDetector::create();
	*/
	std::vector <cv::KeyPoint > key0;
	std::vector <cv::KeyPoint > key1;
	detector->detect(image0,key0,cv::noArray());
	detector->detect(image1, key1, cv::noArray());
	
	/*
	step2:描述子提取器
	*/
	cv::Ptr<cv::xfeatures2d::SURF> Extractor;
	Extractor = cv::xfeatures2d::SURF::create(800);
	/*
       以下都是xfeature2d中的提取器
	-----SURF-----
	-----SIFT-----
	-----LUCID----
	-----BriefDescriptorExtractor----
	-----VGG-----
	-----BoostDesc-----
	*/
	cv::Mat descriptor0, descriptor1;
	Extractor->compute(image0, key0, descriptor0);
	Extractor->compute(image1, key1, descriptor1);
 
	/*
	step3:匹配器
	*/
	cv::BFMatcher matcher;//暴力匹配器
	std::vector<cv::DMatch> matches; // 存放匹配结果
	std::vector<cv::DMatch> good_matches; //存放好的匹配结果
 
	matcher.match(descriptor0, descriptor1, matches);             
	std::sort(matches.begin(), matches.end());     //筛选匹配点，根据match里面特征对的距离从小到大排序
 
	int ptsPairs = std::min(50, (int)(matches.size() * 0.15));
	std::cout << "匹配点数为" << ptsPairs << std::endl;
	for (int i = 0; i < ptsPairs; i++)
	{
		good_matches.push_back(matches[i]);              //距离最小的50个压入新的DMatch
	}
 
	cv::Mat result;
 
	cv::drawMatches(image0, key0,
		image1, key1,
		good_matches, result,
		cv::Scalar::all(-1), cv::Scalar::all(-1),
		std::vector<char>(),
		cv::DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);  //绘制匹配点  
 
	cv::imshow("result", result);
	cv::waitKey(0);
}
```

