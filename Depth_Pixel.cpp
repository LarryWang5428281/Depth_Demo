#include "Depth_Pixel.h"
#include <fstream>

using namespace std;


bool depth_pixel::input_Dmap(string base)
{
	ifstream inputFile;
	inputFile.open(base.c_str());
	if (!inputFile)
		return false;
	while(!inputFile.eof())
	{
		Dpixel tmp;
		int num;
		float idepth;
		inputFile>>num>>idepth>>tmp.depth_var;
		if (idepth < 0) continue;
		tmp.y = num /width;
		tmp.x = num - (tmp.y*width);
		if (idepth> 1.5 || idepth < 0.5) continue;
		tmp.depth = 1.0 /idepth;
		float depth4 = tmp.depth * tmp.depth;
		if(tmp.depth_var * depth4 > 1) continue;

		Dmap.push_back(tmp);

	}
	pixelNum = Dmap.size();
	inputFile.close();
	return true;
}

bool depth_pixel::inputPose(string based)
{
	ifstream inputFile;
	inputFile.open(based.c_str());
	if (!inputFile)
		return false;
	for (int j=0;j<4;j++)
				for (int k=0;k<4;k++)
					inputFile>>Pose.at<float>(j,k);

	return true;
}


Mat depth_pixel::getWorldposition(int pixelNum)
{

	Mat tem_Point(4,1,CV_32FC1);
	float depth =Dmap[pixelNum].depth;
	tem_Point.at<float>(3) = 1.0;
	tem_Point.at<float>(2) = depth;
	tem_Point.at<float>(1) = depth*(Dmap[pixelNum].y-cy)/fy;
	tem_Point.at<float>(0) = depth*(Dmap[pixelNum].x-cx)/fx;
	Mat worldPoint(4,1,CV_32FC1);
	//return worldPoint = Pose.inv(DECOMP_SVD) * tem_Point;
	return worldPoint = Pose * tem_Point;

}
depth_pixel::depth_pixel()
{
	width = 640;
	height = 480;
	pixelNum = 0;	
	id = 0;
	
	fx = 0.39738586545*float(width);
	fy = 0.78319662809*float(height);
	cx = 0.41778421402*float(width);
	cy = 0.48249810536*float(height);

	Pose.create(4,4,CV_32F);
}

depth_pixel::~depth_pixel()
{
	Pose.release();
	Dmap.clear();
}
