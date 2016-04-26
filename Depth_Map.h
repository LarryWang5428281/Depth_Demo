#include <opencv2/opencv.hpp>
using namespace cv;
#include "Depth_Pixel.h"

struct worldPoint
{
	float X,Y,Z;
};


class Depth_Map
{

public:

	void Input_Pose_Depth(string base);
	Depth_Map();
	~Depth_Map();
	int mapNum;
	int getPixels(int num);
	Mat getPoint(int mapNum, int pixelNum);
	void generateMap(int mapNum); 
	worldPoint getWorldPoint(int mapNum,int pixelNum);
	//void createList(int mapNum);
	void compRansac(int mapNum);
	float avx,avy,avz;
	vector<Mat> Ransac_plane;

private:
	vector<depth_pixel> depth_map;
	vector<vector<worldPoint>  > world_map;

};
