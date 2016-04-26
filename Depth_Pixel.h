#include <opencv2/opencv.hpp>
using namespace cv;

struct Dpixel
{
	float depth,depth_var;
	int x,y;
};

class depth_pixel
{
public:
	bool input_Dmap(string basd="");
	depth_pixel();
	~depth_pixel();
	bool inputPose(string based);
	int pixelNum,id;
	Mat getWorldposition(int pixelNum);

private:
	vector<Dpixel> Dmap;
	Mat Pose;
	float fx,fy,cx,cy;
	int width,height;
};
