#include "Depth_Map.h"
#include <fstream>
#include <string>
#include <GL/glut.h>


//using namespace std;

void Depth_Map::Input_Pose_Depth(string base)
{
	string Daddress = base + "/Depth/";
	string Paddress = base + "/Pose/";

	for (int i = 0;i<100;i++)
	{
		std::ifstream PFile;
		std::stringstream ss;
		ss<<i;
		std::string s = ss.str();
		string Dtmp = Daddress + s + ".txt";
		string Ptmp = Paddress + s + ".txt";
		PFile.open(Ptmp.c_str());
		if (PFile)
		{
			PFile.close();
			depth_pixel tmp;
			tmp.id = i;
			if (tmp.inputPose(Ptmp))
				if(tmp.input_Dmap(Dtmp))
				{
					depth_map.push_back(tmp);
					vector<worldPoint> tem;
					Mat t(3,1,CV_32FC1);
					world_map.push_back(tem);
					Ransac_plane.push_back(t);
					printf("Success input file:%d\n",tmp.id);
				}



			mapNum++;
		}
		else
		{
			PFile.close();
		}

	}

}

int Depth_Map::getPixels(int num)
{
	return depth_map[num].pixelNum;
}

Mat Depth_Map::getPoint(int mapNum,int pixelNum)
{
	return depth_map[mapNum].getWorldposition(pixelNum);
}
Depth_Map::Depth_Map()
{
	avx = 0;
	avy = 0;
	avz = 0;
	mapNum = 0;
}

Depth_Map::~Depth_Map()
{
	depth_map.clear();
	delete &mapNum;
}

void Depth_Map::generateMap(int mapNum)
{
	float tavx = 0,tavy = 0 ,tavz = 0;

	for (int i = 0;i<this->getPixels(mapNum);i++)
	{
		Mat P(4,1,CV_32FC1);
		P = this->getPoint(mapNum,i);
		worldPoint tmp;
		tmp.X = P.at<float>(0);
		tmp.Y = P.at<float>(1);
		tmp.Z = P.at<float>(2);
		tavx += tmp.X;
		tavy += tmp.Y;
		tavz += tmp.Z;
		world_map[mapNum].push_back(tmp);
	}
	tavx =tavx / getPixels(mapNum);
	tavy =tavy / getPixels(mapNum);
	tavz =tavz / getPixels(mapNum);
	if (avx == 0 && avy == 0 && avz == 0)
	{
		avx = tavx;
		avy = tavy;
		avz = tavz;
	}
	else
	{
		avx = (avx +tavx)/2;
		avy = (avy+tavy)/2;
		avz = (avz+tavz)/2;
	}

}
/*void Depth_Map::createList(int mapNum)
{
	unsigned int ListName = mapNum;
	glNewList (ListName, GL_COMPILE);
	for (int i = 0;i < this->getPixels(mapNum);i++)
	{
		  worldPoint tmp = this->getWorldPoint(mapNum,i);
		  float var =  -5 * log10(tmp.Z);
				var = var*255;
		  if(var > 255)
				var = 255;
		  if(var < 0)
			  glColor3f(0,0, 1.0);
		  else
			  glColor3f((255-var)/255.0,var/255.0, 0);

		glBegin(GL_POINTS);
			glVertex3f(tmp.X,tmp.Y,tmp.Z);
		glEnd();
	}
	glEndList ();
}*/

void Depth_Map::compRansac(int mapNum)
{
	float e[] = {1,1,1};
	Mat I(3,1,CV_32FC1,e);

	printf("Compute Ransac plane!\n");
	float besterr = 99999;
	float mythresh = 1.1;
	int bestNum = 0;
	Mat ransac(3,1,CV_32FC1);

	for (int i=0;i<500;i++)
	{
		int p1,p2,p3;
		//printf("irr %d\n",i);

		do{
			 srand (time(NULL));
			 p1 = rand()% (this->getPixels(mapNum)/3);
			 p2 = rand()% (this->getPixels(mapNum)/3*2);
			 p3 = rand()% this->getPixels(mapNum);
		//	 printf("%d %d %d \n",p1,p2,p3);
		}while(p1 == p2 || p2 == p3|| p3 == p1);

		float m[] = {this->getWorldPoint(mapNum,p1).X,this->getWorldPoint(mapNum,p1).Y,this->getWorldPoint(mapNum,p1).Z,
					 this->getWorldPoint(mapNum,p2).X,this->getWorldPoint(mapNum,p2).Y,this->getWorldPoint(mapNum,p2).Z,
					 this->getWorldPoint(mapNum,p3).X,this->getWorldPoint(mapNum,p3).Y,this->getWorldPoint(mapNum,p3).Z,
					};

		Mat M(3,3,CV_32FC1,m);
		Mat plane = M.inv(DECOMP_SVD)*I;
		int goodNum = 0;
		float total_err = 0.0;

		//printf("irr %d\n",i);

		for (int j=0;j<this->getPixels(mapNum);j++)
		{
			if (j != p1 && j!= p2 && j != p3)
			{
				float err = plane.at<float>(0) * this->getWorldPoint(mapNum,j).X
							+plane.at<float>(1) * this->getWorldPoint(mapNum,j).Y
							+plane.at<float>(2) * this->getWorldPoint(mapNum,j).Z;
				if (err*err < mythresh)
				{
					goodNum ++;
					total_err+= err*err;
				}
			}
		}

		if (besterr >=  total_err)
		{
			bestNum = goodNum;
			besterr = total_err;
			ransac = plane.clone();
		}
	}
	printf("Final err %.4f\n",besterr);
	Ransac_plane[mapNum] = ransac.clone();
}

worldPoint Depth_Map::getWorldPoint(int mapNum,int pixelNum)
{
	return world_map[mapNum][pixelNum];
}
