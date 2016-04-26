#include <iostream>
#include <fstream>
#include <string>
#include <opencv2/opencv.hpp>
#define GLUT_DISABLE_ATEXIT_HACK
#include <GL/glut.h>

#include "Depth_Map.h"
#include <math.h>
#include <stdio.h> 

using namespace cv;
using namespace std;

Depth_Map* worldMap;
//int LastMap = 1;

static float c=3.1415/180.0f;
static int du=90,oldmy=-1,oldmx=-1;
static float r=1.5f,h=0.0f;

void reshape(int w,int h)
{
		glViewport( 0, 0, w, h );

		glMatrixMode(GL_PROJECTION);
		glLoadIdentity();
		glOrtho(-2.0f,2.0f,-2.0f,2.0f,-2.0f,2.0f);
		glMatrixMode(GL_MODELVIEW);
		glLoadIdentity();
}

void Mouse(int button, int state, int x, int y)
{  
    if(state==GLUT_DOWN)
        oldmx=x,oldmy=y;  
}  
void onMouseMove(int x,int y)
{  
    //printf("%d\n",du);  
    du+=0.5*(x-oldmx);
    h +=0.03f*(y-oldmy);
    if(h>1.0f) h=1.0f;
    else if(h<-1.0f) h=-1.0f;  
    oldmx=x,oldmy=y;
}  


void display()
{
	glClear(GL_COLOR_BUFFER_BIT|GL_DEPTH_BUFFER_BIT);				// Reset The Modelview Matrix

	glLoadIdentity(); 
	glOrtho(worldMap->avx-1.0f,worldMap->avx+1.0f,
			worldMap->avy-1.0f,worldMap->avy+1.0f,
			worldMap->avz-1.0f,worldMap->avz+1.0f);
	gluLookAt(r*cos(c*du), h, r*sin(c*du), worldMap->avx, worldMap->avy, worldMap->avz, 0, 1, 0);
	glPointSize(1.0f);

	glLineWidth(2);
	glColor3f(1.0, 1.0, 1.0);
	glBegin(GL_LINES);
	glVertex3f(0.0, 0.0, 0.0);
	glVertex3f(worldMap->Ransac_plane[1].at<float>(0),worldMap->Ransac_plane[1].at<float>(1),worldMap->Ransac_plane[1].at<float>(2));
	glEnd();


	//Draw the x,y,z axis
	glColor3f(1.0, 0.0, 0.0);
	glBegin(GL_LINES);
	glVertex3f(0.0, 0.0, 0.0);
	glVertex3f(3.0, 0.0, 0.0);
	glEnd();

	glColor3f(0.0, 1.0, 0.0);
	glBegin(GL_LINES);
	glVertex3f(0.0, 0.0, 0.0);
	glVertex3f(0.0, 3.0, 0.0);
	glEnd();

	glColor3f(0.0, 0.0, 1.0);
	glBegin(GL_LINES);
	glVertex3f(0.0, 0.0, 0.0);
	glVertex3f(0.0, 0.0, 3.0);
	glEnd();

	for (int i = 0; i<worldMap->mapNum;i++)
	{
	/*	unsigned int ListName = i;
		glPushMatrix();
			glCallList(ListName);
		glPopMatrix();
		*/
		for (int j=0;j<worldMap->getPixels(i);j++)
		{
				if (j % 10 > 4)
				{
					glPushMatrix();		//Draw a voxel

						worldPoint tmp = worldMap->getWorldPoint(i,j);

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

					glPopMatrix();
				}
		}

	}
	glutSwapBuffers();
}

void keyboard(unsigned char key, int x, int y)
{
	//printf("%d\n",key);
	switch(key)
	{
	case 'W': h += 0.3; break;
	case 'S': h -= 0.3; break;
	case 'A': du += 0.3; break;
	case 'D': du -= 0.3; break;
	}
	glutPostRedisplay();
}

void init()
{
	glEnable(GL_DEPTH_TEST);
	glClearColor(0.0f, 0.0f, 0.0f, 0.5f);							// Black Background
	glClearDepth(1.0f);												// Depth Buffer Setup
	glDepthFunc(GL_LEQUAL);											// The Type Of Depth Testing To Do

	for (int i=0;i<worldMap->mapNum;i++)
	{
		worldMap->generateMap(i);

		//worldMap->createList(i);
	}
	worldMap->compRansac(1);

}
int main(int argc, char* argv[])
{
	worldMap = new Depth_Map;
	string argv_str(argv[0]);
	string base = argv_str.substr(0, argv_str.find_last_of("/"));
	worldMap->Input_Pose_Depth(base);
	init();
	glutInit(&argc, argv);
	glutInitDisplayMode(GLUT_DOUBLE|GLUT_RGB|GLUT_DEPTH);
	glutCreateWindow("Model");
	glutInitWindowSize(1280, 960);
	glutMouseFunc(Mouse);  
	glutIdleFunc(display);
    glutMotionFunc(onMouseMove);  
	glutReshapeFunc(reshape);
	glutDisplayFunc(display);
	glutKeyboardFunc(keyboard);
	//init();
	glutMainLoop();
	system("pause");
}

