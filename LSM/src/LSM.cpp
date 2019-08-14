
extern "C"
{
#include "lsd.h"
};

#include "LSM.h"
#include <stdio.h>
#include <highgui.h>
#include <cv.h>
#include <math.h>
#include <iostream>
#include "IO.h"
#include "LineMatching.h"
#include "PartiallyRecoverConnectivity.h"

// #include "CmdLine.h"

using namespace std;
using namespace cv;

LSM::LSM() {
}

LSM::~LSM() {
}

LSM::LSM(char* imgName1, char* imgName2, char* providedLines1, char* providedLines2, bool isProvideLines): mImgName1(imgName1), mImgName2(imgName2), 
mProvidedLines1(providedLines1), mProvidedLines2(providedLines2), mIsProvideLines(isProvideLines){}

void LSM::detectLine(char* imfile, Mat &mLines, float minLineLength)
{
	IplImage* im = cvLoadImage(imfile, CV_LOAD_IMAGE_GRAYSCALE);
	image_double image = new_image_double(im->width, im->height);
	unsigned char* im_src = (unsigned char*)im->imageData;
	int xsize = image->xsize;
	int ysize = image->ysize;
	int y, x;
	for (y = 0; y < ysize; y++)
	{
		for (x = 0; x < xsize; x++)
		{
			image->data[y * xsize + x] = im_src[y * im->widthStep + x];
		}
	}
	ntuple_list detected_lines = lsd(image);
	free_image_double(image);

	int nLines = detected_lines->size;
	int nCount = 0;
	int i, j;
	int dim = detected_lines->dim;

	Mat tmat;
	for (i = 0; i < nLines; i++)
	{
		float a1 = detected_lines->values[i*dim + 0];
		float a2 = detected_lines->values[i*dim + 1];
		float a3 = detected_lines->values[i*dim + 2];
		float a4 = detected_lines->values[i*dim + 3];
		if (sqrt((a3 - a1)*(a3 - a1) + (a4 - a2)*(a4 - a2)) >= minLineLength)
		{
			tmat = (Mat_<float>(1, 4) << a1, a2, a3, a4);
			mLines.push_back(tmat);
		}
	}
}

void LSM::drawDectectedLine(Mat Img, Mat mLines, string imgName)
{
	int color[6][3];
	color[0][0] = 255; color[0][1] = 0;  color[0][2] = 0;
	color[1][0] = 0;  color[1][1] = 255; color[1][2] = 0;
	color[2][0] = 0;  color[2][1] = 0;  color[2][2] = 255;
	color[3][0] = 255; color[3][1] = 255; color[3][2] = 0;
	color[4][0] = 0; color[4][1] = 255; color[4][2] = 255;
	color[5][0] = 255; color[5][1] = 0; color[5][2] = 255;

	int nline = mLines.rows;
	for (int i = 0; i < nline; i++)
	{
		float *pdat = mLines.ptr<float>(i);
		int color_num = rand() % 6;
		int R = color[color_num][0];
		int G = color[color_num][1];
		int B = color[color_num][2];

		Point2f spt1 = Point2f(pdat[0], pdat[1]);
		Point2f ept1 = Point2f(pdat[2], pdat[3]);
		line(Img, spt1, ept1, cvScalar(R, G, B), 2);
	}
	
	//imshow(imgName, Img);
	////	imwrite(imgName, Img);
	//waitKey(20);
}

void LSM::drawPartiallyConnectedLine(Mat Img, Mat mLines, string imgName, Mat fans)
{
	int color[6][3];
	color[0][0] = 255; color[0][1] = 0;  color[0][2] = 0;
	color[1][0] = 0;  color[1][1] = 255; color[1][2] = 0;
	color[2][0] = 0;  color[2][1] = 0;  color[2][2] = 255;
	color[3][0] = 255; color[3][1] = 255; color[3][2] = 0;
	color[4][0] = 0; color[4][1] = 255; color[4][2] = 255;
	color[5][0] = 255; color[5][1] = 0; color[5][2] = 255;

	int nline = mLines.rows;
	for (int i = 0; i < nline; i++)
	{
		float *pdat = mLines.ptr<float>(i);
		int color_num = rand() % 6;
		int R = color[color_num][0];
		int G = color[color_num][1];
		int B = color[color_num][2];

		Point2f spt1 = Point2f(pdat[0], pdat[1]);
		Point2f ept1 = Point2f(pdat[2], pdat[3]);
		line(Img, spt1, ept1, cvScalar(R, G, B), 2);
	}
	int nFan = fans.rows;
	for (int i = 0; i < nFan; i++)
	{
		int color_num = rand() % 6;
		int R = color[color_num][0];
		int G = color[color_num][1];
		int B = color[color_num][2];
		float *pdat = fans.ptr<float>(i);
		circle(Img, Point2f(pdat[0], pdat[1]), 5, cvScalar(R, G, B), -1);
	}

	//imshow(imgName, Img);
	//waitKey(20);
}

Mat* LSM::lsm(Mat &given_lines1, Mat&given_lines2) {
	Mat colorImg1 = imread(mImgName1, 3);
	Mat colorImg2 = imread(mImgName2, 3);
	Mat img1, img2;
	cvtColor(colorImg1, img1, CV_RGB2GRAY);
	img1.convertTo(img1, CV_32FC1);
	cvtColor(colorImg2, img2, CV_RGB2GRAY);
	img2.convertTo(img2, CV_32FC1);

	Mat nodes1, nodes2, lines1, lines2;
	Mat keyPoints1, keyPoints2;
	if (mIsProvideLines)
	{
		given_lines1.copyTo(lines1);
		given_lines2.copyTo(lines2);

		if (minLineLength != 0)
		{
			Mat tlines1, tlines2;
			Mat ta = lines1.row(2) - lines1.row(0);
			Mat tb = lines1.row(3) - lines1.row(1);
			Mat tc = ta.mul(ta) + tb.mul(tb);
			float *pdat = tc.ptr<float>(0);
			int nline1 = lines1.rows;
			for (int i = 0; i < nline1; i++)
			{
				if (pdat[i] < minLineLength)
					continue;

				tlines1.push_back(lines1.row(i));
			}
			lines1 = tlines1;

			ta = lines2.row(2) - lines2.row(0);
			tb = lines2.row(3) - lines2.row(1);
			tc = ta.mul(ta) + tb.mul(tb);
			pdat = tc.ptr<float>(0);
			int nline2 = lines2.rows;
			for (int i = 0; i < nline2; i++)
			{
				if (pdat[i] < minLineLength)
					continue;

				tlines2.push_back(lines2.row(i));
			}
			lines2 = tlines2;
		}
	}
	else
	{
		detectLine(mImgName1, lines1, minLineLength);
		detectLine(mImgName2, lines2, minLineLength);
	}
	if (isVerbose)
	{
		drawDectectedLine(colorImg1.clone(), lines1, "Detected lines in the first image");
		drawDectectedLine(colorImg2.clone(), lines2, "Detected lines in the second image");
	}

	if (isProvideJunc)
	{
		CIO io;
		io.loadData(providedJunc1, nodes1);
		io.loadData(providedJunc2, nodes2);
	}
	else
	{
		CPartiallyRecoverConnectivity  p1(lines1, expandWidth, nodes1, colorImg1, fanThr);
		CPartiallyRecoverConnectivity  p2(lines2, expandWidth, nodes2, colorImg2, fanThr);
	}
	cout << "Line segments detected in the two images: (" << lines1.rows << "," << lines2.rows << ")" << endl;

	if (isVerbose)
	{
		// cout<<colorImg1.rows<<endl;
		// cout<<colorImg2.rows<<endl;

		drawPartiallyConnectedLine(colorImg1.clone(), lines1, "Detected lines and generated junctions in the first image", nodes1);
		drawPartiallyConnectedLine(colorImg2.clone(), lines2, "Detected lines and generated junctions in the second image", nodes2);
	}
	// waitKey();
	// return 0;

	//waitKey();

	CLineMatching *pLineMatching = new CLineMatching(img1, lines1, nodes1, img2, lines2, nodes2, colorImg1, colorImg2, mlines,
		isVerbose, isBuildingImagePyramids, nAvgDesDist, isProvideJunc, isTwoLineHomography,
		nOctave, nOctaveLayer, desDistThrEpi, desDistThrProg, fmatThr, hmatThr, nNeighborPts, nEnterGroup,
		rotAngThr, sameSideRatio, regionHeight, junctionDistThr, intensityProfileWidth, radiusPointMatchUnique, difAngThr,
		rcircle, truncateThr, fanThr, outLineMatchesFileName);

	CIO *pIO = new CIO;
	pIO->writeData(outLineMatchesFileName, mlines);

	delete pIO;
	pIO = NULL;
	delete pLineMatching;
	pLineMatching = NULL;
	
	return &mlines;
}


//int main(int argc, char** argv)
//{
//	/*char*  imgName1 = argv[1];
//	char*  imgName2 = argv[2];*/
//
//	char*  imgName1 = "D:/Research/afm_cvpr2019-master/data/near_dorm/images/11.jpg";
//	char*  imgName2 = "D:/Research/afm_cvpr2019-master/data/near_dorm/images/12.jpg";
//	bool showViz = true;
//
//	//char*  imgName2 = "D:/Research/afm_cvpr2019-master/data/n5/n5_night/images/1.jpg";
//
//	string outLineMatchesFileName = "result.txt";
//	if (argc == 4)
//		outLineMatchesFileName = argv[3];
//
//	// show intermediate results or not
//	bool  isVerbose = false;
//
//	// whether build image pyramids to deal with scale change betwen images to be matched;
//	// Set false when you know there is not scale or only slight scale change between the images,
//	// This can tremendously accelerate the matching process.
//	bool isBuildingImagePyramids = true;
//
//	// Ture, if load junctions generated by yourself; otherwise false, the program will generate junction by itself
//	bool isProvideJunc = false;
//	// the paths of your junction files, if load junctions generated by yourself.
//	char* providedJunc1;
//	char* providedJunc2;
//
//	// Ture, if load line segments generated by yourself; otherwise false, the program will use LSD to generate segments
//	bool isProvideLines = true;
//	// the paths of your segment files, if load segments generated by yourself.
//	char* providedLines1 = "D:/Research/afm_cvpr2019-master/experiments/atrous/results/near_dorm/10_lines.txt";
//	char* providedLines2 = "D:/Research/afm_cvpr2019-master/experiments/atrous/results/near_dorm/11_lines.txt";
//
//	//char* providedLines2 = "D:/Research/afm_cvpr2019-master/experiments/atrous/results/n5/n5_night/0_lines.txt";
//
//	//// Ture, if load line segments generated by yourself; otherwise false, the program will use LSD to generate segments
//	//bool isProvideLines = false;
//	//// the paths of your segment files, if load segments generated by yourself.
//	//char* providedLines1;
//	//char* providedLines2;
//
//
//	// the length requirement for the detected segments used for matching 
//	float minLineLength = 0;
//
//	// the threshold controling the size of thte impact region of a segment, see paper for detail
//	float  expandWidth = 20.0;
//
//	int   nAvgDesDist = 2;
//	bool isScaleChangeExisting = false;
//
//
//	bool isTwoLineHomography = true;
//
//	// the number of octaves used in the image pyramids
//	int nOctave = 4;
//	// the number of layers in the each octave
//	int nOctaveLayer = 2;
//
//
//	float desDistThrEpi = 0.4;
//	float desDistThrProg = 0.5;
//
//	float  fmatThr = 3.0;
//	float  hmatThr = 5.0;
//
//	int   nNeighborPts = 10;
//	int  nEnterGroup = 4;
//	float rotAngThr = 30 * CV_PI / 180;
//	float sameSideRatio = 0.8;
//	float regionHeight = 4;
//	float junctionDistThr = 5.0;
//
//	float intensityProfileWidth = 3;
//	float radiusPointMatchUnique = 0;
//	float difAngThr = 20 * CV_PI / 180;
//	float rcircle = 10;
//	float truncateThr = 0.3;
//	float fanThr = 1.0 / 4 * CV_PI;
//
//
//	Mat colorImg1 = imread(imgName1, 3);
//	Mat colorImg2 = imread(imgName2, 3);
//	Mat img1, img2;
//	cvtColor(colorImg1, img1, CV_RGB2GRAY);
//	img1.convertTo(img1, CV_32FC1);
//	cvtColor(colorImg2, img2, CV_RGB2GRAY);
//	img2.convertTo(img2, CV_32FC1);
//
//	Mat nodes1, nodes2, lines1, lines2;
//	Mat keyPoints1, keyPoints2;
//	if (isProvideLines)
//	{
//		CIO io;
//		io.loadData(providedLines1, lines1);
//		io.loadData(providedLines2, lines2);
//
//		if (minLineLength != 0)
//		{
//			Mat tlines1, tlines2;
//			Mat ta = lines1.row(2) - lines1.row(0);
//			Mat tb = lines1.row(3) - lines1.row(1);
//			Mat tc = ta.mul(ta) + tb.mul(tb);
//			float *pdat = tc.ptr<float>(0);
//			int nline1 = lines1.rows;
//			for (int i = 0; i < nline1; i++)
//			{
//				if (pdat[i] < minLineLength)
//					continue;
//
//				tlines1.push_back(lines1.row(i));
//			}
//			lines1 = tlines1;
//
//			ta = lines2.row(2) - lines2.row(0);
//			tb = lines2.row(3) - lines2.row(1);
//			tc = ta.mul(ta) + tb.mul(tb);
//			pdat = tc.ptr<float>(0);
//			int nline2 = lines2.rows;
//			for (int i = 0; i < nline2; i++)
//			{
//				if (pdat[i] < minLineLength)
//					continue;
//
//				tlines2.push_back(lines2.row(i));
//			}
//			lines2 = tlines2;
//		}
//	}
//	else
//	{
//		detectLine(imgName1, lines1, minLineLength);
//		detectLine(imgName2, lines2, minLineLength);
//	}
//	if (isVerbose)
//	{
//		drawDectectedLine(colorImg1.clone(), lines1, "Detected lines in the first image");
//		drawDectectedLine(colorImg2.clone(), lines2, "Detected lines in the second image");
//	}
//
//
//	if (isProvideJunc)
//	{
//		CIO io;
//		io.loadData(providedJunc1, nodes1);
//		io.loadData(providedJunc2, nodes2);
//	}
//	else
//	{
//		CPartiallyRecoverConnectivity  p1(lines1, expandWidth, nodes1, colorImg1, fanThr);
//		CPartiallyRecoverConnectivity  p2(lines2, expandWidth, nodes2, colorImg2, fanThr);
//	}
//	cout << "Line segments detected in the two images: (" << lines1.rows << "," << lines2.rows << ")" << endl;
//
//	if (isVerbose)
//	{
//		// cout<<colorImg1.rows<<endl;
//		// cout<<colorImg2.rows<<endl;
//
//		drawPartiallyConnectedLine(colorImg1.clone(), lines1, "Detected lines and generated junctions in the first image", nodes1);
//		drawPartiallyConnectedLine(colorImg2.clone(), lines2, "Detected lines and generated junctions in the second image", nodes2);
//	}
//	// waitKey();
//	// return 0;
//
//	//waitKey();
//
//	Mat mlines;
//	CLineMatching *pLineMatching = new CLineMatching(img1, lines1, nodes1, img2, lines2, nodes2, colorImg1, colorImg2, mlines,
//		isVerbose, isBuildingImagePyramids, nAvgDesDist, isProvideJunc, isTwoLineHomography,
//		nOctave, nOctaveLayer, desDistThrEpi, desDistThrProg, fmatThr, hmatThr, nNeighborPts, nEnterGroup,
//		rotAngThr, sameSideRatio, regionHeight, junctionDistThr, intensityProfileWidth, radiusPointMatchUnique, difAngThr,
//		rcircle, truncateThr, fanThr, outLineMatchesFileName);
//
//	CIO *pIO = new CIO;
//	pIO->writeData(outLineMatchesFileName, mlines);
//
//	delete pIO;
//	pIO = NULL;
//	delete pLineMatching;
//	pLineMatching = NULL;
//	return 0;
//}