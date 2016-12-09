#include<iostream>
#include<opencv2\highgui\highgui.hpp>
#include<opencv2\imgproc\imgproc.hpp>
#include<opencv2\core\core.hpp>
#include<opencv2\video\background_segm.hpp>
#include<Windows.h>
using namespace cv;
using namespace std;

///function prototypes
void on_trackbar(int, void*);
void createTrackbars();
void morphit(Mat &img);
void blurthresh(Mat &img);
void toggle(int);
void showconvex(Mat &img, Mat &frame);
void condefects(vector<Vec4i> convexityDefectsSet, vector<Point> mycontour, Mat &frame);
Mat getroiframe(Mat image);
void workOnDefects(Mat &frame, vector<Point>&fingertips, vector<Point>&deptharr);
int anglebetween(Point, Point);
void drawangle(Mat &frame, Point p1, Point p2, int ang);
///function prototypes




//max number of objects to be detected in frame
const int MAX_NUM_OBJECTS = 50;
//default capture width and height
const int FRAME_WIDTH = 640;
const int FRAME_HEIGHT = 480;
//minimum and maximum object area
const int MIN_OBJECT_AREA = 20 * 20;
const int MAX_OBJECT_AREA = FRAME_HEIGHT*FRAME_WIDTH / 1.5;
//x and y values for the location of the object
void drawObject(int x, int y, Mat &frame);
string intToString(int number);
void trackFilteredObject(int &x, int &y, Mat threshold, Mat &cameraFeed);

Point middle;

int H_MIN = 0;
int H_MAX = 255;
int S_MIN = 0;
int S_MAX = 255;
int V_MIN = 0;
int V_MAX = 255;

int kerode = 1;
int kdilate = 1;
int kblur = 1;
int threshval = 0;

bool domorph = false;
bool doblurthresh = false;
bool showchangedframe = false;
bool trackobjstatus = false;
bool showconvexhull = false;
bool showcondefects = false;
bool showmyhull = false;
bool krokaam = false;
int main(void)
{

	createTrackbars();
	on_trackbar(0, 0);

	int x, y;
	Mat frame, hsvframe, rangeframe;
	int key;
	VideoCapture cap(0);
	while ((key = waitKey(30)) != 27)
	{
		toggle(key);
		cap >> frame;
		flip(frame, frame, 180);
		cvtColor(frame, hsvframe, COLOR_BGR2HSV);

		inRange(hsvframe, Scalar(H_MIN, S_MIN, V_MIN), Scalar(H_MAX, S_MAX, V_MAX), rangeframe);

		if (domorph)
			morphit(rangeframe);

		if (doblurthresh)
			blurthresh(rangeframe);
		if (trackobjstatus)
			trackFilteredObject(x, y, rangeframe, frame);
		if (showconvexhull)
			showconvex(rangeframe, frame);

		if (showchangedframe)
			imshow("Camera", frame);
		else
			imshow("Camera", rangeframe);

	}

}


void on_trackbar(int, void*)
{//This function gets called whenever a
 // trackbar position is changed
	if (kerode == 0)
		kerode = 1;
	if (kdilate == 0)
		kdilate = 1;
	if (kblur == 0)
		kblur = 1;
}

void showconvex(Mat &img, Mat &frame)
{

	int largest_area = 0;
	int largest_contour_index = 0;
	vector<vector<Point> > contours;
	vector<Vec4i> hierarchy;



	findContours(img, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE);
	/// Find the convex hull object for each contour
	vector<vector<Point> >hull(contours.size());
	vector<vector<int> >inthull(contours.size());
	vector<vector<Vec4i> >defects(contours.size());
	for (int i = 0; i < contours.size(); i++)
	{
		convexHull(Mat(contours[i]), hull[i], false);
		convexHull(Mat(contours[i]), inthull[i], false);
		if (inthull[i].size()>3)
			convexityDefects(contours[i], inthull[i], defects[i]);
	}
	//find largest contour
	for (int i = 0; i< contours.size(); i++) // iterate through each contour. 
	{
		double a = contourArea(contours[i], false);  //  Find the area of contour
		if (a>largest_area)
		{
			largest_area = a;
			largest_contour_index = i;                //Store the index of largest contour
		}

	}

	//show contours of biggest and hull as well
	if (contours.size()>0)
	{  //checkforcontourarea function if error occur
		drawContours(frame, contours, largest_contour_index, CV_RGB(0, 255, 0), 2, 8, hierarchy); // Draw the largest contour using previously stored index.

																								  //draw hull as well
		if (showmyhull)
			drawContours(frame, hull, largest_contour_index, CV_RGB(0, 0, 255), 2, 8, hierarchy);
		if (showcondefects)
			condefects(defects[largest_contour_index], contours[largest_contour_index], frame);
	}

}//end of showconvex method

void createTrackbars()
{
	String trackbarWindowName = "TrackBars";
	namedWindow(trackbarWindowName, WINDOW_NORMAL);
	createTrackbar("H_MIN", trackbarWindowName, &H_MIN, H_MAX, on_trackbar);
	createTrackbar("H_MAX", trackbarWindowName, &H_MAX, H_MAX, on_trackbar);
	createTrackbar("S_MIN", trackbarWindowName, &S_MIN, S_MAX, on_trackbar);
	createTrackbar("S_MAX", trackbarWindowName, &S_MAX, S_MAX, on_trackbar);
	createTrackbar("V_MIN", trackbarWindowName, &V_MIN, V_MAX, on_trackbar);
	createTrackbar("V_MAX", trackbarWindowName, &V_MAX, V_MAX, on_trackbar);
	createTrackbar("Erode", trackbarWindowName, &kerode, 31, on_trackbar);
	createTrackbar("Dilate", trackbarWindowName, &kdilate, 31, on_trackbar);
	createTrackbar("Blur", trackbarWindowName, &kblur, 255, on_trackbar);
	createTrackbar("Thresh", trackbarWindowName, &threshval, 255, on_trackbar);

}
void morphit(Mat &img)
{
	erode(img, img, getStructuringElement(MORPH_RECT, Size(kerode, kerode)));
	dilate(img, img, getStructuringElement(MORPH_RECT, Size(kdilate, kdilate)));
}
void blurthresh(Mat &img)
{
	//medianBlur(img,img,kblur%2+3+kblur);
	blur(img, img, Size(kblur, kblur), Point(-1, -1), BORDER_DEFAULT);
	threshold(img, img, threshval, 255, THRESH_BINARY_INV);
}
void toggle(int key)
{

	//toggle line start
	if (key == 'm')
		domorph = !domorph;
	if (key == 'b')
		doblurthresh = !doblurthresh;
	if (key == 'r')
		showchangedframe = !showchangedframe;
	if (key == 't')
		trackobjstatus = !trackobjstatus;
	if (key == 'c')
		showconvexhull = !showconvexhull;
	if (key == 'd')
		showcondefects = !showcondefects;
	if (key == 'h')
		showmyhull = !showmyhull;
	if (key == 'k')
		krokaam = !krokaam;
	//toggle line end
}
void trackFilteredObject(int &x, int &y, Mat threshold, Mat &cameraFeed)
{

	Mat temp;
	threshold.copyTo(temp);
	//these two vectors needed for output of findContours
	vector< vector<Point> > contours;
	vector<Vec4i> hierarchy;
	//find contours of filtered image using openCV findContours function
	findContours(temp, contours, hierarchy, CV_RETR_CCOMP, CV_CHAIN_APPROX_SIMPLE);
	//use moments method to find our filtered object
	double refArea = 0;
	bool objectFound = false;
	if (hierarchy.size() > 0) {
		int numObjects = hierarchy.size();
		//if number of objects greater than MAX_NUM_OBJECTS we have a noisy filter
		if (numObjects < MAX_NUM_OBJECTS) {
			for (int index = 0; index >= 0; index = hierarchy[index][0]) {

				Moments moment = moments((cv::Mat)contours[index]);
				double area = moment.m00;

				//if the area is less than 20 px by 20px then it is probably just noise
				//if the area is the same as the 3/2 of the image size, probably just a bad filter
				//we only want the object with the largest area so we safe a reference area each
				//iteration and compare it to the area in the next iteration.
				if (area > MIN_OBJECT_AREA && area<MAX_OBJECT_AREA && area>refArea) {
					x = moment.m10 / area;
					y = moment.m01 / area;
					objectFound = true;
					refArea = area;
				}
				else objectFound = false;


			}
			//let user know you found an object
			if (objectFound == true) {
				putText(cameraFeed, "Tracking Glove", Point(0, 50), 2, 1, Scalar(0, 255, 0), 2);
				//draw object location on screen
				drawObject(x, y, cameraFeed);
				middle.x = x;
				middle.y = y;
			}

		}
		else putText(cameraFeed, "TOO MUCH NOISE! ADJUST FILTER", Point(0, 50), 1, 2, Scalar(0, 0, 255), 2);
	}
}
///////////////////////------------------------------------------

void drawObject(int x, int y, Mat &frame)
{

	//use some of the openCV drawing functions to draw crosshairs
	//on your tracked image!

	//UPDATE:JUNE 18TH, 2013
	//added 'if' and 'else' statements to prevent
	//memory errors from writing off the screen (ie. (-25,-25) is not within the window!)

	circle(frame, Point(x, y), 20, Scalar(0, 255, 0), 2);
	if (y - 25>0)
		line(frame, Point(x, y), Point(x, y - 25), Scalar(0, 255, 0), 2);
	else line(frame, Point(x, y), Point(x, 0), Scalar(0, 255, 0), 2);
	if (y + 25<FRAME_HEIGHT)
		line(frame, Point(x, y), Point(x, y + 25), Scalar(0, 255, 0), 2);
	else line(frame, Point(x, y), Point(x, FRAME_HEIGHT), Scalar(0, 255, 0), 2);
	if (x - 25>0)
		line(frame, Point(x, y), Point(x - 25, y), Scalar(0, 255, 0), 2);
	else line(frame, Point(x, y), Point(0, y), Scalar(0, 255, 0), 2);
	if (x + 25<FRAME_WIDTH)
		line(frame, Point(x, y), Point(x + 25, y), Scalar(0, 255, 0), 2);
	else line(frame, Point(x, y), Point(FRAME_WIDTH, y), Scalar(0, 255, 0), 2);

	putText(frame, intToString(x) + "," + intToString(y), Point(x, y + 30), 1, 1, Scalar(0, 255, 0), 2);

}
//////////////////////---------------------------------------------
string intToString(int number) {


	std::stringstream ss;
	ss << number;
	return ss.str();
}
/////////////

void condefects(vector<Vec4i> convexityDefectsSet, vector<Point> mycontour, Mat &frame)
{
	Point2f mycenter;
	float myradius;
	float dis;
	vector<Point>fingertips, deptharr;
	for (int cDefIt = 0; cDefIt < convexityDefectsSet.size(); cDefIt++) {

		int startIdx = convexityDefectsSet[cDefIt].val[0]; Point ptStart(mycontour[startIdx]);

		int endIdx = convexityDefectsSet[cDefIt].val[1]; Point ptEnd(mycontour[endIdx]);

		int farIdx = convexityDefectsSet[cDefIt].val[2]; Point ptFar(mycontour[farIdx]);

		double depth = static_cast<double>(convexityDefectsSet[cDefIt].val[3]) / 256;
		//cout << "depth" << depth << endl;
		if (trackobjstatus)
		{
			if (depth > 20 && depth < 80 && ptStart.y < middle.y)
			{
				circle(frame, ptStart, 4, Scalar(100, 0, 255), 4);
				fingertips.push_back(ptStart);
			}
			if (depth > 230)
			{
				circle(frame, ptFar, 4, Scalar(0, 0, 255), 2, 8);
				deptharr.push_back(ptFar);
			}
		}//grab point only if trackobjstatus=true
	}
	//work will start from here
	if (krokaam)
		workOnDefects(frame, fingertips, deptharr);

}// condefects ends here

void workOnDefects(Mat &frame, vector<Point>&fingertips, vector<Point>&deptharr)
{
	bool indexfin = false;
	bool middlefin = false;
	String position;
	if (fingertips.size()<3)
	{
		for (int i = 0; i < fingertips.size(); i++)
		{
			int ang = anglebetween(middle, fingertips[i]);
			drawangle(frame, middle, fingertips[i], ang);
			line(frame, middle, fingertips[i], CV_RGB(255, 0, 0), 2);
			if (ang>45 && ang < 95)
				indexfin = true;
			if (ang>95 && ang < 140)
				middlefin = true;
		}

	}
	//finger check ends here
	if (indexfin == true && middlefin == true)
		position = "move";
	else if (indexfin == false && middlefin == false)
		position = "What's that";
	else if (indexfin == true)
		position = "Left Click";
	else if (middlefin == true)
		position = "Right Click";
	else position = "No Hand";

	//putText(frame,position,Point(400,50),1,2,CV_RGB(255,0,0),2,8);
	//do the real deed

	INPUT ip;
	ip.type = INPUT_MOUSE;
	static bool button = false;
	if (position == "move")
	{
		button = false;
		//SendInput(1, &ip, sizeof(INPUT));
		/*	ip.mi.dwFlags = MOUSEEVENTF_MOVE;
		ip.mi.dx = 1366/640*middle.x;
		ip.mi.dy = 768/480*middle.y;
		ip.mi.dwFlags = MOUSEEVENTF_ABSOLUTE;
		SendInput(1, &ip, sizeof(INPUT));*/
		putText(frame, position, Point(400, 50), 1, 2, CV_RGB(255, 0, 0), 2, 8);
		SetCursorPos(1366 / 400 * (middle.x - 100), 800 / 200 * (middle.y - 200));
	}
	else if (position == "Left Click")
	{
		if (!button)
		{
			putText(frame, position, Point(400, 50), 1, 2, CV_RGB(255, 0, 0), 2, 8);
			button = true;
			ip.mi.dwFlags = MOUSEEVENTF_LEFTDOWN;
			SendInput(1, &ip, sizeof(INPUT));
			ZeroMemory(&ip, sizeof(INPUT));
			ip.type = INPUT_MOUSE;
			ip.mi.dwFlags = MOUSEEVENTF_LEFTUP;
			SendInput(1, &ip, sizeof(INPUT));
			ZeroMemory(&ip, sizeof(INPUT));
		}
	}
	else if (position == "Right Click")
	{
		if (!button)
		{
			putText(frame, position, Point(400, 50), 1, 2, CV_RGB(255, 0, 0), 2, 8);
			button = true;
			ip.mi.dwFlags = MOUSEEVENTF_RIGHTDOWN;
			SendInput(1, &ip, sizeof(INPUT));
			ZeroMemory(&ip, sizeof(INPUT));
			ip.type = INPUT_MOUSE;
			ip.mi.dwFlags = MOUSEEVENTF_RIGHTUP;
			SendInput(1, &ip, sizeof(INPUT));
			ZeroMemory(&ip, sizeof(INPUT));
			ZeroMemory(&ip, sizeof(INPUT));
		}
	}
	else putText(frame, position, Point(400, 50), 1, 2, CV_RGB(255, 0, 0), 2, 8);

	//do the real deed

}

int anglebetween(Point p1, Point p2)
{
	int ang;
	if (p2.x == p1.x)
		return 0;
	ang = int(atan((p1.y - p2.y) / (p2.x - p1.x)) * 180 / 3.14);
	if (ang < 0)
		return ang + 180;
	else
		return ang;

}
void drawangle(Mat &frame, Point p1, Point p2, int ang)
{
	putText(frame, intToString(ang), Point((p1.x + p2.x) / 2, (p1.y + p2.y) / 2), 1, 1, CV_RGB(255, 255, 255), 2, 8);
}