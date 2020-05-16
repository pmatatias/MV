#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <iostream>
#include <sstream>
#include <string>
#include <iostream>
#include <vector>
#include <time.h>
#include "wiringPi.h"
#include "softPwm.h"


using namespace cv;
using namespace std;

#define In1	0
#define In2	1
#define En1 	2
#define In3	3
#define In4	4
#define En2 	5

//bagian PID
#define kp 1
#define ki 0
#define kd 1
//int P, I, D, prev_eror, outpid, outpidnew;
int error_p, error_d, output_pid;
int setpoint = 160;
int lastData = 0;
int data1 = 0;
int kec_kiri, kec_kanan; 
int Vo = 40;

float proportional, derivative;

const int max_value_H = 360 / 2;
const int max_value = 255;
const String window_capture_name = "Video Capture";
const String window_detection_name = "Object Detection";
const String window_hasil = "Video threshold";
//int low_H = 2, low_S = 0, low_V = 183;
//int H_max = 179, S_max = 17, V_max = 255;

//threshold di robotika
//int low_H = 0, low_S = 0, low_V = 153;
//int H_max = 179, S_max = 42, V_max = 217;

//int low_H = 0, low_S = 0, low_V = 15;
//int H_max = 179, S_max = 59, V_max = 255;

int low_H = 0, low_S = 0, low_V = 212; // sore a206
int H_max = 179, S_max = 34, V_max = 255;

//threshold di 202
//int low_H = 0, low_S = 0, low_V = 0;
//int H_max = 179, S_max = 255, V_max = 255;

//int high_H = 84, high_S = 255, high_V = 227;
int high_H = max_value_H, high_S = max_value, high_V = max_value;
int nilai_thr, atas, bawah;

int blob;
int x; int jlh_blob;


double MapValue(double a0, double a1, double b0, double b1, double a)
{
	return b0 + ((b1 - b0) * ((a - a0) / (a1 - a0)));
}

void jalan(int kec_kiri, int kec_kanan) {

	if (kec_kanan < 0) {
		digitalWrite(In1, LOW);
		digitalWrite(In2, HIGH);
	}
	else {
		digitalWrite(In1, HIGH);
		digitalWrite(In2, LOW);
	}
	if (kec_kiri < 0) {
		digitalWrite(In3, LOW);
		digitalWrite(In4, HIGH);
	}
	else {
		digitalWrite(In3, HIGH);
		digitalWrite(In4, LOW);
	}
	softPwmWrite(En1, abs(kec_kiri));
	softPwmWrite(En2, abs(kec_kanan));
}
int main()
{
	wiringPiSetup();
	if (wiringPiSetup()== 1){
		printf ("setup wiringPi salah");
		return 1;
	}
	pinMode(In1, OUTPUT);
	pinMode(In2, OUTPUT);
	pinMode(In3, OUTPUT);
	pinMode(In4, OUTPUT);
	softPwmCreate(En1,0,255);
	softPwmCreate(En2,0,255);


	Mat frame, frame_HSV, frame_threshold, frame_hasil;

	VideoCapture cap(0);

	cap.set(3, 320);
	cap.set(4, 240);

	clock_t time_prev = 0;

	for (;;) {
		cap >> frame;
		if (frame.empty())break;
		//resize(frame, frame, Size(200, 200), INTER_LINEAR);
		namedWindow(window_detection_name, 1);
		/*createTrackbar("Low H", window_detection_name, &low_H, 179);
		createTrackbar("High H", window_detection_name, &H_max, 179);
		createTrackbar("Low S", window_detection_name, &low_S, 255);
		createTrackbar("High S", window_detection_name, &S_max, 255);
		createTrackbar("Low V", window_detection_name, &low_V, 255);
		createTrackbar("High V", window_detection_name, &V_max, 255);*/

		cvtColor(frame, frame_HSV, COLOR_BGR2HSV);

		inRange(frame_HSV, Scalar(low_H, low_S, low_V), Scalar(H_max, S_max, V_max), frame_threshold);
		vector<vector<Point>> contours;
		vector<Vec4i> hierarchy;

		findContours(frame_threshold, contours, hierarchy, RETR_EXTERNAL, CHAIN_APPROX_NONE); //find contours  
		vector<double> areas(contours.size());

		//line(frame, Point(frame.cols / 2, 480), Point(frame.cols / 2, 0), Scalar(255, 128, 128), 3, 8);
		Point p;
		Moments m;

		for (int i = 0; i < contours.size(); i++)
		{
			areas[i] = contourArea(contours[i]);
			//drawContours(frame, contours, i, Scalar(0, 128, 255), 2, 8, hierarchy, 0, Point());

			if (areas[i] > 10) {

				m = moments(contours[i], true);
				//p(m.m10 / m.m00, m.m01 / m.m00);
				p.x = m.m10 / m.m00;
				p.y = m.m01 / m.m00;
				circle(frame, p, 5, Scalar(128, 0, 0), -1);

				data1 = p.x;
			}
			else
			{
				data1 = 0;
			}
		}

		clock_t time_now = clock();

		int selisih = time_now - time_prev;

		if (time_now - time_prev >= 200)
		{
			time_prev = time_now;

			error_p = setpoint - data1;
			proportional = kp * error_p;

			error_d = data1 - lastData;
			lastData = data1;

			derivative = kd * error_d;

			output_pid = proportional + derivative;

			if (output_pid > 35)
				output_pid = 35;
			else if (output_pid < -35)
				output_pid = -35;

		kec_kiri = Vo + output_pid;
		kec_kanan = Vo - output_pid;
		jalan(kec_kiri, kec_kanan);
		}


		//cout << selisih << endl;

		imshow("ok", frame_threshold);
		imshow("frame", frame);


		char key = (char)waitKey(30);
		if (key == 'q' || key == 27)
		{
			break;
		}
	}
	return 0;
}



