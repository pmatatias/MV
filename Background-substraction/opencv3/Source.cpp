#include "opencv2\highgui\highgui.hpp"
#include "opencv2\imgproc\imgproc.hpp"
#include "opencv2\features2d\features2d.hpp"

#include <iostream>
#include <stdio.h>
#include <stdlib.h>

using namespace cv;
using namespace std;


void main()
{
	namedWindow("original", WINDOW_AUTOSIZE);
	namedWindow("result", WINDOW_AUTOSIZE);
	VideoCapture cap("D:/Semester_7/MV/video/mvcu.mp4"); //
	Mat frame; Mat gray;  Mat It0; Mat It1; Mat hasil;
	int cnt = 0;
	Scalar(rata2);
	Mat element = getStructuringElement(MORPH_RECT, Size(7, 7), Point(-1, -1));
	for (;;)
	{
		cap >> frame;   // cap itu video (dari divais) , kemudian ditarik menjadi gambar diberi nama frame
		if (frame.empty())break;
		cvtColor(frame, gray, COLOR_BGR2GRAY);
		resize(gray, gray,Size(640, 480),0.0,0.0,1);
		GaussianBlur(gray, gray, Size(5, 5), 0, 0);
		
		//langkah 1
		It0 = gray.clone(); 
		if (cnt == 0){It1 = It0.clone(); cnt = 1; hasil = It0.clone(); // it1 dan hasil di copy isi dari it0 supaya mempunyai ukuran(tidak NULL)
		}
		//langkah 2
		cv::absdiff (It0, It1, hasil); //  Hasil = |it0-it1|
		//langkah 3
		It1 = It0.clone();
		rata2 = mean(hasil);
		//Canny(hasil, hasil, 0, 100, 3, false);
		printf("nilai rata2 = %f", rata2.val[0]); // val[0] karena mean nya hanya ada 1 nilai . array nya cuma 1 

		if (rata2.val[0] > 2) printf("ada gerakan");
		printf("\n");


		// threshold
		threshold(hasil, hasil, 30, 255, 0);
		dilate(hasil, hasil, element);
		resize(frame, frame, Size(640, 480), 0.0, 0.0, 1);

		imshow("original", frame); //menampilkan frame di window yang bernana original
		imshow("result", hasil);


		
		if (waitKey(1) == 27)break; // menunggu press any key

	}
}