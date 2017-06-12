#include "getobj.h"

void getObj(Mat frame, Mat back)
{
    Mat imgDiff = abs(frame - back);

    double thresh = 10;
    double maxValue = 255;

    Mat mask;
    threshold(imgDiff, mask, thresh, maxValue, THRESH_BINARY);

    Mat detObjects = frame & mask;

//  threshold(src,dst, thresh, maxValue, THRESH_TOZERO);
    cv::namedWindow("diff", WINDOW_NORMAL);
    cv::imshow("diff", detObjects);
    cv::namedWindow("w2", WINDOW_NORMAL);
    cv::imshow("w2", frame);


      vector<vector<Point> > contours;
      vector<Vec4i> hierarchy;

      /// Find contours
      findContours( mask, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0) );

//      for (auto vec : hierarchy)
//          std::cout << vec << std::endl;

//      for (auto vec : contours)
//          for (auto v : vec)
//              std::cout << v << std::endl;

      /// ApproxiMate contours to polygons + get bounding rects and circles
      vector<vector<Point> > contours_poly( contours.size() );
      vector<Rect> boundRect( contours.size() );
      vector<Point2f>center( contours.size() );
      vector<float>radius( contours.size() );

      for( int i = 0; i < contours.size(); i++ )
         { approxPolyDP( Mat(contours[i]), contours_poly[i], 3, true );
           boundRect[i] = boundingRect( Mat(contours_poly[i]) );
           minEnclosingCircle( (Mat)contours_poly[i], center[i], radius[i] );
         }


      /// Draw polygonal contour + bonding rects + circles
//      Mat drawing = Mat::zeros( mask.size(), CV_8UC3 );

      cout << contours.size() <<endl;

      for( int i = 0; i< contours.size(); i++ )
         {
//           Scalar color = Scalar( rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255) );
           int col2 = 255;
           int col3 = 155;

           drawContours( frame, contours_poly, i, col3, CV_FILLED, 8, vector<Vec4i>(), 0, Point() );

           rectangle( frame, boundRect[i].tl(), boundRect[i].br(), col2, 2, 8, 0 );

           //circle( drawing, center[i], (int)radius[i], color, 2, 8, 0 );
         }

      /// Show in a window
      namedWindow( "Contours", CV_WINDOW_AUTOSIZE );
      imshow( "Contours", frame);
}
