#include "Ellipse_Detector.h"
#include "CalculatePose.h"

using namespace cv;
int main()
{
   
    Mat image = imread("F:/projectphoto/中广核图片/photo2.bmp");

    Ellipse_Detector EllDet;
    std::vector<array<double, 5>> EParameters;
    Vector3d Tcenter, Tvector;
    int detect = EllDet.ellipse_detect(image, image, EParameters);
    cout << "detect=" << detect << endl;
    if (detect) {
    
        namedWindow("Ellipse", 0);
        resizeWindow("Ellipse", 612, 512);
        imshow("Ellipse", image);
        Seektrue(EParameters, Tcenter, Tvector);
        waitKey();
        //imwrite("C:/Users/admin/Desktop/2.jpg", image);
    }
    return 0;
}
