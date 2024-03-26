#include <opencv2\opencv.hpp>
#include <iostream>
using namespace std;
using namespace cv;

int main()
{

    /************************************************************************
           测试一张图片
    *************************************************************************/
    if (1)
    {
        cout << "TestImage ..." << endl;
        Mat image = imread("F:/水下机器人代码/旺\Robot_Debug/Robot_Debug/TestOutput.jpg", 1);
        Mat testImage;

        //以下三行为需要手动修改的参数
        //int image_count = 14;                   //图像数量
        //Size board_size = Size(22, 21);            //定标板上每行、列的角点数
        int x_expand = 0, y_expand = 200;		//x,y方向的扩展(x横向，y纵向)，适当增大可以不损失原图像信息
        copyMakeBorder(image, testImage, (int)(y_expand / 2), (int)(y_expand / 2), (int)(x_expand / 2), (int)(x_expand / 2), BORDER_CONSTANT);

        Mat mapx = Mat(image.size(), CV_32FC1);
        Mat mapy = Mat(image.size(), CV_32FC1);
        Mat R = Mat::eye(3, 3, CV_32F);
        //来自matlab一般标定
        //Mat intrinsic_matrix = (Mat_<double>(3, 3) <<
        //    7.014957893955580e+02, 0                    , 1.234660344699637e+03,
        //    0                    , 7.092855488083487e+02, 1.131605886789209e+03,
        //    0                    , 0                    , 1                   );
        //内参-校正图matlab普通标定（圆点标定板）
        //Mat intrinsic_matrix = (Mat_<double>(3, 3) <<
        //    1.021964028441256e+03, 0                    , 1.229205610767032e+03,
        //    0                    , 1.022423073832182e+03, 1.025806111487431e+03,
        //    0                    , 0                    , 1                   );
        //来自Fisheye_biaoding
        Mat intrinsic_matrix = (Mat_<double>(3, 3) <<
            5.35441421e+02, 0.00000000e+00, 1.19914761e+03,
            0.00000000e+00, 5.47906802e+02, 1.09279105e+03,
            0.00000000e+00, 0.00000000e+00, 1.00000000e+00);
        ////来自matlab鱼眼标定
        //Mat distortion_coeffs = (Mat_<double>(4, 1) <<
        //    5.278168146132050e+02, -6.301573078256051e-04, 1.355978629251127e-07, -3.116128841665866e-10);
        Mat distortion_coeffs = (Mat_<double>(4, 1) <<
            -0.7552404, 1.7284778, -1.33230792, 0.28293758);
        fisheye::initUndistortRectifyMap(intrinsic_matrix, distortion_coeffs, R, intrinsic_matrix, image.size(), CV_32FC1, mapx, mapy);
        Mat t = testImage.clone();
        cv::remap(testImage, t, mapx, mapy, INTER_LINEAR);

        imwrite("TestOutput.jpg", t);
        imshow("TestOutput.jpg", t);
        cout << "保存结束" << endl;
    }
    while (1)
    {
        if (cvWaitKey(15) == 27)
        {
            break;
        }
    }

    return 0;
}
