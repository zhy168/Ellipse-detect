#include "Ellipse_Detector.h"

using namespace std;
using namespace cv;

Ellipse_Detector::Ellipse_Detector()
{
}

Ellipse_Detector::~Ellipse_Detector()
{
}


// CheckMode: 0����ȥ��������1����ȥ��������; NeihborMode��0����4����1����8����;
// cited by ellipse_detect func
void Ellipse_Detector::RemoveSmallRegion(Mat& Src, Mat& Dst, int AreaLimit, int CheckMode, int NeihborMode)
{
    int RemoveCount = 0;       //��¼��ȥ�ĸ���  
    //��¼ÿ�����ص����״̬�ı�ǩ��0����δ��飬1�������ڼ��,2�����鲻�ϸ���Ҫ��ת��ɫ����3������ϸ������  
    Mat Pointlabel = Mat::zeros(Src.size(), CV_8UC1);

    if (CheckMode == 1)
    {
        cout << "Mode: ȥ��С����. ";
        for (int i = 0; i < Src.rows; ++i)
        {
            uchar* iData = Src.ptr<uchar>(i);
            uchar* iLabel = Pointlabel.ptr<uchar>(i);
            for (int j = 0; j < Src.cols; ++j)
            {
                if (iData[j] < 10)
                {
                    iLabel[j] = 3;
                }
            }
        }
    }
    else
    {
        cout << "Mode: ȥ���׶�. ";
        for (int i = 0; i < Src.rows; ++i)
        {
            uchar* iData = Src.ptr<uchar>(i);
            uchar* iLabel = Pointlabel.ptr<uchar>(i);
            for (int j = 0; j < Src.cols; ++j)
            {
                if (iData[j] > 10)
                {
                    iLabel[j] = 3;
                }
            }
        }
    }

    vector<Point2i> NeihborPos;  //��¼�����λ��  
    NeihborPos.push_back(Point2i(-1, 0));
    NeihborPos.push_back(Point2i(1, 0));
    NeihborPos.push_back(Point2i(0, -1));
    NeihborPos.push_back(Point2i(0, 1));
    if (NeihborMode == 1)
    {
        cout << "Neighbor mode: 8����." << endl;
        NeihborPos.push_back(Point2i(-1, -1));
        NeihborPos.push_back(Point2i(-1, 1));
        NeihborPos.push_back(Point2i(1, -1));
        NeihborPos.push_back(Point2i(1, 1));
    }
    else cout << "Neighbor mode: 4����." << endl;
    int NeihborCount = 4 + 4 * NeihborMode;
    int CurrX = 0, CurrY = 0;
    //��ʼ���  
    for (int i = 0; i < Src.rows; ++i)
    {
        uchar* iLabel = Pointlabel.ptr<uchar>(i);
        for (int j = 0; j < Src.cols; ++j)
        {
            if (iLabel[j] == 0)
            {
                //********��ʼ�õ㴦�ļ��**********  
                vector<Point2i> GrowBuffer;                                      //��ջ�����ڴ洢������  
                GrowBuffer.push_back(Point2i(j, i));
                Pointlabel.at<uchar>(i, j) = 1;
                int CheckResult = 0;                                               //�����жϽ�����Ƿ񳬳���С����0Ϊδ������1Ϊ����  

                for (int z = 0; z < GrowBuffer.size(); z++)
                {

                    for (int q = 0; q < NeihborCount; q++)                                      //����ĸ������  
                    {
                        CurrX = GrowBuffer.at(z).x + NeihborPos.at(q).x;
                        CurrY = GrowBuffer.at(z).y + NeihborPos.at(q).y;
                        if (CurrX >= 0 && CurrX < Src.cols && CurrY >= 0 && CurrY < Src.rows)  //��ֹԽ��  
                        {
                            if (Pointlabel.at<uchar>(CurrY, CurrX) == 0)
                            {
                                GrowBuffer.push_back(Point2i(CurrX, CurrY));  //��������buffer  
                                Pointlabel.at<uchar>(CurrY, CurrX) = 1;           //���������ļ���ǩ�������ظ����  
                            }
                        }
                    }

                }
                if (GrowBuffer.size() > AreaLimit) CheckResult = 2;                 //�жϽ�����Ƿ񳬳��޶��Ĵ�С����1Ϊδ������2Ϊ����  
                else { CheckResult = 1;   RemoveCount++; }
                for (int z = 0; z < GrowBuffer.size(); z++)                         //����Label��¼  
                {
                    CurrX = GrowBuffer.at(z).x;
                    CurrY = GrowBuffer.at(z).y;
                    Pointlabel.at<uchar>(CurrY, CurrX) += CheckResult;
                }
                //********�����õ㴦�ļ��**********  


            }
        }
    }

    CheckMode = 255 * (1 - CheckMode);
    //��ʼ��ת�����С������  
    for (int i = 0; i < Src.rows; ++i)
    {
        uchar* iData = Src.ptr<uchar>(i);
        uchar* iDstData = Dst.ptr<uchar>(i);
        uchar* iLabel = Pointlabel.ptr<uchar>(i);
        for (int j = 0; j < Src.cols; ++j)
        {
            if (iLabel[j] == 2)
            {
                iDstData[j] = CheckMode;
            }
            else if (iLabel[j] == 3)
            {
                iDstData[j] = iData[j];
            }
        }
    }

    cout << RemoveCount << " objects removed." << endl;
}


void Ellipse_Detector::Cal_score(vector<vector<Point> > contours_final, vector<RotatedRect> box_final, int count, vector<vector<double>>& score) {
    for (int k = 0; k < count; k++)
    {
        double suit=-1.0,complete=-1.0,area=-1.0;

        int consize = contours_final[k].size();

        float _cos = cos(-box_final[k].angle);
        float _sin = sin(-box_final[k].angle);

        double a = std::max(box_final[k].size.height, box_final[k].size.width)/2;
        double b = std::min(box_final[k].size.height, box_final[k].size.width)/2;

        float invA2 = 1.0f/ (a * a);   //1.f   l.f
        float invB2 = 1.0f /(b * b);

        float invNofPoints = 1.f / float(contours_final[k].size());
        int counter_on_perimeter = 0;

        for (ushort l = 0; l < consize; ++l)
        {
            float tx = float(contours_final[k][l].x) - box_final[k].center.x;
            float ty = float(contours_final[k][l].y) - box_final[k].center.y;
            float rx = (tx * _cos - ty * _sin);
            float ry = (tx * _sin + ty * _cos);

            float h = (rx * rx) * invA2 + (ry * ry) * invB2;
            if (abs(h - 1.f) < fDistanceToEllipseContour)   //��Բ��׼ʽ���<0.1
            {
                ++counter_on_perimeter;
            }
        }
        
        // Compute score
        suit = float(counter_on_perimeter) * invNofPoints;//�ŵ�/�ܵ�  
//        cout << "�ܵ㣺" << contours_final[k].size() << endl;
//        cout << "�õ㣺" << counter_on_perimeter << endl;
//        cout << "score=" << score[k] << endl;

        //������
        Eigen::Vector2d start, middle, end, center;
        start << contours_final[k][0].x, contours_final[k][0].y;
        middle << contours_final[k][consize / 2].x, contours_final[k][consize / 2].y;
        end << contours_final[k][consize - 1].x, contours_final[k][consize - 1].y;
        center << box_final[k].center.x, box_final[k].center.y;

        Vector2d vector1 = start - center;
        Vector2d vector2 = middle - center;
        Vector2d vector3 = end - center;

        double cos1 = vector1.dot(vector2) / (vector1.norm() * vector2.norm());   //dot�����norm
        double cos2 = vector2.dot(vector3) / (vector2.norm() * vector3.norm());

        complete = (acos(cos1) + acos(cos2));//rad
//        cout << "complete=" << complete << endl;


        //���
        if(contourArea(contours_final[k]))
                area=0.8*contourArea(contours_final[k]);
            else
                area=0;


        score[k].push_back(suit);					//ƥ�����
        score[k].push_back(complete);				//������
        score[k].push_back(area);					//���
        }

}

// detect function, reffering to removeSmallRegion func.
int Ellipse_Detector::ellipse_detect(Mat src, Mat& dst, vector<array<double, 5>>& parameters)
{
    int count = 0;
    dst = src.clone();
    //dst = Mat::zeros(src.size(), CV_8UC1);
    if (src.empty()) {
        cout << "ͼƬΪ��" << endl;
        return 0;
    }

 /*   Mat img_hsv;
    cvtColor(src, img_hsv, COLOR_BGR2HSV);
    for (int i = 0; i < 10; i++) {
        for (int j = 0; j < 5; j++) {
            Vec3b _p = img_hsv.at<Vec3b>(i,j);
            cout << _p << "\t";
        }     
        cout << endl;
    }*/
	Size dsize = Size(306, 256);
	resize(src, src, dsize, 0, 0, INTER_AREA);
    Mat after_weighted;
    cvtColor(src, after_weighted, COLOR_BGR2GRAY);
    //cv::imwrite("C:/Users/admin/Desktop/after_weighted.jpg", after_weighted);
  
    blur(after_weighted, after_weighted, Size(2, 2));
    //thresh_binary         ������ֵ�Ĳ��ֱ���Ϊ255��С�ڲ��ֱ���Ϊ0
    //thresh_binary_inv     ������ֵ���ֱ���Ϊ0��С�ڲ��ֱ���Ϊ255
    //thresh_trunc          ������ֵ���ֱ���Ϊthreshold��С�ڲ��ֱ���ԭ��  
    //thresh_tozero         С����ֵ���ֱ���Ϊ0�����ڲ��ֱ��ֲ���
    //thresh_tozero_inv     ������ֵ���ֱ���Ϊ0��С�ڲ��ֱ��ֲ��� 
    threshold(after_weighted, after_weighted, 100, 255, THRESH_TOZERO);
	//adaptiveThreshold(after_weighted, after_weighted, 255, ADAPTIVE_THRESH_MEAN_C, THRESH_BINARY, 9, -2);
    imshow("��ֵ",after_weighted);
	waitKey();

    Mat1b after_can;
    Canny(after_weighted, after_can, 8, 10, 3);
    namedWindow("edge", 0);
    resizeWindow("edge", 612, 512);
    imshow("edge", after_can);

    Mat kernel_d = getStructuringElement(MORPH_ELLIPSE, Size(2, 2));
    dilate(after_can, after_can, kernel_d, Point(-1, -1), 2);
	//morphologyEx(after_can, after_can, MORPH_OPEN, kernel_d);
    Mat1b after_filt = Mat::zeros(src.size(), CV_8UC1);
    RemoveSmallRegion(after_can, after_filt, 5, 1, 0);
    namedWindow("after_filt", 0);
    resizeWindow("after_filt", 612, 512);
    imshow("after_filt", after_filt);
	waitKey();
    //dst = after_filt.clone();
    //cvtColor(dst,dst,CV_GRAY2BGR);

    // contours
    vector<vector<Point> > contours_origin;
    vector<vector<Point> > contours;
    vector<vector<Point> > contours_final;
    vector<Vec4i> hier;
    //RETR_LIST�����������������������κβ�νṹ��ϵ��
    //RETR_EXTERNAL�ϸ�����Բ
    //RETR_CCOMP����������������������֯��������νṹ�С��ڶ��㣬����������ⲿ�߽硣�ڵڶ��㣬�п׵ı߽硣�������Ԫ���Ŀ�������һ������������λ�ڶ��㡣
    findContours(after_filt, contours_origin, hier, RETR_CCOMP, CHAIN_APPROX_SIMPLE);//hier��������
	//��������
	Mat contours_1(after_filt.size(), CV_8UC1, Scalar(255));
	drawContours(contours_1, contours_origin, -1, Scalar(0, 0, 250), 3);
	namedWindow("contours_1", 0);
	imshow("contours_1", contours_1);
	


    //hier[i]=[Next, Previous, First_Child, Parent]

    for (int j = 0; j < hier.size(); ++j) {
        //cout << hier[j] << endl;
        if(contours_origin[j].size()>50 && hier[j][2] == -1)  //���������ȷ��һ����Բ��������������50��������������origin����contours
        {
            contours.push_back(contours_origin[j]);
        }
    }
    cout << "all count=" << contours.size() << endl;

	//��2
	Mat contours_2(after_filt.size(), CV_8UC1, Scalar(255));
	drawContours(contours_2, contours, -1, Scalar(0, 0, 250), 3);
	namedWindow("contours_2", 0);
	imshow("contours_2", contours_2);
	

    vector<RotatedRect> box(contours.size());
    vector<RotatedRect> box_final;
 /*   Mat r;
    cvtColor(after_filt, r, COLOR_GRAY2BGR);*/


    //ȥ��������<150�������>2������ڰ�
    for (int i = 0; i < contours.size(); i++) {
        if(contours[i].size() <= 150)
            continue;
        else
        {
            Rect boundRect = boundingRect(Mat(contours[i]));
            double recOrNot = abs(boundRect.width - boundRect.height) / (0.5 * (boundRect.width + boundRect.height));
            if (recOrNot>0.5)  // 1st rect check
                continue;
            else
            {
                //������С������Բ���������о���
                box[i] = fitEllipse(Mat(contours[i]));//���ε�5������Ϊ��������ҵ�x������֮��ļнǣ�0~180�㣩
                Point2f* vertices = new Point2f[4];
                box[i].points(vertices);
                double temp_dist_1 = sqrtf(powf((vertices[0].x - vertices[1].x), 2) + powf(vertices[0].y - vertices[1].y, 2));
                double temp_dist_2 = sqrtf(powf((vertices[2].x - vertices[1].x), 2) + powf(vertices[2].y - vertices[1].y, 2));
                double temp_dist_ratio = temp_dist_1 / temp_dist_2;
                if (temp_dist_ratio > 1.4 || temp_dist_ratio < 0.6)  // 2nd rect check
                {
                    //cout << i << "����������Ȳ�����" << endl;
                    continue;
                }
                else  // rect for sure
                {
                    ////ȥ�����۾�ͷ���Ե
                    //Mat1s DX, DY;
                    //Sobel(after_weighted, DX, CV_16S, 1, 0);
                    //Sobel(after_weighted, DY, CV_16S, 0, 1);

                    //int isize = contours[i].size();
                    //Point p1 = contours[i][0];
                    //Point p2 = contours[i][isize / 2];
                    //Point middle = (p1 + p2) / 2;
                    ////ָ��Բ��
                    //Vector2d v1, v2;
                    //v1 << (middle - p1).x, (middle - p1).y;
                    //v2 << (middle - p2).x, (middle - p2).y;

                    //Vector2d g1, g2;//�ݶ�ʸ��

                    //int up = p1.y, down = p1.y;
                    //short pdy = DY.at<short>(p1.y, p1.x);
                    //int edgey = p1.y;
                    //while (pdy == 0) {

                    //    if (up - 1 > 0 && after_dil.at<uchar>(up - 1, p1.x) != 0) {
                    //        up--;
                    //        pdy = DY.at<short>(up, p1.x);
                    //        edgey = up;
                    //    }

                    //    if (pdy == 0 && down + 1 < DY.rows && after_dil.at<char>(down + 1, p1.x) != 0) {
                    //        down++;
                    //        pdy = DY.at<short>(down, p1.x);
                    //        edgey = down;
                    //    }
                    //    else if (after_dil.at<char>(up - 1, p1.x) != 0 && after_dil.at<char>(down + 1, p1.x) == 0) {
                    //        edgey = p1.y;
                    //        break;
                    //    }
                    //}
                    //g1 << double(DX.at<short>(edgey, p1.x)), double(DY.at<short>(edgey, p1.x));

                    //up = p2.y, down = p2.y;
                    //pdy = DY.at<short>(p2.y, p2.x);
                    //edgey = p2.y;
                    //while (pdy == 0) {

                    //    if (up - 1 > 0 && after_dil.at<char>(up - 1, p2.x) != 0) {
                    //        up--;
                    //        pdy = DY.at<short>(up, p2.x);
                    //        edgey = up;
                    //    }
                    //    else if (after_dil.at<char>(up - 1, p2.x) == 0) {
                    //        edgey = p2.y;
                    //        break;
                    //    }
                    //}
                    //g2 << double(DX.at<short>(edgey, p2.x)), double(DY.at<short>(edgey, p2.x));

                    ////if (v1.dot(g1) > 0 || v2.dot(g2) > 0)
                    //if (0)
                    //{
                    //    //cout << i << "����������ڰ�" << endl;
                    //    continue;
                    //}
                    //else
                    //{
                        count++;
                        contours_final.push_back(contours[i]);
                        box_final.push_back(box[i]);
						


                    //}
                }
            }
        }
    }

	//��3
	Mat contours_3(after_filt.size(), CV_8UC1, Scalar(255));
	drawContours(contours_3, contours, -1, Scalar(0, 0, 250), 3);
	//�������о���box
	//Point2f* vertices = new Point2f[4];
	//vector<Point> points;
	//for (int i = 0; i < box_final.size(); i++)
	//{
	//	for (int j = 0; j < 1000; j++)
	//	{
	//		box_final[j].points(vertices);
	//		line(contours_3, vertices[j], vertices[(j + 1) % 4], Scalar(0, 0, 255));
	//	}
	//}

	namedWindow("contours_3", 0);
	imshow("contours_3", contours_3);
	waitKey();

    vector<vector<double>> score(count);//ƥ������ȡ�Բ�����ȡ����
	//�������
    Cal_score(contours_final, box_final, count, score);//��ԲԽϸ�������Խ��

    //ѡȡ
    vector<vector<double>> ab;
    for (int idx = 0; idx < count; idx++) {
        cout << "ƥ�������[" << idx << "]=" << score[idx][0] << endl;
        cout << "Բ������[" << idx << "]=" << score[idx][1] << endl;
        cout << "���[" << idx << "]=" << score[idx][2] << endl<<endl;

        if(score[idx][0] > 0.2 && score[idx][1] > 0.1 && score[idx][2] > 500)
        {
            bool con1 = 1;
            for (int k = 0; k < ab.size(); k++) {
                bool b1 = sqrt(box_final[idx].center.x - ab[k][0]) < 50;
                bool b2 = sqrt(box_final[idx].center.y - ab[k][1]) < 50;
                bool b3 = sqrt(box_final[idx].size.width - ab[k][2]) < 50;
                bool b4 = sqrt(box_final[idx].size.height - ab[k][3]) < 50;
                if (b1 && b2 && b3 && b4) {
                    con1 = 0;
                }
                else {
                    con1 = 1;
                }
            }
            
            if(con1)
            {
                // plot����Բ  void cv::ellipse(		InputOutputArray    img,
			/*	Point 	            center,
					Size 	            axes,
					double 	            angle,
					double 	            startAngle,
					double 	            endAngle,
					const Scalar & 	    color,
					int 	            thickness = 1,
					int 	            lineType = LINE_8,
					int 	            shift = 0
					)
			*/


                ellipse(dst, box_final[idx].center, Size(box_final[idx].size.width / 2, box_final[idx].size.height / 2), box_final[idx].angle, 0, 360, Scalar(0, 0, 255), 4);
				
                Point2f* vertices = new Point2f[4];
                box_final[idx].points(vertices);
                /*cout << vertices[0] << endl;
                cout << vertices[1] << endl;
                cout << vertices[2] << endl;
                cout << vertices[3] << endl;*/
                ////������������
                //double xmin = vertices[0].x, xmax = vertices[0].y;
                //int ymin = vertices[0].y, ymax = vertices[0].y;
                //int kxmin = 0, kxmax = 0, kymin = 0, kymax = 0;
                //for (int k = 1; k < 4; k++) {
                //    if (vertices[k].x < xmin) {
                //        xmin = vertices[k].x;
                //        kxmin = k;
                //    }
                //    if (vertices[k].x > xmax) {
                //        xmax = vertices[k].x;
                //        kxmax = k;
                //    }
                //    if (vertices[k].y < ymin) {
                //        ymin = vertices[k].y;
                //        kymin = k;
                //    }
                //    if (vertices[k].y > ymax) {
                //        ymax = vertices[k].y;
                //        kymax = k;
                //    }
                //}
                //cv::circle(dst, box_final[idx].center, 2, Scalar(255, 0, 0), 1, 8);
                double x = box_final[idx].center.x;
                double y = box_final[idx].center.y;
                double a = max(box_final[idx].size.width, box_final[idx].size.height) / 2;
                double b = min(box_final[idx].size.width, box_final[idx].size.height) / 2;
                double ang;//angle of shun shi zhen
                //ǰ������������ɳ���
                //�� _____
                //  |_____|
                if (vertices[0].y == vertices[1].y) {
                    ang = 0;//rad
                }
                //�� __
                //  |  |
                //  |__|
                else if (vertices[0].x == vertices[1].x) {
                    ang = 90;//rad
                }
                //�� ��˳ʱ��ת0~90�㣨�õ�������ʱ����ת���󣿣�����
                else if (vertices[0].y > vertices[1].y && vertices[0].x < vertices[1].x) {
                    ang = PI - atan((vertices[0].y - vertices[1].y) / (vertices[1].x - vertices[0].x));//rad
                }
                //�� ��˳ʱ��ת90~180�㣨�õ�������ʱ����ת���󣿣�����
                else if (vertices[0].y < vertices[1].y && vertices[0].x < vertices[1].x) {
                    ang = atan((vertices[1].y - vertices[0].y) / (vertices[1].x - vertices[0].x));//rad
                }
                //cout << "��Բ����Ϊ" << endl;
                //cout << x << endl;
                //cout << y << endl;
                //cout << a << endl;
                //cout << b << endl;
                //cout << ang << endl;

                parameters.push_back({ x,y,a,b,ang });
                //cout << "��Բ����Ϊ" << endl;
                //cout << parameters[0][0] << endl;
                //cout << parameters[0][1] << endl;
                //cout << parameters[0][2] << endl;
                //cout << parameters[0][3] << endl;
                //cout << parameters[0][4] << endl;
            }
            else {
                continue;
            }
        }
    }
    cout << "ʶ��" << parameters.size() << "��Բ" << endl;


    if(parameters.size()>0)
        return 1;
    else
    {
        cout << "parameters.size=" << parameters.size() << endl;
        return 0;
    }
}
