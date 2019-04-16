#include <chrono>
#include <flycapture/FlyCapture2.h>
#include <opencv2/core/ocl.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
//#include <ros/ros.h>
using namespace std;

class CameraNode
{
  private:
	FlyCapture2::Camera camera;
	FlyCapture2::Image rawImage, encodedImage, bgrImage; //Contains the raw and converted frames from the camera.
	FlyCapture2::Error capture;

	int low_B = 200, low_G = 200, low_R = 200, high_B = 255, high_G = 255, high_R = 255; // lower and upper limits for HSV slider
	int sample_Size = 10;
	cv::UMat U_input, U_hsvImage, U_binaryImage, U_erosion, U_transmtx, U_transformed, U_resize; //for use on GPU
	cv::Mat image, intermediate, transmtx;
	cv::Rect ROI = cv::Rect(112, 12, 1670 - 112, 920 - 12);
	std::vector<cv::Point2f> quad_pts, square_pts;
	std::vector<cv::Point> pixelCoordinates;
	std::vector<int> xPos, yPos;

	//empty callback functions but it is the only way to increment the sliders
	static void on_low_B_thresh_trackbar(int, void *) {}
	static void on_high_B_thresh_trackbar(int, void *) {}
	static void on_low_G_thresh_trackbar(int, void *) {}
	static void on_high_G_thresh_trackbar(int, void *) {}
	static void on_low_R_thresh_trackbar(int, void *) {}
	static void on_high_R_thresh_trackbar(int, void *) {}

  public:
	CameraNode()
	{
		capture = camera.Connect(0);
		if (capture != FlyCapture2::PGRERROR_OK)
		{
			cout << "Make sure the Camera is plugged in" << endl;
		}

		capture = camera.StartCapture();
	}
	~CameraNode()
	{
		capture = camera.StopCapture();
		camera.Disconnect();
	}
	void setupWarp(int tl_x, int tl_y, int tr_x, int tr_y, int br_x, int br_y, int bl_x, int bl_y, double r)
	{
		cv::Point Q1 = cv::Point2f(tl_x, tl_y); //top left pixel coordinate
		cv::Point Q2 = cv::Point2f(tr_x, tr_y); //top right
		cv::Point Q3 = cv::Point2f(br_x, br_y); //bottom right
		cv::Point Q4 = cv::Point2f(bl_x, bl_y); //bottom left

		double ratio = r; // width / height of the actual panel on the ground
		double boardH = sqrt((Q3.x - Q2.x) * (Q3.x - Q2.x) + (Q3.y - Q2.y) * (Q3.y - Q2.y));
		double boardW = ratio * boardH;

		cv::Rect R(Q1.x, Q1.y, boardW, boardH);

		cv::Point R1 = cv::Point2f(R.x, R.y);
		cv::Point R2 = cv::Point2f(R.x + R.width, R.y);
		cv::Point R3 = cv::Point2f(cv::Point2f(R.x + R.width, R.y + R.height));
		cv::Point R4 = cv::Point2f(cv::Point2f(R.x, R.y + R.height));

		square_pts = {R1, R2, R3, R4};
		quad_pts = {Q1, Q2, Q3, Q4};

		transmtx = cv::getPerspectiveTransform(quad_pts, square_pts);
		transmtx.copyTo(U_transmtx);
		int width = U_binaryImage.rows;
		int height = U_binaryImage.cols;
		U_transformed = cv::UMat::zeros(height, width, CV_8UC3);
	}
	void setupOCL()
	{
		cv::setUseOptimized(true);
		cv::ocl::setUseOpenCL(true);
		if (cv::useOptimized())
		{
			cout << "OpenCL optimizations enabled" << endl;
		}
		else
		{
			cout << "OpenCL optimizations NOT enabled" << endl;
		}

		if (!cv::ocl::haveOpenCL())
		{
			cout << "no opencl devices detected" << endl;
		}

		cv::ocl::Context context;
		if (!context.create(cv::ocl::Device::TYPE_GPU))
		{
			cout << "failed to initialize device" << endl;
		}
		cout << context.ndevices() << " GPU device(s) detected." << endl;

		cout << "************************" << endl;
		for (size_t i = 0; i < context.ndevices(); i++)
		{
			cv::ocl::Device device = context.device(i);
			cout << "name: " << device.name() << endl;
			cout << "available: " << device.available() << endl;
			cout << "img support: " << device.imageSupport() << endl;
			cout << device.OpenCL_C_Version() << endl;
		}
		cout << "************************" << endl;

		cv::ocl::Device d = cv::ocl::Device::getDefault();
		cout << d.OpenCLVersion() << endl;
	}

	void ptgrey2CVMat()
	{
		capture = camera.RetrieveBuffer(&rawImage);
		if (capture != FlyCapture2::PGRERROR_OK)
		{
			std::cout << "Have you tried plugging it in?" << std::endl;
		}
		// convert to bgr
		FlyCapture2::Image bgrImage;
		rawImage.Convert(FlyCapture2::PIXEL_FORMAT_BGR, &bgrImage);
		// convert to OpenCV Mat
		unsigned int rowBytes =
			(double)bgrImage.GetReceivedDataSize() / (double)bgrImage.GetRows();
		intermediate = cv::Mat(bgrImage.GetRows(), bgrImage.GetCols(), CV_8UC3,
							   bgrImage.GetData(), rowBytes);
		image = intermediate.clone();
		image.copyTo(U_input);
	}

	void imageFiltering()
	{

		cv::inRange(U_input, cv::Scalar(low_B, low_G, low_R), cv::Scalar(high_B, high_G, high_R), U_binaryImage);
		cv::erode(U_binaryImage, U_erosion, cv::Mat(), cv::Point(-1, -1), 2);
	}

	void shiftPerspective()
	{
		cv::warpPerspective(U_erosion, U_transformed, U_transmtx, U_transformed.size());
		//cv::imshow("warp",U_transformed);
		U_resize = U_transformed(ROI);
	}

	void getPixelDistance()
	{

		cv::findNonZero(U_resize, pixelCoordinates);
		for (size_t i = 0; i < pixelCoordinates.size(); i++)
		{
			if (i % sample_Size == 0)
			{
				xPos.push_back(pixelCoordinates[i].x);
				yPos.push_back(pixelCoordinates[i].y);
			}
		}
		std::cout << pixelCoordinates.size() << " " << xPos.size() << "  ";
		xPos.clear();
		yPos.clear();

		//geometry_msgs::Point32 pixelLocation;
		//pixelLocation.x = (A * i) + B;
		//pixelLocation.y = (C * j) + D;
		//msg.pixelLocations.push_back(pixelLocation);
	}
	void createGUI()
	{
		cv::namedWindow("original", CV_WINDOW_FREERATIO);
		cv::namedWindow("erosion", CV_WINDOW_FREERATIO);
		//cv::namedWindow("binary", CV_WINDOW_FREERATIO);
		cv::namedWindow("warp", CV_WINDOW_FREERATIO);
		cv::namedWindow("TRACKBARS", CV_WINDOW_FREERATIO);
		//*****************GUI related *********************************
		cv::createTrackbar("Low Blue", "TRACKBARS", &low_B, 255, on_low_B_thresh_trackbar);
		cv::createTrackbar("Low Green", "TRACKBARS", &low_G, 255, on_low_G_thresh_trackbar);
		cv::createTrackbar("Low Red", "TRACKBARS", &low_R, 255, on_low_R_thresh_trackbar);
		cv::createTrackbar("High Blue", "TRACKBARS", &high_B, 255, on_high_B_thresh_trackbar);
		cv::createTrackbar("High Green", "TRACKBARS", &high_G, 255, on_high_G_thresh_trackbar);
		cv::createTrackbar("High Red", "TRACKBARS", &high_R, 255, on_high_R_thresh_trackbar);
	}
	void display(bool enable)
	{
		if (enable)
		{
			cv::imshow("original", U_input);
			//cv::imshow("binary", U_binaryImage);
			cv::imshow("warp", U_resize);
			cv::imshow("erosion", U_erosion);
		}
	}
};

int main(int argc, char **argv)
{
	//ros::init(argc, argv, "white_line_detection");
	//ros::NodeHandle nh;
	bool enableDebug = false;
	CameraNode whiteLineDetection;
	whiteLineDetection.setupOCL();
	whiteLineDetection.createGUI();
	whiteLineDetection.setupWarp(662, 315, 1263, 318, 1371, 522, 573, 520, 1.5015);

	while ((char)cv::waitKey(1) != 'q')
	{

		whiteLineDetection.ptgrey2CVMat();
		whiteLineDetection.imageFiltering();
		whiteLineDetection.shiftPerspective();

		whiteLineDetection.getPixelDistance();
		whiteLineDetection.display(enableDebug);
	}

	return 0;
}
