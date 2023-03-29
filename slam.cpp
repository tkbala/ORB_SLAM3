#include "slam.h"
#include "global.h"
#include <opencv2/core/core.hpp>
#include <System.h>

void* CreateSLAM()
{
	return new ORB_SLAM3::System(
		"E:\\awilson\\RealityShader\\external\\ORB_SLAM3_Windows\\Vocabulary\\ORBvoc.bin", 
		//"E:\\awilson\\RealityShader\\external\\ORB_SLAM3_Windows\\Examples\\Monocular\\EuRoC.yaml",
		"camera.yaml",
		ORB_SLAM3::System::MONOCULAR,
		true
	);
}

void TrackMonocular(void* slam, void* imageData, int imageWidth, int imageHeight, double timestamp, int* rows, int* cols, float* data)
{
	ORB_SLAM3::System* slamSystem = (ORB_SLAM3::System*)slam;
	cv::Mat image = cv::Mat(imageHeight, imageWidth, CV_8UC4, imageData);
	cv::Mat smallImage = cv::Mat(imageHeight / 2, imageWidth / 2, CV_8UC4);
	cv::resize(image, smallImage, smallImage.size(), 0, 0, cv::INTER_LINEAR);

	auto Tcw = slamSystem->TrackMonocular(smallImage, timestamp);

	*rows = Tcw.rows;
	*cols = Tcw.cols;

	memcpy(data, Tcw.data, Tcw.rows * Tcw.cols * sizeof(float));

	//std::cout << Tcw << std::endl; // stored row-major


}

void ShutdownSLAM(void* slam)
{
	ORB_SLAM3::System* slamSystem = (ORB_SLAM3::System*)slam;
	slamSystem->Shutdown();
}