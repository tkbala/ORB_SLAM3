#include "slam.h"
#include "global.h"
#include <opencv2/core/core.hpp>
#include <System.h>
#include <shlwapi.h>

void* CreateSLAM()
{
	// vocubulary file is found in RealityShader\\external\\ORB_SLAM3_Windows\\Vocabulary\\ORBvoc.bin
	// but copied to executable directory at build time
	char vocabularyFilePath[MAX_PATH];
	auto result = GetModuleFileNameA(GetModuleHandle(NULL), vocabularyFilePath, MAX_PATH);
	PathRemoveFileSpecA(vocabularyFilePath);
	PathAppendA(vocabularyFilePath, "ORBvoc.bin");
	if (!PathFileExistsA(vocabularyFilePath))
		return nullptr;

	return new ORB_SLAM3::System(
		vocabularyFilePath,
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

	if (Tcw.rows * Tcw.cols > 0)
		memcpy(data, Tcw.data, Tcw.rows * Tcw.cols * sizeof(float));

	// OpenCV coordinate system: x right, y down, z forward
}

void GetKeypoints(void* slam, std::vector<SLAMKeyPoint>& out)
{
	ORB_SLAM3::System* slamSystem = (ORB_SLAM3::System*)slam;
	auto keyPoints = slamSystem->GetTrackedKeyPointsUn();
	auto mapPoints = slamSystem->GetTrackedMapPoints();

	out.clear();

	int n = keyPoints.size();
	for (int i = 0; i < n; i++)
	{
		if (mapPoints[i] && mapPoints[i]->Observations() > 0)
			out.push_back(SLAMKeyPoint{ keyPoints[i].pt.x, keyPoints[i].pt.y });
	}
}

void ShutdownSLAM(void* slam)
{
	ORB_SLAM3::System* slamSystem = (ORB_SLAM3::System*)slam;
	slamSystem->Shutdown();
}