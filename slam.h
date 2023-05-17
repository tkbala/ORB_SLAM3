#pragma once

#include <vector>

#ifdef SLAM_EXPORTS
#define SLAM_API extern "C" __declspec(dllexport)
#else
#define SLAM_API extern "C" __declspec(dllimport)
#endif

struct SLAMKeyPoint
{
	float x, y;
};

SLAM_API void* CreateSLAM();

SLAM_API void TrackMonocular(void* slam, void* imageData, int imageWidth, int imageHeight, double timestamp, int* rows, int* cols, float* data);

SLAM_API void GetKeypoints(void* slam, std::vector<SLAMKeyPoint>& keypoints);

SLAM_API void ShutdownSLAM(void* slam);