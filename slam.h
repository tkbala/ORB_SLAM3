#pragma once

#ifdef SLAM_EXPORTS
#define SLAM_API extern "C" __declspec(dllexport)
#else
#define SLAM_API extern "C" __declspec(dllimport)
#endif


SLAM_API void* CreateSLAM();

SLAM_API void TrackMonocular(void* slam, void* imageData, int imageWidth, int imageHeight, double timestamp, int* rows, int* cols, float* data);

SLAM_API void ShutdownSLAM(void* slam);