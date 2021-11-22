#ifndef COMMON_H
#define COMMON_H

#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <chrono>
#include <unistd.h>
#include <cmath>

#define PI M_PI
using namespace std;

void LoadImages(vector<int> &vnFileindex, vector<double> &vdTimestamps, string strSessionFolder, string strSceneName, string strSensorID);

void LoadScans(int LiDAR_Start_Sequence, vector<int> &vnFileFolder, vector<int> &vnFileindex, vector<double> &vdTimestamps, string strSequenceFolder, string strSensorID);

void ReadScans(string strScanFilePath, float &fMax_range, float &fMin_range, vector<float> &vfRanges);

cv::Mat PlotScans(float Max_range, float Min_range, vector<float> &vfRanges, int Scale);

cv::Mat PlotScanLines(float Max_range, float Min_range, vector<float> &vfRanges, int Scale);

#endif