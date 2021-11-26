#include <Common.h>

#include <fstream>
#include <iostream>

void LoadImages(vector<int> &vnFileindex, vector<double> &vdTimestamps, string strSessionFolder, string strSceneName, string strSensorID)
{
    string fname(strSessionFolder+strSceneName+".txt");
    ifstream fSequence(fname.c_str());

    if(fSequence.fail())
    {
        cout << "[Robot@Home Parser] Scanning " << fname << "Failed" << endl;
        return;
    }
    else
    {
        cout << "[Robot@Home Parser] Scanning " << strSceneName+".txt" << endl;
    }
    string s;
    while(getline(fSequence,s))
    {
        if (s.rfind("#", 0) == 0) 
        {
            continue;
        }

        stringstream ss;
        ss << s;
        int nObservation_id;
        string sSensor_label;
        float fPx, fPy, fPz, fOy, fOp, fOr;
        long lTstamp;

        ss >> nObservation_id >> sSensor_label >> fPx >> fPy >> fPz >> fOy >> fOp >> fOr >> lTstamp;

        if(sSensor_label.compare(strSensorID)==0)
        {
            vnFileindex.push_back(nObservation_id);
            vdTimestamps.push_back(double(lTstamp*1e-7));
        }
    }
    fSequence.close();
}

void LoadScans(int LiDAR_Start_Sequence, vector<int> &vnFileFolder, vector<int> &vnFileindex, vector<double> &vdTimestamps, string strSequenceFolder, string strSensorID)
{
    int folder_index = LiDAR_Start_Sequence;
    while(true)
    {
        string fname(strSequenceFolder+to_string(folder_index)+"_hokuyo_processed.txt");
        ifstream fSequence(fname.c_str());

        if(fSequence.fail())
        {
            cout << "[Robot@Home Parser] Scanning " << fname << " ...Failed" << endl;
            cout << "[Robot@Home Parser] Load " << folder_index-LiDAR_Start_Sequence << " LiDAR Sequences" << endl;
            break;
        }
        else
        {
            cout << "[Robot@Home Parser] Scanning " << to_string(folder_index)+"_hokuyo_processed.txt" << endl;
        }

        string s;
        while(getline(fSequence,s))
        {
            if (s.rfind("#", 0) == 0) 
            {
                continue;
            }

            stringstream ss;
            ss << s;
            int nObservation_id;
            string sSensor_label;
            float fPx, fPy, fPz, fOy, fOp, fOr;
            long lTstamp;

            ss >> nObservation_id >> sSensor_label >> fPx >> fPy >> fPz >> fOy >> fOp >> fOr >> lTstamp;

            if(sSensor_label.compare(strSensorID)==0)
            {
                vnFileFolder.push_back(folder_index);
                vnFileindex.push_back(nObservation_id);
                vdTimestamps.push_back(double(lTstamp*1e-7));
            }
        }
        fSequence.close();
        folder_index++;
    }
}

void ReadScans(string strScanFilePath, float &fMax_range, float &fMin_range, vector<float> &vfRanges)
{
    ifstream fSequence(strScanFilePath.c_str());

    if(fSequence.fail())
    {
        cout << "[Robot@Home Parser] Read Scan " << strScanFilePath << "Failed" << endl;
        return;
    }

    string s;
    int LineNumber = 0;
    int number_of_scans = 0;
    vector<float> vfTmpRanges;
    vector<int> vnValid;
    float fMax = 0;
    float fMin = 10;
    while(getline(fSequence,s))
    {
        LineNumber++;
        if (s.rfind("#", 0) == 0) 
        {
            continue;
        }
        stringstream ss;
        ss << s;

        if(LineNumber==9)
        {
            ss >> fMax;
        }
        if(LineNumber==10)
        {
            ss >> number_of_scans;
            vfTmpRanges.reserve(number_of_scans);
            vnValid.reserve(number_of_scans);

        }
        if(LineNumber==11)
        {
            for(int i=0; i < number_of_scans; i++)
            {
                float fRange;
                ss >> fRange;
                vfTmpRanges.push_back(fRange);
            }
        }
        if(LineNumber==12)
        {
            for(int i=0; i < number_of_scans; i++)
            {
                int nValid;
                ss >> nValid;
                vnValid.push_back(nValid);
            }
        }
    }
    fSequence.close();

    vfRanges.clear();
    vfRanges.reserve(number_of_scans);
    for(int i=0; i < number_of_scans; i++)
    {
        if(vnValid[i] == 1)
        {
            float range = vfTmpRanges[i];
            if(range > fMax)
            {
                fMax = range;
            }
            if(range < fMin)
            {
                fMin = range;
            }
            vfRanges.push_back(range);
        }
        else
        {
            vfRanges.push_back(-1);
        }
    }
    fMax_range = fMax;
    fMin_range = fMin;
}

cv::Mat PlotScans(float Max_range, float Min_range, vector<float> &vfRanges, int Scale)
{
    cv::Mat scan = cv::Mat::ones(cv::Size(600,600),CV_8UC3);
    scan = cv::Scalar(50,50,50);
    cv::Point2f center(scan.cols/2, scan.rows/2);
    float angle_increment = 0.352;
    float angle_start = -120.032;

    cv::arrowedLine(scan, center, center+cv::Point2f(20,0), cv::Scalar(0,0,255), 2, 8, 0, 0.2);
    cv::arrowedLine(scan, center, center+cv::Point2f(0,20), cv::Scalar(0,255,0), 2, 8, 0, 0.2);
    cv::drawMarker(scan, center, cv::Scalar(255,0,0), cv::MARKER_TILTED_CROSS, 10, 2);
    for(int i=0; i < vfRanges.size(); i++)
    {
        float range = vfRanges[i];
        if(range < Min_range || range > Max_range)
        {
            continue;
        }
        else
        {
            float current_angle = angle_start + i * angle_increment;

            float px = range * cos(current_angle/180*PI) * Scale;
            float py = range * sin(current_angle/180*PI) * Scale;
            cv::Point2f ScanPosition = center + cv::Point2f(px, py);
            cv::circle(scan, ScanPosition, 1, cv::Scalar(0,0,255), 1);
        }
    }
    return scan;
}

cv::Mat PlotScanLines(float Max_range, float Min_range, vector<float> &vfRanges, int Scale)
{
    cv::Mat scan = cv::Mat::ones(cv::Size(600,600),CV_8UC3);
    scan = cv::Scalar(50,50,50);
    cv::Point2f center(scan.cols/2, scan.rows/2);
    float angle_increment = 0.352;
    float angle_start = -120.032;

    // Draw Lines First
    for(int i=0; i < vfRanges.size(); i++)
    {
        float range = vfRanges[i];
        if(range < Min_range || range > Max_range)
        {
            continue;
        }
        else
        {
            float current_angle = angle_start + i * angle_increment;

            float px = range * cos(current_angle/180*PI) * Scale;
            float py = range * sin(current_angle/180*PI) * Scale;

            cv::Point2f ScanPosition = center + cv::Point2f(px, py);
            cv::line(scan, center, ScanPosition, cv::Scalar(127,0,0), 1);
        }
    }

    // Draw marker after lines
    cv::arrowedLine(scan, center, center+cv::Point2f(20,0), cv::Scalar(0,0,255), 2, 8, 0, 0.2);
    cv::arrowedLine(scan, center, center+cv::Point2f(0,20), cv::Scalar(0,255,0), 2, 8, 0, 0.2);
    cv::drawMarker(scan, center, cv::Scalar(255,0,0), cv::MARKER_TILTED_CROSS, 10, 1);
    return scan;
}
