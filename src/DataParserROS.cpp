#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <chrono>
#include <unistd.h>
#include <cmath>

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>

#define PI M_PI

using namespace std;

void LoadImages(vector<int> &vnFileindex, vector<double> &vlTimestamps, string strSessionFolder, string strSceneName, string strSensorID)
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
            vlTimestamps.push_back(double(lTstamp*1e-7));
        }
    }
    fSequence.close();
}

void LoadScans(vector<int> &vnFileFolder, vector<int> &vnFileindex, vector<double> &vlTimestamps, string strSequenceFolder, string strSensorID)
{
    int folder_index = 1;
    while(true)
    {
        string fname(strSequenceFolder+to_string(folder_index)+"_hokuyo_processed.txt");
        ifstream fSequence(fname.c_str());

        if(fSequence.fail())
        {
            cout << "[Robot@Home Parser] Scanning " << fname << " ...Failed" << endl;
            cout << "[Robot@Home Parser] Load " << folder_index-1 << " Sequences" << endl;
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
                vlTimestamps.push_back(double(lTstamp*1e-7));
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
    scan = cv::Scalar(255,255,255);
    cv::Point2f center(scan.cols/2, scan.rows/2);
    float angle_increment = 0.352;
    float angle_start = -120.032;

    cv::circle(scan, center, 1, cv::Scalar(0,0,0), 4);
    cv::line(scan, center, center+cv::Point2f(10,0), cv::Scalar(0,0,0), 2);
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
            cv::circle(scan, ScanPosition, 1, cv::Scalar(0,0,255), 2);
        }
    }
    return scan;
}

void PubScans(ros::Publisher &pub_, float Max_range, float Min_range, vector<float> &vfRanges)
{
    sensor_msgs::LaserScan msg;

    msg.header.frame_id = string("laser");
    msg.header.stamp = ros::Time::now();
    msg.angle_min = -120.032/180*PI;
    msg.angle_max = 120.032/180*PI;
    msg.angle_increment = 0.352/180*PI;

    msg.time_increment = 0.00015;
    msg.scan_time = 0.1;

    msg.range_min = Min_range;
    msg.range_max = Max_range;

    msg.ranges = vfRanges;
    pub_.publish(msg);
}

int main(int argc, char **argv)
{
    if(argc != 3)
    {
        cerr << endl << "Usage: ./data_parser_ros path_to_atHomeDataset path_to_atHomeDataset_settings " << endl;
        return 1;
    }

    // ROS setup
    ros::init(argc, argv, "RobotAtHome");
    ros::NodeHandle n_;
    ros::Publisher pub_ = n_.advertise<sensor_msgs::LaserScan>("/scan", 1000);

    // Load Parameters
    cv::FileStorage fSettings(string(argv[2]), cv::FileStorage::READ);
    if(!fSettings.isOpened())
    {
       cerr << "Failed to open settings file at: " << string(argv[2]) << endl;
       exit(-1);
    }
    string DatasetRoot(argv[1]);
    string Session = fSettings["Dataset.session"];
    string Sensor = fSettings["Dataset.sensor_type"];
    string Sensor_ID = fSettings["Dataset.sensor_id"];
    string TargetScene = fSettings["Dataset.scene"];
    
    // Combine parameters to generate full path
    string SessionFolder(DatasetRoot+"/Robot@Home-dataset_"+Sensor+"-plain_text-all/Robot@Home-dataset_"+Sensor+"-plain_text-"+Session+"/"+Session+"/");

    if(Sensor.compare("laser_scans")==0)
    {
        string SequenceFolder(SessionFolder+TargetScene+"/");
        // Load Scnas
        vector<int> vnFolderIndex;
        vector<int> vnFileindex;
        vector<double> vlTimestamps;
        LoadScans(vnFolderIndex, vnFileindex, vlTimestamps, SequenceFolder, Sensor_ID);

        long lCurrentTimestamp = 0;
        int nScans = vnFileindex.size();
        for(int i=0; i<nScans; i++)
        {
            std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();

            double tframe = vlTimestamps[i];
            string ScanFolder = SequenceFolder + to_string(vnFolderIndex[i])+"_hokuyo_processed/";
            string Scanpath = ScanFolder+to_string(vnFileindex[i])+"_scan.txt";

            float max_ranges, min_ranges;
            vector<float> vfRanges;
            ReadScans(Scanpath, max_ranges, min_ranges, vfRanges);
            cv::Mat ScanPlot = PlotScans(max_ranges, min_ranges, vfRanges, 60);
            PubScans(pub_, max_ranges, min_ranges, vfRanges);

            cv::imshow("Scan", ScanPlot);
            if(cv::waitKey(1)==27)
            {
                cv::destroyAllWindows();
                ros::shutdown();
                break;
            }

            std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();

            double ttrack= std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count();
            double T=0;
            if(i<nScans-1)
                T = vlTimestamps[i+1]-tframe;
            else if(i>0)
                T = tframe-vlTimestamps[i-1];
            if(ttrack<T)
                usleep((T-ttrack)*1e6);
        }
    }
    else if(Sensor.compare("rgbd_data")==0)
    {
        string rgbd_TargetScene(TargetScene + "_rgbd");
        string SequenceFolder(SessionFolder+rgbd_TargetScene);
        // Load Images
        vector<int> vnFileindex;
        vector<double> vlTimestamps;
        LoadImages(vnFileindex, vlTimestamps, SessionFolder, rgbd_TargetScene, Sensor_ID);

        long lCurrentTimestamp = 0;
        int nImages = vnFileindex.size();
        for(int i=0; i<nImages; i++)
        {
            std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();

            double tframe = vlTimestamps[i];
            string ImgPrefix = SequenceFolder + "/" + to_string(vnFileindex[i]);
            
            string ImgRGBpath = ImgPrefix+"_intensity.png";
            string ImgDepthPath = ImgPrefix+"_depth.png";

            // Load and rotate images
            cv::Mat mImgRGB = cv::imread(ImgRGBpath.c_str());
            cv::rotate(mImgRGB, mImgRGB, cv::ROTATE_90_COUNTERCLOCKWISE);
            cv::Mat mImgDepth = cv::imread(ImgDepthPath.c_str(), cv::IMREAD_ANYDEPTH);
            cv::rotate(mImgDepth, mImgDepth, cv::ROTATE_90_COUNTERCLOCKWISE);

            cv::imshow("RGB", mImgRGB);
            cv::imshow("Depth", mImgDepth);
            cv::waitKey(1);

            std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();

            double ttrack= std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count();
            double T=0;
            if(i<nImages-1)
                T = vlTimestamps[i+1]-tframe;
            else if(i>0)
                T = tframe-vlTimestamps[i-1];
            if(ttrack<T)
                usleep((T-ttrack)*1e6);
        }
    }
    ros::shutdown();
}
