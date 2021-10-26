#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <opencv2/highgui/highgui.hpp>
#include <chrono>
#include <unistd.h>

using namespace std;

void LoadImages(vector<int> &vnFileindex, vector<double> &vlTimestamps, string strFname, string strSensorID)
{
    ifstream fSequence(strFname.c_str());
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

}

int main(int argc, char **argv)
{
    if(argc != 3)
    {
        cerr << endl << "Usage: ./rgbd_atHome path_to_atHomeDataset path_to_atHomeDataset_settings " << endl;
        return 1;
    }

    // Load Parameters
    cv::FileStorage fSettings(string(argv[2]), cv::FileStorage::READ);
    string DatasetRoot(argv[1]);
    string Session = fSettings["Dataset.session"];
    string Sensor = fSettings["Dataset.sensor_type"];
    string Sensor_ID = fSettings["Dataset.sensor_id"];
    string TargetScene = fSettings["Dataset.scene"];
    
    // Combine parameters to generate full path
    string SequenceFolder(DatasetRoot+"/Robot@Home-dataset_"+Sensor+"-plain_text-all/Robot@Home-dataset_"+Sensor+"-plain_text-"+Session+"/"+Session+"/");
    string Target(TargetScene+".txt");
    string FPath = SequenceFolder+Target;
    string ImgPath = SequenceFolder+TargetScene;

    // Load Images
    cout << "Target: " << endl << SequenceFolder+Target << endl;
    vector<int> vnFileindex;
    vector<double> vlTimestamps;
    LoadImages(vnFileindex, vlTimestamps, FPath, Sensor_ID);

    long lCurrentTimestamp = 0;
    int nImages = vnFileindex.size();
    for(int i=0; i<nImages; i++)
    {
        std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();

        double tframe = vlTimestamps[i];
        string ImgPrefix = ImgPath + "/" + to_string(vnFileindex[i]);
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
