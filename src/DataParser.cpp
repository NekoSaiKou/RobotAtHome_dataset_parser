#include <Common.h>

using namespace std;

int main(int argc, char **argv)
{
    if(argc != 3)
    {
        cerr << endl << "Usage: ./data_parser path_to_atHomeDataset path_to_atHomeDataset_settings " << endl;
        return 1;
    }

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
    string TargetScene = fSettings["Dataset.scene"];
    
    // Create LiDAR and RGBD data path
    string LiDAR_Sensor_ID = fSettings["Dataset.laser_sensor_id"];
    string LiDAR_sensor("laser_scans");
    string LiDAR_SessionFolder(DatasetRoot+"/Robot@Home-dataset_"+LiDAR_sensor+"-plain_text-all/Robot@Home-dataset_"+LiDAR_sensor+"-plain_text-"+Session+"/"+Session+"/");
    string LiDAR_SequenceFolder(LiDAR_SessionFolder+TargetScene+"/");
    vector<int> vnLiDAR_FolderIndex;
    vector<int> vnLiDAR_Fileindex;
    vector<double> vdLiDAR_Timestamps;
    int LiDAR_Start_Sequence = fSettings["Dataset.laser_start_sequence"];

    string RGBD_Sensor_ID = fSettings["Dataset.rgbd_sensor_id"];
    string RGBD_sensor("rgbd_data");
    string RGBD_SessionFolder(DatasetRoot+"/Robot@Home-dataset_"+RGBD_sensor+"-plain_text-all/Robot@Home-dataset_"+RGBD_sensor+"-plain_text-"+Session+"/"+Session+"/");
    string RGBD_TargetScene(TargetScene + "_rgbd");
    string RGBD_SequenceFolder(RGBD_SessionFolder+RGBD_TargetScene);
    vector<int> vnRGBD_Fileindex;
    vector<double> vdRGBD_Timestamps;

    if(Sensor.compare("laser_scans")==0)
    {
        LoadScans(LiDAR_Start_Sequence, vnLiDAR_FolderIndex, vnLiDAR_Fileindex, vdLiDAR_Timestamps, LiDAR_SequenceFolder, LiDAR_Sensor_ID);

        int nScans = vnLiDAR_Fileindex.size();
        for(int i=0; i<nScans; i++)
        {
            std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();

            double tframe = vdLiDAR_Timestamps[i];
            string ScanFolder = LiDAR_SequenceFolder + to_string(vnLiDAR_FolderIndex[i])+"_hokuyo_processed/";
            string Scanpath = ScanFolder+to_string(vnLiDAR_Fileindex[i])+"_scan.txt";

            float max_ranges, min_ranges;
            vector<float> vfRanges;
            ReadScans(Scanpath, max_ranges, min_ranges, vfRanges);

            cv::Mat ScanPlot = PlotScans(max_ranges, min_ranges, vfRanges, 60);
            cv::Mat ScanLinePlot = PlotScanLines(max_ranges, min_ranges, vfRanges, 60);
            cv::imshow("Scan", ScanPlot);
            cv::imshow("Scan Line", ScanLinePlot);
            
            if(cv::waitKey(1)==27)
            {
                break;
            }

            std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();

            double ttrack= std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count();
            double T=0;
            if(i<nScans-1)
                T = vdLiDAR_Timestamps[i+1]-tframe;
            else if(i>0)
                T = tframe-vdLiDAR_Timestamps[i-1];
            if(ttrack<T)
                usleep((T-ttrack)*1e6);
        }
    }
    else if(Sensor.compare("rgbd_data")==0)
    {
        LoadImages(vnRGBD_Fileindex, vdRGBD_Timestamps, RGBD_SessionFolder, RGBD_TargetScene, RGBD_Sensor_ID);

        int nImages = vnRGBD_Fileindex.size();
        for(int i=0; i<nImages; i++)
        {
            std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();

            double tframe = vdRGBD_Timestamps[i];
            string ImgPrefix = RGBD_SequenceFolder + "/" + to_string(vnRGBD_Fileindex[i]);
            
            string ImgRGBpath = ImgPrefix+"_intensity.png";
            string ImgDepthPath = ImgPrefix+"_depth.png";

            // Load and rotate images
            cv::Mat mImgRGB = cv::imread(ImgRGBpath.c_str());
            cv::rotate(mImgRGB, mImgRGB, cv::ROTATE_90_COUNTERCLOCKWISE);
            cv::Mat mImgDepth = cv::imread(ImgDepthPath.c_str(), cv::IMREAD_ANYDEPTH);
            cv::rotate(mImgDepth, mImgDepth, cv::ROTATE_90_COUNTERCLOCKWISE);

            cv::imshow("RGB", mImgRGB);
            cv::imshow("Depth", mImgDepth);
            if(cv::waitKey(1)==27)
            {
                break;
            }

            std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();

            double ttrack= std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count();
            double T=0;
            if(i<nImages-1)
                T = vdRGBD_Timestamps[i+1]-tframe;
            else if(i>0)
                T = tframe-vdRGBD_Timestamps[i-1];
            if(ttrack<T)
                usleep((T-ttrack)*1e6);
        }
    }
    else if(Sensor.compare("both")==0)
    {
        LoadScans(LiDAR_Start_Sequence, vnLiDAR_FolderIndex, vnLiDAR_Fileindex, vdLiDAR_Timestamps, LiDAR_SequenceFolder, LiDAR_Sensor_ID);
        LoadImages(vnRGBD_Fileindex, vdRGBD_Timestamps, RGBD_SessionFolder, RGBD_TargetScene, RGBD_Sensor_ID);

        // Start show data
        int nImages = vnRGBD_Fileindex.size();
        int nScans = vnLiDAR_Fileindex.size();
        int nData = nImages + nScans;
        int nIdxImages = 0;
        int nIdxScans = 0;

        for(int i=0; i<nData; i++)
        {
            std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();

            double tImage = vdRGBD_Timestamps[nIdxImages];
            double tScan = vdLiDAR_Timestamps[nIdxScans];
            double tframe = 0;

            // If Image is closer, then read image
            if(tImage < tScan)
            {
                tframe = tImage;
                string ImgPrefix = RGBD_SequenceFolder + "/" + to_string(vnRGBD_Fileindex[nIdxImages]);
                string ImgRGBpath = ImgPrefix+"_intensity.png";
                string ImgDepthPath = ImgPrefix+"_depth.png";

                // Load and rotate images
                cv::Mat mImgRGB = cv::imread(ImgRGBpath.c_str());
                cv::rotate(mImgRGB, mImgRGB, cv::ROTATE_90_COUNTERCLOCKWISE);
                cv::Mat mImgDepth = cv::imread(ImgDepthPath.c_str(), cv::IMREAD_ANYDEPTH);
                cv::rotate(mImgDepth, mImgDepth, cv::ROTATE_90_COUNTERCLOCKWISE);

                cv::imshow("RGB", mImgRGB);
                cv::imshow("Depth", mImgDepth);
                nIdxImages++;
            }
            else
            {
                tframe = tScan;
                string ScanFolder = LiDAR_SequenceFolder + to_string(vnLiDAR_FolderIndex[nIdxScans])+"_hokuyo_processed/";
                string Scanpath = ScanFolder+to_string(vnLiDAR_Fileindex[nIdxScans])+"_scan.txt";

                float max_ranges, min_ranges;
                vector<float> vfRanges;
                ReadScans(Scanpath, max_ranges, min_ranges, vfRanges);

                cv::Mat ScanPlot = PlotScans(max_ranges, min_ranges, vfRanges, 60);
                cv::Mat ScanLinePlot = PlotScanLines(max_ranges, min_ranges, vfRanges, 60);
                cv::imshow("Scan", ScanPlot);
                cv::imshow("Scan Line", ScanLinePlot);
                nIdxScans++;
            }

            if(cv::waitKey(1)==27)
            {
                break;
            }

            std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();

            double ttrack= std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count();
            double T=0;
            // If there are still other data:
            if(i<nData-1)
            {
                double Tnext = 0;
                if(nIdxImages == nImages)
                {
                    Tnext = vdLiDAR_Timestamps[nIdxScans];
                }
                else if(nIdxScans == nScans)
                {
                    Tnext = vdRGBD_Timestamps[nIdxImages];
                }
                else
                {   
                    double dRGBNext = vdRGBD_Timestamps[nIdxImages];
                    double dScanNext = vdLiDAR_Timestamps[nIdxScans];
                    Tnext = (dRGBNext < dScanNext)?dRGBNext:dScanNext;
                }
                T = Tnext-tframe;
            }
            else if(i>0)
                T = ttrack+1;
            if(ttrack<T)
                usleep((T-ttrack)*1e6);
        }
    }
}
