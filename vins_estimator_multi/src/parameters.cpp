#include "parameters.h"

double INIT_DEPTH;
double MIN_PARALLAX;
double ACC_N, ACC_W;
double GYR_N, GYR_W;

std::vector<Eigen::Matrix3d> RIC;
std::vector<Eigen::Vector3d> TIC;

Eigen::Vector3d G{0.0, 0.0, 9.8};

double BIAS_ACC_THRESHOLD;
double BIAS_GYR_THRESHOLD;
double SOLVER_TIME;
int NUM_ITERATIONS;
int ESTIMATE_EXTRINSIC;
int ESTIMATE_TD;
int ROLLING_SHUTTER;
std::string EX_CALIB_RESULT_PATH;
std::string VINS_RESULT_PATH;
std::string IMU_TOPIC;
double ROW, COL;
double TD, TR;

// add by sst
Eigen::Vector3d tc21;
Eigen::Quaterniond qc21;

template <typename T>
T readParam(ros::NodeHandle &n, std::string name)
{
    T ans;
    if (n.getParam(name, ans))
    {
        ROS_INFO_STREAM("Loaded " << name << ": " << ans);
    }
    else
    {
        ROS_ERROR_STREAM("Failed to load " << name);
        n.shutdown();
    }
    return ans;
}

void readParameters(ros::NodeHandle &n)
{
    std::string config_file;
    config_file = readParam<std::string>(n, "config_file");
    cv::FileStorage fsSettings(config_file, cv::FileStorage::READ);
    if (!fsSettings.isOpened())
    {
        std::cerr << "ERROR: Wrong path to settings" << std::endl;
    }

    fsSettings["imu_topic"] >> IMU_TOPIC;

    SOLVER_TIME = fsSettings["max_solver_time"];
    NUM_ITERATIONS = fsSettings["max_num_iterations"];
    MIN_PARALLAX = fsSettings["keyframe_parallax"];
    MIN_PARALLAX = MIN_PARALLAX / FOCAL_LENGTH;

    std::string OUTPUT_PATH;
    fsSettings["output_path"] >> OUTPUT_PATH;
    VINS_RESULT_PATH = OUTPUT_PATH + "/vins_result_no_loop.csv";
    std::cout << "result path " << VINS_RESULT_PATH << std::endl;
    std::ofstream fout(VINS_RESULT_PATH, std::ios::out);
    fout.close();

    ACC_N = fsSettings["acc_n"];
    ACC_W = fsSettings["acc_w"];
    GYR_N = fsSettings["gyr_n"];
    GYR_W = fsSettings["gyr_w"];
    G.z() = fsSettings["g_norm"];
    ROW = fsSettings["image_height"];
    COL = fsSettings["image_width"];
    ROS_INFO("ROW: %f COL: %f ", ROW, COL);

    ESTIMATE_EXTRINSIC = fsSettings["estimate_extrinsic"];
    if (ESTIMATE_EXTRINSIC == 2)
    {
        ROS_WARN("have no prior about extrinsic param, calibrate extrinsic param");
        RIC.push_back(Eigen::Matrix3d::Identity());
        TIC.push_back(Eigen::Vector3d::Zero());
        EX_CALIB_RESULT_PATH = OUTPUT_PATH + "/extrinsic_parameter.csv";
    }
    else
    {
        if (ESTIMATE_EXTRINSIC == 1)
        {
            ROS_WARN(" Optimize extrinsic param around initial guess!");
            EX_CALIB_RESULT_PATH = OUTPUT_PATH + "/extrinsic_parameter.csv";
        }
        if (ESTIMATE_EXTRINSIC == 0)
            ROS_WARN(" fix extrinsic param ");

        for (int i = 0; i < NUM_OF_CAM; i++)
        {
            cv::Mat cv_R, cv_T;
            char str_R[64] = "extrinsicRotation";
            char str_T[64] = "extrinsicTranslation";
            if (i > 1)
            {
                sprintf(str_R, "extrinsicRotation_%d", i);
                sprintf(str_T, "extrinsicTranslation_%d", i);
            }
            fsSettings[str_R] >> cv_R;
            fsSettings[str_T] >> cv_T;

            cv_T = -cv_R.t() * cv_T;
            cv_R = cv_R.t();

            Eigen::Matrix3d eigen_R;
            Eigen::Vector3d eigen_T;
            cv::cv2eigen(cv_R, eigen_R);
            cv::cv2eigen(cv_T, eigen_T);
            Eigen::Quaterniond Q(eigen_R);
            eigen_R = Q.normalized();
            RIC.push_back(eigen_R);
            TIC.push_back(eigen_T);
            ROS_INFO_STREAM("Extrinsic_R_" << i << std::endl
                                           << RIC[i]);
            ROS_INFO_STREAM("Extrinsic_T_" << i << std::endl
                                           << TIC[i].transpose());
        }
    }

#if 1
    // cam0 2 cam1
    cv::Mat cv_R21, cv_T21;
    fsSettings["stereoRotation"] >> cv_R21;
    fsSettings["stereoTranslation"] >> cv_T21;

    Eigen::Matrix3d eigen_R21;
    Eigen::Vector3d eigen_T21;
    cv::cv2eigen(cv_R21, eigen_R21);
    cv::cv2eigen(cv_T21, eigen_T21);
    Eigen::Quaterniond Q21(eigen_R21);
    eigen_R21 = Q21.normalized();
    qc21 = Q21.normalized();
    tc21 = eigen_T21;
    ROS_INFO_STREAM("stereo_R : " << std::endl
                                  << eigen_R21);
    ROS_INFO_STREAM("stereo_T : " << std::endl
                                  << tc21.transpose());
#endif

    INIT_DEPTH = 5.0;
    BIAS_ACC_THRESHOLD = 0.1;
    BIAS_GYR_THRESHOLD = 0.1;

    TD = fsSettings["td"];
    ESTIMATE_TD = fsSettings["estimate_td"];
    if (ESTIMATE_TD)
        ROS_INFO_STREAM("Unsynchronized sensors, online estimate time offset, initial td: " << TD);
    else
        ROS_INFO_STREAM("Synchronized sensors, fix time offset: " << TD);

    ROLLING_SHUTTER = fsSettings["rolling_shutter"];
    if (ROLLING_SHUTTER)
    {
        TR = fsSettings["rolling_shutter_tr"];
        ROS_INFO_STREAM("rolling shutter camera, read out time per line: " << TR);
    }
    else
    {
        TR = 0;
    }

    fsSettings.release();
}
