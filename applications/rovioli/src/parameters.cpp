#include "rovioli/parameters.h"

#include <string>
#include <vector>

double INIT_DEPTH;
double MIN_PARALLAX;
double ACC_N, ACC_W;
double GYR_N, GYR_W;

Eigen::Vector3d ACC_SCALE, ACC_BIAS, GYRO_SCALE, GYRO_BIAS;

std::vector<Eigen::Matrix3d> RIC;
std::vector<Eigen::Vector3d> TIC;

Eigen::Vector3d G{0.0, 0.0, 9.8};

double BIAS_ACC_THRESHOLD;
double BIAS_GYR_THRESHOLD;
double SOLVER_TIME;
int NUM_ITERATIONS;
int ESTIMATE_EXTRINSIC;

int LOOP_CLOSURE =
    0;  // configration for wether run loop closure. Load from fsSettings.
std::atomic<int> LOCALIZE_2D3D(0);
int MIN_LOOP_NUM;

int nb_skip_frame = 5;

int IMAGE_ROW, IMAGE_COL;

int MAX_KEYFRAME_NUM;

Encoder::DifferentialEncoderParameters ENCODER_PARAM;

double FOCAL_LENGTH = 460.0;

bool CameraParamsTxtParser::loadDataParamFromPlainText(
    const std::string cameraConfig) {
  extrinsicTrans =
      cv::Mat(3, 1, CV_64FC1);             // fisheye camera frame to IMU frame
  extrinsicRot = cv::Mat(3, 3, CV_64FC1);  // fisheye camera frame to IMU frame

  std::string cameraParamFile = cameraConfig;

  std::ifstream ifs(cameraParamFile);
  if (!ifs.is_open()) {
    std::cerr << "Failed to open cameraParam file: " << cameraParamFile
              << std::endl;
    return false;
  }

  std::string oneLine;

  std::string s;

  std::stringstream stream(oneLine);
  while (!ifs.eof()) {
    std::getline(ifs, oneLine);

    if (oneLine.find("fisheye intrinsic::") != std::string::npos) {
      std::getline(ifs, oneLine);  // skip
      std::getline(ifs, oneLine);
      stream.clear();
      stream.str(oneLine);
      std::getline(stream, s, ' ');
      fx = atof(s.c_str());

      std::getline(stream, s, ' ');  // skip
      std::getline(stream, s, ' ');
      cx = atof(s.c_str());

      std::getline(ifs, oneLine);
      stream.clear();
      stream.str(oneLine);
      std::getline(stream, s, ' ');  // skip
      std::getline(stream, s, ' ');
      fy = atof(s.c_str());

      std::getline(stream, s, ' ');
      cy = atof(s.c_str());

      std::getline(ifs, oneLine);  // skip
      std::getline(ifs, oneLine);
      stream.clear();
      stream.str(oneLine);
      std::getline(stream, s, ' ');  // skip
      std::getline(stream, s, ' ');  // skip
      std::getline(stream, s, ' ');  // skip
      std::getline(stream, s, ' ');

      w = atof(s.c_str());
    }
    if (oneLine.find("fisheye to imu extrinsic::") != std::string::npos) {
      Eigen::Vector3d t;
      Eigen::Matrix3d R;

      std::getline(ifs, oneLine);
      stream.clear();
      stream.str(oneLine);
      std::getline(stream, s, ' ');  // skip
      std::getline(stream, s, ' ');  // skip
      std::getline(stream, s, ' ');
      extrinsicTrans.at<double>(0) = atof(s.c_str());
      std::getline(stream, s, ' ');
      extrinsicTrans.at<double>(1) = atof(s.c_str());
      std::getline(stream, s, ' ');
      extrinsicTrans.at<double>(2) = atof(s.c_str());

      std::getline(ifs, oneLine);  // skip

      std::getline(ifs, oneLine);
      stream.clear();
      stream.str(oneLine);
      std::getline(stream, s, ' ');
      extrinsicRot.at<double>(0, 0) = atof(s.c_str());
      std::getline(stream, s, ' ');
      extrinsicRot.at<double>(0, 1) = atof(s.c_str());
      std::getline(stream, s, ' ');
      extrinsicRot.at<double>(0, 2) = atof(s.c_str());

      std::getline(ifs, oneLine);
      stream.clear();
      stream.str(oneLine);
      std::getline(stream, s, ' ');
      extrinsicRot.at<double>(1, 0) = atof(s.c_str());
      std::getline(stream, s, ' ');
      extrinsicRot.at<double>(1, 1) = atof(s.c_str());
      std::getline(stream, s, ' ');
      extrinsicRot.at<double>(1, 2) = atof(s.c_str());

      std::getline(ifs, oneLine);
      stream.clear();
      stream.str(oneLine);
      std::getline(stream, s, ' ');
      extrinsicRot.at<double>(2, 0) = atof(s.c_str());
      std::getline(stream, s, ' ');
      extrinsicRot.at<double>(2, 1) = atof(s.c_str());
      std::getline(stream, s, ' ');
      extrinsicRot.at<double>(2, 2) = atof(s.c_str());
    }

    if (oneLine.find("IMU Accel intrinsic::") != std::string::npos) {
      std::getline(ifs, oneLine);
      stream.clear();
      stream.str(oneLine);

      std::string tem;
      stream >> tem >> tem >> accel_scale[0] >> accel_scale[1] >>
          accel_scale[2];
      // std::cout<<accel_scale[0]<<" "<<accel_scale[1]<<"
      // "<<accel_scale[2]<<std::endl;

      std::getline(ifs, oneLine);
      stream.clear();
      stream.str(oneLine);
      stream >> tem >> tem >> accel_bias[0] >> accel_bias[1] >> accel_bias[2];
      // std::cout<<accel_bias[0]<<" "<<accel_bias[1]<<"
      // "<<accel_bias[2]<<std::endl;

      std::getline(ifs, oneLine);
      stream.clear();
      stream.str(oneLine);
      std::getline(stream, s, ' ');  // skip
      std::getline(stream, s, ' ');  // skip
      std::getline(stream, s, ' ');  // skip
      std::getline(stream, s, ' ');
      acc_w = std::sqrt(atof(s.c_str()));

      std::getline(ifs, oneLine);
      stream.clear();
      stream.str(oneLine);
      std::getline(stream, s, ' ');  // skip
      std::getline(stream, s, ' ');  // skip
      std::getline(stream, s, ' ');  // skip
      std::getline(stream, s, ' ');
      acc_n = std::sqrt(atof(s.c_str()));
    }

    if (oneLine.find("IMU Gyro intrinsic::") != std::string::npos) {
      std::getline(ifs, oneLine);
      stream.clear();
      stream.str(oneLine);
      std::string tem;
      stream >> tem >> tem >> gyro_scale[0] >> gyro_scale[1] >> gyro_scale[2];

      std::getline(ifs, oneLine);
      stream.clear();
      stream.str(oneLine);
      stream >> tem >> tem >> gyro_bias[0] >> gyro_bias[1] >> gyro_bias[2];

      std::getline(ifs, oneLine);
      stream.clear();
      stream.str(oneLine);
      std::getline(stream, s, ' ');  // skip
      std::getline(stream, s, ' ');  // skip
      std::getline(stream, s, ' ');  // skip
      std::getline(stream, s, ' ');

      gyr_w = std::sqrt(atof(s.c_str()));

      std::getline(ifs, oneLine);
      stream.clear();
      stream.str(oneLine);
      std::getline(stream, s, ' ');  // skip
      std::getline(stream, s, ' ');  // skip
      std::getline(stream, s, ' ');  // skip
      std::getline(stream, s, ' ');
      gyr_n = std::sqrt(atof(s.c_str()));
    }
  }
  return true;
}
//    const ninebot_algo::MotionModuleCalibration getMotionModuleCalibration()
//    {
//        ninebot_algo::MotionModuleCalibration mmc;
//        float (*fk)[3] = mmc.fishEyeIntrinsics.kf;
//        fk[0][0] = fx; fk[0][1] = 0;  fk[0][2] = cx;
//        fk[1][0] = 0;  fk[1][1] = fy; fk[1][2] = cy;
//        fk[2][0] = 0;  fk[2][1] = 0;  fk[2][2] = 1;

//        mmc.fishEyeIntrinsics.distortion[0] = w;
//        mmc.fishEyeIntrinsics.distortion[1] = 0;
//        mmc.fishEyeIntrinsics.distortion[2] = 0;
//        mmc.fishEyeIntrinsics.distortion[3] = 0;
//        mmc.fishEyeIntrinsics.distortion[4] = 0;

//        float (*fp)[3] = mmc.fishEyeToImu.rotation;
//        fp[0][0] = extrinsicRot.at<double>(0,0); fp[0][1] =
//        extrinsicRot.at<double>(0,1); fp[0][2] = extrinsicRot.at<double>(0,2);
//        fp[1][0] = extrinsicRot.at<double>(1,0); fp[1][1] =
//        extrinsicRot.at<double>(1,1); fp[1][2] = extrinsicRot.at<double>(1,2);
//        fp[2][0] = extrinsicRot.at<double>(2,0); fp[2][1] =
//        extrinsicRot.at<double>(2,1); fp[2][2] = extrinsicRot.at<double>(2,2);

//        mmc.fishEyeToImu.x = extrinsicTrans.at<double>(0);
//        mmc.fishEyeToImu.y = extrinsicTrans.at<double>(1);
//        mmc.fishEyeToImu.z = extrinsicTrans.at<double>(2);

//        return mmc;
//    }

// const ninebot_algo::MotionModuleCalibration parseMotionSystemParamFile(const
// std::string motionSystemParamFile)
//{
//    CameraParamsTxtParser parser;
//    parser.loadDataParamFromPlainText(motionSystemParamFile);
//    return parser.getMotionModuleCalibration();
//}
