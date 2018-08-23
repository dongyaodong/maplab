#ifndef ROVIOLI_PARAMETERS_H_  // NOLINT
#define ROVIOLI_PARAMETERS_H_

#include <atomic>
#include <fstream>
#include <string>
#include <vector>

#include <Eigen/Dense>

#include <opencv2/core/eigen.hpp>
#include <opencv2/opencv.hpp>

// #include "ninebot_log/switchableRos.h"

// #include "utility/RawDataUtility.hpp" // MotionModuleCalibration
// #include "utility/utility.h"

#define OBSOLETE_IMPL_EXIT()                                              \
  do {                                                                    \
    char buffer[512];                                                     \
    snprintf(                                                             \
        buffer, 512,                                                      \
        "Attempt to use an obsolete implementation of %s at %s line[%d]", \
        __func__, __FILE__, __LINE__);                                    \
    throw std::runtime_error(buffer);                                     \
  } while (0)

namespace Encoder {

struct DifferentialEncoderParameters {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  DifferentialEncoderParameters()
      : TickToLength(0.0092),
        BaseLength(0.4620),
        spineHeight(0.3885),
        Kl(0.000005),
        Kr(0.000005),
        vector_head_imu(0.0465),
        Use_encoder_constraint(true),
        Add_consttraints_threshold(10.0) {}

  DifferentialEncoderParameters(
      double tickToLength, double baseLength, double spineheight, double kl,
      double kr, double vec_head_imu, bool use_encoder_constraint,
      double add_consttraints_threshold)
      : TickToLength(tickToLength),
        BaseLength(baseLength),
        spineHeight(spineheight),
        Kl(kl),
        Kr(kr),
        vector_head_imu(vec_head_imu),
        Use_encoder_constraint(use_encoder_constraint),
        Add_consttraints_threshold(add_consttraints_threshold) {}

  double TickToLength;
  double BaseLength;
  double spineHeight;
  double Kl;  // two params ralated to encoder input covariance
  double Kr;
  double vector_head_imu;
  bool Use_encoder_constraint;
  double Add_consttraints_threshold;
};
}  // namespace Encoder

extern double FOCAL_LENGTH;
const int WINDOW_SIZE = 10;
const int NUM_OF_CAM = 1;
const int NUM_OF_F = 1000;
const double LOOP_INFO_VALUE = 50.0;

#define UNIT_SPHERE_ERROR

extern double INIT_DEPTH;
extern double MIN_PARALLAX;
extern int ESTIMATE_EXTRINSIC;

extern double ACC_N, ACC_W;
extern double GYR_N, GYR_W;

extern Eigen::Vector3d ACC_SCALE;
extern Eigen::Vector3d ACC_BIAS;

extern Eigen::Vector3d GYRO_SCALE;
extern Eigen::Vector3d GYRO_BIAS;

extern std::vector<Eigen::Matrix3d> RIC;  // Assume this parameter is only
                                          // changed in initialization and
                                          // estimation of extrinsic parameters,
                                          // Otherwise, it is fixed and read
                                          // safe
extern std::vector<Eigen::Vector3d> TIC;
extern Eigen::Vector3d G;

extern double BIAS_ACC_THRESHOLD;
extern double BIAS_GYR_THRESHOLD;
extern double SOLVER_TIME;
extern int NUM_ITERATIONS;
extern std::string EX_CALIB_RESULT_PATH;

extern std::string VINS_FOLDER_PATH;  // e.g., /home/USERNAME/vins_mono_segway

extern int LOOP_CLOSURE;
extern int nb_skip_frame;
extern std::atomic<int> LOCALIZE_2D3D;
extern int MIN_LOOP_NUM;
extern int MAX_KEYFRAME_NUM;
extern std::string PATTERN_FILE;
extern std::string VOC_FILE;
extern std::string CAM_NAMES;

extern int IMAGE_ROW, IMAGE_COL;

// extern Encoder::DifferentialEncoderParameters ENCODER_PARAM;

enum SIZE_PARAMETERIZATION {
  SIZE_POSE = 7,
  SIZE_SPEEDBIAS = 9,
  SIZE_FEATURE = 1
};

enum StateOrder { O_P = 0, O_R = 3, O_V = 6, O_BA = 9, O_BG = 12 };

enum NoiseOrder { O_AN = 0, O_GN = 3, O_AW = 6, O_GW = 9 };

struct CameraParamsTxtParser {
  double fx;
  double fy;
  double cx;
  double cy;

  double w;
  cv::Mat extrinsicTrans;  // fisheye camera frame to IMU frame
  cv::Mat extrinsicRot;    // fisheye camera frame to IMU frame

  double acc_n;
  double gyr_n;
  double acc_w;
  double gyr_w;

  double accel_scale[3];
  double accel_bias[3];
  double gyro_scale[3];
  double gyro_bias[3];

  // TODO(jhuai): Currently, this function only retrieves a portion of all
  // parameters
  // pertaining the fisheye camera, its intrinsic and extrinsic parameters.
  // In the future, load also the rest of all parameters
  bool loadDataParamFromPlainText(const std::string cameraConfig);

  //    const ninebot_algo::MotionModuleCalibration
  //    getMotionModuleCalibration();
};
// const ninebot_algo::MotionModuleCalibration parseMotionSystemParamFile(const
// std::string motionSystemParamFile);

#endif  // ROVIOLI_PARAMETERS_H_ // NOLINT
