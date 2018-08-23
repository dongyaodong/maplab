#include <memory>

#include <aslam/cameras/camera-pinhole.h>
#include <aslam/cameras/distortion-fisheye.h>
#include <aslam/cameras/ncamera.h>
#include <gflags/gflags.h>
#include <glog/logging.h>

#include "rovioli/parameters.h"

DEFINE_string(
    data_folder, "",
    "data folder of the loomo dataset which contains cameraParam.txt.");
DEFINE_string(
    vio_localization_map_folder, "",
    "Path to a localization summary map or a full VI-map used for "
    "localization.");
DEFINE_string(
    ncamera_calibration, "ncamera.yaml",
    "Path to the camera calibration yaml.");
// TODO(schneith): Unify these two noise definitions.
DEFINE_string(
    imu_parameters_rovio, "imu-rovio.yaml",
    "Path to the imu configuration yaml "
    "for ROVIO.");
DEFINE_string(
    imu_parameters_maplab, "imu-maplab.yaml",
    "Path to the imu configuration yaml for MAPLAB.");
DEFINE_string(
    save_map_folder, "", "Save map to folder; if empty nothing is saved.");
DEFINE_bool(
    overwrite_existing_map, false,
    "If set to true, an existing map will be overwritten on save. Otherwise, a "
    "number will be appended to save_map_folder to obtain an available "
    "folder.");
DEFINE_bool(
    optimize_map_to_localization_map, false,
    "Optimize and process the map into a localization map before "
    "saving it.");

DECLARE_bool(map_builder_save_image_as_resources);

aslam::NCamera::Ptr createTestNFisheyeCamera(size_t num_cameras) {
  std::vector<aslam::Camera::Ptr> cameras;
  Aligned<std::vector, aslam::Transformation> T_C_B_vector;

  for (size_t camera_idx = 0u; camera_idx < num_cameras; ++camera_idx) {
    cameras.push_back(
        aslam::PinholeCamera::createTestCamera<aslam::FisheyeDistortion>());

    // Offset each camera 0.1 m in x direction and rotate it to face forward.
    Eigen::Vector3d position(0.1 * (camera_idx + 1), 0.0, 0.0);
    aslam::Quaternion q_C_B(0.5, 0.5, -0.5, 0.5);
    aslam::Transformation T_C_B(q_C_B, position);
    T_C_B_vector.push_back(T_C_B);
  }

  aslam::NCameraId rig_id;
  rig_id.randomize();
  std::string label("Test camera rig");
  return aslam::NCamera::Ptr(
      new aslam::NCamera(rig_id, T_C_B_vector, cameras, label));
}

aslam::NCamera::Ptr readCameraFromLoomoLog(
    const std::string motionSystemParamFile) {
  CameraParamsTxtParser parser;
  parser.loadDataParamFromPlainText(motionSystemParamFile);

  Eigen::VectorXd params(1);
  params << parser.w;
  aslam::Distortion::UniquePtr distortion(new aslam::FisheyeDistortion(params));

  aslam::PinholeCamera::Ptr camera(
      new aslam::PinholeCamera(
          parser.fx, parser.fy, parser.cx, parser.cy, 640, 480, distortion));
  aslam::CameraId id;
  id.randomize();
  camera->setId(id);
  camera->setLabel("cam0");

  std::vector<aslam::Camera::Ptr> cameras;
  Aligned<std::vector, aslam::Transformation> T_C_B_vector;
  size_t num_cameras = 1;
  for (size_t camera_idx = 0u; camera_idx < num_cameras; ++camera_idx) {
    cameras.push_back(camera);

    Eigen::Vector3d position;
    cv::cv2eigen(parser.extrinsicTrans, position);
    LOG(INFO) << "From cameraParam.txt t_SC " << parser.extrinsicTrans.t()
              << std::endl
              << "R_SC" << std::endl
              << parser.extrinsicRot;
    Eigen::Matrix3d rotation;
    cv::cv2eigen(parser.extrinsicRot, rotation);

    Eigen::JacobiSVD<Eigen::MatrixXd> svd(
        rotation, Eigen::ComputeThinU | Eigen::ComputeThinV);

    rotation = svd.matrixU() * svd.matrixV().transpose();
    LOG(INFO) << "regularized rotation " << std::endl
              << std::setprecision(9) << rotation;

    aslam::Quaternion q_S_C(rotation);
    q_S_C.normalize();

    aslam::Transformation T_C_B(
        q_S_C.conjugated(), -q_S_C.conjugated().rotate(position));

    T_C_B_vector.push_back(T_C_B);
  }

  aslam::NCameraId rig_id;
  rig_id.randomize();
  std::string label("Segway loomo factory calibration");
  return aslam::NCamera::Ptr(
      new aslam::NCamera(rig_id, T_C_B_vector, cameras, label));
}

int main(int argc, char** argv) {
  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, true);
  google::InstallFailureSignalHandler();
  FLAGS_stderrthreshold = 0;  // INFO: 0, WARNING: 1, ERROR: 2, FATAL: 3
  FLAGS_alsologtostderr = true;
  FLAGS_colorlogtostderr = true;

  std::string dataPath(FLAGS_data_folder);
  std::string motionSystemParamFile = dataPath + "/cameraParam.txt";
  aslam::NCamera::Ptr ncamera = readCameraFromLoomoLog(motionSystemParamFile);
  std::string filename = dataPath + "/ncamera_cla.yaml";
  ncamera->saveToYaml(filename);

  aslam::NCamera::Ptr ncamera_loaded = aslam::NCamera::loadFromYaml(filename);
  CHECK_EQ(ncamera_loaded->getNumCameras(), 1) << "#camera loaded from the "
                                                  "newly generated loomo "
                                                  "ncamera yaml should be 1";

  return 0;
}
