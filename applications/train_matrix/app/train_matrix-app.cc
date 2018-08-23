#include <fstream>
#include <iostream>  // NOLINT
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <descriptor-projection/build-projection-matrix.h>
#include <descriptor-projection/descriptor-projection.h>
#include <descriptor-projection/flags.h>
#include <descriptor-projection/map-track-extractor.h>
#include <descriptor-projection/train-projection-matrix.h>
#include <gflags/gflags.h>
#include <glog/logging.h>
#include <localization-summary-map/localization-summary-map-creation.h>
#include <localization-summary-map/localization-summary-map.h>
#include <loopclosure-common/flags.h>
#include <loopclosure-common/types.h>
#include <maplab-common/binary-serialization.h>
#include <ros/ros.h>
#include <vi-map/vi-map-serialization.h>
#include <vi-map/vi-map.h>

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
DEFINE_string(vi_map_folder, "", "Path to a full VI-map used for training.");
DEFINE_string(
    vi_map_list, "", "Path to a name.txt containing paths of VI-map folders.");

DECLARE_bool(map_builder_save_image_as_resources);

struct SubSetCovariance {
  unsigned int sample_size_matches;
  unsigned int sample_size_non_matches;
  Eigen::MatrixXf cov_matches;
  Eigen::MatrixXf cov_non_matches;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

int main(int argc, char** argv) {
  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, true);
  google::InstallFailureSignalHandler();
  FLAGS_alsologtostderr = true;
  FLAGS_colorlogtostderr = true;

  ros::init(argc, argv, "train_matrix");
  ros::NodeHandle nh;

  // Load VI-map

  CHECK_NE(FLAGS_lc_projection_matrix_filename, "")
      << "You have to provide a filename to write the projection matrix to.";

  CHECK(FLAGS_vi_map_folder != "" || FLAGS_vi_map_list != "")
      << "You have to provide a VI map or a file which contains the paths of "
         "VI map folders.";

  std::vector<std::string> vi_maps;
  if (!FLAGS_vi_map_list.empty()) {
    std::ifstream infile;
    infile.open(FLAGS_vi_map_list.c_str());
    CHECK(infile.is_open()) << "Can not open the map list.";
    std::string map_name;
    while (infile >> map_name) {
      if (map_name != "") {
        vi_maps.push_back(map_name);
      }
    }
  }
  if (!FLAGS_vi_map_folder.empty()) {
    vi_maps.push_back(FLAGS_vi_map_folder);
  }

  Eigen::MatrixXf cov_matches;
  Eigen::MatrixXf cov_non_matches;
  double total_sample_size_matches = 0;
  double total_sample_size_non_matches = 0;

  // Get the descriptor-length.
  unsigned int descriptor_size = -1;
  unsigned int raw_descriptor_matching_threshold = 70;
  if (FLAGS_feature_descriptor_type == loop_closure::kFeatureDescriptorFREAK) {
    descriptor_size = loop_closure::kFreakDescriptorLengthBits;
  } else if (
      FLAGS_feature_descriptor_type == loop_closure::kFeatureDescriptorBRISK) {
    descriptor_size = loop_closure::kBriskDescriptorLengthBits;
  } else {
    CHECK(false) << "Unknown feature descriptor "
                 << FLAGS_feature_descriptor_type;
  }

  std::vector<SubSetCovariance> covariances;
  for (std::string map_folder_in_list : vi_maps) {
    vi_map::VIMap map;
    CHECK(vi_map::serialization::loadMapFromFolder(map_folder_in_list, &map))
        << "Loading " << map_folder_in_list << " failed. ";

    // A list to find the dataset with most data points.
    std::vector<std::pair<vi_map::MissionId, unsigned int> >
        match_counts_and_datasets;

    // For memory reasons we compute sample covariances on every dataset
    // and compute a weighted average afterwards.

    vi_map::MissionIdList all_mission_ids;
    map.getAllMissionIds(&all_mission_ids);

    for (const vi_map::MissionId& mission_id : all_mission_ids) {
      Eigen::MatrixXf all_descriptors;
      SubSetCovariance subset_covariance;
      std::vector<descriptor_projection::Track> tracks;

      using descriptor_projection::CollectAndConvertDescriptors;
      CollectAndConvertDescriptors(
          map, mission_id, descriptor_size, raw_descriptor_matching_threshold,
          &all_descriptors, &tracks);

      descriptor_projection::BuildCovarianceMatricesOfMatchesAndNonMatches(
          descriptor_size, all_descriptors, tracks,
          &subset_covariance.sample_size_matches,
          &subset_covariance.sample_size_non_matches,
          &subset_covariance.cov_matches, &subset_covariance.cov_non_matches);

      match_counts_and_datasets.emplace_back(
          mission_id, subset_covariance.sample_size_matches);

      total_sample_size_matches += subset_covariance.sample_size_matches;
      total_sample_size_non_matches +=
          subset_covariance.sample_size_non_matches;
      covariances.push_back(subset_covariance);
    }
  }

  CHECK(!covariances.empty());
  const int descriptor_dims = covariances[0].cov_matches.cols();

  cov_matches.resize(descriptor_dims, descriptor_dims);
  cov_matches.setZero();
  cov_non_matches.resize(descriptor_dims, descriptor_dims);
  cov_non_matches.setZero();

  for (const SubSetCovariance& subset_covariance : covariances) {
    cov_matches += subset_covariance.cov_matches *
                   subset_covariance.sample_size_matches /
                   total_sample_size_matches;
    cov_non_matches += subset_covariance.cov_non_matches *
                       subset_covariance.sample_size_non_matches /
                       total_sample_size_non_matches;
  }

  // Compute projection matrix

  Eigen::MatrixXf A;
  descriptor_projection::ComputeProjectionMatrix(
      cov_matches, cov_non_matches, &A);

  VLOG(0) << "Trained projection matrix:";
  VLOG(0) << "\tcols: " << A.cols();
  VLOG(0) << "\trows: " << A.rows();
  VLOG(0) << "\tmin: " << A.minCoeff();
  VLOG(0) << "\tmax: " << A.maxCoeff();
  VLOG(0) << "\tmatrix: " << A;

  std::ofstream serializer(
      FLAGS_lc_projection_matrix_filename, std::ofstream::binary);
  common::Serialize(A, &serializer);
  serializer.flush();

  std::cout << "Stored projection matrix to "
            << FLAGS_lc_projection_matrix_filename << std::endl;

  return 0;
}
