//
// Created by shapelim on 21. 10. 18..
//

#include "erasor2/Config.hpp"

#include "dataloader/dataloader.h"
#include "rosparam_server.hpp"
#include "tools/erasor_utils.hpp"

using namespace std;

using PointType = pcl::PointXYZI;

void loadGTLabel(const string &gt_label_dir, const size_t idx, vector<uint32_t> &labels) {
  string label_name = (boost::format("%s/%06d.label") % gt_label_dir % idx).str();
  //    cout << label_name << endl;
  std::ifstream label_input(label_name, std::ios::binary);
  if (!label_input.is_open()) {
    throw invalid_argument("Could not open the label!");
  }
  label_input.seekg(0, std::ios::end);
  uint32_t num_points = label_input.tellg() / sizeof(uint32_t);
  label_input.seekg(0, std::ios::beg);

  labels.resize(num_points);
  label_input.read((char *)&labels[0], num_points * sizeof(uint32_t));
}

void loadEstLabel(const string &est_label_dir, const int i, std::vector<uint32_t> &instance_label) {
  string inst_label_name = (boost::format("%s/%06d.label") % est_label_dir % i).str();
  erasor_utils::load_labels(inst_label_name, instance_label);

  static bool is_initial = true;
  if (is_initial) {
    std::vector<uint32_t> tmp_instance_label = instance_label;
    std::sort(tmp_instance_label.begin(), tmp_instance_label.end());
    auto last = std::unique(tmp_instance_label.begin(), tmp_instance_label.end());
    tmp_instance_label.erase(last, tmp_instance_label.end());
    std::cout << "\033[1;33m[NOTE] Est. label contains ";
    for (int j = 0; j < tmp_instance_label.size(); ++j) {
      std::cout << tmp_instance_label[j];
      if (j < tmp_instance_label.size() - 1) {
        std::cout << ", ";
      } else {
        std::cout << std::endl;
      }
    }
    std::cout << "(check the function `assignLabel()` in `dataloader.cpp`\033[0m" << std::endl;
    is_initial = false;
  }
}

inline vector<float> splitLine(const string &input, char delimiter) {
  vector<float> answer;
  stringstream ss(input);
  string temp;

  while (getline(ss, temp, delimiter)) {
    if (temp.empty() || temp == "\r") {
      continue;
    }
    answer.push_back(stof(temp));
  }
  return answer;
}

inline void vec2tf4x4(vector<float> &pose, Eigen::Matrix4f &tf4x4) {
  for (int idx = 0; idx < 12; ++idx) {
    int i       = idx / 4;
    int j       = idx % 4;
    tf4x4(i, j) = pose[idx];
  }
}

void loadAllPoses(string pose_path, vector<Eigen::Matrix4f> &poses) {
  // Custom/Adaptive-LIO export mode: poses are already T_map_lidar in KITTI
  // row-major 3x4 format. Do not apply SemanticKITTI/SuMa camera-to-lidar
  // conversion here, otherwise accumulated clouds are rotated/misaligned.
  poses.clear();
  poses.reserve(20000);

  std::ifstream in(pose_path);
  if (!in.is_open()) {
    throw invalid_argument("Fail to open pose file: " + pose_path);
  }

  string line;
  int count = 0;
  while (std::getline(in, line)) {
    if (line.empty()) {
      continue;
    }
    vector<float> pose = splitLine(line, ' ');
    if (pose.size() != 12) {
      throw invalid_argument("Invalid pose format in " + pose_path + ": expected 12 KITTI values");
    }

    Eigen::Matrix4f tf4x4_lidar = Eigen::Matrix4f::Identity();
    vec2tf4x4(pose, tf4x4_lidar);
    poses.emplace_back(tf4x4_lidar);
    count++;
  }
  in.close();

  std::cout << "Total " << count
            << " poses are loaded as direct T_map_lidar poses" << std::endl;

  if (count == 0) {
    throw invalid_argument("Fail to load poses. Please check the `pose_path_`");
  }
}

template <typename T>
int loadCloud(const string &cloud_dir, size_t idx, pcl::PointCloud<T> &cloud) {
  string filename = (boost::format("%s/%06d.bin") % cloud_dir % idx).str();
  FILE *file      = fopen(filename.c_str(), "rb");
  if (!file) {
    std::cerr << "Error: failed to load " << filename << std::endl;
    return -1;
  }

  std::vector<float> buffer(2000000);
  size_t num_points =
      fread(reinterpret_cast<char *>(buffer.data()), sizeof(float), buffer.size(), file) / 4;
  fclose(file);

  cloud.resize(num_points);
  if (std::is_same<T, pcl::PointXYZ>::value) {
    for (int i = 0; i < num_points; i++) {
      auto &pt = cloud.at(i);
      pt.x     = buffer[i * 4];
      pt.y     = buffer[i * 4 + 1];
      pt.z     = buffer[i * 4 + 2];
    }
  } else if (std::is_same<T, pcl::PointXYZI>::value) {
    for (int i = 0; i < num_points; i++) {
      auto &pt     = cloud.at(i);
      pt.x         = buffer[i * 4];
      pt.y         = buffer[i * 4 + 1];
      pt.z         = buffer[i * 4 + 2];
      pt.intensity = buffer[i * 4 + 3];
    }
  }
  return 0;
}

int main(int argc, char **argv) {
  if (argc < 2) {
    std::cerr << "Usage: accum_4dmos <config.yaml> [target_mos_type]\n";
    return 1;
  }
  string target_mos_type =
      (argc >= 3) ? std::string(argv[2]) : std::string("benedikt_4dmos_labels");

  std::cout << "4D MOS mapping started" << std::endl;

  const auto cfg = erasor2::Config::fromYaml(argv[1]);
  unique_ptr<RosParamServer> params(new RosParamServer(cfg));

  cout << "From " << params->start_frame_ << " to " << params->end_frame_ << endl;
  cout << params->robot_body_size_ << endl;

  int start_frame    = params->start_frame_;
  int end_frame      = params->end_frame_;
  int accum_interval = params->accum_interval_;

  string abs_data_dir = params->abs_data_dir_;
  string sequence     = params->sequence_;

  string abs_gt_label_dir = abs_data_dir + "/" + sequence + "/labels";
  string abs_label_dir    = abs_data_dir + "/" + sequence + "/" + target_mos_type;
  string abs_cloud_dir    = abs_data_dir + "/" + sequence + "/velodyne";
  string abs_pose_path    = abs_data_dir + "/" + sequence + "/kiss_icp_poses.txt";

  cout << "\033[1;32m" << abs_gt_label_dir << "\n";
  cout << abs_label_dir << "\n";
  cout << abs_cloud_dir << "\n";
  cout << abs_pose_path << "\033[0m\n";

  vector<Eigen::Matrix4f> poses;
  loadAllPoses(abs_pose_path, poses);

  int cnt = 0;
  pcl::PointCloud<pcl::PointXYZI>::Ptr static_map_accum(new pcl::PointCloud<pcl::PointXYZI>);
  pcl::PointCloud<pcl::PointXYZI>::Ptr static_map_voxelized(new pcl::PointCloud<pcl::PointXYZI>);
  static_map_accum->reserve(2000000);
  static_map_voxelized->reserve(2000000);

  Eigen::Matrix4f tf_h_of_ground_to_be_zero = Eigen::Matrix4f::Identity();
  tf_h_of_ground_to_be_zero(2, 3)           = params->sensor_height_;

  for (int i = start_frame; i < end_frame + accum_interval; ++i) {
    signal(SIGINT, erasor_utils::signal_callback_handler);
    if (i % 10 == 0) {
      cout << "[DataLoader] " << i << "th frame comes!\n";
    }

    // if `accum_interval` == 1, the below condition is not used
    if (accum_interval > 1 && ++cnt / accum_interval >= 1) {
      cnt = 0;
      continue;
    }

    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_gt_label(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr est_static_transformed(
        new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr est_static(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr est_dynamic(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr noise(new pcl::PointCloud<pcl::PointXYZI>);

    Eigen::Matrix4f pose = Eigen::Matrix4f::Identity();
    pose                 = poses[i];
    vector<uint32_t> est_labels, gt_labels;
    loadGTLabel(abs_gt_label_dir, i, gt_labels);
    loadEstLabel(abs_label_dir, i, est_labels);
    loadCloud(abs_cloud_dir, i, *cloud_gt_label);

    est_static->reserve(gt_labels.size());
    est_dynamic->reserve(gt_labels.size());
    static_map_accum->reserve(gt_labels.size());

    for (int j = 0; j < cloud_gt_label->points.size(); ++j) {
      auto &pt     = cloud_gt_label->points[j];
      pt.intensity = static_cast<float>(gt_labels[j]);
    }

    // Filtering out dynamic & noisy points
    float max_dist_square = pow(params->robot_body_size_, 2);
    int count             = 0;
    for (auto const &pt : cloud_gt_label->points) {
      double dist_square = pow(pt.x, 2) + pow(pt.y, 2);
      if (dist_square < max_dist_square) {
        noise->points.emplace_back(pt);
      } else {
        if (est_labels[count] == 9.0) {
          est_static->points.emplace_back(pt);
        } else {  // Maybe 251 is dynamic label!
          est_dynamic->points.emplace_back(pt);
        }
      }
      ++count;
    }
    pcl::transformPointCloud(
        *est_static, *est_static_transformed, pose * tf_h_of_ground_to_be_zero);
    (*static_map_accum) += (*est_static_transformed);
  }

  erasor_utils::voxelize_preserving_labels_by_nanoflann(
      static_map_accum, *static_map_voxelized, params->voxel_size_);

  static_map_voxelized->width  = static_map_voxelized->points.size();
  static_map_voxelized->height = 1;
  std::cout << "[Debug]: (" << static_map_voxelized->width << ", " << static_map_voxelized->height
            << ") => " << static_map_voxelized->points.size() << std::endl;
  string static_map_path = params->abs_save_dir_ + "/" + sequence + "_from_" +
                           to_string(start_frame) + "_to_" + to_string(end_frame) + "_" +
                           target_mos_type + ".pcd";
  std::cout << "\033[1;32mSaving the map to pcd...: " << static_map_path << "\033[0m" << std::endl;
  pcl::io::savePCDFileASCII(static_map_path, *static_map_voxelized);

  cout << "[ERASOR2] Complete to set scans and poses\n";

  return 0;
}
