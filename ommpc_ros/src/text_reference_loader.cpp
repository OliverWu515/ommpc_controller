#include "ommpc_ros/text_reference_loader.hpp"

#include <algorithm>
#include <fstream>
#include <sstream>

#include <ros/package.h>

namespace ommpc_ros
{

bool TextReferenceLoader::loadFromPackagePath(const std::string &package_name,
                                              const ommpc_core::TextReferenceConfig &config,
                                              ommpc_core::ReferenceTrajectory &trajectory,
                                              std::string *error_message) const
{
  const std::string path = ros::package::getPath(package_name) + config.ref_filename;
  std::ifstream file(path.c_str());
  if (!file.is_open())
  {
    if (error_message != nullptr)
    {
      *error_message = "Failed to open text trajectory file: " + path;
    }
    return false;
  }

  const bool loaded = load(file, config.time_step, trajectory, error_message);
  file.close();
  return loaded;
}

void TextReferenceLoader::sampleWindow(const ommpc_core::ReferenceTrajectory &source,
                                       int start_index,
                                       int horizon_steps,
                                       double sample_dt_s,
                                       ommpc_core::ReferenceTrajectory &window) const
{
  if (source.empty())
  {
    window.clear();
    window.sample_dt_s = sample_dt_s;
    return;
  }

  window.sample_dt_s = sample_dt_s;
  window.resize(horizon_steps + 1);

  const int last_idx = static_cast<int>(source.size()) - 1;
  const int clamped_start_idx = std::max(0, std::min(start_index, last_idx));
  for (int i = 0; i <= horizon_steps; ++i)
  {
    const int ref_idx = std::min(clamped_start_idx + i, last_idx);
    const auto &src_point = source.points.at(ref_idx);
    auto &dst_point = window.points.at(i);
    dst_point = src_point;
    dst_point.time_from_start_s = i * sample_dt_s;
    if (ref_idx == last_idx)
    {
      dst_point.velocity = Eigen::Vector3d::Zero();
      dst_point.acceleration.setZero();
      dst_point.jerk.setZero();
    }
  }
}

bool TextReferenceLoader::load(std::istream &input,
                               double sample_dt_s,
                               ommpc_core::ReferenceTrajectory &trajectory,
                               std::string *error_message) const
{
  trajectory.clear();
  trajectory.sample_dt_s = sample_dt_s;

  std::string line;
  int index = 0;
  while (std::getline(input, line))
  {
    if (line.empty())
    {
      continue;
    }

    ommpc_core::ReferencePoint point;
    if (!parseLine(line, index * sample_dt_s, point))
    {
      continue;
    }
    trajectory.points.push_back(point);
    ++index;
  }

  if (trajectory.empty())
  {
    if (error_message != nullptr)
    {
      *error_message = "Text trajectory file does not contain any valid reference point.";
    }
    return false;
  }

  return true;
}

bool TextReferenceLoader::parseLine(const std::string &line,
                                    double time_from_start_s,
                                    ommpc_core::ReferencePoint &point)
{
  std::istringstream linestream(line);
  std::vector<double> values;
  double value = 0.0;
  while (linestream >> value)
  {
    values.push_back(value);
  }

  if (values.size() < 7)
  {
    return false;
  }

  point.time_from_start_s = time_from_start_s;
  point.position = Eigen::Vector3d(values[0], values[1], values[2]);
  point.velocity = Eigen::Vector3d(values[3], values[4], values[5]);
  point.acceleration.setZero();
  point.jerk.setZero();
  point.yaw = values.size() > 6 ? values[6] : 0.0;
  point.yaw_rate = values.size() > 7 ? values[7] : 0.0;
  return true;
}

} // namespace ommpc_ros
