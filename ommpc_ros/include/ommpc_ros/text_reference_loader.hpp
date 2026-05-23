#ifndef OMMPC_ROS_TEXT_REFERENCE_LOADER_HPP
#define OMMPC_ROS_TEXT_REFERENCE_LOADER_HPP

#include <istream>
#include <string>

#include "ommpc_core/types.hpp"

namespace ommpc_ros
{

class TextReferenceLoader
{
public:
  bool loadFromPackagePath(const std::string &package_name,
                           const ommpc_core::TextReferenceConfig &config,
                           ommpc_core::ReferenceTrajectory &trajectory,
                           std::string *error_message = nullptr) const;

  void sampleWindow(const ommpc_core::ReferenceTrajectory &source,
                    int start_index,
                    int horizon_steps,
                    double sample_dt_s,
                    ommpc_core::ReferenceTrajectory &window) const;

private:
  bool load(std::istream &input,
            double sample_dt_s,
            ommpc_core::ReferenceTrajectory &trajectory,
            std::string *error_message) const;

  static bool parseLine(const std::string &line,
                        double time_from_start_s,
                        ommpc_core::ReferencePoint &point);
};

} // namespace ommpc_ros

#endif
