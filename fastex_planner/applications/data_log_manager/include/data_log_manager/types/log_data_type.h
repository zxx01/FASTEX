/*
 * @Author: Xiaoxun Zhang
 * @Date: 2024-12-22 10:51:25
 * @LastEditTime: 2026-05-31 21:53:04
 * @Description:
 */

#ifndef _LOG_DATA_TYPE_H_
#define _LOG_DATA_TYPE_H_

#include <memory>
#include <string>

#include "file_utils/file_rw.h"

namespace data_log_manager
{
class TimeLogData
{
  public:
    using SharedPtr = std::shared_ptr<TimeLogData>;
    using UniquePtr = std::unique_ptr<TimeLogData>;

    TimeLogData()
        : clean_(true), iteration_(0), duration_(0.0), avg_time_(0.0), sum_time_(0.0),
          total_runtime_(0.0), txt_name_(""), first_start_time_(""), start_time_(""), end_time_("")
    {
    }
    ~TimeLogData() = default;

    bool clean_;
    int iteration_;
    double duration_, avg_time_, sum_time_, total_runtime_;
    std::string txt_name_;
    std::string first_start_time_, start_time_, end_time_;

    // Initialize log file and ensure directory exists
    void initializeLog(const std::string& dir, const std::string& file_name)
    {
        file_utils::createDirectory(dir);
        txt_name_ = dir + "/" + file_name;

        if (!file_utils::fs::exists(txt_name_) || clean_)
        {
            file_utils::writeToFileByTrunc(txt_name_, "\t", "iteration_num", "start_time(s)",
                                           "end_time(s)", "elapsed_time(s)", "average_time(s)",
                                           "sum_time(s)", "total_runtime(s)");
            clean_ = false;
        }
    }

    // Update the log data and write to file
    bool updateAndWrite(int iteration, const std::string& start_time, const std::string& end_time,
                        bool write_file = true)
    {
        try
        {
            iteration_ = iteration;
            start_time_ = start_time;
            end_time_ = end_time;
            duration_ = std::stod(end_time_) - std::stod(start_time_);
            sum_time_ += duration_;
            avg_time_ = sum_time_ / iteration_;

            if (iteration_ == 1)
                first_start_time_ = start_time_;
            total_runtime_ = std::stod(end_time_) - std::stod(first_start_time_);

            if (write_file)
            {
                file_utils::writeToFileByAppend(txt_name_, "\t", iteration_, start_time_, end_time_,
                                                duration_, avg_time_, sum_time_, total_runtime_);
            }
            return true;
        } catch (const std::exception& e)
        {
            std::cerr << "Error in TimeLogData::updateAndWrite: " << e.what() << std::endl;
            return false;
        }
    }
};

class ExplorationDataLog
{
  public:
    using SharedPtr = std::shared_ptr<ExplorationDataLog>;
    using UniquePtr = std::unique_ptr<ExplorationDataLog>;

    ExplorationDataLog()
        : clean_(true), cumulative_time_(0.0), cumulative_distance_(0.0), known_space_volume_(0.0),
          x_(0.0), y_(0.0), z_(0.0), known_voxel_num_(0), txt_name_("")
    {
    }
    ~ExplorationDataLog() = default;

    bool clean_;
    double cumulative_time_, cumulative_distance_, known_space_volume_, x_, y_, z_;
    int known_voxel_num_;
    std::string txt_name_;

    // Initialize log file and ensure directory exists
    void initializeLog(const std::string& dir, const std::string& file_name)
    {
        file_utils::createDirectory(dir);
        txt_name_ = dir + "/" + file_name;

        if (!file_utils::fs::exists(txt_name_) || clean_)
        {
            file_utils::writeToFileByTrunc(
                txt_name_, "\t", "cumulative_time(s)", "cumulative_distance(m)", "known_voxel_num",
                "known_space_volume(m^3)", "odom_x(m)", "odom_y(m)", "odom_z(m)");
            clean_ = false;
        }
    }

    // Update the log data and write to file
    void updateAndWrite(double time, double distance, int voxel_num, double volume, double x,
                        double y, double z, bool write_file = true)
    {
        cumulative_time_ = time;
        cumulative_distance_ = distance;
        known_voxel_num_ = voxel_num;
        known_space_volume_ = volume;
        x_ = x;
        y_ = y;
        z_ = z;

        if (write_file)
        {
            file_utils::writeToFileByAppend(txt_name_, "\t", cumulative_time_, cumulative_distance_,
                                            known_voxel_num_, known_space_volume_, x_, y_, z_);
        }
    }
};
}; // namespace data_log_manager

#endif
