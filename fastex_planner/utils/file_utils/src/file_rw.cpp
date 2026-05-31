/*
 * @Author: Xiaoxun Zhang
 * @Date: 2024-03-27 09:23:47
 * @LastEditTime: 2024-04-14 15:45:27
 * @Description:
 */
#include "file_utils/file_rw.h"

namespace file_utils
{
  void createDirectory(const std::string &dir_path)
  {
    if (!fs::exists(dir_path))
    {
      if (!fs::create_directories(dir_path))
      {
        std::cerr << "create directory " << dir_path.c_str() << " fail!" << std::endl;
      }
    }
  }

  std::string formatDouble(double value, int precision)
  {
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(precision) << value;
    return oss.str();
  }
}