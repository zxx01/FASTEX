/*
 * @Author: Xiaoxun Zhang
 * @Date: 2024-03-26 16:41:52
 * @LastEditTime: 2024-12-23 20:49:38
 * @Description: 
 */

#ifndef FILE_UTILS_H
#define FILE_UTILS_H

#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <iomanip>
#include <boost/filesystem.hpp>

namespace file_utils
{
  namespace fs = boost::filesystem;

  void createDirectory(const std::string &dir_path);

  std::string formatDouble(double value, int precision = 3);

  template <typename... Args>
  void writeToFileByTrunc(const std::string &file_name, const std::string &delimiter, Args &&...args)
  {
    std::ofstream fout(file_name, std::ios_base::out | std::ios_base::trunc);
    if (fout.is_open())
    {
      std::ostringstream oss;
      ((oss << std::forward<Args>(args) << delimiter), ...);
      std::string line = oss.str();
      line.pop_back();
      fout << line << std::endl;
      fout.close();
      std::cout << "File written successfully." << std::endl;
    }
    else
    {
      std::cerr << "Failed to open the file: " << file_name << std::endl;
    }
  }

  template <typename... Args>
  void writeToFileByAppend(const std::string &file_name, const std::string &delimiter, Args &&...args)
  {
    std::ofstream fout(file_name, std::ios_base::out | std::ios_base::app);
    if (fout.is_open())
    {
      std::ostringstream oss;
      ((oss << std::forward<Args>(args) << delimiter), ...);
      std::string line = oss.str();
      line.pop_back();
      fout << line << std::endl;
      fout.close();
      std::cout << "File written successfully." << std::endl;
    }
    else
    {
      std::cerr << "Failed to open the file: " << file_name << std::endl;
    }
  }
}

#endif