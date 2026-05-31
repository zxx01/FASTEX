/*
 * @Author: Xiaoxun Zhang
 * @Date: 2024-03-03 20:17:24
 * @LastEditTime: 2024-12-25 12:09:40
 * @Description:
 */

#include <ctime>   // std::time_t, std::tm, std::localtime
#include <iomanip> // std::put_time
#include <sstream>
#include <stdexcept>
#include "time_utils/time_utils.h"

namespace time_utils
{
  Timer::Timer()
  {
    cnt_ = 0u;
    name_ = "Timer";
    is_started_ = false;
    timer_start_ = std::chrono::time_point<std::chrono::high_resolution_clock>();
    timer_stop_ = std::chrono::time_point<std::chrono::high_resolution_clock>();
    duration_ = std::chrono::duration<double>::zero();
    duration_total_ = std::chrono::duration<double>::zero();
  };

  Timer::Timer(std::string name)
  {
    cnt_ = 0u;
    name_ = name;
    is_started_ = false;
    timer_start_ = std::chrono::time_point<std::chrono::high_resolution_clock>();
    timer_stop_ = std::chrono::time_point<std::chrono::high_resolution_clock>();
    duration_ = std::chrono::duration<double>::zero();
    duration_total_ = std::chrono::duration<double>::zero();
  }

  /**
   * @brief Start the timer
   *
   */
  void Timer::start(bool show_log)
  {
    if (show_log)
      std::cerr << name_ << " starts!" << std::endl;

    is_started_ = true;
    timer_start_ = std::chrono::high_resolution_clock::now();
  }

  /**
   * @brief Stop the timer, and show the duration if show is true
   *
   * @param show  whether to show the duration
   * @param unit  the unit of the duration, can be "ms" (milliseconds), "us" (microseconds), "ns" (nanoseconds), or "s" (seconds)
   */
  void Timer::stop(bool show_log, std::string unit)
  {
    is_started_ = false;
    timer_stop_ = std::chrono::high_resolution_clock::now();

    ++cnt_;
    duration_ = timer_stop_ - timer_start_;
    duration_total_ += duration_;

    if (show_log)
    {
      if (unit == "ms")
      {
        std::cerr << name_ << " takes " << getDurationThisRound("ms") << " ms" << std::endl;
      }
      else if (unit == "us")
      {
        std::cerr << name_ << " takes " << getDurationThisRound("us") << " us" << std::endl;
      }
      else if (unit == "ns")
      {
        std::cerr << name_ << " takes " << getDurationThisRound("ns") << " ns" << std::endl;
      }
      else if (unit == "s")
      {
        std::cerr << name_ << " takes " << getDurationThisRound("s") << " s" << std::endl;
      }
      else
      {
        throw std::invalid_argument("timer unit error!");
      }
    }
  }

  /**
   * @brief Reset the timer
   *
   */
  void Timer::reset()
  {
    cnt_ = 0;
    is_started_ = false;
    timer_start_ = std::chrono::time_point<std::chrono::high_resolution_clock>();
    timer_stop_ = std::chrono::time_point<std::chrono::high_resolution_clock>();
    duration_ = std::chrono::duration<double>::zero();
    duration_total_ = std::chrono::duration<double>::zero();
  }

  /**
   * @brief Reset the timer
   *
   * @param name the name of the timer
   */
  void Timer::reset(std::string name)
  {
    cnt_ = 0;
    name_ = name;
    is_started_ = false;
    timer_start_ = std::chrono::time_point<std::chrono::high_resolution_clock>();
    timer_stop_ = std::chrono::time_point<std::chrono::high_resolution_clock>();
    duration_ = std::chrono::duration<double>::zero();
    duration_total_ = std::chrono::duration<double>::zero();
  }

  /**
   * @brief Check if the timer is started
   *
   * @return true
   * @return false
   */
  bool Timer::isStart()
  {
    return is_started_;
  }

  /**
   * @brief Get the start time
   *
   * @param unit the unit of the time, can be "ms" (milliseconds), "us" (microseconds), "ns" (nanoseconds), or "s" (seconds)
   * @return int64_t
   */
  int64_t Timer::getStartTime(std::string unit)
  {
    if (unit == "s")
    {
      return std::chrono::duration_cast<std::chrono::seconds>(timer_start_.time_since_epoch()).count();
    }
    else if (unit == "ms")
    {
      return std::chrono::duration_cast<std::chrono::milliseconds>(timer_start_.time_since_epoch()).count();
    }
    else if (unit == "us")
    {
      return std::chrono::duration_cast<std::chrono::microseconds>(timer_start_.time_since_epoch()).count();
    }
    else if (unit == "ns")
    {
      return std::chrono::duration_cast<std::chrono::nanoseconds>(timer_start_.time_since_epoch()).count();
    }
    else
    {
      throw std::invalid_argument("timer unit error!");
    }
  }

  /**
   * @brief Get the stop time
   *
   * @param unit  the unit of the time, can be "ms" (milliseconds), "us" (microseconds), "ns" (nanoseconds), or "s" (seconds)
   * @return int64_t
   */
  int64_t Timer::getStopTime(std::string unit)
  {
    if (unit == "s")
    {
      return std::chrono::duration_cast<std::chrono::seconds>(timer_stop_.time_since_epoch()).count();
    }
    else if (unit == "ms")
    {
      return std::chrono::duration_cast<std::chrono::milliseconds>(timer_stop_.time_since_epoch()).count();
    }
    else if (unit == "us")
    {
      return std::chrono::duration_cast<std::chrono::microseconds>(timer_stop_.time_since_epoch()).count();
    }
    else if (unit == "ns")
    {
      return std::chrono::duration_cast<std::chrono::nanoseconds>(timer_stop_.time_since_epoch()).count();
    }
    else
    {
      throw std::invalid_argument("timer unit error!");
    }
  }

  /**
   * @brief Get the duration
   *
   * @tparam U
   * @return int64_t
   */
  template <class U>
  int64_t Timer::getDurationCountThisRound()
  {
    return std::chrono::duration_cast<U>(duration_).count();
  }

  /**
   * @brief Get the duration
   *
   * @param unit the unit of the duration, can be "ms" (milliseconds), "us" (microseconds), "ns" (nanoseconds), or "s" (seconds)
   * @return double the duration
   */
  double Timer::getDurationThisRound(std::string unit)
  {
    if (unit == "ms")
    {
      return std::chrono::duration<double, std::milli>(duration_).count();
    }
    else if (unit == "us")
    {
      return std::chrono::duration<double, std::micro>(duration_).count();
    }
    else if (unit == "ns")
    {
      return std::chrono::duration<double, std::nano>(duration_).count();
    }
    else if (unit == "s")
    {
      return std::chrono::duration<double, std::ratio<1>>(duration_).count();
    }
    else
    {
      throw std::invalid_argument("timer unit error!");
    }
  }

  /**
   * @brief Get the duration
   *
   * @param unit the unit of the duration, can be "ms" (milliseconds), "us" (microseconds), "ns" (nanoseconds), or "s" (seconds)
   * @return double the duration
   */
  double Timer::getDurationThisRoundUntilNow(std::string unit)
  {
    if (!is_started_)
      throw std::logic_error("timer is not started!");

    auto time_now = std::chrono::high_resolution_clock::now();

    if (unit == "ms")
    {
      return std::chrono::duration<double, std::milli>(time_now - timer_start_).count();
    }
    else if (unit == "us")
    {
      return std::chrono::duration<double, std::micro>(time_now - timer_start_).count();
    }
    else if (unit == "ns")
    {
      return std::chrono::duration<double, std::nano>(time_now - timer_start_).count();
    }
    else if (unit == "s")
    {
      return std::chrono::duration<double, std::ratio<1>>(time_now - timer_start_).count();
    }
    else
    {
      throw std::invalid_argument("timer unit error!");
    }
  }

  /**
   * @brief Get the current time as an integer value
   *
   * @param unit The time unit, can be "ms" (milliseconds), "us" (microseconds), "ns" (nanoseconds), or "s" (seconds)
   * @return int64_t The current time as an integer value, in the specified unit
   * @throws std::invalid_argument If the unit is not a valid time unit
   */
  int64_t Timer::getTimeNow(std::string unit)
  {
    if (unit == "ms")
    {
      return std::chrono::duration_cast<std::chrono::milliseconds>(
                 std::chrono::high_resolution_clock::now().time_since_epoch())
          .count();
    }
    else if (unit == "us")
    {
      return std::chrono::duration_cast<std::chrono::microseconds>(
                 std::chrono::high_resolution_clock::now().time_since_epoch())
          .count();
    }
    else if (unit == "ns")
    {
      return std::chrono::duration_cast<std::chrono::nanoseconds>(
                 std::chrono::high_resolution_clock::now().time_since_epoch())
          .count();
    }
    else if (unit == "s")
    {
      return std::chrono::duration_cast<std::chrono::seconds>(
                 std::chrono::high_resolution_clock::now().time_since_epoch())
          .count();
    }
    else
    {
      throw std::invalid_argument("timer unit error!");
    }
  }

  /**
   * @brief Get the current time as a formatted string
   *
   * This function retrieves the current system time, converts it to a
   * local time representation, and formats it as a string in the
   * "YYYY-MM-DD HH:MM:SS" format.
   *
   * @return std::string The current time as a formatted string
   */
  std::string Timer::getCurrentTimeString()
  {
    // get the current time point
    auto now = std::chrono::system_clock::now();

    // convert to time_t
    std::time_t now_c = std::chrono::system_clock::to_time_t(now);

    // convert to local time (tm struct)
    std::tm localTime = *std::localtime(&now_c);

    // format output using stringstream
    std::ostringstream oss;
    oss << std::put_time(&localTime, "%Y-%m-%d_%H-%M-%S");

    return oss.str();
  }

  /**
   * @brief Get the number of times the timer has been run
   *
   * @return uint32_t The number of times the timer has been run
   */
  uint32_t Timer::getTimerRunCount()
  {
    return cnt_;
  }

  /**
   * @brief Get the total duration
   *
   * @param unit the unit of the duration, can be "ms" (milliseconds), "us" (microseconds), "ns" (nanoseconds), or "s" (seconds)
   * @return double the total duration
   */
  double Timer::getTotalDuration(std::string unit)
  {
    if (cnt_ == 0u)
      throw std::logic_error(name_ + ": timer is not started!");

    if (unit == "ms")
    {
      return std::chrono::duration<double, std::milli>(duration_total_).count();
    }
    else if (unit == "us")
    {
      return std::chrono::duration<double, std::micro>(duration_total_).count();
    }
    else if (unit == "ns")
    {
      return std::chrono::duration<double, std::nano>(duration_total_).count();
    }
    else if (unit == "s")
    {
      return std::chrono::duration<double, std::ratio<1>>(duration_total_).count();
    }
    else
    {
      throw std::invalid_argument("timer unit error!");
    }
  }

  /**
   * @brief Get the average duration
   *
   * @param unit the unit of the duration, can be "ms" (milliseconds), "us" (microseconds), "ns" (nanoseconds), or "s" (seconds)
   * @return double the average duration
   */
  double Timer::getAverageDuration(std::string unit)
  {
    if (cnt_ == 0u)
      throw std::logic_error(name_ + ": timer is not started!");

    if (unit == "ms")
    {
      return std::chrono::duration<double, std::milli>(duration_total_ / static_cast<double>(cnt_)).count();
    }
    else if (unit == "us")
    {
      return std::chrono::duration<double, std::micro>(duration_total_ / static_cast<double>(cnt_)).count();
    }
    else if (unit == "ns")
    {
      return std::chrono::duration<double, std::nano>(duration_total_ / static_cast<double>(cnt_)).count();
    }
    else if (unit == "s")
    {
      return std::chrono::duration<double, std::ratio<1>>(duration_total_ / static_cast<double>(cnt_)).count();
    }
    else
    {
      throw std::invalid_argument("timer unit error!");
    }
  }
}
