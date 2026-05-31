/*
 * @Author: Xiaoxun Zhang
 * @Date: 2024-03-03 20:17:24
 * @LastEditTime: 2024-12-25 11:57:46
 * @Description:
 */
#ifndef _TIME_UTILS_H_
#define _TIME_UTILS_H_

#include <iostream>
#include <chrono>
#include <string>
#include <memory>

namespace time_utils
{
  class Timer
  {
  public:
    typedef std::shared_ptr<Timer> Ptr;

    Timer();

    Timer(std::string name);

    ~Timer() = default;

    void start(bool show_log = false);
    void stop(bool show_log = false, std::string unit = "ms");
    void reset();
    void reset(std::string name);

    bool isStart();

    int64_t getStartTime(std::string unit = "ms");
    int64_t getStopTime(std::string unit = "ms");
    static int64_t getTimeNow(std::string unit = "ms");
    static std::string getCurrentTimeString();
    uint32_t getTimerRunCount();

    template <class U>
    int64_t getDurationCountThisRound();
    double getDurationThisRound(std::string unit = "ms");
    double getDurationThisRoundUntilNow(std::string unit = "ms");

    double getTotalDuration(std::string unit = "ms");
    double getAverageDuration(std::string unit = "ms");

  private:
    uint32_t cnt_;
    bool is_started_;
    std::string name_;
    std::chrono::time_point<std::chrono::high_resolution_clock> timer_start_;
    std::chrono::time_point<std::chrono::high_resolution_clock> timer_stop_;
    std::chrono::duration<double> duration_;
    std::chrono::duration<double> duration_total_;
  };
}

#endif