#ifndef _UTILS_H
#define _UTILS_H

#include <unordered_map>
#include <Eigen/Eigen>

#include <asuka/modules/freedom/common_types.h>

namespace freedom{
// 计时器
class Timer{
public:
    struct TimerData
    {
        std::string name;
        std::chrono::high_resolution_clock::time_point start_time;
        double total_time = 0;
        int count = 0;
        bool is_running = false;

        TimerData(const std::string& timer_name = "") : name(timer_name) {}

        void start()
        {
            start_time = std::chrono::high_resolution_clock::now();
            is_running = true;
        }

        void stop() {
            if (!is_running) {
                std::cout << "Timer is not running!" << std::endl;
                return;
            }
            auto end_time = std::chrono::high_resolution_clock::now();
            double elapsed = std::chrono::duration<double, std::milli>(end_time - start_time).count();
            total_time += elapsed;
            ++ count;
            is_running = false;

            std::cout << "[" << name << "] time: " << elapsed << " ms" << std::endl;
        }

        void reset()
        {
            total_time = 0.0;
            count = 0;
            is_running = false;
        }
    };

    TimerData& operator[](const std::string& timer_name)
    {
        for (auto& timer : timers_) {
            if (timer.name == timer_name) {
                return timer;
            }
        }
        timers_.emplace_back(timer_name);
        return timers_.back();
    }

    Timer(){}
    ~Timer()
    {
        if(timers_.size() > 0)
        {
            std::cout << "\n\033[1;33m================= Timer Summary =================\033[0m\n";
            double total_average_time = 0.0;
            for (const auto& timer : timers_) {
                if (timer.count > 0)
                {
                    std::cout << "  [" << timer.name << "] Average: " << (timer.total_time / timer.count) << " ms" << std::endl;
                    total_average_time += timer.total_time / timer.count;
                }
                else
                    std::cout << "  [" << timer.name << "] No recorded intervals" << std::endl;
            }
            std::cout << "  [total] Average: " << total_average_time << " ms" << std::endl;
            std::cout << "\033[1;33m=================================================\033[0m\n";
        }
    }

private:
    std::vector<TimerData> timers_;
};

// 线程安全的vector遍历分配
// 需要保证get_ptr的指针使用周期内vector不发生改变
template <typename T>
class VectorElementGetter {
public:
    explicit VectorElementGetter(std::vector<T,Eigen::aligned_allocator<T>>& elements): elements_(elements), current_index_(0){}

    bool get_ptr(T*& element) {
        std::lock_guard<std::mutex> lock(mutex_);
        if (current_index_ >= elements_.size())
            return false;

        element = &elements_[current_index_];
        ++ current_index_;
        return true;
    }

    void reset()
    {
        std::lock_guard<std::mutex> lock(mutex_);
        current_index_ = 0;
    }

private:
    std::mutex mutex_;
    std::vector<T,Eigen::aligned_allocator<T>>& elements_;
    size_t current_index_;
};

template <typename T>
class constVectorElementGetter {
public:
    explicit constVectorElementGetter(const std::vector<T,Eigen::aligned_allocator<T>>& elements): elements_(elements), current_index_(0){}

    bool get_ptr(const T*& element) {
        std::lock_guard<std::mutex> lock(mutex_);
        if (current_index_ >= elements_.size())
            return false;

        element = &elements_[current_index_];
        ++ current_index_;
        return true;
    }

    void reset()
    {
        std::lock_guard<std::mutex> lock(mutex_);
        current_index_ = 0;
    }

private:
    std::mutex mutex_;
    const std::vector<T,Eigen::aligned_allocator<T>>& elements_;
    size_t current_index_;
};

// 默认最小idx为0,0,0且最大值相同
inline void incrementIdx(Index& idx, const unsigned int& max_idx)
{
    idx.z()++;
    if (idx.z() >= max_idx)
    {
        idx.z() = 0;
        idx.y()++;

        if (idx.y() >= max_idx)
        {
            idx.y() = 0;
            idx.x()++;
        }
    }
}

// 默认最小idx为0,0,0
inline void incrementIdx(Index& idx, const Index& max_idx)
{
    idx.z()++;
    if (idx.z() >= max_idx.z())
    {
        idx.z() = 0;
        idx.y()++;

        if (idx.y() >= max_idx.y())
        {
            idx.y() = 0;
            idx.x()++;
        }
    }
}

inline void incrementIdx(Index& idx, const Index& min_idx, const Index& max_idx)
{
    idx.z()++;
    if (idx.z() >= max_idx.z())
    {
        idx.z() = min_idx.z();
        idx.y()++;

        if (idx.y() >= max_idx.y())
        {
            idx.y() = min_idx.y();
            idx.x()++;
        }
    }
}

class Neighbours{
public:
    Neighbours(){}
    inline void set_params(unsigned int connectivity);
    std::vector<Index> offsets;
};

inline void Neighbours::set_params(unsigned int connectivity)
{
    offsets.reserve(26);
    offsets.emplace_back(Index(-1,  0,  0));
    offsets.emplace_back(Index( 1,  0,  0));
    offsets.emplace_back(Index( 0, -1,  0));
    offsets.emplace_back(Index( 0,  1,  0));
    offsets.emplace_back(Index( 0,  0, -1));
    offsets.emplace_back(Index( 0,  0,  1));
    offsets.emplace_back(Index(-1, -1,  0));
    offsets.emplace_back(Index(-1,  1,  0));
    offsets.emplace_back(Index( 1, -1,  0));
    offsets.emplace_back(Index( 1,  1,  0));
    offsets.emplace_back(Index( 0, -1, -1));
    offsets.emplace_back(Index( 0, -1,  1));
    offsets.emplace_back(Index( 0,  1, -1));
    offsets.emplace_back(Index( 0,  1,  1));
    offsets.emplace_back(Index(-1,  0, -1));
    offsets.emplace_back(Index( 1,  0, -1));
    offsets.emplace_back(Index(-1,  0,  1));
    offsets.emplace_back(Index( 1,  0,  1));
    offsets.emplace_back(Index(-1, -1, -1));
    offsets.emplace_back(Index(-1, -1,  1));
    offsets.emplace_back(Index(-1,  1, -1));
    offsets.emplace_back(Index(-1,  1,  1));
    offsets.emplace_back(Index( 1, -1, -1));
    offsets.emplace_back(Index( 1, -1,  1));
    offsets.emplace_back(Index( 1,  1, -1));
    offsets.emplace_back(Index( 1,  1,  1));

    if(connectivity == 6 || connectivity == 18 || connectivity == 26)
        offsets.resize(connectivity);
    else
        std::cout << "Connectivity not supproted" << std::endl;
}

}
#endif