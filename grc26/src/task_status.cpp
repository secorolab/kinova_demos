#include "grc26/task_status.hpp"

void TaskStatus::update(const TaskStatusData& new_status)
{
    std::lock_guard<std::mutex> lock(mutex_);

    current_ = new_status;
    current_.sequence_number = sequence_counter_++;
    current_.timestamp = std::chrono::high_resolution_clock::now();
}

bool TaskStatus::getLatest(TaskStatusData& out) const
{
    std::lock_guard<std::mutex> lock(mutex_);
    out = current_;
    return true;
}