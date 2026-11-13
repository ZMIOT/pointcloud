#include "DebugController.h"
namespace fw {
void DebugController::reset(bool debug,quint64 run,quint64 budget) {
    std::lock_guard<std::mutex> lock(mutex_);
    debug_=debug; stop_=false; step_=true; run_=run; budget_=budget; waiting_=0; permit_=0;
}
bool DebugController::enabled() const { std::lock_guard<std::mutex> lock(mutex_); return debug_; }
bool DebugController::isCancelled() const { std::lock_guard<std::mutex> lock(mutex_); return stop_; }
DebugAction DebugController::checkpoint(const DebugFrame& source) {
    quint64 sequence,run,budget;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if(stop_) return DebugAction::Stop;
        if(!debug_) return DebugAction::Continue;
        sequence=++serial_; waiting_=sequence; run=run_; budget=budget_;
    }
    DebugFrame frame=source; frame.sequence=sequence; frame.runId=run;
    auto frozen=snapshot(frame,budget);
    if(onFrame) onFrame(frozen); else { stop(); return DebugAction::Stop; }
    std::unique_lock<std::mutex> lock(mutex_);
    // Render acknowledgement bounds the queue even during Continue mode.
    cv_.wait(lock,[&] { return stop_||acknowledged_>=sequence; });
    const bool pause=!stop_&&step_&&permit_!=sequence;
    lock.unlock(); if(pause&&onPause) onPause(true); lock.lock();
    cv_.wait(lock,[&] { return stop_||!step_||permit_==sequence; });
    waiting_=0; const auto result=stop_?DebugAction::Stop:DebugAction::Continue;
    lock.unlock(); if(onPause) onPause(false);
    return result;
}
void DebugController::next() { std::lock_guard<std::mutex> lock(mutex_); if(waiting_) permit_=waiting_; cv_.notify_all(); }
void DebugController::resume() { std::lock_guard<std::mutex> lock(mutex_); step_=false; cv_.notify_all(); }
void DebugController::singleStep() { std::lock_guard<std::mutex> lock(mutex_); step_=true; }
void DebugController::stop() { std::lock_guard<std::mutex> lock(mutex_); stop_=true; cv_.notify_all(); }
void DebugController::presented(quint64 seq) { std::lock_guard<std::mutex> lock(mutex_); acknowledged_=std::max(acknowledged_,seq); cv_.notify_all(); }
}
