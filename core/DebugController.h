#pragma once
#include <fw/Algorithm.h>
#include <condition_variable>
#include <mutex>
#include <functional>

namespace fw {
class DebugController final : public IExecutionContext, public IDebugContext {
public:
    void reset(bool debug, quint64 run, quint64 budget);
    bool enabled() const override;
    bool isCancelled() const override;
    IDebugContext& debug() override { return *this; }
    DebugAction checkpoint(const DebugFrame&) override;
    void log(const QString& s) override { if(onLog) onLog(s); }
    void progress(int p) override { if(onProgress) onProgress(p); }
    void next();
    void resume();
    void singleStep();
    void stop();
    void presented(quint64);
    std::function<void(FramePtr)> onFrame;
    std::function<void(bool)> onPause;
    std::function<void(const QString&)> onLog;
    std::function<void(int)> onProgress;
private:
    mutable std::mutex mutex_;
    std::condition_variable cv_;
    bool debug_=false, stop_=false, step_=true;
    quint64 run_=0, serial_=0, waiting_=0, acknowledged_=0, permit_=0, budget_=0;
};
}
