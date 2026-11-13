#pragma once

#include "SeamTypes.h"

#include <deque>
#include <vector>

namespace weld {

class SeamHistory {
public:
    explicit SeamHistory(std::size_t capacity = 5);

    void setCapacity(std::size_t capacity);
    void add(const CloudT::ConstPtr& seam, Axis axis);
    void clear();

    bool empty() const { return passes_.empty(); }
    std::size_t size() const { return passes_.size(); }
    const std::deque<CloudT::Ptr>& passes() const { return passes_; }

    HistoryPrediction predict(float station,
                              Axis axis,
                              float maxExtrapolation,
                              int inwardAxis = -1,
                              int inwardDirection = 0) const;

private:
    static bool interpolate(const CloudT& seam,
                            float station,
                            Axis axis,
                            Eigen::Vector3f* point);

    std::size_t capacity_;
    std::deque<CloudT::Ptr> passes_;
};

} // namespace weld
