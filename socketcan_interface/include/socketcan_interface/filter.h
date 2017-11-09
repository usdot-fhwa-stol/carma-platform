#ifndef SOCKETCAN_INTERFACE_FILTER_H
#define SOCKETCAN_INTERFACE_FILTER_H

#include <vector>

#include "interface.h"

namespace can {

class FrameFilter {
public:
  typedef boost::shared_ptr<FrameFilter> Ptr;
  virtual bool pass(const can::Frame &frame) const = 0;
  virtual ~FrameFilter() {}
};

class FrameMaskFilter : public FrameFilter {
public:
  static const uint32_t MASK_ALL = 0xffffffff;
  static const uint32_t MASK_RELAXED = ~Frame::EXTENDED_MASK;
  FrameMaskFilter(uint32_t can_id, uint32_t mask = MASK_RELAXED, bool invert = false)
  : mask_(mask), masked_id_(can_id & mask), invert_(invert)
  {}
  virtual bool pass(const can::Frame &frame) const{
    return ((mask_ & frame) == masked_id_) != invert_;
  }
private:
  const uint32_t mask_;
  const uint32_t masked_id_;
  const bool invert_;
};

class FrameRangeFilter  : public FrameFilter {
public:
  FrameRangeFilter(uint32_t min_id, uint32_t max_id, bool invert = false)
  : min_id_(min_id), max_id_(max_id), invert_(invert)
  {}
  virtual bool pass(const can::Frame &frame) const{
    return (min_id_ <= frame && frame <= max_id_) != invert_;
  }
private:
  const uint32_t min_id_;
  const uint32_t max_id_;
  const bool invert_;
};

class FilteredFrameListener : public CommInterface::FrameListener {
public:
  typedef std::vector<FrameFilter::Ptr> FilterVector;
  FilteredFrameListener(boost::shared_ptr<CommInterface> comm, const Callable &callable, const FilterVector &filters)
  : CommInterface::FrameListener(callable),
    filters_(filters),
    listener_(comm->createMsgListener(Callable(this, &FilteredFrameListener::filter)))
  {}
private:
  void filter(const Frame &frame) {
    for(FilterVector::const_iterator it=filters_.begin(); it != filters_.end(); ++it) {
      if((*it)->pass(frame)){
        (*this)(frame);
        break;
      }
    }
  }
  const std::vector<FrameFilter::Ptr> filters_;
  CommInterface::FrameListener::Ptr listener_;
};

} // namespace can

#endif /*SOCKETCAN_INTERFACE_FILTER_H*/
