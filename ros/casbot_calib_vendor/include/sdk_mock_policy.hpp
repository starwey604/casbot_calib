#pragma once
#include <chrono>
namespace casbot::sdk {
// The host forwards five fields; device-local PD does not use host feedback.
// Only the synthetic protection/gravity hold actually depends on a plant pose.
inline bool needs_plant_pose(bool active, bool awaiting_first_command, bool internal_gravity, bool protection_gravity=true) {
  return ((!active || awaiting_first_command) && protection_gravity) || internal_gravity;
}
// Vendor feedback repeats the latest cached frame with a publication stamp.
// Publishing that cache never certifies its physical acquisition time/freshness.
inline bool publishes_cached_feedback(bool have_cache, bool feedback_enabled) {
  return have_cache && feedback_enabled;
}
// A late step must catch up immediately, not add a whole idle period after
// its computation. Once caught up, the ordinary periodic deadline resumes.
template<class TimePoint,class Duration>
TimePoint next_physics_deadline(TimePoint previous,Duration period,TimePoint now) {
  const auto planned=previous+std::chrono::duration_cast<typename TimePoint::duration>(period);
  return planned<now?now:planned;
}
}
