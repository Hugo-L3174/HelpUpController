#pragma once
#include <memory>
#include <type_traits>

/**
 * Helper to create a task with two additional boolean arguments
 * This is used to call constructors that do not support these extra arguments
 */
template<typename T, bool showTarget = false, bool showPose = false, typename... Args>
inline auto make_task_optional_gui(Args &&... args)
{
  if constexpr(std::is_constructible_v<T, Args..., bool, bool>)
  {
    return std::make_unique<T>(std::forward<Args>(args)..., showTarget, showPose);
  }
  else
  {
    return std::make_unique<T>(std::forward<Args>(args)...);
  }
}
