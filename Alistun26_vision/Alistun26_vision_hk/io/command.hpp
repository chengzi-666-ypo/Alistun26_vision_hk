#ifndef IO__COMMAND_HPP
#define IO__COMMAND_HPP

namespace io
{
struct Command
{
  bool control;
  bool shoot;
  double yaw;
  double pitch;
  double horizon_distance = 0;  //无人机专有
  
  // MPC fields (for buff mode)
  double yaw_vel = 0;
  double yaw_acc = 0;
  double pitch_vel = 0;
  double pitch_acc = 0;
};

}  // namespace io

#endif  // IO__COMMAND_HPP
