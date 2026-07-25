#include "io/cboard_comm.hpp"
#include <cmath>  // 包含 std::lrint 所需的头文件
#include <arpa/inet.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>

#include <cstring>
#include <iostream>

namespace io {

CBoardComm::CBoardComm(const std::string &ifname, uint32_t can_id) : ifname_(ifname), can_id_(can_id) {}

void CBoardComm::set_interface(const std::string &ifname) { ifname_ = ifname; }
void CBoardComm::set_can_id(uint32_t can_id) { can_id_ = can_id; }

void CBoardComm::encode_command(const io::Command &cmd, uint8_t *out)
{
  // New layout (12 bytes):
  // Byte 0: Control (1B)
  // Byte 1: Shoot (1B)
  // Byte 2-5: Yaw (4B float, radians)
  // Byte 6-9: Pitch (4B float, radians)
  // Byte 10-11: Reserved (2B)
  
  std::memset(out, 0, 12);
  out[0] = cmd.control ? 1 : 0;
  out[1] = cmd.shoot ? 1 : 0;

  // Send yaw and pitch directly in radians (no conversion)
  float yaw_rad = static_cast<float>(cmd.yaw);
  float pitch_rad = static_cast<float>(cmd.pitch);

  std::memcpy(out + 2, &yaw_rad, sizeof(float));
  std::memcpy(out + 6, &pitch_rad, sizeof(float));
  
  // Reserved bytes remain 0
  out[10] = 0;
  out[11] = 0;
}

io::Command CBoardComm::decode_frame(const uint8_t *in)
{
  io::Command cmd;
  // Byte 0: Control
  cmd.control = in[0] != 0;
  // Byte 1: Shoot
  cmd.shoot = in[1] != 0;

  float yaw_rad;
  float pitch_rad;

  std::memcpy(&yaw_rad, in + 2, sizeof(float));
  std::memcpy(&pitch_rad, in + 6, sizeof(float));

  // Data is already in radians, no conversion needed
  cmd.yaw = static_cast<double>(yaw_rad);
  cmd.pitch = static_cast<double>(pitch_rad);
  
  cmd.horizon_distance = 0; // Not used/available in new protocol

  return cmd;
}

int CBoardComm::send_socketcan(const io::Command &cmd) const
{
  /* CAN disabled by user request
  int s = socket(PF_CAN, SOCK_RAW, CAN_RAW);
  if (s < 0) {
    std::perror("socket");
    return -1;
  }

  struct ifreq ifr;
  std::memset(&ifr, 0, sizeof(ifr));
  std::strncpy(ifr.ifr_name, ifname_.c_str(), IFNAMSIZ - 1);
  if (ioctl(s, SIOCGIFINDEX, &ifr) < 0) {
    std::perror("ioctl");
    close(s);
    return -1;
  }

  struct sockaddr_can addr;
  std::memset(&addr, 0, sizeof(addr));
  addr.can_family = AF_CAN;
  addr.can_ifindex = ifr.ifr_ifindex;

  if (bind(s, reinterpret_cast<struct sockaddr *>(&addr), sizeof(addr)) < 0) {
    std::perror("bind");
    close(s);
    return -1;
  }

  struct can_frame frame;
  std::memset(&frame, 0, sizeof(frame));
  frame.can_id = can_id_;
  frame.can_dlc = 8;

  // encode_command(cmd, frame.data); 
  // DANGER: encode_command now writes 11 bytes, but frame.data is 8 bytes.
  // Since user disabled CAN and protocol is now larger than CAN frame, we skip encoding full payload.
  // We can just zero it out or send partial if needed, but for now we safeguard against overflow.
  std::memset(frame.data, 0, 8);
  
  // To avoid confusion, we might want to log or just send empty frame if this is ever called.
  // Ideally, this method should be updated to use CAN FD or removed if CAN is deprecated.

  ssize_t n = write(s, &frame, sizeof(frame));
  if (n != sizeof(frame)) {
    std::cerr << "send write error: " << std::strerror(errno) << "\n";
    close(s);
    return -1;
  }

  close(s);
  */
  return 0;
}

}  // namespace io
