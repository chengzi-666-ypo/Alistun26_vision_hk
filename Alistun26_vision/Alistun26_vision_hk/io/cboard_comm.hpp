#ifndef IO__CBOARD_COMM_HPP
#define IO__CBOARD_COMM_HPP

#include "io/command.hpp"

#include <string>
#include <cstdint>

namespace io {

// A small helper for encoding/decoding the 8-byte CBoard CAN frame and
// optionally sending via SocketCAN.
class CBoardComm
{
public:
  CBoardComm() = default;
  CBoardComm(const std::string &ifname, uint32_t can_id);

  // configure network interface and can id
  void set_interface(const std::string &ifname);
  void set_can_id(uint32_t can_id);

  // encode an io::Command into 12-byte payload (Control + Shoot + Yaw(float) + Pitch(float) + Res(2))
  static void encode_command(const io::Command &cmd, uint8_t *out);

  // decode 12-byte payload into io::Command
  static io::Command decode_frame(const uint8_t *in);

  // send using SocketCAN (returns 0 on success)
  int send_socketcan(const io::Command &cmd) const;

private:
  std::string ifname_ = "can0";
  uint32_t can_id_ = 0xff;
};

}  // namespace io

#endif  // IO__CBOARD_COMM_HPP
