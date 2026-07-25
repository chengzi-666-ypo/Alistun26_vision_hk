/*
 * tests/test_send_can.cpp
 * 简单的 SocketCAN 发送程序，用来发送符合 io::CBoard::send 的 8 字节帧
 * 用法：
 *   ./test_send_can [ifname] [can_id(hex)] [control] [shoot] [yaw_deg] [pitch_deg] [horizon_m]
 * 示例：
 *   ./test_send_can can0 0xff 1 0 10.5 -2.3 3.2
 * 表示 control=1, shoot=0, yaw=10.5deg, pitch=-2.3deg, horizon_distance=3.2m
 */

#include <arpa/inet.h>
#include <errno.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>

#include <cmath>
#include <cstring>
#include <iostream>
#include <sstream>

int main(int argc, char **argv)
{
  const char *ifname = (argc > 1) ? argv[1] : "can0";
  unsigned int can_id = 0xff;
  if (argc > 2) {
    std::stringstream ss(argv[2]);
    ss >> std::hex >> can_id;
  }

  uint8_t control = (argc > 3) ? static_cast<uint8_t>(std::atoi(argv[3])) : 0;
  uint8_t shoot = (argc > 4) ? static_cast<uint8_t>(std::atoi(argv[4])) : 0;
  double yaw_deg = (argc > 5) ? std::atof(argv[5]) : 0.0;
  double pitch_deg = (argc > 6) ? std::atof(argv[6]) : 0.0;
  double horizon_m = (argc > 7) ? std::atof(argv[7]) : 0.0;

  // scale as in CBoard::send: int16(value * 1e4)
  auto to_int16_scaled = [](double v) -> int16_t {
    double s = v * 1e4;
    if (s > 32767) s = 32767;
    if (s < -32768) s = -32768;
    return static_cast<int16_t>(std::lrint(s));
  };

  int16_t yaw_s = to_int16_scaled(yaw_deg);
  int16_t pitch_s = to_int16_scaled(pitch_deg);
  int16_t horizon_s = to_int16_scaled(horizon_m);

  // open socketcan
  int s = socket(PF_CAN, SOCK_RAW, CAN_RAW);
  if (s < 0) {
    std::perror("socket");
    return 1;
  }

  struct ifreq ifr;
  std::strncpy(ifr.ifr_name, ifname, IFNAMSIZ - 1);
  if (ioctl(s, SIOCGIFINDEX, &ifr) < 0) {
    std::perror("ioctl");
    close(s);
    return 1;
  }

  struct sockaddr_can addr;
  std::memset(&addr, 0, sizeof(addr));
  addr.can_family = AF_CAN;
  addr.can_ifindex = ifr.ifr_ifindex;

  if (bind(s, reinterpret_cast<struct sockaddr *>(&addr), sizeof(addr)) < 0) {
    std::perror("bind");
    close(s);
    return 1;
  }

  struct can_frame frame;
  std::memset(&frame, 0, sizeof(frame));
  frame.can_id = can_id;
  frame.can_dlc = 8;

  frame.data[0] = control;
  frame.data[1] = shoot;

  auto pack_int16_be = [&](int16_t v, int idx) {
    uint16_t uv = static_cast<uint16_t>(v);
    frame.data[idx] = static_cast<uint8_t>((uv >> 8) & 0xff);
    frame.data[idx + 1] = static_cast<uint8_t>(uv & 0xff);
  };

  pack_int16_be(yaw_s, 2);
  pack_int16_be(pitch_s, 4);
  pack_int16_be(horizon_s, 6);

  ssize_t nbytes = write(s, &frame, sizeof(frame));
  if (nbytes != sizeof(frame)) {
    std::cerr << "Write error: " << std::strerror(errno) << " (wrote " << nbytes << ")\n";
    close(s);
    return 1;
  }

  std::cout << "Sent CAN frame on " << ifname << " id=0x" << std::hex << can_id << std::dec << " dlc="
            << static_cast<int>(frame.can_dlc) << " data:";
  for (int i = 0; i < frame.can_dlc; ++i) std::printf(" %02x", frame.data[i]);
  std::cout << std::endl;

  close(s);
  return 0;
}
