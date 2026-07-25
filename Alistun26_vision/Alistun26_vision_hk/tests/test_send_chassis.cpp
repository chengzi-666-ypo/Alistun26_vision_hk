/*
 * tests/test_send_chassis.cpp
 * 发送 4 个 int16 到固件约定的 CAN ID（默认 0x300），用于控制 3508 车体电机（主机 -> C 板）。
 * 用法：
 *   ./test_send_chassis can0 300 m1 m2 m3 m4
 * 示例：
 *   ./test_send_chassis can0 300 1000 1000 1000 1000
 */

#include <arpa/inet.h>
#include <errno.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>

#include <cstring>
#include <iostream>
#include <sstream>

int main(int argc, char **argv)
{
  const char *ifname = (argc > 1) ? argv[1] : "can0";
  unsigned int can_id = 0x300;
  if (argc > 2) {
    std::stringstream ss(argv[2]);
    ss >> std::hex >> can_id;
  }

  int16_t m1 = (argc > 3) ? static_cast<int16_t>(std::atoi(argv[3])) : 0;
  int16_t m2 = (argc > 4) ? static_cast<int16_t>(std::atoi(argv[4])) : 0;
  int16_t m3 = (argc > 5) ? static_cast<int16_t>(std::atoi(argv[5])) : 0;
  int16_t m4 = (argc > 6) ? static_cast<int16_t>(std::atoi(argv[6])) : 0;

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

  auto pack_be = [&](int16_t v, int idx) {
    uint16_t uv = static_cast<uint16_t>(v);
    frame.data[idx] = static_cast<uint8_t>((uv >> 8) & 0xff);
    frame.data[idx + 1] = static_cast<uint8_t>(uv & 0xff);
  };

  pack_be(m1, 0);
  pack_be(m2, 2);
  pack_be(m3, 4);
  pack_be(m4, 6);

  ssize_t nbytes = write(s, &frame, sizeof(frame));
  if (nbytes != sizeof(frame)) {
    std::cerr << "Write error: " << std::strerror(errno) << " (wrote " << nbytes << ")\n";
    close(s);
    return 1;
  }

  std::cout << "Sent chassis control CAN frame on " << ifname << " id=0x" << std::hex << can_id
            << std::dec << " dlc=" << static_cast<int>(frame.can_dlc) << " data:";
  for (int i = 0; i < frame.can_dlc; ++i) std::printf(" %02x", frame.data[i]);
  std::cout << std::endl;

  close(s);
  return 0;
}
