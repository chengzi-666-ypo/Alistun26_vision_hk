// Simple referee listener: listens on SocketCAN interface for ID 0x700 and writes
// 'red' / 'blue' / 'both' into /tmp/robot_enemy_color

#include <cstring>
#include <fstream>
#include <iostream>
#include <string>

#include <sys/socket.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <unistd.h>

int main(int argc, char ** argv)
{
  if (argc < 2) {
    std::cerr << "Usage: referee_listener <can_iface>\n";
    return 2;
  }

  const char * ifname = argv[1];

  int s = socket(PF_CAN, SOCK_RAW, CAN_RAW);
  if (s < 0) {
    perror("socket");
    return 1;
  }

  struct ifreq ifr;
  std::strncpy(ifr.ifr_name, ifname, IFNAMSIZ - 1);
  if (ioctl(s, SIOCGIFINDEX, &ifr) < 0) {
    perror("SIOCGIFINDEX");
    close(s);
    return 1;
  }

  struct sockaddr_can addr;
  std::memset(&addr, 0, sizeof(addr));
  addr.can_family = AF_CAN;
  addr.can_ifindex = ifr.ifr_ifindex;

  if (bind(s, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
    perror("bind");
    close(s);
    return 1;
  }

  std::cout << "referee_listener: listening on " << ifname << " for CAN 0x700\n";

  const canid_t REF_ID = 0x700;
  struct can_frame frame;

  while (true) {
    int n = read(s, &frame, sizeof(frame));
    if (n < 0) {
      perror("read");
      break;
    }
    if ((frame.can_id & CAN_SFF_MASK) != REF_ID) continue;

    // Expect payload: first byte: 0=red,1=blue,2=both; optional ascii string
    std::string color = "both";
    if (frame.can_dlc >= 1) {
      uint8_t b = frame.data[0];
      if (b == 0) color = "red";
      else if (b == 1) color = "blue";
      else if (b == 2) color = "both";
      else {
        // fallback: try parse ascii
        std::string s((char *)frame.data, frame.can_dlc);
        if (s.find("red") != std::string::npos) color = "red";
        else if (s.find("blue") != std::string::npos) color = "blue";
        else color = "both";
      }
    }

    std::ofstream ofs("/tmp/robot_enemy_color");
    if (ofs) {
      ofs << color << std::endl;
      ofs.close();
      std::cout << "referee_listener: set color=" << color << "\n";
    } else {
      std::cerr << "referee_listener: failed to write /tmp/robot_enemy_color\n";
    }
  }

  close(s);
  return 0;
}
