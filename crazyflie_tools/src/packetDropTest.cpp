#include <iostream>
#include <thread>

#include <boost/program_options.hpp>
#include <crazyflie_cpp/Crazyflie.h>

int main(int argc, char **argv)
{

  std::string uri;
  std::string defaultUri("radio://0/80/2M/E7E7E7E7E7");
  uint32_t numPackets = 1000;
  uint32_t delay = 10;

  namespace po = boost::program_options;

  po::options_description desc("Allowed options");
  desc.add_options()
    ("help", "produce help message")
    ("uri", po::value<std::string>(&uri)->default_value(defaultUri), "unique ressource identifier")
    ("numPackets", po::value<uint32_t>(&numPackets)->default_value(numPackets), "number of packets to send")
    ("delay", po::value<uint32_t>(&delay)->default_value(delay), "delay in ms between packets")
  ;

  try
  {
    po::variables_map vm;
    po::store(po::parse_command_line(argc, argv, desc), vm);
    po::notify(vm);

    if (vm.count("help")) {
      std::cout << desc << "\n";
      return 0;
    }
  }
  catch(po::error& e)
  {
    std::cerr << e.what() << std::endl << std::endl;
    std::cerr << desc << std::endl;
    return 1;
  }

  try
  {
    // broadcast packets
    CrazyflieBroadcaster cfb(uri);

    for (uint64_t seq = 1; seq <= numPackets; ++seq) {
      cfb.sendPacketDropTest(seq);
      // send at 100 Hz
      std::this_thread::sleep_for(std::chrono::milliseconds(delay));
    }

    return 0;
  }
  catch(std::exception& e)
  {
    std::cerr << e.what() << std::endl;
    return 1;
  }
}
