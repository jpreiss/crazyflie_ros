#include <iostream>
#include <chrono>
#include <thread>

#include <boost/program_options.hpp>
#include <crazyflie_cpp/Crazyradio.h>

int main(int argc, char **argv)
{

  std::string addressStr;
  std::string defaultAddressStr("0xE7E7E7E7E7");

  namespace po = boost::program_options;

  po::options_description desc("Allowed options");
  desc.add_options()
    ("help", "produce help message")
    ("address", po::value<std::string>(&addressStr)->default_value(defaultAddressStr), "device address")
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
    uint64_t address;
    std::stringstream sstr;
    sstr << std::hex << addressStr;
    sstr >> address;

    Crazyradio radio(0);
    radio.setAddress(address);
    radio.setDatarate(Crazyradio::Datarate_2MPS);
    radio.setChannel(100);
    radio.setAckEnable(false);
    // radio.setArc(0);

    uint8_t test[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
    for (size_t i = 0; i < 255; ++i) {
      test[0] = i;
      radio.sendPacketNoAck(test, sizeof(test));
      // We need to wait 500us to make sure that the receiver is ready for the next packet
      std::this_thread::sleep_for(std::chrono::microseconds(500));
    }

    // radio.setAckEnable(true);
    // radio.sendPacketNoAck(test, sizeof(test));

    return 0;
  }
  catch(std::exception& e)
  {
    std::cerr << e.what() << std::endl;
    return 1;
  }
}
