#include <iostream>
#include <chrono>
#include <thread>

#include <boost/program_options.hpp>
#include <crazyflie_cpp/Crazyflie.h>

struct log {
  float accx;
  float accy;
  float accz;
  float gyrox;
  float gyroy;
  float gyroz;
} __attribute__((packed));

volatile bool g_done = false;

void onLogData(uint32_t time_in_ms, struct log* data)
{
  static int count = 0;

  std::cout << time_in_ms << ","
            << data->accx << "," << data->accy << "," << data->accz << ","
            << data->gyrox << "," << data->gyroy << "," << data->gyroz << std::endl;

  ++count;
  // std::cout << count << std::endl;
  if (count > 1000) {
    g_done = true;
  }
}

int main(int argc, char **argv)
{

  std::string uri;
  std::string defaultUri("radio://0/80/2M/E7E7E7E7E7");

  namespace po = boost::program_options;

  po::options_description desc("Allowed options");
  desc.add_options()
    ("help", "produce help message")
    ("uri", po::value<std::string>(&uri)->default_value(defaultUri), "unique ressource identifier")
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
    Crazyflie cf(uri);
    cf.logReset();
    cf.requestLogToc();

    std::unique_ptr<LogBlock<struct log> > logBlock;
    std::function<void(uint32_t, struct log*)> cb = std::bind(&onLogData, std::placeholders::_1, std::placeholders::_2);

    logBlock.reset(new LogBlock<struct log>(
      &cf,{
        {"acc", "x"},
        {"acc", "y"},
        {"acc", "z"},
        {"gyro", "x"},
        {"gyro", "y"},
        {"gyro", "z"}
      }, cb));

    std::cout << "t,accx,accy,accz,gyrox,gyroy,gyroz" << std::endl;

    logBlock->start(1); // 10ms

    while (!g_done) {
      cf.sendPing();
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }

    return 0;
  }
  catch(std::exception& e)
  {
    std::cerr << e.what() << std::endl;
    return 1;
  }
}
