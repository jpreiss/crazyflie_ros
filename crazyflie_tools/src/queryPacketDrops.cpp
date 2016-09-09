#include <iostream>
#include <chrono>
#include <thread>

#include <boost/program_options.hpp>
#include <crazyflie_cpp/Crazyflie.h>

struct log1 {
  uint32_t d0;
  uint32_t d1;
  uint32_t d2;
  uint32_t d3;
  uint32_t d4;
  uint32_t d5;
} __attribute__((packed));

struct log2 {
  uint32_t d6;
  uint32_t d7;
  uint32_t d8;
  uint32_t d9p;
  uint32_t total;
} __attribute__((packed));

volatile bool g_done1 = false;
volatile bool g_done2 = false;


void onLogData1(uint32_t time_in_ms, struct log1* data)
{
  if (!g_done1) {
    std::cout << "0," << data->d0 << std::endl;
    std::cout << "1," << data->d1 << std::endl;
    std::cout << "2," << data->d2 << std::endl;
    std::cout << "3," << data->d3 << std::endl;
    std::cout << "4," << data->d4 << std::endl;
    std::cout << "5," << data->d5 << std::endl;

    g_done1 = true;
  }
}

void onLogData2(uint32_t time_in_ms, struct log2* data)
{
  if (g_done1) {
    std::cout << "6," << data->d6 << std::endl;
    std::cout << "7," << data->d7 << std::endl;
    std::cout << "8," << data->d8 << std::endl;
    std::cout << "9+," << data->d9p << std::endl;
    std::cout << "total," << data->total << std::endl;

    g_done2 = true;
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

    std::unique_ptr<LogBlock<struct log1> > logBlock1;
    std::function<void(uint32_t, struct log1*)> cb1 = std::bind(&onLogData1, std::placeholders::_1, std::placeholders::_2);

    logBlock1.reset(new LogBlock<struct log1>(
      &cf,{
        {"pacDrop", "d0"},
        {"pacDrop", "d1"},
        {"pacDrop", "d2"},
        {"pacDrop", "d3"},
        {"pacDrop", "d4"},
        {"pacDrop", "d5"},
      }, cb1));
    logBlock1->start(10); // 100ms

    std::unique_ptr<LogBlock<struct log2> > logBlock2;
    std::function<void(uint32_t, struct log2*)> cb2 = std::bind(&onLogData2, std::placeholders::_1, std::placeholders::_2);

    logBlock2.reset(new LogBlock<struct log2>(
      &cf,{
        {"pacDrop", "d6"},
        {"pacDrop", "d7"},
        {"pacDrop", "d8"},
        {"pacDrop", "d9p"},
        {"pacDrop", "total"},
      }, cb2));
    logBlock2->start(10); // 100ms

    while (!g_done1 || !g_done2) {
      cf.sendPing();
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    return 0;
  }
  catch(std::exception& e)
  {
    std::cerr << e.what() << std::endl;
    return 1;
  }
}
