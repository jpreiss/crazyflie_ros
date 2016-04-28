#include <ros/ros.h>
#include <tf/transform_listener.h>

#include <sensor_msgs/Joy.h>
#include <std_srvs/Empty.h>

namespace Xbox360Buttons {

    enum {
        Green  = 0,
        Red    = 1,
        Blue   = 2,
        Yellow = 3,
    };

}

class Manager
{
public:

    Manager()
        : m_subscribeJoy()
        , m_serviceEmergency()
        , m_serviceTakeoff()
        , m_serviceLand()
        , m_serviceStartTrajectory()
    {
        ros::NodeHandle nh;
        m_subscribeJoy = nh.subscribe("/joy", 1, &Manager::joyChanged, this);

        m_serviceEmergency = nh.serviceClient<std_srvs::Empty>("/emergency");
        m_serviceTakeoff = nh.serviceClient<std_srvs::Empty>("/takeoff");
        m_serviceLand = nh.serviceClient<std_srvs::Empty>("/land");
        m_serviceStartTrajectory = nh.serviceClient<std_srvs::Empty>("/start_trajectory");
    }

    ~Manager()
    {
    }

private:
    void joyChanged(
        const sensor_msgs::Joy::ConstPtr& msg)
    {
        static std::vector<int> lastButtonState(4);

        if (msg->buttons.size() >= 3
            && lastButtonState.size() >= 3)
        {
            if (msg->buttons[Xbox360Buttons::Red] == 1 && lastButtonState[Xbox360Buttons::Red] == 0) {
                emergency();
            }
            if (msg->buttons[Xbox360Buttons::Blue] == 1 && lastButtonState[Xbox360Buttons::Blue] == 0) {
                takeoff();
            }
            if (msg->buttons[Xbox360Buttons::Green] == 1 && lastButtonState[Xbox360Buttons::Green] == 0) {
                land();
            }
            if (msg->buttons[Xbox360Buttons::Yellow] == 1 && lastButtonState[Xbox360Buttons::Yellow] == 0) {
                startTrajectory();
            }
        }

        lastButtonState = msg->buttons;
    }

    void emergency()
    {
        std_srvs::Empty srv;
        m_serviceEmergency.call(srv);
    }

    void takeoff()
    {
        std_srvs::Empty srv;
        m_serviceTakeoff.call(srv);
    }

    void land()
    {
        std_srvs::Empty srv;
        m_serviceLand.call(srv);
    }

    void startTrajectory()
    {
        std_srvs::Empty srv;
        m_serviceStartTrajectory.call(srv);
    }

    void callEmptyService(const std::string&)
    {
        ros::NodeHandle n;
        ros::ServiceClient client = n.serviceClient<std_srvs::Empty>("add_two_ints");
        std_srvs::Empty srv;
        if (!client.call(srv)) {
            ROS_ERROR("Failed to call emergency service");
        }
    }

private:
    ros::Subscriber m_subscribeJoy;

    ros::ServiceClient m_serviceEmergency;
    ros::ServiceClient m_serviceTakeoff;
    ros::ServiceClient m_serviceLand;
    ros::ServiceClient m_serviceStartTrajectory;
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "manager");

  // Read parameters
  // ros::NodeHandle n("~");
  // std::string worldFrame;
  // n.param<std::string>("worldFrame", worldFrame, "/world");
  // std::string frame;
  // n.getParam("frame", frame);
  // double frequency;
  // n.param("frequency", frequency, 50.0);

  // Controller controller(worldFrame, frame, n);
  // controller.run(frequency);

  Manager manager;
  ros::spin();

  return 0;
}
