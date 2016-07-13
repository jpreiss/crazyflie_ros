#include <fstream>
#include <sstream>

#include <ros/ros.h>
#include <tf/transform_listener.h>

#include <sensor_msgs/Joy.h>
#include <std_srvs/Empty.h>

#include <crazyflie_driver/UploadTrajectory.h>

namespace Xbox360Buttons {

    enum {
        Green  = 0,
        Red    = 1,
        Blue   = 2,
        Yellow = 3,
        Back   = 6,
        Start  = 7,
        COUNT  = 8,
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
        static std::vector<int> lastButtonState(Xbox360Buttons::COUNT);

        if (msg->buttons.size() >= Xbox360Buttons::COUNT
            && lastButtonState.size() >= Xbox360Buttons::COUNT)
        {
            if (msg->buttons[Xbox360Buttons::Red] == 1 && lastButtonState[Xbox360Buttons::Red] == 0) {
                emergency();
            }
            if (msg->buttons[Xbox360Buttons::Start] == 1 && lastButtonState[Xbox360Buttons::Start] == 0) {
                takeoff();
            }
            if (msg->buttons[Xbox360Buttons::Back] == 1 && lastButtonState[Xbox360Buttons::Back] == 0) {
                land();
            }
            if (msg->buttons[Xbox360Buttons::Yellow] == 1 && lastButtonState[Xbox360Buttons::Yellow] == 0) {
                startTrajectory();
            }
            if (msg->buttons[Xbox360Buttons::Blue] == 1 && lastButtonState[Xbox360Buttons::Blue] == 0) {
                uploadTrajectory();
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

    void uploadTrajectory()
    {
        ros::NodeHandle n("~");
        int numCFs;
        n.getParam("num_cfs", numCFs);

        for (size_t i = 1; i <= numCFs; ++i)
        {
            std::stringstream sstr;
            sstr << "crazyflie" << i;

            std::string csvFile;
            double x_offset;
            double y_offset;
            double z_offset;
            double timescale; // 2 means it will take 2x longer
            n.getParam(sstr.str() + "/csv_file", csvFile);
            n.getParam(sstr.str() + "/x_offset", x_offset);
            n.getParam(sstr.str() + "/y_offset", y_offset);
            n.getParam(sstr.str() + "/z_offset", z_offset);
            n.getParam(sstr.str() + "/timescale", timescale);

            crazyflie_driver::UploadTrajectory srv;

            std::ifstream stream(csvFile);
            bool firstLine = true;
            for (std::string line; std::getline(stream, line); ) {
                if (!firstLine) {
                    std::stringstream sstr;
                    char dummy;
                    float duration, accX, accY, accZ;
                    crazyflie_driver::QuadcopterTrajectoryPoly poly;
                    sstr << line;
                    sstr >> duration >> dummy;
                    poly.poly_x.resize(8);
                    for (size_t i = 0; i < 8; ++i) {
                        sstr >> poly.poly_x[i] >> dummy;
                    }
                    poly.poly_y.resize(8);
                    for (size_t i = 0; i < 8; ++i) {
                        sstr >> poly.poly_y[i] >> dummy;
                    }
                    poly.poly_z.resize(8);
                    for (size_t i = 0; i < 8; ++i) {
                        sstr >> poly.poly_z[i] >> dummy;
                    }
                    poly.poly_yaw.resize(8);
                    for (size_t i = 0; i < 8; ++i) {
                        sstr >> poly.poly_yaw[i] >> dummy;
                    }
                    poly.poly_x[0] += x_offset;
                    poly.poly_y[0] += y_offset;
                    poly.poly_z[0] += z_offset;

                    // // e.g. if s==2 the new polynomial will be stretched to take 2x longer
                    float recip = 1.0f / timescale;
                    float scale = recip;
                    for (int i = 1; i < 8; ++i) {
                        poly.poly_x[i] *= scale;
                        poly.poly_y[i] *= scale;
                        poly.poly_z[i] *= scale;
                        poly.poly_yaw[i] *= scale;
                        scale *= recip;
                    }
                    poly.duration = ros::Duration(duration * timescale);

                  // point.position.x += x_offset;
                    // point.position.y += y_offset;
                    srv.request.polygons.push_back(poly);
                }
                firstLine = false;
            }

            // for (auto& poly : srv.request.polygons) {
            //     // point.time_from_start.toSec();
            //     ROS_INFO("%f,%f,%f,%f,%f,%f,%f,%f", poly.duration.toSec(), );
            // }

            ros::service::call(sstr.str() + "/upload_trajectory", srv);
        }

    }

    // void callEmptyService(const std::string&)
    // {
    //     ros::NodeHandle n;
    //     ros::ServiceClient client = n.serviceClient<std_srvs::Empty>("add_two_ints");
    //     std_srvs::Empty srv;
    //     if (!client.call(srv)) {
    //         ROS_ERROR("Failed to call emergency service");
    //     }
    // }

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

  Manager manager;
  ros::spin();

  return 0;
}
