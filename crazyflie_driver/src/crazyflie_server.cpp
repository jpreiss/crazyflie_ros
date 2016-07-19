#include "ros/ros.h"
#include <tf/transform_listener.h>

#include "crazyflie_driver/AddCrazyflie.h"
#include "crazyflie_driver/LogBlock.h"
#include "crazyflie_driver/GenericLogData.h"
#include "crazyflie_driver/UpdateParams.h"
#include "crazyflie_driver/UploadTrajectory.h"
#include "std_srvs/Empty.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/Temperature.h"
#include "sensor_msgs/MagneticField.h"
#include "std_msgs/Float32.h"

#include <sensor_msgs/Joy.h>

#include "vicon_ros/NamedPoseArray.h"

//#include <regex>
#include <thread>
#include <mutex>

#include <crazyflie_cpp/Crazyflie.h>

// debug test
#include <signal.h>
#include <csignal> // or C++ style alternative
//

constexpr double pi() { return std::atan(1)*4; }

double degToRad(double deg) {
    return deg / 180.0 * pi();
}

double radToDeg(double rad) {
    return rad * 180.0 / pi();
}

class CrazyflieROS
{
public:
  CrazyflieROS(
    const std::string& link_uri,
    const std::string& tf_prefix,
    const std::string& frame,
    const std::string& worldFrame,
    bool enable_parameters,
    int id,
    const std::vector<crazyflie_driver::LogBlock>& log_blocks,
    ros::CallbackQueue& queue)
    : m_cf(link_uri)
    , m_tf_prefix(tf_prefix)
    , m_frame(frame)
    , m_worldFrame(worldFrame)
    , m_enableParameters(enable_parameters)
    , m_id(id)
    , m_serviceUpdateParams()
    , m_serviceUploadTrajectory()
    , m_listener()
    , m_logBlocks(log_blocks)
  {
    ros::NodeHandle n;
    n.setCallbackQueue(&queue);
    m_serviceUploadTrajectory = n.advertiseService(tf_prefix + "/upload_trajectory", &CrazyflieROS::uploadTrajectory, this);

    for (auto& logBlock : m_logBlocks)
    {
      m_pubLogDataGeneric.push_back(n.advertise<crazyflie_driver::GenericLogData>(tf_prefix + "/" + logBlock.topic_name, 10));
    }

    m_subscribeJoy = n.subscribe("/joy", 1, &CrazyflieROS::joyChanged, this);
  }

  const std::string& frame() const {
    return m_frame;
  }

  const int id() const {
    return m_id;
  }

  void sendPing() {
    m_cf.sendPing();
  }

  void joyChanged(
        const sensor_msgs::Joy::ConstPtr& msg)
  {
    static float x = 0.0;
    static float y = 0.0;
    static float z = 1.0;
    static float yaw = 0;
    bool changed = false;

    float dx = msg->axes[4];
    if (fabs(dx) > 0.1) {
      x += dx * 0.01;
      changed = true;
    }
    float dy = msg->axes[3];
    if (fabs(dy) > 0.1) {
      y += dy * 0.01;
      changed = true;
    }
    float dz = msg->axes[1];
    if (fabs(dz) > 0.1) {
      z += dz * 0.01;
      changed = true;
    }
    float dyaw = msg->axes[0];
    if (fabs(dyaw) > 0.1) {
      yaw += dyaw * 1.0;
      changed = true;
    }

    if (changed) {
      ROS_INFO("[%f, %f, %f, %f]", x, y, z, yaw);
      m_cf.trajectoryHover(x, y, z, yaw);
    }
  }

public:

  template<class T, class U>
  void updateParam(uint8_t id, const std::string& ros_param) {
      U value;
      ros::param::get(ros_param, value);
      m_cf.setParam<T>(id, (T)value);
  }

  bool updateParams(
    crazyflie_driver::UpdateParams::Request& req,
    crazyflie_driver::UpdateParams::Response& res)
  {
    ROS_INFO("Update parameters");
    for (auto&& p : req.params) {
      std::string ros_param = "/" + m_tf_prefix + "/" + p;
      size_t pos = p.find("/");
      std::string group(p.begin(), p.begin() + pos);
      std::string name(p.begin() + pos + 1, p.end());

      auto entry = m_cf.getParamTocEntry(group, name);
      if (entry)
      {
        switch (entry->type) {
          case Crazyflie::ParamTypeUint8:
            updateParam<uint8_t, int>(entry->id, ros_param);
            break;
          case Crazyflie::ParamTypeInt8:
            updateParam<int8_t, int>(entry->id, ros_param);
            break;
          case Crazyflie::ParamTypeUint16:
            updateParam<uint16_t, int>(entry->id, ros_param);
            break;
          case Crazyflie::ParamTypeInt16:
            updateParam<int16_t, int>(entry->id, ros_param);
            break;
          case Crazyflie::ParamTypeUint32:
            updateParam<uint32_t, int>(entry->id, ros_param);
            break;
          case Crazyflie::ParamTypeInt32:
            updateParam<int32_t, int>(entry->id, ros_param);
            break;
          case Crazyflie::ParamTypeFloat:
            updateParam<float, float>(entry->id, ros_param);
            break;
        }
      }
      else {
        ROS_ERROR("Could not find param %s/%s", group.c_str(), name.c_str());
      }
    }
    return true;
  }


  bool uploadTrajectory(
    crazyflie_driver::UploadTrajectory::Request& req,
    crazyflie_driver::UploadTrajectory::Response& res)
  {
    ROS_INFO("[%s] Upload trajectory", m_frame.c_str());

    m_cf.trajectoryReset();

    for (auto& p : req.polygons) {
      m_cf.trajectoryAdd(
        p.duration.toSec(),
        p.poly_x,
        p.poly_y,
        p.poly_z,
        p.poly_yaw);
    }

    ROS_INFO("[%s] Uploaded trajectory", m_frame.c_str());


    return true;
  }


  // bool prepareTakeoff()
  // {
  //   ROS_INFO("Prepare Takeoff");

  //   tf::StampedTransform transform;
  //   m_listener.lookupTransform(m_worldFrame, m_frame, ros::Time(0), transform);

  //   m_cf.trajectoryReset();
  //   m_cf.trajectoryAdd(
  //     transform.getOrigin().x(),
  //     transform.getOrigin().y(),
  //     transform.getOrigin().z(),
  //     0, 0, 0, 0, 0);
  //   m_cf.trajectoryAdd(
  //     transform.getOrigin().x(),
  //     transform.getOrigin().y(),
  //     0.5, //transform.getOrigin().z + 0.5,
  //     0, 0, 0, 0, 2 * 1000);

  //   return true;
  // }

  // bool prepareLand()
  // {
  //   ROS_INFO("Prepare Land");

  //   tf::StampedTransform transform;
  //   m_listener.lookupTransform(m_worldFrame, m_frame, ros::Time(0), transform);

  //   m_cf.trajectoryReset();
  //   m_cf.trajectoryAdd(
  //     transform.getOrigin().x(),
  //     transform.getOrigin().y(),
  //     transform.getOrigin().z(),
  //     0, 0, 0, 0, 0);
  //   m_cf.trajectoryAdd(
  //     transform.getOrigin().x(),
  //     transform.getOrigin().y(),
  //     0,
  //     0, 0, 0, 0, 2 * 1000);

  //   return true;
  // }

  void run(
    ros::CallbackQueue& queue)
  {
    // m_cf.reboot();

    auto start = std::chrono::system_clock::now();

    std::function<void(float)> cb_lq = std::bind(&CrazyflieROS::onLinkQuality, this, std::placeholders::_1);
    m_cf.setLinkQualityCallback(cb_lq);



    m_cf.logReset();

    if (m_enableParameters)
    {
      ROS_INFO("Requesting parameters...");
      m_cf.requestParamToc();
      for (auto iter = m_cf.paramsBegin(); iter != m_cf.paramsEnd(); ++iter) {
        auto entry = *iter;
        std::string paramName = "/" + m_tf_prefix + "/" + entry.group + "/" + entry.name;
        switch (entry.type) {
          case Crazyflie::ParamTypeUint8:
            ros::param::set(paramName, m_cf.getParam<uint8_t>(entry.id));
            break;
          case Crazyflie::ParamTypeInt8:
            ros::param::set(paramName, m_cf.getParam<int8_t>(entry.id));
            break;
          case Crazyflie::ParamTypeUint16:
            ros::param::set(paramName, m_cf.getParam<uint16_t>(entry.id));
            break;
          case Crazyflie::ParamTypeInt16:
            ros::param::set(paramName, m_cf.getParam<int16_t>(entry.id));
            break;
          case Crazyflie::ParamTypeUint32:
            ros::param::set(paramName, (int)m_cf.getParam<uint32_t>(entry.id));
            break;
          case Crazyflie::ParamTypeInt32:
            ros::param::set(paramName, m_cf.getParam<int32_t>(entry.id));
            break;
          case Crazyflie::ParamTypeFloat:
            ros::param::set(paramName, m_cf.getParam<float>(entry.id));
            break;
        }
      }
      ros::NodeHandle n;
      n.setCallbackQueue(&queue);
      m_serviceUpdateParams = n.advertiseService(m_tf_prefix + "/update_params", &CrazyflieROS::updateParams, this);
    }

    // Logging
    ROS_INFO("Requesting Logging variables...");
    m_cf.requestLogToc();

    m_logBlocksGeneric.resize(m_logBlocks.size());
    // custom log blocks
    size_t i = 0;
    for (auto& logBlock : m_logBlocks)
    {
      std::function<void(uint32_t, std::vector<double>*, void* userData)> cb =
        std::bind(
          &CrazyflieROS::onLogCustom,
          this,
          std::placeholders::_1,
          std::placeholders::_2,
          std::placeholders::_3);

      m_logBlocksGeneric[i].reset(new LogBlockGeneric(
        &m_cf,
        logBlock.variables,
        (void*)&m_pubLogDataGeneric[i],
        cb));
      m_logBlocksGeneric[i]->start(logBlock.frequency / 10);
      ++i;
    }


    ROS_INFO("Ready...");
    auto end = std::chrono::system_clock::now();
    std::chrono::duration<double> elapsedSeconds = end-start;
    ROS_INFO("Elapsed: %f s", elapsedSeconds.count());

  }

  void sendPositionExternalBringup(
    const stateExternalBringup& data)
  {
    m_cf.sendPositionExternalBringup(data);
  }


  void onLinkQuality(float linkQuality) {
      if (linkQuality < 0.7) {
        ROS_WARN("Link Quality low (%f)", linkQuality);
      }
  }

  void onLogCustom(uint32_t time_in_ms, std::vector<double>* values, void* userData) {

    ros::Publisher* pub = reinterpret_cast<ros::Publisher*>(userData);

    crazyflie_driver::GenericLogData msg;
    msg.header.stamp = ros::Time(time_in_ms/1000.0);
    msg.values = *values;

    pub->publish(msg);
  }

private:
  Crazyflie m_cf;
  std::string m_tf_prefix;
  std::string m_frame;
  std::string m_worldFrame;
  bool m_enableParameters;
  int m_id;

  ros::ServiceServer m_serviceUpdateParams;
  ros::ServiceServer m_serviceUploadTrajectory;

  tf::TransformListener m_listener;

  std::vector<crazyflie_driver::LogBlock> m_logBlocks;
  std::vector<ros::Publisher> m_pubLogDataGeneric;
  std::vector<std::unique_ptr<LogBlockGeneric> > m_logBlocksGeneric;

  ros::Subscriber m_subscribeJoy;
};

class CrazyflieServer
{
public:
  CrazyflieServer(
    const std::string& link_uri,
    const std::string& worldFrame,
    const std::string& posesTopic)
    : m_worldFrame(worldFrame)
    , m_isEmergency(false)
    , m_cfbc(link_uri)
    , m_serviceEmergency()
    , m_serviceStartTrajectory()
    , m_serviceTakeoff()
    , m_serviceLand()
    , m_listener()
  {
    ros::NodeHandle nhFast;
    nhFast.setCallbackQueue(&m_fastQueue);

    m_subscribePoses = nhFast.subscribe(posesTopic, 1, &CrazyflieServer::posesChanged, this);
    m_serviceEmergency = nhFast.advertiseService("emergency", &CrazyflieServer::emergency, this);

    ros::NodeHandle nhSlow;
    nhSlow.setCallbackQueue(&m_slowQueue);

    m_serviceStartTrajectory = nhSlow.advertiseService("start_trajectory", &CrazyflieServer::startTrajectory, this);
    m_serviceTakeoff = nhSlow.advertiseService("takeoff", &CrazyflieServer::takeoff, this);
    m_serviceLand = nhSlow.advertiseService("land", &CrazyflieServer::land, this);
  }

  ~CrazyflieServer()
  {
    for(auto cf : m_cfs) {
      delete cf;
    }
  }

  void posesChanged(
    const vicon_ros::NamedPoseArray::ConstPtr& msg)
  {
    // use direct communication if we have only one CF
    // This allows us to stream back data
    // Otherwise, use broadcasts
    if (m_cfs.size() == 1) {
      if (msg->poses.size() > 0) {
        bool success = false;
        for (auto& pose: msg->poses) {
          if (pose.name == m_cfs[0]->frame()) {
            stateExternalBringup stateExternalBringup;
            stateExternalBringup.id = m_cfs[0]->id();
            stateExternalBringup.x = pose.pose.position.x;
            stateExternalBringup.y = pose.pose.position.y;
            stateExternalBringup.z = pose.pose.position.z;
            stateExternalBringup.q0 = pose.pose.orientation.x;
            stateExternalBringup.q1 = pose.pose.orientation.y;
            stateExternalBringup.q2 = pose.pose.orientation.z;
            stateExternalBringup.q3 = pose.pose.orientation.w;

            m_cfs[0]->sendPositionExternalBringup(
              stateExternalBringup);
            success = true;
            break;
          }
        }
        if (!success) {
          ROS_WARN("Could not find pose for CF %s", m_cfs[0]->frame().c_str());
        }
      } else {
        ROS_WARN("Not enough poses");
      }
    } else {
      std::vector<stateExternalBringup> states;
      size_t i = 0;
      for (auto cf : m_cfs) {
        bool success = false;
        for (auto& pose: msg->poses) {
          if (pose.name == cf->frame()) {
            states.resize(i + 1);
            states[i].id = cf->id();
            states[i].x = pose.pose.position.x;
            states[i].y = pose.pose.position.y;
            states[i].z = pose.pose.position.z;
            states[i].q0 = pose.pose.orientation.x;
            states[i].q1 = pose.pose.orientation.y;
            states[i].q2 = pose.pose.orientation.z;
            states[i].q3 = pose.pose.orientation.w;
            ++i;
            success = true;
            break;
          }
        }
        if (!success) {
          ROS_WARN("Could not find pose for CF %s", cf->frame().c_str());
        }
      }
      m_cfbc.sendPositionExternalBringup(states);
    }
  }

  // void start()
  // {
  //   std::thread t(&CrazyflieServer::run, this);
  //   t.detach();
  // }

  void run()
  {
//    std::thread tFast(&CrazyflieServer::runFast, this);
    std::thread tSlow(&CrazyflieServer::runSlow, this);

//    tFast.join();
    runFast();
    tSlow.join();
  }

  void runFast()
  {
    while(ros::ok() && !m_isEmergency) {
      m_fastQueue.callAvailable(ros::WallDuration(0.1));
    }
  }

  void runSlow()
  {
    while(ros::ok() && !m_isEmergency) {
      if (m_cfs.size() == 1) {
        m_cfs[0]->sendPing();
      }
      m_slowQueue.callAvailable(ros::WallDuration(0.01));
    }
  }

  // void run()
  // {
  //   ROS_INFO("Waiting for transforms...");

  //   for (auto cf : m_cfs) {
  //     m_listener.waitForTransform(m_worldFrame, cf->frame(), ros::Time(0), ros::Duration(1000.0) );
  //   }

  //   ROS_INFO("Found all transforms!");

  //   // m_numCFs = 25;
  //   // std::vector<CrazyflieBroadcaster::stateExternal> stateExternal(m_numCFs);
  //   stateExternalBringup stateExternalBringup;

  //   // for (size_t i = 0; i < m_numCFs; ++i) {
  //   //   stateExternal[i].x = i + 0.1;
  //   //   stateExternal[i].y = i + 0.2;
  //   //   stateExternal[i].z = i + 0.3;
  //   //   stateExternal[i].yaw = i + 0.4;
  //   // }

  //   while(ros::ok() && !m_isEmergency) {

  //     // for (size_t i = 0; i < m_numCFs; ++i) {
  //     //   stateExternal[i].x += 1.0;
  //     //   stateExternal[i].y += 1.0;
  //     //   stateExternal[i].z += 1.0;
  //     //   stateExternal[i].yaw += 1.0;
  //     // }

  //     for (size_t i = 0; i < m_numCFs; ++i) {
  //       tf::StampedTransform transform;
  //       m_listener.lookupTransform(m_worldFrame, m_cfs[i]->frame(), ros::Time(0), transform);

  //       // tfScalar current_euler_roll, current_euler_pitch, current_euler_yaw;
  //       // tf::Matrix3x3(transform.getRotation()).getRPY(
  //       //     current_euler_roll,
  //       //     current_euler_pitch,
  //       //     current_euler_yaw);

  //       stateExternalBringup.id = m_cfs[i]->id();
  //       // stateExternal[i].x = transform.getOrigin().x();
  //       // stateExternal[i].y = transform.getOrigin().y();
  //       // stateExternal[i].z = transform.getOrigin().z();
  //       // stateExternal[i].yaw = atan2(sin(current_euler_yaw), cos(current_euler_yaw));
  //       stateExternalBringup.x = transform.getOrigin().x();
  //       stateExternalBringup.y = transform.getOrigin().y();
  //       stateExternalBringup.z = transform.getOrigin().z();
  //       stateExternalBringup.q0 = transform.getRotation().x();
  //       stateExternalBringup.q1 = transform.getRotation().y();
  //       stateExternalBringup.q2 = transform.getRotation().z();
  //       stateExternalBringup.q3 = transform.getRotation().w();

  //       m_cfs[i]->sendPositionExternalBringup(
  //         stateExternalBringup);

  //     }

  //     // ros::Time t = ros::Time::now();
  //     // while (ros::Time::now() < t + ros::Duration(0.03)) {
  //     //   m_cfs[0]->sendPing();
  //     // }

  //     // m_cfbc.sendPositionExternal(
  //     //   stateExternal);

  //     // m_cfbc.sendPositionExternalBringup(
  //     //   stateExternalBringup);

  //     std::this_thread::sleep_for(std::chrono::milliseconds(1));

  //     // ros::spinOnce();
  //   }
  // }

  void addCrazyflie(
    const std::string& uri,
    const std::string& tf_prefix,
    const std::string& frame,
    int id,
    const std::vector<crazyflie_driver::LogBlock>& logBlocks)
  {
    ROS_INFO("Adding CF: %s (%s, %s)...", tf_prefix.c_str(), uri.c_str(), frame.c_str());
    CrazyflieROS* cf = new CrazyflieROS(
      uri,
      tf_prefix,
      frame,
      m_worldFrame,
      true,
      id,
      logBlocks,
      m_slowQueue
      );
    cf->run(m_slowQueue);
    m_cfs.push_back(cf);
  }


private:

  bool emergency(
    std_srvs::Empty::Request& req,
    std_srvs::Empty::Response& res)
  {
    ROS_FATAL("Emergency requested!");
    m_isEmergency = true;

    return true;
  }

  bool takeoff(
    std_srvs::Empty::Request& req,
    std_srvs::Empty::Response& res)
  {
    ROS_INFO("Takeoff!");

    for (size_t i = 0; i < 10; ++i) {
      m_cfbc.takeoff();
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }

    return true;
  }

  bool land(
    std_srvs::Empty::Request& req,
    std_srvs::Empty::Response& res)
  {
    ROS_INFO("Land!");

    for (size_t i = 0; i < 10; ++i) {
      m_cfbc.land();
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }

    return true;
  }

  bool startTrajectory(
    std_srvs::Empty::Request& req,
    std_srvs::Empty::Response& res)
  {
    ROS_INFO("Start trajectory");

    for (size_t i = 0; i < 10; ++i) {
      m_cfbc.trajectoryStart();
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }

    return true;
  }

private:
  std::string m_worldFrame;
  bool m_isEmergency;
  CrazyflieBroadcaster m_cfbc;
  ros::ServiceServer m_serviceEmergency;
  ros::ServiceServer m_serviceStartTrajectory;
  ros::ServiceServer m_serviceTakeoff;
  ros::ServiceServer m_serviceLand;
  ros::Subscriber m_subscribePoses;

public:
  // TODO: make me private again!
  std::vector<CrazyflieROS*> m_cfs;

private:
  tf::TransformListener m_listener;


  // We have two callback queues
  // 1. Fast queue handles pose and emergency callbacks. Those are high-priority and can be served quickly
  // 2. Slow queue handles all other requests.
  // Each queue is handled in its own thread. We don't want a thread per CF to make sure that the fast queue
  //  gets called frequently enough.

  ros::CallbackQueue m_fastQueue;
  ros::CallbackQueue m_slowQueue;
};

int main(int argc, char **argv)
{
  // raise(SIGSTOP);

  ros::init(argc, argv, "crazyflie_server");

  ros::NodeHandle n("~");
  std::string worldFrame;
  n.param<std::string>("world_frame", worldFrame, "/world");
  std::string broadcastUri;
  n.getParam("broadcast_uri", broadcastUri);
  std::string posesTopic;
  n.getParam("poses_topic", posesTopic);

  CrazyflieServer server(broadcastUri, worldFrame, posesTopic);

  // custom log blocks
  std::vector<std::string> genericLogTopics;
  n.param("genericLogTopics", genericLogTopics, std::vector<std::string>());
  std::vector<int> genericLogTopicFrequencies;
  n.param("genericLogTopicFrequencies", genericLogTopicFrequencies, std::vector<int>());

  std::vector<crazyflie_driver::LogBlock> logBlocks;

  if (genericLogTopics.size() == genericLogTopicFrequencies.size())
  {
    size_t i = 0;
    for (auto& topic : genericLogTopics)
    {
      crazyflie_driver::LogBlock logBlock;
      logBlock.topic_name = topic;
      logBlock.frequency = genericLogTopicFrequencies[i];
      n.getParam("genericLogTopic_" + topic + "_Variables", logBlock.variables);
      logBlocks.push_back(logBlock);
      ++i;
    }
  }
  else
  {
    ROS_ERROR("Cardinality of genericLogTopics and genericLogTopicFrequencies does not match!");
  }

  // read CF config
  ros::NodeHandle nGlobal;

  XmlRpc::XmlRpcValue crazyflies;
  nGlobal.getParam("crazyflies", crazyflies);
  ROS_ASSERT(crazyflies.getType() == XmlRpc::XmlRpcValue::TypeArray);

  for (int32_t i = 0; i < crazyflies.size(); ++i)
  {
    ROS_ASSERT(crazyflies[i].getType() == XmlRpc::XmlRpcValue::TypeStruct);
    XmlRpc::XmlRpcValue crazyflie = crazyflies[i];
    std::string id = crazyflie["id"];
    // XmlRpc::XmlRpcValue pos = crazyflie["initialPosition"];
    // ROS_ASSERT(pos.getType() == XmlRpc::XmlRpcValue::TypeArray);
    // for (int32_t j = 0; j < pos.size(); ++j)
    // {
    //   ROS_ASSERT(pos[j].getType() == XmlRpc::XmlRpcValue::TypeDouble);
    //   double f = static_cast<double>(pos[j]);
    //   std::cout << pos[j].getType() << std::endl;

    // }

    std::string uri = "radio://0/100/2M/E7E7E7E7" + id;
    std::string tf_prefix = "cf" + id;
    std::string frame = "cf" + id + "/cf" + id;
    int idNumber;
    std::sscanf(id.c_str(), "%x", &idNumber);
    server.addCrazyflie(uri, tf_prefix, frame, idNumber, logBlocks);

    // update parameters
    XmlRpc::XmlRpcValue firmwareParams;
    n.getParam("firmwareParams", firmwareParams);

    crazyflie_driver::UpdateParams::Request request;
    crazyflie_driver::UpdateParams::Response response;

    // ROS_ASSERT(firmwareParams.getType() == XmlRpc::XmlRpcValue::TypeArray);
    auto iter = firmwareParams.begin();
    for (; iter != firmwareParams.end(); ++iter) {
      std::string group = iter->first;
      XmlRpc::XmlRpcValue v = iter->second;
      auto iter2 = v.begin();
      for (; iter2 != v.end(); ++iter2) {
        std::string param = iter2->first;
        XmlRpc::XmlRpcValue value = iter2->second;
        if (value.getType() == XmlRpc::XmlRpcValue::TypeBoolean) {
          bool b = value;
          nGlobal.setParam(tf_prefix + "/" + group + "/" + param, b);
        } else if (value.getType() == XmlRpc::XmlRpcValue::TypeInt) {
          int b = value;
          nGlobal.setParam(tf_prefix + "/" + group + "/" + param, b);
        } else if (value.getType() == XmlRpc::XmlRpcValue::TypeDouble) {
          double b = value;
          nGlobal.setParam(tf_prefix + "/" + group + "/" + param, b);
        } else {
          ROS_WARN("No known type for %s.%s!", group.c_str(), param.c_str());
        }
        request.params.push_back(group + "/" + param);
      }
    }

    server.m_cfs.back()->updateParams(request, response);
  }

  ROS_INFO("All CFs are ready!");

  // server.start();

  server.run();

  // ros::spin();

  return 0;
}
