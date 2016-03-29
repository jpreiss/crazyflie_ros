#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>
#include <std_srvs/Empty.h>
#include <geometry_msgs/Twist.h>

#include <Eigen/Dense>

#include <crazyflie_controller/ExecuteTrajectoryAction.h>
#include <actionlib/server/simple_action_server.h>

#include <crazyflie_driver/GenericLogData.h>


#include "pid.hpp"

double get(
    const ros::NodeHandle& n,
    const std::string& name) {
    double value;
    n.getParam(name, value);
    return value;
}

class Controller
{
public:

    Controller(
        const std::string& worldFrame,
        const std::string& frame,
        const ros::NodeHandle& n)
        : m_worldFrame(worldFrame)
        , m_frame(frame)
        , m_pubNav()
        , m_listener()
        , m_pidYaw(
            get(n, "PIDs/Yaw/kp"),
            get(n, "PIDs/Yaw/kd"),
            get(n, "PIDs/Yaw/ki"),
            get(n, "PIDs/Yaw/minOutput"),
            get(n, "PIDs/Yaw/maxOutput"),
            get(n, "PIDs/Yaw/integratorMin"),
            get(n, "PIDs/Yaw/integratorMax"),
            "yaw",
            true)
        , m_state(Idle)
        , m_subscribeStabilizer()
        // , m_goal()
        // , m_subscribeGoal()
        , m_serviceTakeoff()
        , m_serviceLand()
        , m_actionServerExecuteTrajectory(nullptr)
        , m_thrust(0)
        , m_kp(get(n, "PIDs/Body/kp"))
        , m_kd(get(n, "PIDs/Body/kd"))
        , m_ki(get(n, "PIDs/Body/ki"))
        , m_oldPosition(0,0,0)
        , m_current_r_error_integration(0,0,0)
        , m_massThrust(get(n, "MassThrust"))
        , m_maxAngle(get(n, "MaxAngle"))
        , m_roll(0)
        , m_pitch(0)
        , m_yaw(0)
    {
        ros::NodeHandle nh;
        m_pubNav = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);
        // m_subscribeGoal = nh.subscribe("goal", 1, &Controller::goalChanged, this);
        // ToDo switch to actions!
        m_serviceTakeoff = nh.advertiseService("takeoff", &Controller::takeoff, this);
        m_serviceLand = nh.advertiseService("land", &Controller::land, this);
        // m_serviceGoTo = nh.advertiseService("go_to", &Controller::go_to, this);

        m_subscribeStabilizer = nh.subscribe("/crazyflie/stabilizer", 1, &Controller::stabilizerChanged, this);

        m_actionServerExecuteTrajectory = new actionlib::SimpleActionServer<crazyflie_controller::ExecuteTrajectoryAction>(
            nh, "execute_trajectory",
            std::bind(&Controller::executeTrajectory, this, std::placeholders::_1),
            true);
    }

    ~Controller()
    {
        delete m_actionServerExecuteTrajectory;
    }

    void run(double frequency)
    {
        ros::NodeHandle node;
        ros::Timer timer = node.createTimer(ros::Duration(1.0/frequency), &Controller::iteration, this);
        ros::spin();
    }

private:
    // void goalChanged(
    //     const geometry_msgs::PoseStamped::ConstPtr& msg)
    // {
    //     m_goal = *msg;
    // }

    void stabilizerChanged(
        const crazyflie_driver::GenericLogData::ConstPtr& msg)
    {
        m_roll = -msg->values[0] / 180 * M_PI;
        m_pitch = msg->values[1] / 180 * M_PI;
        m_yaw = msg->values[2] / 180 * M_PI;
    }

    bool takeoff(
        std_srvs::Empty::Request& req,
        std_srvs::Empty::Response& res)
    {
        ROS_INFO("Takeoff requested!");
        m_state = TakingOff;

        return true;
    }

    bool land(
        std_srvs::Empty::Request& req,
        std_srvs::Empty::Response& res)
    {
        ROS_INFO("Landing requested!");
        m_state = Landing;

        return true;
    }

    bool executeTrajectory(
        const crazyflie_controller::ExecuteTrajectoryGoalConstPtr& goal)
    {
        ROS_INFO("execute trajectory requested!");

        m_trajectory = goal->trajectory;
        // m_startTime = ros::Time::now();

        // m_actionServerExecuteTrajectory->setSucceeded();
    }

    void getCurrentTrajectoryPoint(
        crazyflie_controller::QuadcopterTrajectoryPoint& result)
    {
        // TODO: we could also linearely interpolate here!
        ros::Duration d = ros::Time::now() - m_trajectory.header.stamp;
        for (auto& pt : m_trajectory.points) {
            if (pt.time_from_start > d) {
                result = pt;
                return;
            }
        }
        result = m_trajectory.points.back();
    }

    void getTransform(
        const std::string& sourceFrame,
        const std::string& targetFrame,
        tf::StampedTransform& result)
    {
        m_listener.lookupTransform(sourceFrame, targetFrame, ros::Time(0), result);
    }

    void pidReset()
    {
        m_current_r_error_integration = Eigen::Vector3d(0,0,0);
        m_pidYaw.reset();
    }

    void iteration(const ros::TimerEvent& e)
    {
        float dt = e.current_real.toSec() - e.last_real.toSec();

        switch(m_state)
        {
        case TakingOff:
            {
                tf::StampedTransform transform;
                m_listener.lookupTransform(m_worldFrame, m_frame, ros::Time(0), transform);
                if (transform.getOrigin().z() > 0.05 || m_thrust > 50000)
                {
                    pidReset();
                    m_state = Automatic;
                    m_thrust = 0;
                    m_trajectory.header.stamp = ros::Time::now();
                    m_trajectory.points.clear();
                    crazyflie_controller::QuadcopterTrajectoryPoint pt;
                    pt.position.z = 0.5;
                    m_trajectory.points.push_back(pt);
                }
                else
                {
                    m_thrust += 10000 * dt;
                    geometry_msgs::Twist msg;
                    msg.linear.z = m_thrust;
                    m_pubNav.publish(msg);
                }

            }
            break;
        case Landing:
            {
                // m_goal.pose.position.z = 0.05;
                m_trajectory.header.stamp = ros::Time::now();
                m_trajectory.points.clear();
                crazyflie_controller::QuadcopterTrajectoryPoint pt;
                pt.position.z = 0.05;
                m_trajectory.points.push_back(pt);

                tf::StampedTransform transform;
                m_listener.lookupTransform(m_worldFrame, m_frame, ros::Time(0), transform);
                if (transform.getOrigin().z() <= 0.05) {
                    m_state = Idle;
                    geometry_msgs::Twist msg;
                    m_pubNav.publish(msg);
                }
            }
            // intentional fall-thru
        case Automatic:
            {
                tf::StampedTransform tf_transform;
                m_listener.lookupTransform(m_worldFrame, m_frame, ros::Time(0), tf_transform);

                // CURRENT STATES
                Eigen::Affine3d transform;
                tf::transformTFToEigen(tf_transform, transform);

                Eigen::Vector3d position = transform.translation();
                Eigen::Vector3d current_velocity = (position - m_oldPosition) / dt;
                m_oldPosition = position;

                // tf::Quaternion q;
                // //q.setEuler(-m_pitch, m_roll, m_yaw);//m_yaw, m_roll, m_pitch);
                // q.setEuler(m_yaw, m_roll, -m_pitch);
                // tf::Transform t(q);
                // tf::Vector3 current_z_axis = t(tf::Vector3(0, 0, 1));
                Eigen::Vector3d current_z_axis( -sin(m_pitch)*cos(m_roll),
                                            sin(m_roll),
                                            cos(m_pitch)*cos(m_roll));

                // q.setRPY(m_pitch, m_roll, m_yaw);

                // tf::Vector3 current_z_axis = q.getAxis().normalized();//transform.getRotation().getAxis();
                // // current_z_axis.normalize();
                // if (current_z_axis[2] < 0) {
                //     current_z_axis *= -1;
                // }

                //ROS_INFO("%f, %f, %f", current_z_axis[0], current_z_axis[1], current_z_axis[2]);
                //break;

                crazyflie_controller::QuadcopterTrajectoryPoint trajectoryPoint;
                getCurrentTrajectoryPoint(trajectoryPoint);


                // ROS_INFO("%f", position[2]);

                Eigen::Vector3d target_position(
                    trajectoryPoint.position.x,
                    trajectoryPoint.position.y,
                    trajectoryPoint.position.z);

                Eigen::Vector3d target_velocity(
                    trajectoryPoint.velocity.x,
                    trajectoryPoint.velocity.y,
                    trajectoryPoint.velocity.z);

                double target_euler_yaw = trajectoryPoint.yaw;

                // tf::Quaternion target_quaternion(
                //     m_goal.pose.orientation.x,
                //     m_goal.pose.orientation.y,
                //     m_goal.pose.orientation.z,
                //     m_goal.pose.orientation.w);

                // tfScalar target_euler_roll, target_euler_pitch, target_euler_yaw;
                // tf::Matrix3x3(target_quaternion).getRPY(
                //     target_euler_roll,
                //     target_euler_pitch,
                //     target_euler_yaw);

                tfScalar current_euler_roll, current_euler_pitch, current_euler_yaw;
                tf::Matrix3x3(tf_transform.getRotation()).getRPY(
                    current_euler_roll,
                    current_euler_pitch,
                    current_euler_yaw);

                Eigen::Vector3d current_r_error = target_position - position;
                Eigen::Vector3d current_r_error_unit = current_r_error.normalized();

                m_current_r_error_integration += current_r_error * dt;
                if (m_current_r_error_integration.norm() >= 6) {
                    m_current_r_error_integration = 6.0 * m_current_r_error_integration.normalized();
                }

                // double r_error_norm = current_r_error.length();
                // target_velocity = 1.3 * (r_error_norm/5.0)*current_r_error_unit;
                // if (r_error_norm >= 5.0) {
                //     target_velocity = 1.3 * current_r_error_unit; // The velocity in the target position direction is 1 m/s
                // }

                // compute z-axis-desired
                Eigen::Vector3d z_axis_desired = m_massThrust * Eigen::Vector3d(0,0,1) + m_kp*current_r_error + m_kd*(target_velocity - current_velocity) + m_ki*m_current_r_error_integration;
                double angle = acos(z_axis_desired.normalized().dot(Eigen::Vector3d(0,0,1)));
                double kp = m_kp;
                double kd = m_kd;
                double ki = m_ki;

                while (angle >= m_maxAngle) {
                    kp *= 0.9;
                    kd *= 0.9;
                    ki *= 0.9;
                    z_axis_desired = m_massThrust * Eigen::Vector3d(0,0,1) + kp*current_r_error + kd*(target_velocity - current_velocity) + ki*m_current_r_error_integration;
                    angle = acos(z_axis_desired.normalized().dot(Eigen::Vector3d(0,0,1)));
                }
                Eigen::Vector3d z_axis_desired_unit = z_axis_desired.normalized();

                // control
                double thrust = z_axis_desired.dot(current_z_axis);
                ROS_INFO("%f", thrust);
                if (thrust < 0) {
                    thrust = 0;
                }
                if (thrust > 65536) {
                    thrust = 65536;
                }

                Eigen::Vector3d x_axis_desired = z_axis_desired_unit.cross(Eigen::Vector3d(sin(target_euler_yaw), cos(target_euler_yaw), 0));
                x_axis_desired.normalize();
                Eigen::Vector3d y_axis_desired = z_axis_desired_unit.cross(x_axis_desired);

                double pitch_angle = asin(x_axis_desired[2]) * 180.0 / M_PI;
                double yaw_angle = target_euler_yaw;
                // double yaw_angle = atan2(x_axis_desired.getY(), x_axis_desired.getX());
                double roll_angle = atan2(y_axis_desired[2], z_axis_desired_unit[2]) * 180.0 / M_PI;

                // ROS_INFO("%f", pitch_angle);
                // break;

                geometry_msgs::Twist msg;
                msg.linear.x = pitch_angle;
                msg.linear.y = -roll_angle;
                msg.linear.z = thrust;
                msg.angular.z = m_pidYaw.update(current_euler_yaw, yaw_angle);
                m_pubNav.publish(msg);


            }
            break;
        case Idle:
            {
                geometry_msgs::Twist msg;
                m_pubNav.publish(msg);
            }
            break;
        }
    }

private:

    enum State
    {
        Idle = 0,
        Automatic = 1,
        TakingOff = 2,
        Landing = 3,
    };

private:
    std::string m_worldFrame;
    std::string m_frame;
    ros::Publisher m_pubNav;
    tf::TransformListener m_listener;
    PID m_pidYaw;
    State m_state;
    // geometry_msgs::PoseStamped m_goal;
    // ros::Subscriber m_subscribeGoal;
    ros::Subscriber m_subscribeStabilizer;
    ros::ServiceServer m_serviceTakeoff;
    ros::ServiceServer m_serviceLand;
    actionlib::SimpleActionServer<crazyflie_controller::ExecuteTrajectoryAction>* m_actionServerExecuteTrajectory;
    float m_thrust;

    double m_kp;
    double m_kd;
    double m_ki;
    Eigen::Vector3d m_oldPosition;
    Eigen::Vector3d m_current_r_error_integration;
    double m_massThrust;
    double m_maxAngle;

    crazyflie_controller::QuadcopterTrajectory m_trajectory;

    double m_roll;
    double m_pitch;
    double m_yaw;

    // ros::time m_startTime;
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "controller");

  // Read parameters
  ros::NodeHandle n("~");
  std::string worldFrame;
  n.param<std::string>("worldFrame", worldFrame, "/world");
  std::string frame;
  n.getParam("frame", frame);
  double frequency;
  n.param("frequency", frequency, 50.0);

  Controller controller(worldFrame, frame, n);
  controller.run(frequency);

  return 0;
}
