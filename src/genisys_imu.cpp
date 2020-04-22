#include <iostream>
#include <sstream>
#include <iomanip>
#include <numeric>

#include "genisys_imu.h"
#include "serial/serial.h"
#include "ros/ros.h"
#include "sensor_msgs/Imu.h"
#include "tf/tf.h"
#include "tf/transform_broadcaster.h"

// using namespace std;

genisys_imu imu;

// IMU imu_data;
sensor_msgs::Imu imu_msg;
ros::Publisher imu_pub;
geometry_msgs::TransformStamped imu_tf;
// void publishImuTimer(const ros::TimerEvent &event);

int main(int argc, char **argv)
{

    std::cout << "Hello World" << std::endl;
    ros::init(argc, argv, "pitbull_imu");
    ros::NodeHandle nh;

    imu_pub = nh.advertise<sensor_msgs::Imu>("imu", 10);
    // ros::Timer publishImu = nh.createTimer(ros::Duration(0.05), publishOdomTimer);
    if (imu.begin("/dev/ttyIMU", 500000))
    {
        imu.update();
        // tf::Vector3 a3x1;
        // tf::Vector3 g3x1 = tf::Vector3(0, 0, -9.83453);
        // double vx = 0.001;
        // double vy = 0;
        // double vz = 0;
        // double px = 0;
        // double py = 0;
        // double pz = 0;
        ros::Time last_time;
        // imu.pose.x = 0.0;
        // imu.pose.y = 0.0;
        // imu.pose.z = 0.0;
        // std::cout << std::fixed << std::setw(10) << std::setprecision(5) << std::right;
        ros::Duration dt;
        while (ros::ok())
        {
            imu.update();
            static tf::TransformBroadcaster tf_broadcaster;
            ros::Time current_time = ros::Time::now();
            // dt = (current_time - last_time);
            // last_time = current_time;
            // tf::Quaternion q(imu.data.orientation.x, imu.data.orientation.y, imu.data.orientation.z, imu.data.orientation.w);
            // tf::Matrix3x3 m(q);
            // a3x1 = tf::Vector3(imu.data.linear_acceleration.x, imu.data.linear_acceleration.y, imu.data.linear_acceleration.z);

            // double ax = m.tdotx(a3x1);
            // double ay = m.tdoty(a3x1);
            // double az = m.tdotz(a3x1) + m.tdotz(g3x1); // - 9.83453;

            // imu.twist.x += imu.twist.x + ax * dt.toSec();
            // imu.twist.y += imu.twist.y + ay * dt.toSec();
            // imu.twist.z += imu.twist.z + az * dt.toSec();
            // imu.twist.x += std::isnan(ax) ? 0 : ax * dt.toSec();
            // imu.twist.x = 1.0;
            // imu.twist.y += std::isnan(ay) ? 0 : ay * dt.toSec();
            // imu.twist.z += std::isnan(az) ? 0 : az * dt.toSec();

            // imu.pose.x = imu.pose.x + (1.0 * dt.toSec());
            // px += double(vx * dt.toSec());
            // imu.pose.y += imu.twist.y * dt.toSec();
            // imu.pose.z += imu.twist.z * dt.toSec();

            // std::cout << col_cyan << data.orientation.x << " " << data.orientation.y << " " << data.orientation.z << " " << data.orientation.w << " ";
            // std::cout << col_green << imu.data.linear_acceleration.x << " " << imu.data.linear_acceleration.y << " " << imu.data.linear_acceleration.z << " ";
            // std::cout << col_red << data.angular_velocity.x << " " << data.angular_velocity.y << " " << data.angular_velocity.z << " ";
            // std::cout << col_yellow << imu.twist.x << " " << imu.twist.y << " " << imu.twist.z << " ";
            // std::cout << col_yellow << imu.pose.x << " " << imu.pose.y << " " << imu.pose.z << " ";
            // std::cout << col_yellow << px << " " << py << " " << pz << " ";
            // std::cout << col_reset << std::endl;

            imu_msg.header.stamp = current_time;
            imu_msg.header.frame_id = "imu_link";
            imu_msg.orientation.x = imu.data.orientation.x;
            imu_msg.orientation.y = imu.data.orientation.y;
            imu_msg.orientation.z = imu.data.orientation.z;
            imu_msg.orientation.w = imu.data.orientation.w;
            imu_msg.angular_velocity.x = imu.data.angular_velocity.x;
            imu_msg.angular_velocity.y = imu.data.angular_velocity.y;
            imu_msg.angular_velocity.z = imu.data.angular_velocity.z;
            imu_msg.linear_acceleration.x = imu.data.linear_acceleration.x;
            imu_msg.linear_acceleration.y = imu.data.linear_acceleration.y;
            imu_msg.linear_acceleration.z = imu.data.linear_acceleration.z;
            for (size_t i = 0; i < 9; i++)
            {
                imu_msg.orientation_covariance[i] = imu.data.orientation_covariance[i];
                imu_msg.angular_velocity_covariance[i] = imu.data.angular_velocity_covariance[i];
                imu_msg.linear_acceleration_covariance[i] = imu.data.linear_acceleration_covariance[i];
            }
            imu_pub.publish(imu_msg);
            // imu_tf.header.stamp = current_time;
            // imu_tf.header.frame_id = "base_link";
            // imu_tf.child_frame_id = "imu_link";
            // imu_tf.transform.translation.x = od;
            // odom_tf.transform.translation.y = odom.pose.pose.position.y;
            // imu_tf.transform.translation.z = 0.5;
            // imu_tf.transform.rotation = imu_msg.orientation;

            // tf_broadcaster.sendTransform(imu_tf);
            ros::spinOnce();
        }
    }

    return 0;
}
