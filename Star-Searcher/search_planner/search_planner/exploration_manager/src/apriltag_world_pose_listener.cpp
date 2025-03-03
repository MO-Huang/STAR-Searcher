#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <map>
#include <string>

// 存储每个 AprilTag 在 world 坐标系下的位姿
std::map<int, geometry_msgs::TransformStamped> tag_poses;

int main(int argc, char** argv) {
    ros::init(argc, argv, "apriltag_world_pose_listener");
    ros::NodeHandle nh;

    // 初始化 TF2 监听器和缓存
    tf2_ros::Buffer tf_buffer;
    tf2_ros::TransformListener tf_listener(tf_buffer);

    // 定义要监听的 AprilTag 名称范围（apriltag_0 到 apriltag_7）
    const int num_tags = 8;
    ros::Rate rate(10.0);  // 更新频率：10Hz

    while (nh.ok()) {
        for (int tag_id = 0; tag_id < num_tags; ++tag_id) {
            std::string tag_frame = "apriltag_" + std::to_string(tag_id);
            std::string target_frame = "world";

            try {
                // 获取从 world 到 tag 的变换（时间戳为最新可用）
                geometry_msgs::TransformStamped transform = 
                    tf_buffer.lookupTransform(
                        target_frame, 
                        tag_frame,
                        ros::Time(0)  // 获取最新可用的变换
                    );

                // 更新位姿数据
                tag_poses[tag_id] = transform;

                // 输出位姿信息（调试用）
                // ROS_INFO_STREAM(
                //     "Tag " << tag_id << " in world frame:\n"
                //     << "  Position: [" 
                //     << transform.transform.translation.x << ", "
                //     << transform.transform.translation.y << ", "
                //     << transform.transform.translation.z << "]\n"
                //     << "  Orientation: ["
                //     << transform.transform.rotation.x << ", "
                //     << transform.transform.rotation.y << ", "
                //     << transform.transform.rotation.z << ", "
                //     << transform.transform.rotation.w << "]"
                // );
            } catch (tf2::TransformException &ex) {
                // 处理异常（例如标签未检测到）
                ROS_WARN("Failed to get transform for %s: %s", 
                    tag_frame.c_str(), ex.what());
                continue;
            }
        }
        // 统一打印当前检测到的所有标签位姿
        ROS_INFO("===== Current AprilTag Poses in World Frame =====");
        for (const auto& pair : tag_poses) {
            int tag_id = pair.first;
            const geometry_msgs::TransformStamped& transform = pair.second;
            ROS_INFO_STREAM(
                "[Tag " << tag_id << "]\n"
                << "  Position:    [" 
                << transform.transform.translation.x << ", "
                << transform.transform.translation.y << ", "
                << transform.transform.translation.z << "]\n"
                << "  Orientation: ["
                << transform.transform.rotation.x << ", "
                << transform.transform.rotation.y << ", "
                << transform.transform.rotation.z << ", "
                << transform.transform.rotation.w << "]"
            );
        }
        ROS_INFO("================================================\n");

        rate.sleep();
    }
    return 0;
}