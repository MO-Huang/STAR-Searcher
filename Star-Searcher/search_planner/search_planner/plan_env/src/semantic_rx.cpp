#include <ros/ros.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <sensor_msgs/PointCloud2.h>

// 定义点云类型
typedef pcl::PointCloud<pcl::PointXYZL> PointCloudXYZL;

class PointCloudConverter {
private:
    ros::NodeHandle nh_;
    ros::Subscriber sub_;
    ros::Publisher pub_;  // 可选：发布转换后的点云

public:
    PointCloudConverter() {
        // 订阅 /velodyne_points 话题
        sub_ = nh_.subscribe<sensor_msgs::PointCloud2>(
            "/velodyne_points_semantic", 1, &PointCloudConverter::cloudCallback, this);

        // 可选：发布转换后的点云到新话题
        // pub_ = nh_.advertise<PointCloudXYZL>("converted_cloud", 1);

        ROS_INFO("Node initialized. Waiting for PointCloud2 messages...");
    }

    void cloudCallback(const sensor_msgs::PointCloud2ConstPtr& input_cloud) {
        // 步骤 1：尝试自动转换（如果字段匹配）
        PointCloudXYZL::Ptr pcl_cloud(new PointCloudXYZL);
        bool conversion_success = false;

        try {
            // pcl::fromROSMsg(*input_cloud, *pcl_cloud);
            convertPointCloud2ToPclXYZL(*input_cloud, *pcl_cloud);
            conversion_success = true;
        } catch (const std::exception& e) {
            ROS_WARN("Auto conversion failed: %s. Attempting manual conversion...", e.what());
        }

        // 步骤 2：如果自动转换失败，手动复制字段
        // if (!conversion_success) {
            // pcl_cloud->header = pcl_conversions::toPCL(input_cloud->header);
            // pcl_cloud->height = input_cloud->height;
            // pcl_cloud->width = input_cloud->width;
            // pcl_cloud->is_dense = input_cloud->is_dense;
            // pcl_cloud->resize(input_cloud->width * input_cloud->height);

            // // 使用迭代器访问字段（假设存在 x, y, z, label）
            // sensor_msgs::PointCloud2ConstIterator<float> iter_x(*input_cloud, "x");
            // sensor_msgs::PointCloud2ConstIterator<float> iter_y(*input_cloud, "y");
            // sensor_msgs::PointCloud2ConstIterator<float> iter_z(*input_cloud, "z");
            // sensor_msgs::PointCloud2ConstIterator<uint32_t> iter_label(*input_cloud, "label");

            // for (size_t i = 0; i < pcl_cloud->size(); ++i, ++iter_x, ++iter_y, ++iter_z, ++iter_label) {
            //     pcl_cloud->points[i].x = *iter_x;
            //     pcl_cloud->points[i].y = *iter_y;
            //     pcl_cloud->points[i].z = *iter_z;
            //     pcl_cloud->points[i].label = *iter_label;
            // }
        // }

        ROS_INFO_STREAM("Successfully converted cloud with " << pcl_cloud->size() << " points.");

        const bool print_all_points = false;  // 设置为true打印全部点
        const int max_print_points = 100;      // 最大打印点数

        ROS_INFO_STREAM("Cloud contents:");
        size_t printed_count = 0;


        for (const auto& point : *pcl_cloud) {
            // 打印前N个点
            // if (!print_all_points && printed_count >= max_print_points) break;

            if(point.label == 1)
            {
                // 使用ROS_INFO_STREAM保证线程安全输出
                ROS_INFO_STREAM(
                    "Point " << printed_count << ": "
                    << "(x=" << point.x
                    << ", y=" << point.y
                    << ", z=" << point.z
                    << ", label=" << point.label << ")"
                );
            }
            
            printed_count++;
        }

        // 可选：发布转换后的点云
        // pub_.publish(pcl_cloud);
    }

    void convertPointCloud2ToPclXYZL(const sensor_msgs::PointCloud2& msg, pcl::PointCloud<pcl::PointXYZL>& pcl_pc) {
        // 确保 msg 包含 x, y, z, label 字段
        if (msg.fields.size() < 7) {
            ROS_ERROR("PointCloud2 message does not contain enough fields.");
            return;
        }

        // 获取字段的偏移量
        int x_offset = msg.fields[0].offset;
        int y_offset = msg.fields[1].offset;
        int z_offset = msg.fields[2].offset;
        int label_offset = msg.fields[6].offset;

        // 点云大小
        pcl_pc.clear();
        pcl_pc.resize(msg.height * msg.width);

        // 遍历每个点
        for (size_t i = 0; i < msg.height * msg.width; ++i) {
            // 获取当前点的起始位置
            const uint8_t* data_ptr = &msg.data[i * msg.point_step];

            // 解析 x, y, z
            float x = *reinterpret_cast<const float*>(data_ptr + x_offset);
            float y = *reinterpret_cast<const float*>(data_ptr + y_offset);
            float z = *reinterpret_cast<const float*>(data_ptr + z_offset);

            // 解析 label
            uint8_t label = *reinterpret_cast<const uint8_t*>(data_ptr + label_offset);

            // 填充到 pcl::PointXYZL
            pcl_pc[i].x = x;
            pcl_pc[i].y = y;
            pcl_pc[i].z = z;
            pcl_pc[i].label = label;
        }
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "velodyne_converter");
    PointCloudConverter converter;
    ros::spin();
    return 0;
}