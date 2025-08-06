#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <iostream>
#include <chrono>
#include <ostream>

namespace gazebo
{
  class WallAndDoorPlugin : public ModelPlugin
  {
    public:
      WallAndDoorPlugin() : ModelPlugin()
      {
        // 构造函数
      }

      virtual ~WallAndDoorPlugin()
      {
        // 析构函数
      }

      // 加载插件
      void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
      {
        // 存储模型指针
        this->model = _model;

        // 获取关节指针
        this->joint = this->model->GetJoint("Door_JOINT");
        if (!this->joint)
        {
          gzerr << "关节 Door_JOINT 未找到！" << std::endl;
          return;
        }

        // 设置关节位置控制器
        this->pid = common::PID(100, 0.01, 1.0);
        this->model->GetJointController()->SetPositionPID(this->joint->GetScopedName(), this->pid);

        // 连接到更新事件
        this->updateConnection = event::Events::ConnectWorldUpdateBegin(
            std::bind(&WallAndDoorPlugin::OnUpdate, this));
        
        // 获取是否切换的标志和切换周期
        this->lastSwitchTime = std::chrono::steady_clock::now();
        switchFlag = true;
        if (_sdf->HasElement("switch_flag")) {
          switchFlag = _sdf->Get<bool>("switch_flag");
          std::cout << "\033[32minitial_position set to " << switchFlag <<".\033[0m" << std::endl;
        }
        // this->switchDuration = std::chrono::seconds((rand() % 11) + 15);
        // this->switchDuration = std::chrono::seconds(85);
        if (_sdf->HasElement("switch_duration")) {
          double sec = _sdf->Get<double>("switch_duration");
          this->switchDuration = std::chrono::seconds(static_cast<int>(sec));
          printf("\033[32mswitch_duration set to %fs.\033[0m\n", sec);
        } else {
          this->switchDuration = std::chrono::seconds((rand() % 11) + 15);
        }

        double initPos = 0.0;
        if (_sdf->HasElement("initial_position")) {
          initPos = _sdf->Get<double>("initial_position");
          printf("\033[32minitial_position set to %frad.\033[0m\n", initPos);
        }
        this->targetPos = initPos;

        this->model->GetJointController()->SetPositionTarget(this->joint->GetScopedName(), this->targetPos);
      }

      // 每次仿真更新时调用
      void OnUpdate()
      {
        this->currentAngle = this->joint->Position(0);
        // if(abs(this->currentAngle - this->targetPos) > 0.01)
        //   printf("\033[34m currentAngle = %f\033[0m\n", this->currentAngle);

        // 获取当前时间
        auto now = std::chrono::steady_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::seconds>(now - this->lastSwitchTime);

        if(switchFlag) {
          // 如果超过设定的切换时间，就切换速度方向
          if (duration >= this->switchDuration)
          {
            if(this->targetPos == 1.57)
              this->targetPos = 0.0;
            else
              this->targetPos = 1.57;
            this->model->GetJointController()->SetPositionTarget(this->joint->GetScopedName(), this->targetPos);
            // std::cout << "切换门的目标位置: " << this->targetPos << std::endl;

            // 更新上次切换时间
            this->lastSwitchTime = now;
          }
        }
      }

    private:
      physics::ModelPtr model;
      physics::JointPtr joint;
      common::PID pid;
      event::ConnectionPtr updateConnection;
      
      double targetPos;
      double currentAngle;

      bool switchFlag;

      // 切换控制的时间
      std::chrono::steady_clock::time_point lastSwitchTime;
      std::chrono::seconds switchDuration;
  };

  // 注册插件
  GZ_REGISTER_MODEL_PLUGIN(WallAndDoorPlugin)
}