#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <iostream>
#include <chrono>

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
      void Load(physics::ModelPtr _model, sdf::ElementPtr /*_sdf*/)
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

        // 设置初始目标速度
        // this->velocityTarget = 1.0; // 初始目标速度为1.0

        // 连接到更新事件
        this->updateConnection = event::Events::ConnectWorldUpdateBegin(
            std::bind(&WallAndDoorPlugin::OnUpdate, this));
        
        // 设置切换周期时间
        this->lastSwitchTime = std::chrono::steady_clock::now();
        this->switchDuration = std::chrono::seconds(20);

        this->targetPos = 0;

        this->model->GetJointController()->SetPositionTarget(this->joint->GetScopedName(), this->targetPos);
      }

      // 每次仿真更新时调用
      void OnUpdate()
      {
        this->currentAngle = this->joint->Position(0);
        if(abs(this->currentAngle - this->targetPos) > 0.01)
          printf("\033[34m currentAngle = %f\033[0m\n", this->currentAngle);

        // 获取当前时间
        auto now = std::chrono::steady_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::seconds>(now - this->lastSwitchTime);

        // 如果超过设定的切换时间，就切换速度方向
        if (duration >= this->switchDuration)
        {
          if(this->targetPos == 1.57)
            this->targetPos = 0.0;
          else
            this->targetPos = 1.57;
          this->model->GetJointController()->SetPositionTarget(this->joint->GetScopedName(), this->targetPos);
          std::cout << "切换门的目标位置: " << this->targetPos << std::endl;

          // 更新上次切换时间
          this->lastSwitchTime = now;
        }
      }

    private:
      physics::ModelPtr model;
      physics::JointPtr joint;
      common::PID pid;
      event::ConnectionPtr updateConnection;
      
      double targetPos;
      double currentAngle;
      
      // 控制目标速度的变量
      double velocityTarget;

      // 切换控制的时间
      std::chrono::steady_clock::time_point lastSwitchTime;
      std::chrono::seconds switchDuration;
  };

  // 注册插件
  GZ_REGISTER_MODEL_PLUGIN(WallAndDoorPlugin)
}

// #include <gazebo/gazebo.hh>
// #include <gazebo/physics/physics.hh>
// #include <gazebo/common/common.hh>

// namespace gazebo
// {
//   class WallAndDoorPlugin : public ModelPlugin
//   {
//     public:
//       WallAndDoorPlugin() : ModelPlugin()
//       {
//         // 构造函数
//       }

//       virtual ~WallAndDoorPlugin()
//       {
//         // 析构函数
//       }

//       // 加载插件
//       void Load(physics::ModelPtr _model, sdf::ElementPtr /*_sdf*/)
//       {
//         // 存储模型指针
//         this->model = _model;

//         // 获取关节指针
//         this->joint = this->model->GetJoint("Wall_7_JOINT_0");
//         if (!this->joint)
//         {
//           gzerr << "关节 Wall_7_JOINT_0 未找到！" << std::endl;
//           return;
//         }

//         // 设置关节速度控制器
//         this->pid = common::PID(2, 0.01, 0.001); // 比例增益为0.1，积分增益为0.01，微分增益为0.001
//         this->model->GetJointController()->SetVelocityPID(this->joint->GetScopedName(), this->pid);

//         // 初始目标角度
//         this->targetAngle = 1.57; // 90度（开启）
//         this->currentAngle = this->joint->Position(0);

//         // 设置目标速度
//         this->model->GetJointController()->SetVelocityTarget(this->joint->GetScopedName(), 1.0); // 设置目标速度为1.0

//         printf("\033[32m set velocity target\033[0m\n");

//         // 连接到更新事件
//         this->updateConnection = event::Events::ConnectWorldUpdateBegin(
//             std::bind(&WallAndDoorPlugin::OnUpdate, this));
//       }

//       // 每次仿真更新时调用
//       void OnUpdate()
//       {
//         // 获取当前关节角度
//         this->currentAngle = this->joint->Position(0);
//         printf("\033[34m currentAngle = %f\033[0m\n", this->currentAngle);

//         // 计算误差
//         double error = this->targetAngle - this->currentAngle;
//         printf("\033[32m error = %f\033[0m\n", error);

//         // 设置目标速度
//         double targetSpeed = error; // 比例增益为1
//         this->model->GetJointController()->SetVelocityTarget(this->joint->GetScopedName(), targetSpeed);

//         // 检查是否达到目标角度
//         if (std::abs(error) < 0.03) // 如果当前角度接近目标角度
//         {
//           // 切换目标角度
//           if (this->targetAngle > 1.5) // 如果当前目标是开启
//           {
//             this->targetAngle = 0.0; // 设置目标为关闭
//             printf("\033[34m set closed \033[0m\n");
//           }
//           else
//           {
//             this->targetAngle = 1.57; // 设置目标为开启
//             printf("\033[34m set opened \033[0m\n");
//           }
//         }

//         // 打印当前关节角度
//         // std::cout << "当前关节角度: " << this->currentAngle << " 弧度" << std::endl;
//       }

//     private:
//       physics::ModelPtr model;
//       physics::JointPtr joint;
//       common::PID pid;
//       event::ConnectionPtr updateConnection;
//       double targetAngle; // 目标角度
//       double currentAngle; // 当前角度
//   };

//   // 注册插件
//   GZ_REGISTER_MODEL_PLUGIN(WallAndDoorPlugin)
// }

// #include <gazebo/gazebo.hh>
// #include <gazebo/physics/physics.hh>
// #include <gazebo/common/common.hh>
// #include <iostream>
// #include <chrono>

// namespace gazebo
// {
//   class WallAndDoorPlugin : public ModelPlugin
//   {
//     public:
//       WallAndDoorPlugin() : ModelPlugin()
//       {
//         // 构造函数
//       }

//       virtual ~WallAndDoorPlugin()
//       {
//         // 析构函数
//       }

//       // 加载插件
//       void Load(physics::ModelPtr _model, sdf::ElementPtr /*_sdf*/)
//       {
//         // 存储模型指针
//         this->model = _model;

//         // 获取关节指针
//         this->joint = this->model->GetJoint("Wall_7_JOINT_0");
//         if (!this->joint)
//         {
//           gzerr << "关节 Wall_7_JOINT_0 未找到！" << std::endl;
//           return;
//         }

//         // 设置关节速度控制器
//         this->pid = common::PID(0.1, 0, 0); // 比例增益为0.1
//         this->model->GetJointController()->SetVelocityPID(this->joint->GetScopedName(), this->pid);

//         // 设置初始目标速度
//         this->velocityTarget = 1.0; // 初始目标速度为1.0

//         // 连接到更新事件
//         this->updateConnection = event::Events::ConnectWorldUpdateBegin(
//             std::bind(&WallAndDoorPlugin::OnUpdate, this));
        
//         // 设置切换周期时间（例如2秒）
//         this->lastSwitchTime = std::chrono::steady_clock::now();
//         this->switchDuration = std::chrono::seconds(2); // 2秒切换一次
//       }

//       // 每次仿真更新时调用
//       void OnUpdate()
//       {
//         // 获取当前时间
//         auto now = std::chrono::steady_clock::now();
//         auto duration = std::chrono::duration_cast<std::chrono::seconds>(now - this->lastSwitchTime);

//         // 如果超过设定的切换时间，就切换速度方向
//         if (duration >= this->switchDuration)
//         {
//           // 切换目标速度（正负切换）
//           this->velocityTarget = -this->velocityTarget; // 如果是正方向，就切换到负方向，反之亦然

//           // 更新上次切换时间
//           this->lastSwitchTime = now;

//           // 设置新的目标速度
//           this->model->GetJointController()->SetVelocityTarget(this->joint->GetScopedName(), this->velocityTarget);
//           std::cout << "切换门的目标速度: " << this->velocityTarget << std::endl;
//         }
//       }

//     private:
//       physics::ModelPtr model;
//       physics::JointPtr joint;
//       common::PID pid;
//       event::ConnectionPtr updateConnection;
      
//       // 控制目标速度的变量
//       double velocityTarget;

//       // 切换控制的时间
//       std::chrono::steady_clock::time_point lastSwitchTime;
//       std::chrono::seconds switchDuration;
//   };

//   // 注册插件
//   GZ_REGISTER_MODEL_PLUGIN(WallAndDoorPlugin)
// }
