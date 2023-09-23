#include <functional>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <ignition/math/Vector3.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>

#include <thread>
#include <ros/ros.h>
#include <std_msgs/Float32MultiArray.h>
namespace gazebo
{
	class UR10JointController : public ModelPlugin
	{
		public: void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
		{
			double Kp=0, Kd=0, Ki=0;
			std::cout << "This Control plugin is attached to [" << _parent->GetName() << "]\n";

			if (_parent->GetJointCount() == 0)
			{
				std::cerr << "Invalid joint count. Plugin not loaded correctly\n";
				exit(EXIT_FAILURE);
			}
			this->model = _parent;
			this->jointCount = this->model->GetJointCount();

			if (_sdf->HasElement("Kp"))
			{
				Kp = _sdf->Get<double>("Kp");
				std::cout << "Loaded Kp component: " << Kp << std::endl;
			}
			if (_sdf->HasElement("Ki"))
			{
				Ki = _sdf->Get<double>("Ki");
				std::cout << "Loaded Ki component: " << Ki << std::endl;
			}
			if (_sdf->HasElement("Kd"))
			{
				Kd = _sdf->Get<double>("Kd");
				std::cout << "Loaded Kd component: " << Kd << std::endl;
			}
			this->controller = common::PID(Kp, Ki, Kd);
			for(auto joint:this->model->GetJoints())
			{
				this->model->GetJointController()->SetPositionPID(
					joint->GetScopedName(), this->controller
				);
			}

			// interface with ROS
			this->rosNode.reset(new ros::NodeHandle("ur10_plugin"));

			// create a named topic and subscribe to it
			ros::SubscribeOptions so = ros::SubscribeOptions::create<std_msgs::Float32MultiArray> (
				"/"+this->model->GetName() + "/joint_position_cmd",
				1,
				boost::bind(&UR10JointController::OnMsg, this, _1),
				ros::VoidPtr(), &this->rosQueue
			);

			// subscribe to topic
			this->rosSub = this->rosNode->subscribe(so);

			// spin the thread of the ros node
			this->rosQueueThread = std::thread(std::bind(&UR10JointController::QueueThread, this));
		}
		public: void SetPosition(double x, double y, double z)
		{
			this->model->GetJoints()[1]->SetPosition(0, x);
			this->model->GetJoints()[2]->SetPosition(0, y);
			this->model->GetJoints()[3]->SetPosition(0, z);
		}
		private: void  QueueThread()
		{
			static const double timeout = 0.01;
			while(this->rosNode->ok())
			{
				this->rosQueue.callAvailable(ros::WallDuration(timeout));
			}
		}
		private: void OnMsg(ConstVector3dPtr &_msg)
		{
			this->SetPosition(_msg->x(), _msg->y(), _msg->z());
		}
		// Called by the world update start event
		public: void OnUpdate()
		{
			// Apply a small linear velocity to the model.
			// this->model->GetJoints()[1]->SetPosition(0, 2);
		}

		// Pointer to the model
		private: physics::ModelPtr model;

		// Pointer to the update event connection
		private: event::ConnectionPtr updateConnection;

		std::vector<physics::JointPtr> joints;
		common::PID controller;
		unsigned int jointCount;

		std::unique_ptr<ros::NodeHandle> rosNode;
		ros::Subscriber rosSub;
		ros::Publisher rosPub;

		ros::CallbackQueue rosQueue;
		std::thread rosQueueThread;
	};

	// Register this plugin with the simulator
	GZ_REGISTER_MODEL_PLUGIN(UR10JointController)
}
