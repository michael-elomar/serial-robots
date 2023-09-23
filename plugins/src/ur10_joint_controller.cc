#include <functional>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <ignition/math/Vector3.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>

namespace gazebo
{
	class UR10JointController : public ModelPlugin
	{
		public: void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
		{
			// Store the pointer to the model
			this->model = _parent;
			// this->model->GetWorld()->SetGravity(ignition::math::v6::Vector3d(0,0,0));
			// Listen to the update event. This event is broadcast every
			// simulation iteration.
			// this->model->SetGravityMode(true);
			this->updateConnection = event::Events::ConnectWorldUpdateBegin(
			std::bind(&UR10JointController::OnUpdate, this));

			// Create the node
			this->node = transport::NodePtr(new transport::Node());
			#if GAZEBO_MAJOR_VERSION < 8
			this->node->Init(this->model->GetWorld()->GetName());
			#else
			this->node->Init(this->model->GetWorld()->Name());
			#endif

			// Create a topic name
			std::string topicName = "~/" + this->model->GetName() + "/vel_cmd";

			// Subscribe to the topic, and register a callback
			this->sub = this->node->Subscribe(topicName,
			&UR10JointController::OnMsg, this);
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

		public: void SetPosition(double x, double y, double z)
		{
			this->model->GetJoints()[1]->SetPosition(0, x);
			this->model->GetJoints()[2]->SetPosition(0, y);
			this->model->GetJoints()[3]->SetPosition(0, z);
		}

		// Pointer to the model
		private: physics::ModelPtr model;

		// Pointer to the update event connection
		private: event::ConnectionPtr updateConnection;

		/// \brief A node used for transport
		private: transport::NodePtr node;

		/// \brief A subscriber to a named topic.
		private: transport::SubscriberPtr sub;

	};

	// Register this plugin with the simulator
	GZ_REGISTER_MODEL_PLUGIN(UR10JointController)
}
