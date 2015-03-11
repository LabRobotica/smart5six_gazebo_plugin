#include "ComauGazeboPlugin.h"

static const double JOINT_MAX_FORCE = 10000;
static const std::string PATH_URDF = "/robot/comau.urdf";

namespace gazebo {

//ComauGazeboPlugin::ComauGazeboPlugin()
//{
//pgain_ = 2.0;
//dgain_ = 0.0;
// check PID values
//std::string name = "comau_controller/follow_joint_trajectory";

//as_(nh_, name, boost::bind(&ComauGazeboPlugin::goalCB, this, _1), false);
//action_name_(name);

//as_(nh_, "comau_controller/follow_joint_trajectory", boost::bind(&ComauGazeboPlugin::goalCB, this, _1), false);
//action_name_ ("comau_controller/follow_joint_trajectory")

//Register callback functions:
//as_.registerGoalCallback();
//as_.registerPreemptCallback(boost::bind(&preemptCB, this));

//as_.start();
//}

ComauGazeboPlugin::~ComauGazeboPlugin()
{
#ifdef COMAU
	delete comau;
#else
	delete pid;
#endif
	delete rosnode_;
}

void ComauGazeboPlugin::goalCB(const control_msgs::FollowJointTrajectoryGoal::ConstPtr& goal)
{
	//If the server has been killed, don't process
	if(!as_.isActive()||as_.isPreemptRequested())
		return;
	//Run the processing at 5Hz
	ros::Rate rate(5);
	//Setup some local variables
	bool success= true;

	ROS_INFO("%s is Processing Goal", action_name_.c_str());

	//Check for Ros kill
	if(!ros::ok())
	{
		success = false;
		ROS_INFO("%s Shutting Down", action_name_.c_str());
	}
	else
	{
		as_.publishFeedback(feedback);

		//Sleep for rate time
		rate.sleep();
	}

	//Publish the result if the goal wasn't preempted
	if(success)
	{
		ROS_INFO("%s Succeeded at Getting to Goal", action_name_.c_str());
		as_.setSucceeded(result);

		//goal_r.trajectory = goal->trajectory;
		//pointc=0;

		int traj_size = goal->trajectory.points.size();
		for(int i=0; i<traj_size; i++)
		{
			trajectory_msgs::JointTrajectory gj;
			gj.header.stamp = ros::Time::now();
			gj.points.resize(1);

			//aggiungere eventuale controllo che le dimensioni di names, positions e velocities siano le stesse

			int size = goal->trajectory.joint_names.size();
			gj.joint_names.resize(size);
			gj.points[0].positions.resize(size);
			gj.points[0].velocities.resize(size);

			for(int j=0; j< size; j++)
			{
				gj.joint_names[j] = goal->trajectory.joint_names[j];
				gj.points[0].positions[j] = goal->trajectory.points[i].positions[j];
				gj.points[0].velocities[j] = goal->trajectory.points[i].velocities[j];
			}
			//gj.points[0] = goal->trajectory.points[i];

			goal_pub.publish(gj);
		}

	}
	else
	{
		as_.setAborted(result,"I Failed!");
	}

	/*************** verificare se vi e' un comando che abilita la ricezione di altre traiettorie *****************/
	// accept the new goal
	//goal_ = as_.acceptNewGoal()->samples;
}
/*
void ComauGazeboPlugin::preemptCB()
{
	ROS_INFO("%s: Preempted", action_name_.c_str());
	//result.result = progress;

	// set the action state to preempted
	as_.setPreempted();
}*/

void ComauGazeboPlugin::trajCallback(const trajectory_msgs::JointTrajectory::ConstPtr& jointTraj)
{
#ifdef COMAU
	comau->setMove(jointTraj);
#else
	pid->setMove(jointTraj);
#endif

	for (unsigned int i = 0; i < jointTraj->points[0].positions.size(); i++)
	{
            desired.positions[i] = jointTraj->points[0].positions[i];
            desired.velocities[i] = jointTraj->points[0].velocities[i];
	//desired = jointTraj->points[0];

	//ROS_INFO("received joint_%d trajectory value: %f", i + 1, jointTraj->points[0].positions[i]);
	 }

	feedback.desired = desired;
	startMovement = ros::WallTime::now();
}

void ComauGazeboPlugin::cmdCallback(const sensor_msgs::JointState::ConstPtr& jointState)
{

#ifdef COMAU
	comau->setMove(jointState);
#else
	pid->setMove(jointState);
#endif

	for (unsigned int i = 0; i < jointState->position.size(); i++) {
		desired.positions[i] = jointState->position[i];
		//desired.velocities[i] = jointState->velocity[i];

		//ROS_INFO("received joint_%d state value: %f", i + 1, jointState->position[i]);
	}

	feedback.desired = desired;
	startMovement = ros::WallTime::now();
}

void ComauGazeboPlugin::OnUpdate()
{
	//if(pointc < goal_r.trajectory.points.size())
	//{
		/*trajectory_msgs::JointTrajectory gj;
		gj.header.stamp = ros::Time::now();
		gj.points.resize(1);
		gj.points[0].positions.resize(goal_r.trajectory.points[i].positions.size());
		gj.joint_names.resize(goal_r.trajectory.joint_names.size());

		gj.points[0] = goal_r.trajectory.points[pointc];

		#ifdef COMAU
			comau->setMove(goal_r.trajectory);
		#else
			//pid->setMove(jointTraj);
		#endif

		desired = goal_r.trajectory.points[pointc];

		feedback.desired = desired;
		startMovement = ros::WallTime::now();
		//ROS_INFO("Iterazione numero %d", pointc);
		 */

		/*sensor_msgs::JointState gj;
		gj.position.resize(goal_r.trajectory.points[pointc].positions.size());
		gj.name.resize(goal_r.trajectory.joint_names.size());
		gj.header.stamp = ros::Time::now();
		for(int j=0; j< gj.position.size(); j++)
		{
			gj.name[j]=goal_r.trajectory.joint_names[j];
			gj.position[j] = goal_r.trajectory.points[pointc].positions[j];
		}

		#ifdef COMAU
			comau->setMove(gj);
		#else
			//pid->setMove(jointState);
		#endif

		for (unsigned int i = 0; i < gj.position.size(); i++)
		{
			desired.positions[i] = gj.position[i];
			//desired.velocities[i] = jointState->velocity[i];

			//ROS_INFO("received joint_%d value: %f", i + 1, jointState->position[i]);
		}

		feedback.desired = desired;
		startMovement = ros::WallTime::now();*/

	//	pointc++;
	//}


	//increases Gazebo's real time factor
	if (ros::WallTime::now() - this->prevUpdateTime < this->updateRate)
		return;

#ifdef COMAU
	//comau->getNextInterpolationVelocities(jointsAngles, jointsVelocities);
	comau->getNextInterpolationAngles(jointsAngles);
#else
	pid->getNextInterpolationVelocities(jointsAngles, jointsVelocities);
	//pid->getNextInterpolationAngles(jointsAngles);
#endif

	for (unsigned int i = 0; i < this->joints_.size(); i++) {
		physics::JointPtr joint = this->joints_[i];

		gazebo::math::Angle gazeboAngle(jointsAngles[i]);
		/*************** verificare primo parametro ******************/
		joint->SetAngle(0, gazeboAngle);
		joint->SetVelocity(0, jointsVelocities[i]);

		actual.positions[i] = joint->GetAngle(0).Radian();
		actual.velocities[i] = joint->GetVelocity(0);
	}

	ros::WallDuration d = ros::WallTime::now() - startMovement;
	actual.time_from_start = ros::Duration(d.toSec());
	feedback.actual = actual;
	feedback_states_pub.publish(feedback);

	this->prevUpdateTime = ros::WallTime::now();
}

void ComauGazeboPlugin::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
{

	ROS_INFO("Gazebo's plugin is loading...");

	this->parent_model_ = _parent;
	if(!this->parent_model_)
		ROS_ERROR("ComauGazeboPlugin controller requires a Model as its parent");

	if(!_sdf->HasElement("robotNamespace"))
		ROS_ERROR("Missing <robotNamespace>");
	else
		robotNamespace = _sdf->GetElement("robotNamespace")->Get<std::string>();

	int argc = 0;
	char** argv = NULL;
	jointsNumber = 0;

	// read urdf
	if(!this->urdf_model_.initFile(ros::package::getPath("smart5six_description") + PATH_URDF))
	{
		ROS_ERROR("load urdf error!");
	}

	std::vector< boost::shared_ptr<urdf::Link> > links;
	urdf_model_.getLinks(links);
	ROS_INFO("links [%u]", links.size());

	std::map<std::string, boost::shared_ptr<urdf::Joint> > jointMap;
	jointMap = urdf_model_.joints_;
	ROS_INFO("jointState->position Map [%u]", jointMap.size());


	// set the update rate (2 ms)
	this->updateRate = ros::WallDuration(0.002);
	// initialize the prevUpdateTime
	this->prevUpdateTime = ros::WallTime::now();


	//robot jointState->position initialization
	std::map<std::string, boost::shared_ptr<urdf::Joint> >::iterator it = jointMap.begin();
	for(unsigned int i = 0; it != jointMap.end(); i++, it++) {

		std::string jointName = (*it).first;
		ROS_INFO("name=%s", jointName.c_str());

		if((*it).second->type != urdf::Joint::UNKNOWN && (*it).second->type != urdf::Joint::FIXED)
		{
			gazebo::physics::JointPtr joint = this->parent_model_->GetJoint(jointName);

			if(joint)
			{
				this->joints_.push_back(joint);
				ROS_INFO("Add joint: %s", (*it).second->name.c_str());

				jointsNumber++;
				jointsAngles.push_back(0.0);
				jointsVelocities.push_back(0.0);

				joint->SetVelocity(0, 0);
				joint->SetMaxForce(0, JOINT_MAX_FORCE);
				gazebo::math::Angle angle(jointsAngles[i]);
				joint->SetAngle(0, angle);
				joint->SetVelocity(0, jointsVelocities[i]);
			}
			else
				ROS_WARN("A joint named \"%s\" is not part of Mechanism Controlled jointState->position.\n", jointName.c_str());
		}
	}

	ROS_INFO("Joints inserted: %u", joints_.size());

	ros::init(argc, argv, "gazebo_ros", ros::init_options::NoSigintHandler | ros::init_options::AnonymousName);
	this->rosnode_ = new ros::NodeHandle(this->robotNamespace); // namespace comes from gazebo_model spawner

	//resize of the actual and desired msg with Comau's jointState->position number
	actual.positions.resize(jointsNumber);
	desired.positions.resize(jointsNumber);
	actual.velocities.resize(jointsNumber);
	desired.velocities.resize(jointsNumber);

	for (unsigned int i = 1; i <= jointsNumber; i++)
	{
		std::stringstream name;
		name << "joint_" << i;
		feedback.joint_names.push_back(name.str());
	}
	startMovement = ros::WallTime::now();

	goal_pub = rosnode_->advertise<trajectory_msgs::JointTrajectory>("joint_trajectory", 1);
	feedback_states_pub = rosnode_->advertise<control_msgs::FollowJointTrajectoryFeedback>("feedback_states", 1);
	cmd_angles_sub_ = rosnode_->subscribe<sensor_msgs::JointState>("joint_states", 1, &ComauGazeboPlugin::cmdCallback, this);
	trajectory_sub = rosnode_->subscribe<trajectory_msgs::JointTrajectory>("joint_trajectory", 1, &ComauGazeboPlugin::trajCallback, this);

#ifdef COMAU
	comau = new RobotMovementComauLib(jointsNumber);
#else
	pid = new RobotMovementPID(jointsNumber, updateRate.toSec());
	pid->setPGain(pgain_);
	pid->setDGain(dgain_);
#endif  

	ROS_INFO("starting gazebo plugin in namespace: %s", this->robotNamespace.c_str());
	this->updateConnection = event::Events::ConnectWorldUpdateBegin(boost::bind(&ComauGazeboPlugin::OnUpdate, this));
	ROS_INFO("Gazebo's plugin loading ended!");
}
} /* namespace gazebo */
