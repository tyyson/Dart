#include <iostream>
#include "dart/dart.hpp"
#include "dart/gui/gui.hpp"
#include <math.h>

using namespace dart::dynamics;
using namespace dart::simulation;

const double default_width = 0.2;
const double default_height = 1;
const double default_torque = 10.0;
const double default_rest_position = 0.0;
const double zero_x = 321.0;
const double zero_y = 305.0;
const double radian = M_PI / 180.0;
const double DiffLimit = 0.03;
const double kp = 1.5;
const double kv = pow(kp , 2.0);
const int default_countdown = 100;
const int length_BodyNode = 110.0;

double first = 0.0;
double second = 0.0;

class MyWindow : public dart::gui::SimWindow

{
public:
	MyWindow(WorldPtr world)
	: j1_rotation(0.0),
		j2_rotation(0.0),
		j1d(0.0),
		j2d(0.0)
	{
		setWorld(world);
		mRobot = world->getSkeleton("robot");
		assert(mRobot != nullptr);


	}



	void Calc(int _x,int _y)
	{
		double new_x = _x - zero_x;
		double new_y = zero_y - _y;

		//std::cout<< "new_x : " << new_x << " new_y : " << new_y << std::endl;
		double numerator = pow(new_x,2.0) + pow(new_y,2.0) - (2 * pow(length_BodyNode,2.0));
		//std::cout<<"length_BodyNode power(2.0) val equals to " << pow(length_BodyNode,2.0) << std::endl;
		double denominator = 2 * pow(length_BodyNode,2.0);

		//std::cout<<"num / denom val equals to : " << numerator / denominator << std::endl;
		second = acos(numerator/denominator);
		if(new_x < 0) second = -second;

		double k1 = length_BodyNode + length_BodyNode * cos(second);
		double k2 = length_BodyNode * sin(second);

		first = atan2(new_x,new_y) - atan2(k2,k1);


		if(second == second) {
		/*std::cout<<
		"###########  IN ANGLE  #################"<<std::endl<<
		"calculated theta #1 : " << first / radian << std::endl<<
		"calculated theta #2 : " << second / radian <<std::endl<<std::endl;*/
		}
		else{
			std::cout<< "unreachable ! ...  position reset . . ." <<std::endl;
		}

		Joint* j1 = mRobot->getDof(4)->getJoint();
		Joint* j2 = mRobot->getDof(7)->getJoint();
		std::size_t index = 1;

		j1d = first;
		j2d = second;

		//j1->setPosition( index , first);
		//j2->setPosition( index , second);

		mRobot->computeForwardKinematics();
	}

	double diff(double desired, double current){return desired - current;}
	double abs(double d){return d > 0 ? d : -d;}
	bool isNan(double d){return d == d ? false : true;}

	void timeStepping() override
	{
		DegreeOfFreedom* dofj1 = mRobot->getDof(4);
		DegreeOfFreedom* dofj2 = mRobot->getDof(7);

		j1_rotation = dofj1->getJoint()->getPosition(1);
		j2_rotation = dofj2->getJoint()->getPosition(1);

		std::cout<<
		"j1 rotation is  : " << j1_rotation << std::endl <<
		"j2 rotation is : " << j2_rotation << std::endl;

		double j1diff = diff(j1d, j1_rotation);
		double j2diff = diff(j2d, j2_rotation);

		std::cout<<
		"difference with j1d is : " << j1diff << std::endl <<
		"difference with j2d is : " << j2diff << std::endl;
		/*std::cout<<
		"current joint #1 acceleration : " << docurrentfj1->getJoint()->getVelocity(1) << std::endl<<
		"current joint #2 acceleration : " << dofj2->getJoint()->getVelocity(1) << std::endl;*/


		double Force1 = (kp * j1diff - kv * dofj1->getJoint()->getVelocity(1));
		double Force2 = (kp * j2diff - kv * dofj2->getJoint()->getVelocity(1));


		if(abs(j1diff) > DiffLimit && !isNan(Force1)) {
			dofj1->setForce(Force1);
			std::cout<<"force applied #1 : " << Force1 << std::endl;
		}
		else {
			dofj1->resetVelocity();
			std::cout<<"#1 velocity reset "<< std::endl;
		}

		if(abs(j2diff) > DiffLimit && !isNan(Force2))
		{
			dofj2->setForce(Force2);
			std::cout<<"force applied #2 : " << Force2 << std::endl;
		}
		else {
			dofj2->resetVelocity();
			std::cout<<"#2 velocity reset "<< std::endl;
		}



		mRobot->getDof(1)->resetVelocity();

	   /*for(std::size_t i = 0; i < mRobot->getNumDofs(); ++i)
	   {
			 if(mForceCountDown[i] > 0)
        {
          DegreeOfFreedom* dof = mRobot->getDof(i);
          dof->setForce(default_torque);

          BodyNode* bn = dof->getChildBodyNode();
          auto visualShapeNodes = bn->getShapeNodesWith<VisualAspect>();
          visualShapeNodes[0]->getVisualAspect()->setColor(dart::Color::Red());

          --mForceCountDown[i];
        }
				else{
					DegreeOfFreedom* dof = mRobot->getDof(i);
					dof->resetVelocity();
				}
	   }*/
	    SimWindow::timeStepping();
  }

	void click(int _button, int _state, int _x,int _y) override
  {
		//std::cout<< "clicked  x : " << _x << " y : " << _y << std::endl;
		Calc(_x,_y);
 		//Win3D::click(_button,_state,_x,_y);
  }

	void initWindow(int _w, int _h, const char* _name) override
	{
  		Win3D::initWindow(_w,_h,_name);

  		mTrackBall.setQuaternion(Eigen::Quaterniond(0.707,-0.707,0,0));
  		/*
			std::cout<<
			"current quaternion : " <<
  		mTrackBall.getCurrQuat().x()<<" "<<
  		mTrackBall.getCurrQuat().y()<<" "<<
  		mTrackBall.getCurrQuat().z()<<" "<<
  		mTrackBall.getCurrQuat().w()<<" "<<
  		std::endl;
			*/
			mTrans = Eigen::Vector3d(-12.0, 299.518, -1596.52);

			mZoom = 0.2;
	}


	/*void keyboard(unsigned char key, int x,int y) override
	{
		switch(key)
		{

			case '1':
				applyTorque(4);
				break;

			case '2':
				applyTorque(7);
				break;

			default:
				SimWindow::keyboard(key,x,y);
		}
	}*/

protected:
	SkeletonPtr mRobot;
	double j1_rotation;
	double j2_rotation;
	double j1d;
	double j2d;
};

void setGeometry(const BodyNodePtr& bn)
{
  std::shared_ptr<BoxShape> box(new BoxShape(
      Eigen::Vector3d(default_width, default_width, default_height)));

  // Create a shape node for visualization and collision checking
  auto shapeNode
      = bn->createShapeNodeWith<VisualAspect, CollisionAspect, DynamicsAspect>(box);
  shapeNode->getVisualAspect()->setColor(dart::Color::Blue());

  // Set the location of the shape node
  Eigen::Isometry3d box_tf(Eigen::Isometry3d::Identity());
  Eigen::Vector3d center = Eigen::Vector3d(0, 0, default_height / 1.82);
  box_tf.translation() = center;
  shapeNode->setRelativeTransform(box_tf);

  // Move the center of mass to the center of the object
  bn->setLocalCOM(center);
}

BodyNode* makeRB(const SkeletonPtr& robot, const std::string& name){
	BallJoint::Properties properties;
	properties.mName = name + "_joint";

	BodyNodePtr bn = robot->createJointAndBodyNodePair<BallJoint>(nullptr,properties,BodyNode::AspectProperties(name)).second;
	std::shared_ptr<BoxShape> box(new BoxShape(Eigen::Vector3d(default_width,default_width,default_width)));

  auto shapeNode = bn->createShapeNodeWith<VisualAspect>(box);
  shapeNode->getVisualAspect()->setColor(dart::Color::Blue());

  setGeometry(bn);

	return bn;
}

BodyNode* addBD(const SkeletonPtr& robot, BodyNode* parent, const std::string& name){
	BallJoint::Properties properties;
	properties.mName = name + "_joint";
	properties.mT_ParentBodyToJoint.translation() =
	      Eigen::Vector3d(0, 0, default_height);

	std::pair<JointPtr, BodyNodePtr> jnb = robot->createJointAndBodyNodePair<BallJoint>(parent,properties,BodyNode::AspectProperties(name));
	JointPtr jr = jnb.first;
	BodyNodePtr bn = jnb.second;

	//jr->setAxis(Eigen::Vector3d(0.0, 1.0, 0.0));

	const double R = default_width / 2.0;
	const double h = default_width;
	std::shared_ptr<CylinderShape> cyl(new CylinderShape(R,h));

	Eigen::Isometry3d tf(Eigen::Isometry3d::Identity());
  tf.linear() = dart::math::eulerXYZToMatrix(Eigen::Vector3d(90.0 * M_PI / 180.0, 0, 0));

	auto shapeNode = bn->createShapeNodeWith<VisualAspect>(cyl);
  shapeNode->getVisualAspect()->setColor(dart::Color::Blue());
	shapeNode->setRelativeTransform(tf);

	 setGeometry(bn);

	return bn;
}
int main(int argc,char **argv)
{
	WorldPtr world(new World);
	world->setGravity(Eigen::Vector3d(0.0, 0.0, 0.0));
	SkeletonPtr mRobot = Skeleton::create("robot");

	BodyNode* bn = makeRB(mRobot,"link #1");
	bn = addBD(mRobot, bn, "link #2");
	bn = addBD(mRobot, bn, "link #3");

	world->addSkeleton(mRobot);



	MyWindow window(world);
	glutInit(&argc, argv);
	window.initWindow(640,480,"3-link-robot-simulation");
	glutMainLoop();


	return 0;
}
