#include <iostream>
#include "dart/dart.hpp"
#include "dart/gui/gui.hpp"

using namespace dart::dynamics;
using namespace dart::simulation;

const double default_width = 0.2;
const double default_height = 1;
const double default_torque = 15.0;
class MyWindow : public dart::gui::SimWindow
{
public:
	MyWindow(WorldPtr world){
		setWorld(world);
		mRobot = world->getSkeleton("robot");
		mForceCountDown.resize(mRobot->getNumDofs(), 0);
		assert(mRobot != nullptr);

	}

	void timeStepping() override
	{
	    for(std::size_t i = 0; i < mRobot->getNumBodyNodes(); i++)
	    {
	      BodyNode* bn = mRobot->getBodyNode(i);
	      auto visualShapeNodes = bn->getShapeNodesWith<VisualAspect>();

	    }

	   // Apply joint torques based on user input, and color the Joint shape red
	   for(std::size_t i = 0; i < mRobot->getNumDofs(); ++i)
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
	   }
	    SimWindow::timeStepping();
  }

	//void click(int _button, int _state, int _x,int _y) override
  //{
  //  std::cout<<"coordinate _x : " <<_x<<" _y : "<<_y<<std::endl;
  //}

	void initWindow(int _w, int _h, const char* _name) override
	{
  	GlutWindow::initWindow(_w, _h, _name);

  	int smaller = _w < _h ? _w : _h;
  	mTrackBall.setTrackball(Eigen::Vector2d(_w*0.5, _h*0.5), smaller/2.5);
		mTrackBall.startBall(10,10);
}

protected:
	SkeletonPtr mRobot;
	std::vector<int> mForceCountDown;
};

void setGeometry(const BodyNodePtr& bn)
{
  // Create a BoxShape to be used for both visualization and collishttps://github.com/tyyson/Dartion checking
  std::shared_ptr<BoxShape> box(new BoxShape(
      Eigen::Vector3d(default_width, default_width, default_height)));

  // Create a shape node for visualization and collision checking
  auto shapeNode
      = bn->createShapeNodeWith<VisualAspect, CollisionAspect, DynamicsAspect>(box);
  shapeNode->getVisualAspect()->setColor(dart::Color::Blue());

  // Set the location of the shape node
  Eigen::Isometry3d box_tf(Eigen::Isometry3d::Identity());
  Eigen::Vector3d center = Eigen::Vector3d(0, 0, default_height / 1.8);
  box_tf.translation() = center;
  shapeNode->setRelativeTransform(box_tf);

  // Move the center of mass to the center of the object
  bn->setLocalCOM(center);
}

BodyNode* makeRB(const SkeletonPtr& robot, const std::string& name){
	PlanarJoint::Properties properties;
	properties.mName = name + "_joint";
	BodyNodePtr bn = robot->createJointAndBodyNodePair<PlanarJoint>(nullptr,properties,BodyNode::AspectProperties(name)).second;
	std::shared_ptr<BoxShape> box(new BoxShape(Eigen::Vector3d(default_width,default_width,default_width)));

  auto shapeNode = bn->createShapeNodeWith<VisualAspect>(box);
  shapeNode->getVisualAspect()->setColor(dart::Color::Blue());

  setGeometry(bn);

	return bn;
}

BodyNode* addBD(const SkeletonPtr& robot, BodyNode* parent, const std::string& name){
	RevoluteJoint::Properties properties;
	properties.mName = name + "_joint";
	properties.mT_ParentBodyToJoint.translation() =
	      Eigen::Vector3d(0, 0, default_height);

	BodyNodePtr bn = robot->createJointAndBodyNodePair<RevoluteJoint>(parent,properties,BodyNode::AspectProperties(name)).second;

	const double R = default_width / 2.0;
	const double h = default_width;
	std::shared_ptr<CylinderShape> cyl(new CylinderShape(R,h));

	//Eigen::Isometry3d tf(Eigen::Isometry3d::Identity());
  //tf.linear() = dart::math::eulerXYZToMatrix(Eigen::Vector3d(90.0 * M_PI / 180.0, 0, 0));

	auto shapeNode = bn->createShapeNodeWith<VisualAspect>(cyl);
  shapeNode->getVisualAspect()->setColor(dart::Color::Blue());
	//shapeNode->setRelativeTransform(tf);

	 setGeometry(bn);

	return bn;
}
int main(int argc,char **argv)
{
	WorldPtr world = std::make_shared<World>();
	SkeletonPtr mRobot = Skeleton::create("robot");

	BodyNode* bn = makeRB(mRobot,"link #1");
	bn = addBD(mRobot, bn, "link #2");
	bn = addBD(mRobot, bn, "link #3");

	world->addSkeleton(mRobot);

	BodyNode* tip;
	for(int i=0;i<mRobot->getNumBodyNodes();i++){
		tip = mRobot->getBodyNode(i);
		Eigen::Vector3d location = tip->getTransform() * Eigen::Vector3d(0.0, 0.0, default_height);
		std::cout<<"index #"<< i<<" location is : "<<location << std::endl;
	}

	//mRobot->getDof(0)->setPosition(75 * M_PI / 180.0);
	MyWindow window(world);
	glutInit(&argc, argv);
	window.initWindow(640,480,"3-link-robot-simulation");
	glutMainLoop();


	return 0;
}
