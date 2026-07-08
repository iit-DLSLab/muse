#ifndef ROBOT_HPP
#define ROBOT_HPP

#include <memory>

namespace state_estimator {

class Robot;


class RobotFactory {
public:
	static std::unique_ptr<Robot> create();
};

class Robot : public RobotFactory {
public:
	friend class RobotFactory;
	
	Robot() { }
	virtual ~Robot() { }

	virtual void Hello() = 0;

	Robot(Robot&& that) { }

	Robot(Robot const &) = delete;
	Robot& operator=(const Robot &) = delete;

};


} //namespace state_estimator

#endif