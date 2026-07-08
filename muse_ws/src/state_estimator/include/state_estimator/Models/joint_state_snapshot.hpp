#ifndef STATE_ESTIMATOR_MODELS_JOINT_STATE_SNAPSHOT_HPP
#define STATE_ESTIMATOR_MODELS_JOINT_STATE_SNAPSHOT_HPP

#include <string>
#include <vector>

namespace state_estimator
{

struct JointStateSnapshot
{
	std::vector<std::string> name;
	std::vector<double> position;
	std::vector<double> velocity;
	std::vector<double> acceleration;
	std::vector<double> effort;

	std::size_t size() const
	{
		return name.size();
	}
};

} // namespace state_estimator

#endif // STATE_ESTIMATOR_MODELS_JOINT_STATE_SNAPSHOT_HPP
