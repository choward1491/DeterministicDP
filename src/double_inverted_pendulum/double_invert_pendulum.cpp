#include "double_invert_pendulum.hpp"

namespace ddp {

	double_invert_pendulum::double_invert_pendulum()
	{
	}


	double_invert_pendulum::~double_invert_pendulum()
	{
	}

	void double_invert_pendulum::init()
	{
	}

	double double_invert_pendulum::finalCost(int state_idx)
	{
		return 0.0;
	}

	int double_invert_pendulum::getNextState(int state_idx, int control_idx)
	{
		return 0;
	}

	double double_invert_pendulum::cost(int current_iter, int max_iter, int state_idx, int control_idx)
	{
		return 0.0;
	}

	int double_invert_pendulum::numControls()
	{
		return 0;
	}

	int double_invert_pendulum::numStates()
	{
		return 0;
	}

}