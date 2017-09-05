#include "ddp_brachistochrone.hpp"
#include <math.h>


namespace ddp {

	brachistochrone::brachistochrone(){
		setNumVars(1);
	}


	brachistochrone::~brachistochrone(){
	}

	void brachistochrone::init()
	{
	}

	double brachistochrone::finalCost(uint32_t state_idx)
	{
		auto & s = this->getState(state_idx);
		double dy = s[y] + 1.0;
		return 1e5*(dy*dy);
	}

	uint32_t brachistochrone::getNextState(uint32_t state_idx, uint32_t control_idx)
	{
		auto & s = this->getState(state_idx);
		auto   u = this->getControlAt(control_idx);
		double y_ = s[y];

		double yn = y_ + dx*u;
		std::vector<uint32_t> & indices_ = this->getIndices();
		indices_[0] = getStateIdx(yn, 0);
		//indices_[1] = getStateIdx(u, 1);

		// return net state index
		return getNetIdx(indices_);
	}

	double brachistochrone::cost(uint32_t current_iter, uint32_t max_iter, uint32_t state_idx, uint32_t control_idx)
	{
		double g = 9.81;
		auto & s = this->getState(state_idx);
		auto   u = this->getControlAt(control_idx);
		double dy = -s[y] + 1e-6;
		return dx*( sqrt( ( 1 + u*u ) / (2.0*g*dy) ) );
	}

	void brachistochrone::setMaxIterations(uint32_t max_iter)
	{
		dx = 1.0 / (static_cast<double>(max_iter));
	}

	

}