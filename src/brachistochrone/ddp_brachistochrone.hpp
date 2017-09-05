#pragma once

#include <vector>
#include "src/deterministic_dp_control_solver/ddp_abstract_dyn.hpp"

namespace ddp {
	class brachistochrone : public dynamics {
	public:

		enum state_name { y=0, u_o };

		brachistochrone();
		~brachistochrone();

		// methods that must be implemented
		virtual void		init();
		virtual double		finalCost(uint32_t state_idx);
		virtual uint32_t	getNextState(uint32_t state_idx, uint32_t control_idx);
		virtual double		cost(uint32_t current_iter, uint32_t max_iter, uint32_t state_idx, uint32_t control_idx);
		void setMaxIterations(uint32_t max_iter);

	private:
		double p1[2], p2[2];
		double dx;
		
	};

}
