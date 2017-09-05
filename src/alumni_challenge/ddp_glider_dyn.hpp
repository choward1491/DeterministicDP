
#ifndef _ddp_glider_dyn_
#define _ddp_glider_dyn_

#include "src/deterministic_dp_control_solver/ddp_solver.hpp"
#include "src/deterministic_dp_control_solver/ddp_abstract_dyn.hpp"

#include <vector>

namespace ddp {
	class glider_dyn : public dynamics {
	public:

		enum state_name {
			/*x = 0, y,*/ theta=0, phi, xd, yd, thetad
		};

		glider_dyn();
		~glider_dyn();

		// methods that must be implemented
		virtual void		init();
		virtual double		finalCost(uint32_t state_idx);
		virtual uint32_t	getNextState(uint32_t state_idx, uint32_t control_idx);
		virtual double		cost(uint32_t current_iter, uint32_t max_iter, uint32_t state_idx, uint32_t control_idx);

		// other methods
		void setDt(double dt_);

	private:

		// time step
		double dt;

		// temporary storage for state propogation
		std::vector<double> q;

		// physical constants
		const double m, I;
		const double g;
		const double rho;
		const double Sw, Se;
		const double l, lw, le;


		// helper functionality
		static double bound_0_2pi(double angle);
		static double bound_npi_pi(double angle);
	};

}

#endif
