#pragma once

#include <vector>

namespace ddp {

	class control1d {
	public:
		control1d();
		~control1d();

		// interface methods
		void setStateRange(double xmin, double xmax, int Nx);
		void setControlRange(double umin, double umax, int Nu);
		void setQf(double qf_);
		void setQ(double q_);
		void setR(double r_);
		void setDt(double dt_);

		// template-dependent methods
		void init();
		double finalCost(int state_idx);
		int getNextState(int state_idx, int control_idx);
		double cost(int current_iter, int max_iter, int state_idx, int control_idx);
		int numControls();
		int numStates();
		double getStateAt(int idx) { return x[idx]; }

	private:
		double Qf, Q, R, dt, alpha, beta, Ns, Nc;
		double xb[2], dx;
		double ub[2], du;
		std::vector<double> x, u;

	};


}
