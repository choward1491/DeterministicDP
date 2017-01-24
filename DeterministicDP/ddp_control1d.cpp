#include "ddp_control1d.hpp"

namespace ddp {

	/*
	double Qf, Q, R, dt, alpha, beta, Ns, Nc;
	double xb[2], dx;
	double ub[2], du;
	std::vector<double> x, u;
	*/

	control1d::control1d(){

	}


	control1d::~control1d(){

	}

	void control1d::setStateRange(double xmin, double xmax, int Nx)
	{
		xb[0] = xmin; xb[1] = xmax; dx = (xmax - xmin) / static_cast<double>(Nx - 1);
		Ns = Nx;
		if (x.size() != Nx) { x.resize(Nx); }
		for (int i = 0; i < Nx; ++i) {
			x[i] = xmin + i*dx;
		}
	}

	void control1d::setControlRange(double umin, double umax, int Nu)
	{
		ub[0] = umin; ub[1] = umax; du = (umax - umin) / static_cast<double>(Nu - 1);
		Nc = Nu;
		if (u.size() != Nu) { u.resize(Nu); }
		for (int i = 0; i < Nu; ++i) {
			u[i] = umin + i*du;
		}
	}

	void control1d::setQf(double qf_)
	{
		Qf = qf_;
	}

	void control1d::setQ(double q_)
	{
		Q = q_;
	}

	void control1d::setR(double r_)
	{
		R = r_;
	}

	void control1d::setDt(double dt_)
	{
		dt = dt_;
	}

	void control1d::init() {
		alpha = 1 - dt;
		beta = dt * 1.0;
		alpha = 1.0;
		beta = 1.0;
	}

	double control1d::finalCost(int state_idx) {
		const double & xv = x[state_idx];
		return Qf*xv*xv;
	}

	int control1d::getNextState(int state_idx, int control_idx) {
		const double & xv = x[state_idx];
		const double & uv = u[control_idx];
		double xn = alpha*xv + beta*uv;
		int sn_idx = static_cast<int>(0.5 + (xn-xb[0])/dx);
		if (sn_idx < 0) { sn_idx = 0; }
		if (sn_idx >= Ns) { sn_idx = Ns - 1; }
		return sn_idx;
	}

	double control1d::cost(int current_iter, int max_iter, int state_idx, int control_idx) {
		const double & xv = x[state_idx];
		const double & uv = u[control_idx];
		return Q*xv*xv + R*uv*uv;
	}

	int control1d::numControls() {
		return Nc;
	}

	int control1d::numStates() {
		return Ns;
	}

}