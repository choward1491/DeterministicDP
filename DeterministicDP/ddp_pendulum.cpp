#include "ddp_pendulum.hpp"
#define _USE_MATH_DEFINES
#include <math.h>

namespace ddp {

	pendulum::pendulum()
	{
	}

	pendulum::~pendulum()
	{
	}

	void pendulum::init()
	{
	}

	double pendulum::finalCost(int state_idx)
	{
		int idx1 = 0, idx2 = 0;
		getIndicies(state_idx, idx1, idx2);
		struct state s = { t[idx1],td[idx2] };
		double dtheta = fabs(s.theta) - M_PI;
		return Qf*dtheta*dtheta + s.thetadot*s.thetadot ;
	}

	int pendulum::getNextState(int state_idx, int control_idx)
	{
		double k = 5, c = 0;
		int idx1 = 0, idx2 = 0;
		getIndicies(state_idx, idx1, idx2);
		struct state s = { t[idx1],td[idx2] };
		const double & u = control[control_idx];
		double tn  = s.theta + s.thetadot*dt;
		double tdn = s.thetadot + u*dt - dt*(sin(s.theta)*k + s.thetadot*c );
		const double pi_2 = 2.0*M_PI;
		if (tn < -M_PI) { tn = pi_2 + tn; }
		if (tn > M_PI) { tn = tn - pi_2; }

		int idx_t = getStateIdx(tn, 0);
		int idx_td = getStateIdx(tdn, 1);

		return getNetIdx(idx_t, idx_td);
	}

	double pendulum::cost(int current_iter, int max_iter, int state_idx, int control_idx)
	{
		int idx1 = 0, idx2 = 0;
		getIndicies(state_idx, idx1, idx2);
		struct state s = { t[idx1],td[idx2] };
		const double & u = control[control_idx];
		double dtheta = fabs(s.theta) - M_PI;
		return R*u*u + Q*dtheta*dtheta;
	}

	int pendulum::numControls()
	{
		return Nc;
	}

	int pendulum::numStates()
	{
		return Nn;
	}

	void pendulum::setAngleRange(double t1, double t2, int num_ang)
	{
		N1 = num_ang;
		setRange(t1, t2, num_ang, t);
		Nn = N1*N2;
	}

	void pendulum::setAngVelRange(double av1, double av2, int num_angvel)
	{
		N2 = num_angvel;
		setRange(av1, av2, num_angvel, td); 
		Nn = N1*N2;
	}

	void pendulum::setControlRange(double c1, double c2, int num_controls)
	{
		Nc = num_controls;
		setRange(c1, c2, Nc, control);
	}

	void pendulum::setDt(double dt_)
	{
		dt = dt_;
	}

	void pendulum::setCostFuncVars(double Q_, double Qf_, double R_)
	{
		Q	= Q_;
		Qf	= Qf_;
		R	= R_;
	}

	int pendulum::getStateIdx(double val, int state_dim)
	{
		std::vector<double> * sv = &t;
		if (state_dim == 1) { 
			sv = &td;
		}
		const double invN = 1.0 / static_cast<double>(sv->size() - 1);
		const double lb = (*sv)[0], del = (*sv)[1] - (*sv)[0];
		int idx = (val - lb)/del;
		if (idx >= sv->size()) { idx = sv->size() - 1; }
		if (idx < 0) { idx = 0; }
		return idx;
	}

	int pendulum::getNetIdx(int idx1, int idx2) const
	{
		return idx1 + N1*idx2;
	}

	struct pendulum::state pendulum::getStateAtNetIdx(int net_idx)
	{
		int idx1 = 0, idx2 = 0;
		getIndicies(net_idx, idx1, idx2);
		struct state state_ = {t[idx1],td[idx2]};
		return state_;
	}

	double pendulum::getControlAt(int c_idx) {
		return control[c_idx];
	}

	void pendulum::getIndicies(int net_idx, int & idx1, int & idx2) const
	{
		idx1 = net_idx % N1;
		idx2 = net_idx / N1;
	}

	void pendulum::setRange(double start, double end, int N, std::vector<double>& vec)
	{
		if (vec.size() != N) { vec.resize(N); }
		double del = (end - start) / static_cast<double>(N - 1);
		for (int i = 0; i < N; ++i) {
			vec[i] = start + i*del;
		}
	}


}
