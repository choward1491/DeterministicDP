
#include "ddp_glider_dyn.hpp"
#define _USE_MATH_DEFINES
#include <math.h>

namespace ddp {

	glider_dyn::glider_dyn():
		/*m(0.05),
		I(6e-3),
		g(9.81),
		rho(1.292),
		Sw(0.1),Se(0.025),
		l(0.35),lw(0.03),le(0.04)*/
		m(0.05),
		I(6e-3),
		g(9.81),
		rho(1.292),
		Sw(0.1), Se(0.025),
		l(0.35), lw(-0.03), le(0.05)
	{
		unsigned int num_state_vars = 5;
		setNumVars	(num_state_vars);
		q.resize	(num_state_vars);

	}

	glider_dyn::~glider_dyn()
	{
	}

	void glider_dyn::init()
	{
	}

	double glider_dyn::finalCost(uint32_t state_idx)
	{
		auto & s = this->getState(state_idx);
		double w = 1 * (s[yd] >= 0) - (s[yd] < 0)*s[yd];
		return -s[xd] - 1e-1*s[yd];
	}

	unsigned int glider_dyn::getNextState(uint32_t state_idx, uint32_t control_idx)
	{
		// get current state and control
		auto &	s = this->getState(state_idx);
		double	u = this->getControlAt(control_idx);
		/*std::vector<unsigned int> indices0(7);

		s[x = 1;
		s[y = 1;
		s[theta = 1;
		s[phi = 1;
		s[xd = 1;
		s[yd = 1;
		s[thetad = 1;

		q[x] = s[x;
		q[y] = s[y;
		q[theta] = s[theta;
		q[phi] = s[phi;
		q[xd] = s[xd;
		q[yd] = s[yd;
		q[thetad] = s[thetad;

		for (unsigned int i = 0; i < num_vars; ++i) {
			indices0[i] = getStateIdx(q[i], i);
		}

		unsigned int i1 = getNetIdx(indices0);
		*/

		// update state using discrete dynamics
		/*q[x]		= s[x		+ s[xd*dt;
		q[y]		= s[y		+ s[yd*dt;*/
		q[theta]	= bound_npi_pi(s[theta]	+ s[thetad]*dt);
		q[phi]		= bound_npi_pi(s[phi]	+ u*dt);
		s[phi]		= q[phi];
		double st		= sin(s[theta]), ct = cos(s[theta]);
		double stp		= sin(s[theta] + s[phi]), ctp = cos(s[theta] + s[phi]);
		double xd_w[2]	= {	s[xd] + lw*s[thetad]*st,
							s[yd] - lw*s[thetad]*ct};
		double xd_e[2]	= {	s[xd] + l*s[thetad]*st + le*(s[thetad] + u)*stp,
							s[yd] - l*s[thetad]*ct - le*(s[thetad] + u)*ctp };
		double s2_w		= xd_w[0] * xd_w[0] + xd_w[1] * xd_w[1];
		double s2_e		= xd_e[0] * xd_e[0] + xd_e[1] * xd_e[1];
		double alpha_w	= s[theta] - atan2(xd_w[1],xd_w[0]);
		double fw		= rho*Sw*sin(alpha_w)*s2_w;
		double alpha_e	= s[theta] + s[phi] - atan2(xd_e[1],xd_e[0]);
		double fe		= rho*Se*sin(alpha_e)*s2_e;

		q[xd]		= s[xd]		- dt*(fw*st + fe*stp)/m;
		q[yd]		= s[yd]		+ dt*((fw*ct + fe*ctp)/m - g);
		q[thetad]	= s[thetad]	- dt*(fe*(le + l*cos(s[phi])) + fw*lw)/I;

		// compute indices for state
		std::vector<uint32_t> & indices_ = this->getIndices();
		for (unsigned int i = 0; i < s.size(); ++i) {
			indices_[i] = getStateIdx(q[i], i);
		}

		// return net state index
		return getNetIdx(indices_);
	}

	double glider_dyn::cost(uint32_t current_iter, uint32_t max_iter, uint32_t state_idx, uint32_t control_idx)
	{
		std::vector<double> &	s = this->getState(state_idx);
		double					u = this->getControlAt(control_idx);
		double w = 1 * (s[yd] >= 0) - (s[yd] < 0)*s[yd];
		return -s[xd] - 1e-1*s[yd];
	}

	void glider_dyn::setDt(double dt_)
	{
		dt = dt_;
	}


	double glider_dyn::bound_0_2pi(double angle)
	{
		const double _2pi = 2.0*M_PI;
		while (angle > _2pi)	{ angle -= _2pi; }
		while (angle < 0.0)		{ angle += _2pi; }
		return angle;
	}

	double glider_dyn::bound_npi_pi(double angle)
	{
		const double _2pi = 2.0*M_PI;
		while (angle > M_PI) { angle -= _2pi; }
		while (angle < -M_PI) { angle += _2pi; }
		return angle;
	}

}