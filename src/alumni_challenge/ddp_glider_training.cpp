
#define _USE_MATH_DEFINES
#include <math.h>
#include "ddp_glider_training.hpp"
#include "ddp_glider_dyn.hpp"
#include "src/deterministic_dp_control_solver/ddp_solver.hpp"

typedef ddp::solver<ddp::glider_dyn,unsigned char,double> p_ddp;

namespace ddp {

	void savePolicy(p_ddp & p, std::string out_name) {
		FILE* fptr = fopen(out_name.c_str(), "wb");

		if (fptr) {
			p_ddp::DDP & ddp_ = p.ddp();
			p_ddp::Policy & policy = p.getPolicy();

			for (unsigned int i = 0; i < policy.size(); ++i) {
				fwrite(&policy[i], sizeof(unsigned char), 1, fptr);
			}

			fclose(fptr); fptr = nullptr;
		}
	}

	enum state_name {
		x = 0, y, theta, phi, xd, yd, thetad
	};

	double bound_npi_pi(double angle)
	{
		const double _2pi = 2.0*M_PI;
		while (angle > M_PI) { angle -= _2pi; }
		while (angle < -M_PI) { angle += _2pi; }
		return angle;
	}

	void dynamics(std::vector<double> & s, double u, std::vector<double> & q) {
		/*double m(0.05),
			I(6e-3),
			g(9.81),
			rho(1.292),
			Sw(0.1), Se(0.025),
			l(0.35), lw(-0.03), le(0.05);
		double dt = 1e-2;

		q[x]		= s.x		+ s.xd*dt;
		q[y]		= s.y		+ s.yd*dt;
		q[theta] = bound_npi_pi(s.theta + s.thetad*dt);
		q[phi] = bound_npi_pi(s.phi + u*dt);

		double st = sin(s.theta), ct = cos(s.theta);
		double stp = sin(s.theta + s.phi), ctp = cos(s.theta + s.phi);
		double xd_w[2] = { s.xd + lw*s.thetad*st,
			s.yd - lw*s.thetad*ct };
		double xd_e[2] = { s.xd + l*s.thetad*st + le*(s.thetad + u)*stp,
			s.yd - l*s.thetad*ct - le*(s.thetad + u)*ctp };
		double s2_w = xd_w[0] * xd_w[0] + xd_w[1] * xd_w[1];
		double s2_e = xd_e[0] * xd_e[0] + xd_e[1] * xd_e[1];
		double alpha_w = s.theta - atan2(xd_w[1], xd_w[0]);
		double fw = rho*Sw*sin(alpha_w)*s2_w;
		double alpha_e = s.theta + s.phi - atan2(xd_e[1], xd_e[0]);
		double fe = rho*Se*sin(alpha_e)*s2_e;

		double df[3] = { (fw*st + fe*stp) / m , ((fw*ct + fe*ctp) / m - g) , (fe*(le + l*cos(s.phi)) + fw*lw) / I };
		q[xd] = s.xd - dt*(fw*st + fe*stp) / m;
		q[yd] = s.yd + dt*((fw*ct + fe*ctp) / m - g);
		q[thetad] = s.thetad - dt*(fe*(le + l*cos(s.phi)) + fw*lw) / I;*/
	}

	void trainGliderController(const std::string & filepath)
	{
		int N = 100, Nc = 7;
		p_ddp pddp;
		pddp.setNumIterations(N);
		p_ddp::DDP & ddp_ = pddp.ddp();

		// set timestep for dynamics
		ddp_.setDt(0.1);
		
		// set the discretized states
		/*ddp_.setState(ddp::glider_dyn::state_name::x,		0, 1, 15);
		ddp_.setState(ddp::glider_dyn::state_name::y,		0, 1, 15);*/
		ddp_.setState(ddp::glider_dyn::state_name::theta,	-M_PI/2, M_PI/2, 31);
		ddp_.setState(ddp::glider_dyn::state_name::phi,		-M_PI/2, M_PI/2, 31);
		ddp_.setState(ddp::glider_dyn::state_name::xd,		4, 10, 81);
		ddp_.setState(ddp::glider_dyn::state_name::yd,		-5, 5, 81);
		ddp_.setState(ddp::glider_dyn::state_name::thetad,	-M_PI/2, M_PI/2, 31);

		// set the control
		ddp_.setControlRange(-1, 1, Nc);

		// get the policy for the solution
		p_ddp::Policy & policy = pddp.getPolicy();

#ifndef LOAD_EXISTING
		// solve dynamic programming problem
		pddp.solve();

		// save the policy
		savePolicy(pddp, filepath);
#else
		policy.resize(ddp_.numStates());

		//load policy
		FILE* fptr = fopen(filepath.c_str(), "rb");

		if (fptr) {
			p_ddp::DDP & ddp_ = pddp.ddp();
			p_ddp::Policy & policy = pddp.getPolicy();

			for (unsigned int i = 0; i < policy.size(); ++i) {
				fread(&policy[i], sizeof(unsigned char), 1, fptr);
			}

			fclose(fptr); fptr = nullptr;
		}
#endif
		
		// test the policy
		/*std::vector<unsigned int> indices(5);
		std::vector<double> q {0, 2, 0, 0, 6, 0, 0};
		p_ddp::DDP::state s = { 0 };
		double dt = 1e-2, u = 0;;
		while (q[1] > 0) {
			s.x = q[x];
			s.y = q[y];
			s.theta = q[theta];
			s.phi = q[phi]; 
			s.xd = q[xd]; 
			s.yd = q[yd]; 
			s.thetad = q[thetad];
			for (int i = 0; i < 5; ++i) {
				indices[i] = ddp_.getStateIdx(q[i + 2], i);
			}
			unsigned int sidx = ddp_.getNetIdx(indices);
			u = ddp_.getControlAt(policy[sidx]);
			printf("x = %lf, y = %lf, xd = %lf, yd = %lf, u = %lf\n", s.x, s.y, s.xd, s.yd, u);
			dynamics(s, u, q);
		}

		printf("Max distance = %lf\n", q[0]);
		*/
		

	}

}
