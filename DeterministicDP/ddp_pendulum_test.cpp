
#define _USE_MATH_DEFINES
#include <math.h>
#include "ddp_pendulum_test.hpp"
#include "ddp_pendulum.hpp"
#include "ddp_solver.hpp"

typedef ddp::solver<ddp::pendulum> p_ddp;

void savePolicy(p_ddp & p, std::string out_name, int Ns1, int Ns2) {
	FILE* fptr = fopen(out_name.c_str(), "w");

	if (fptr) {
		p_ddp::DDP & ddp_ = p.ddp();
		p_ddp::Policy & policy = p.getPolicy();

		for (int j = 0; j < Ns2; ++j) {
			for (int i = 0; i < Ns1; ++i) {

				int s_idx = ddp_.getNetIdx(i, j);
				int c_idx = policy[0][s_idx];
				double u = ddp_.getControlAt(c_idx);

				fprintf(fptr, "%0.6lf", u);
				if (j < (Ns1-1) ) {
					fprintf(fptr, ",");
				}

			} fprintf(fptr, "\n");
		}


		fclose(fptr); fptr = nullptr;
	}
}

void ddp::pendulumTest(std::string out_file)
{

	int N = 80, Ns1 = 4000, Ns2 = Ns1, Nc = 31;
	double Q = 10.0, Qf = 100, R = 5;
	p_ddp pddp;
	pddp.setNumIterations(N);
	p_ddp::DDP & ddp_ = pddp.ddp();
	ddp_.setDt(1e-1);
	ddp_.setCostFuncVars(Q, Qf, R);
	ddp_.setAngleRange(-M_PI, M_PI, Ns1);
	ddp_.setAngVelRange(-3 * M_PI, 3 * M_PI, Ns2);
	ddp_.setControlRange(-1, 1, Nc);

	pddp.solve();
	p_ddp::Policy & policy = pddp.getPolicy();

	/*
	int si_1 = ddp_.getStateIdx(0.0, 0);
	int si_2 = ddp_.getStateIdx(0, 1);
	int s_idx = ddp_.getNetIdx(si_1, si_2);
	printf("theta = %lf\n", ddp_.getStateAtNetIdx(s_idx).theta * 180 / 3.1415);
	for (int i = 0; i < (N - 1); ++i) {
		int c_idx = policy[0][s_idx];
		double u = ddp_.getControlAt(c_idx);
		s_idx = ddp_.getNextState(s_idx, c_idx);
		ddp::pendulum::state s = ddp_.getStateAtNetIdx(s_idx);
		printf("theta = %4.3lf, theta_dot = %4.3lf, u = %4.3lf\n", s.theta * 180 / 3.1415, s.thetadot, u);
	}
	*/

	// save the policy
	savePolicy(pddp, out_file.c_str(), Ns1, Ns2);
}
