


#include <stdio.h>
#include "ddp_solver.hpp"
#include "ddp_brachistochrone.hpp"
#include "ddp_control1d.hpp"
#include "ddp_pendulum.hpp"

typedef ddp::solver<ddp::brachistochrone> brach_ddp;
typedef ddp::solver<ddp::control1d> c1d_ddp;
typedef ddp::solver<ddp::pendulum> p_ddp;

int main(int argc, char** argv) {

	/*int N = 20, Nv = 10;
	brach_ddp bddp;
	bddp.setNumIterations(N);
	brach_ddp::DDP & ddp_ = bddp.ddp();
	ddp_.setNumPointsCurve(N);
	ddp_.setNumVerticlePositions(Nv);
	ddp_.setStartAndEndPose(0, 5, 5, 0);
	bddp.solve();
	brach_ddp::Policy& policy = bddp.getPolicy();

	// test policy
	int ys_idx = ddp_.getYStateIdx(5.0);

	// get start idx
	int s_idx = ddp_.computeNetIdx(ys_idx-3, ys_idx);


	// print out trajectory
	double y = ddp_.getYStateAtNetIdx(s_idx);
	printf("%lf\n", 5.0);
	//printf("y1 = %lf, s = %i\n", y,s_idx);
	printf("%lf\n", y);
	for (int i = 0; i < (N-1); ++i) {
		s_idx = ddp_.getNextState(s_idx, policy[i][s_idx]);
		y = ddp_.getYStateAtNetIdx(s_idx);
		//printf("y%i = %lf, s = %i\n", i+2, y,s_idx);
		printf("%lf\n", y);
	}

	// get cost graph
	brach_ddp::CostGraph & cg = bddp.getCostGraph();
	*/

	/*
	int N = 100, Ns = 101, Nc = 21;
	c1d_ddp cddp;
	cddp.setNumIterations(N);
	c1d_ddp::DDP & ddp_ = cddp.ddp();
	ddp_.setDt(1e-2);
	ddp_.setQ(1);
	ddp_.setR(10);
	ddp_.setQf(100);
	ddp_.setStateRange(0, 10, Ns);
	ddp_.setControlRange(-10, 10, Nc);
	
	cddp.solve();
	c1d_ddp::Policy & policy = cddp.getPolicy();
	c1d_ddp::CostGraph & cg = cddp.getCostGraph();
	
	int s_idx = Ns - 1;
	printf("%lf\n", ddp_.getStateAt(s_idx));
	for (int i = 0; i < (N - 1); ++i) {
		s_idx = ddp_.getNextState(s_idx, policy[i][s_idx]);
		printf("%lf\n", ddp_.getStateAt(s_idx));
	}
	*/

	int N = 100, Ns1 = 100, Ns2 = 100, Nc = 11;
	double Q = 0, Qf = 100, R = 0;
	p_ddp pddp;
	pddp.setNumIterations(N);
	p_ddp::DDP & ddp_ = pddp.ddp();
	ddp_.setDt(1e-1);
	ddp_.setCostFuncVars(Q, Qf, R);
	ddp_.setAngleRange(-3.1415, 3.1415, Ns1);
	ddp_.setAngVelRange(-3*3.1415, 3*3.1415, Ns2);
	ddp_.setControlRange(-1, 1, Nc);

	pddp.solve();
	p_ddp::Policy & policy = pddp.getPolicy();
	p_ddp::CostGraph & cg  = pddp.getCostGraph();

	int si_1 = ddp_.getStateIdx(0,0);
	int si_2 = ddp_.getStateIdx(0, 1);
	int s_idx = ddp_.getNetIdx(si_1, si_2);
	printf("theta = %lf\n", ddp_.getStateAtNetIdx(s_idx).theta*180/3.1415);
	for (int i = 0; i < 16 /*(N - 1)*/; ++i) {
		int c_idx = policy[i][s_idx];
		double u = ddp_.getControlAt(c_idx);
		s_idx = ddp_.getNextState(s_idx, c_idx);
		ddp::pendulum::state s = ddp_.getStateAtNetIdx(s_idx);
		printf("theta = %4.3lf, theta_dot = %4.3lf, u = %4.3lf\n", s.theta * 180 / 3.1415, s.thetadot, u);
	}


	return 0;
}