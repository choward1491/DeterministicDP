


#include <stdio.h>
#include "ddp_solver.hpp"
#include "ddp_brachistochrone.hpp"
#include "ddp_control1d.hpp"

typedef ddp::solver<ddp::brachistochrone> brach_ddp;
typedef ddp::solver<ddp::control1d> c1d_ddp;

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


	return 0;
}