


#include <stdio.h>
#include "ddp_solver.hpp"
#include "ddp_brachistochrone.hpp"

typedef ddp::solver<ddp::brachistochrone> brach_ddp;

int main(int argc, char** argv) {

	int N = 20, Nv = 10;
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


	return 0;
}