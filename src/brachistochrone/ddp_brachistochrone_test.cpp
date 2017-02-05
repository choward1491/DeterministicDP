
#include "ddp_brachistochrone_test.hpp"
#include "ddp_brachistochrone.hpp"
#include "ddp_solver.hpp"
#include <stdio.h>

typedef ddp::solver<ddp::brachistochrone, int, double> brach_ddp;


void ddp::brachistochroneTest(const std::string & output_path)
{
	int N = 500, Ny = 101, Nc = 201;
	brach_ddp bddp;
	bddp.setNumIterations(N);
	brach_ddp::DDP & ddp_ = bddp.ddp();
	ddp_.setNumPointsCurve(N);
	ddp_.setNumVerticlePositions(Ny);
	ddp_.setStartAndEndPose(0, 1e-6, 5, -5);
	ddp_.setControlRange(-100, 0, Nc);
	bddp.solve();
	brach_ddp::Policy& policy = bddp.getPolicy();

	// test policy
	int n_idx	= ddp_.getStateIdx(0.0);

	// print out trajectory
	FILE* fptr = fopen(output_path.c_str(), "w");
	if (fptr) {
		double y = ddp_.getStateAtNetIdx(n_idx).y;
		
		for (int i = 0; i < (N - 1); ++i) {
			double c = ddp_.getControlAt(policy[i][n_idx]);
			fprintf(fptr, "%lf, %lf\n", y, c);
			n_idx = ddp_.getNextState(n_idx, policy[i][n_idx]);
			y = ddp_.getStateAtNetIdx(n_idx).y;
		}
		fprintf(fptr, "%lf, 0.0\n", y);
		fclose(fptr); fptr = nullptr;
	}

	// get cost graph
	brach_ddp::CostGraph & cg = bddp.getCostGraph();
}
