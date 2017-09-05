
//#define USE_WHOLE_POLICY
#include "ddp_brachistochrone_test.hpp"
#include "ddp_brachistochrone.hpp"
#include "src/deterministic_dp_control_solver/ddp_solver.hpp"
#include <stdio.h>


typedef ddp::solver<ddp::brachistochrone,unsigned int, double> brach_ddp;


void ddp::brachistochroneTest(const std::string & output_path)
{
	int N = 1001, Ny = 1001, Nc = 11;
	brach_ddp bddp;
	bddp.setNumIterations(N);
	brach_ddp::DDP & ddp_ = bddp.ddp();
	ddp_.setMaxIterations(N);
	ddp_.setState(brach_ddp::DDP::state_name::y, 0, -1, Ny);
	//ddp_.setState(1, -10, 10, Ny);
	//ddp_.setControlRange(-10000, 0, Nc);
	ddp_.setControlSet({-1e4, -5e3, -2e3, -1e3, -5e2, -1e2, -50, -25, -10, -5, -2, -1, -0.5, -0.1, 0.0});
	bddp.solve();
	brach_ddp::Policy& policy = bddp.getPolicy();

	// test policy
	unsigned int n_idx	= ddp_.getStateIdx(0.0, 0);
	//unsigned int n_idx2	= ddp_.getStateIdx(-10, 1);
	//unsigned int n_idx	= ddp_.getNetIdx({ n_idx1, n_idx2 });

	std::vector<double> y(1);
	//ddp_.setMaxIterations(101);

	// print out trajectory
	FILE* fptr = fopen(output_path.c_str(), "w");
	if (fptr) {
		ddp_.getStateAtNetIdx(n_idx,y);
		
		for (int i = 0; i < (N - 1); ++i) {
			double c = ddp_.getControlAt(policy[n_idx]);
			fprintf(fptr, "%lf, %lf\n", y[0], c);
			printf("%lf, %lf\n", y[0], c);
			n_idx = ddp_.getNextState(n_idx, policy[n_idx]);
			ddp_.getStateAtNetIdx(n_idx, y);
		}
		fprintf(fptr, "%lf, 0.0\n", y[0]);
		fclose(fptr); fptr = nullptr;
	}

	// get cost graph
	//brach_ddp::CostGraph & cg = bddp.getCostGraph();
}
