
#include "ddp_control1d_test.hpp"
#include "src/deterministic_dp_control_solver/ddp_solver.hpp"
#include "ddp_control1d.hpp"

/*typedef ddp::solver<ddp::control1d> c1d_ddp;

void ddp::control1dTest()
{
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
	//c1d_ddp::CostGraph & cg = cddp.getCostGraph();

	int s_idx = Ns - 1;
	printf("%lf\n", ddp_.getStateAt(s_idx));
	for (int i = 0; i < (N - 1); ++i) {
		s_idx = ddp_.getNextState(s_idx, policy[i][s_idx]);
		printf("%lf\n", ddp_.getStateAt(s_idx));
	}

}
*/