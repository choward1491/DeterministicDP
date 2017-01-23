#ifndef ddp_solver
#define ddp_solver

#include <vector>

namespace ddp {

	/*
	ddp_def class info:
	Must have the following methods implemented:

	ddp_def(); // null constructor
	void init();
	double finalCost(int state_idx);
	int getNextState(int state_idx, int control_idx);
	double cost(int current_iter, int max_iter, int state_idx, int control_idx);
	int numControls();
	int numStates();
	
	*/

	template<typename ddp_def>
	class solver {
	public:

		typedef std::vector<std::vector<int>> Policy;
		typedef std::vector<std::vector<double>> CostGraph;
		typedef ddp_def DDP;
		solver();
		ddp_def & ddp();
		const ddp_def & ddp() const;
		void setNumIterations(int N);
		void solve();
		Policy & getPolicy();
		CostGraph & getCostGraph();

	private:
		int num_cost_iters;
		ddp_def ddp_;
		CostGraph V;

		Policy policy;

		// helper methods
		void init_cost();
		void init_policy();
	};


}

#include "dpp_solver_impl.hpp"

#endif 