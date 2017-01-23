

#ifndef ddp_solver_impl
#define ddp_solver_impl

#include "ddp_solver.hpp"

namespace ddp {

	template<typename ddp_def>
	solver<ddp_def>::solver():ddp_(),num_cost_iters(1){

	}

	template<typename ddp_def>
	ddp_def & solver<ddp_def>::ddp() {
		return ddp_;
	}

	template<typename ddp_def>
	const ddp_def & solver<ddp_def>::ddp() const {
		return ddp_;
	}

	template<typename ddp_def>
	void solver<ddp_def>::setNumIterations(int N) {
		num_cost_iters = N;
	}

	template<typename ddp_def>
	void solver<ddp_def>::solve() {
		const int N = num_cost_iters;
		const int Ns = ddp_.numStates();
		const int Nc = ddp_.numControls();

		// do initialization
		ddp_.init();
		init_policy();
		init_cost();

		// do dynamic programming computation and compute optimal policy
		{
			int xn_idx = 0;
			double local_cost = 0.0;
			double best_cost = 1e100;
			//std::vector<double> *v1 = &V, *v2 = &Vn, *tmp = &V;

			// compute final condition
			for (int i = 0; i < Ns; ++i) {
				V[N-1][i] = ddp_.finalCost(i);
			}

			// compute V and control for i = 1 to N-1
			for (int i = (N - 1) - 1; i >= 0; --i) {
				for (int k = 0; k < Ns; ++k) { // loop through states
					for (int j = 0; j < Nc; ++j) { // loop through controls

						if (k == Ns - 1 && j == 0) {
							int i = 0;
						}
						// get index of next state using state k and control j
						xn_idx = ddp_.getNextState(k,j);
						double y1_ = ddp_.getYStateAtNetIdx(xn_idx);
						double s1_ = ddp_.getSStateAtNetIdx(xn_idx);


						// compute local cost in time due to state k and control j
						local_cost = ddp_.cost(i, N, k, j) 
									+ V[i + 1][xn_idx];/*(*v2)[xn_idx];*/

						// update the best cost if needed
						if (local_cost < best_cost) {
							best_cost = local_cost;
							policy[i][k] = j;
						}
					}// end control loop

					// set the cost-to-go to found best cost
					//(*v1)[k] = best_cost;
					double y_ = ddp_.getYStateAtNetIdx(k);
					double s_ = ddp_.getSStateAtNetIdx(k);
					V[i][k] = best_cost;


					// reset best cost for use in next state
					best_cost = 1e100;
				}// end state loop

				// update cost references
				/*
				tmp = v2;
				v2 = v1;
				v1 = tmp;
				*/

			}// end time loop
		}

		// clean up
	}

	template<typename ddp_def>
	typename solver<ddp_def>::Policy & solver<ddp_def>::getPolicy(){
		return policy;
	}

	template<typename ddp_def>
	typename solver<ddp_def>::CostGraph & solver<ddp_def>::getCostGraph() {
		return V;
	}

	template<typename ddp_def>
	void solver<ddp_def>::init_policy() {
		const int N = num_cost_iters;
		const int Nm = N - 1;
		const int Ns = ddp_.numStates();

		if (policy.size() != Nm) { policy.resize(Nm); }
		for (int i = 0; i < Nm; ++i) {
			if (policy[i].size() != Ns) { policy[i].resize(Ns); }
		}
	}

	template<typename ddp_def>
	void solver<ddp_def>::init_cost() {
		const int N = num_cost_iters;
		const int Ns = ddp_.numStates();

		if (V.size() != N) { V.resize(N); }
		for (int i = 0; i < N; ++i) {
			if (V[i].size() != Ns) { V[i].resize(Ns); }
		}

	}


	/*
	ddp_def ddp_;
	std::vector<double> Vn;
	std::vector<int> policy;
	*/


}

#endif