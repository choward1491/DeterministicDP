

#ifndef ddp_solver_impl
#define ddp_solver_impl

#include "ddp_solver.hpp"

namespace ddp {

	template<typename ddp_def, typename ctype, typename vtype>
	solver<ddp_def,ctype,vtype>::solver():ddp_(),num_cost_iters(1){

	}

	template<typename ddp_def, typename ctype, typename vtype>
	ddp_def & solver<ddp_def,ctype,vtype>::ddp() {
		return ddp_;
	}

	template<typename ddp_def, typename ctype, typename vtype>
	const ddp_def & solver<ddp_def,ctype,vtype>::ddp() const {
		return ddp_;
	}

	template<typename ddp_def, typename ctype, typename vtype>
	void solver<ddp_def,ctype,vtype>::setNumIterations(int N) {
		num_cost_iters = N;
	}

	template<typename ddp_def, typename ctype, typename vtype>
	void solver<ddp_def,ctype,vtype>::solve() {
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
			std::vector<vtype> *v1 = &V1, *v2 = &V2, *tmp = &V1;

			// compute final condition
			for (int i = 0; i < Ns; ++i) {
				(*v2)[i] = ddp_.finalCost(i);
			}

			// compute V and control for i = 1 to N-1
			for (int i = (N - 1) - 1; i >= 0; --i) {
				for (int k = 0; k < Ns; ++k) { // loop through states

					(*v1)[k] = 1e100; // reset cost for use in next state
					for (int j = 0; j < Nc; ++j) { // loop through controls

						// compute local cost in time due to state k and control j
						xn_idx = ddp_.getNextState(k, j);
						double Vn = (*v2)[xn_idx];
						double gi = ddp_.cost(i, N, k, j);
						local_cost = gi + Vn; 

						// update the best cost if needed
						if (local_cost < (*v1)[k]) {
							(*v1)[k] = local_cost;
							policy[i][k] = j;
						}
					}// end control loop

				}// end state loop

				// update cost references
				tmp = v2; v2 = v1; v1 = tmp;
				printf("Step (%i) Complete\n",i);
			}// end time loop
		}

		// clean up
	}

	template<typename ddp_def, typename ctype, typename vtype>
	typename solver<ddp_def,ctype,vtype>::Policy & solver<ddp_def,ctype,vtype>::getPolicy(){
		return policy;
	}

	template<typename ddp_def, typename ctype, typename vtype>
	void solver<ddp_def,ctype,vtype>::init_policy() {
		const int N = num_cost_iters;
		const int Nm = N - 1;
		const int Ns = ddp_.numStates();

		if (policy.size() != Nm) { policy.resize(Nm); }
		for (int i = 0; i < Nm; ++i) {
			if (policy[i].size() != Ns) { policy[i].resize(Ns); }
		}
	}

	template<typename ddp_def, typename ctype, typename vtype>
	void solver<ddp_def,ctype,vtype>::init_cost() {
		const int N = num_cost_iters;
		const int Ns = ddp_.numStates();
		if (V1.size() != Ns) { V1.resize(Ns); }
		if (V2.size() != Ns) { V2.resize(Ns); }
	}


}

#endif