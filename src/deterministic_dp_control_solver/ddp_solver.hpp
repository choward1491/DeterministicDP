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

	template<typename ddp_def,			// deterministic dynamic programming model class
		typename ctype = unsigned char, // type of variable to store index of control in set of possible control values
		typename vtype = float			// type of variable used to represent value function 
	>
	class solver {
	public:
		// specify solver typedefs
		typedef std::vector<std::vector<ctype>> Policy;
		typedef ddp_def DDP;

		// constructor
		solver();

		// method to return reference to internal DDP model
		ddp_def & ddp();
		const ddp_def & ddp() const;

		// set the number of iterations/time steps to make the
		// solver work over
		void setNumIterations(int N);

		// find an optimal policy using dynamic programming
		void solve();

		// get reference to resulting policy
		Policy & getPolicy();

	private:
		int num_cost_iters;			// number of iterations/timesteps to find optimal policies over
		ddp_def ddp_;				// instance of ddp model
		std::vector<vtype> V1, V2;	// light storage of value function for each timestep/iteration

		// optimal policy storage
		Policy policy;

		// helper methods
		void init_cost();
		void init_policy();
	};


}

#include "dpp_solver_impl.hpp"

#endif 