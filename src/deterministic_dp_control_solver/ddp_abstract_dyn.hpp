

#ifndef _ddp_abstract_dyn_
#define _ddp_abstract_dyn_

#include <vector>
#include <stdint.h>

namespace ddp {

	class dynamics {
	public:

		dynamics();
		virtual ~dynamics() {}

		// methods that must be implemented
		virtual void		init() {}
		virtual double		finalCost(uint32_t state_idx) { return 0.0; }
		virtual uint32_t	getNextState(uint32_t state_idx, uint32_t control_idx) = 0;
		virtual double		cost(uint32_t current_iter, uint32_t max_iter, uint32_t state_idx, uint32_t control_idx) { return 0.0; }

		// other getter/setter methods
		void		setNumVars(uint32_t numvars);
		void		setState(uint32_t idx, double start, double end, uint32_t num_vals);
		void		setControlRange(double c1, double c2, uint32_t num_controls);
		void		setControlSet(const std::vector<double> & control_set);

		uint32_t	numControls();
		uint32_t	numStates();
		uint32_t	getStateIdx(double val, uint32_t state_dim) const;
		uint32_t	getNetIdx( const std::vector<uint32_t> & indices_) const;
		void		getStateAtNetIdx(uint32_t net_idx, std::vector<double> & state) const;
		double		getControlAt(uint32_t c_idx) const;

	protected:
		std::vector<double> & getState(uint32_t net_idx);
		std::vector<uint32_t> & getIndices();

	private:

		uint32_t num_discrete_states, num_vars;
		std::vector<std::vector<double>>	states;
		std::vector<double>					control;
		std::vector<double>					state;
		mutable std::vector<uint32_t>		indices;


		// helper functionality
		void getIndices(uint32_t net_idx) const;
		void updateNumDiscretizedStates();
		static void setRange(double start, double end, uint32_t N, std::vector<double> & v);

	};

}


#endif