#pragma once
#include <vector>

namespace ddp {
	class pendulum {
	public:

		struct state {
			double theta, thetadot;
		};

		pendulum();
		~pendulum();

		// methods that must be implemented
		void init();
		double finalCost(int state_idx);
		int getNextState(int state_idx, int control_idx);
		double cost(int current_iter, int max_iter, int state_idx, int control_idx);
		int numControls();
		int numStates();

		// other methods
		void setAngleRange(double t1, double t2, int num_ang);
		void setAngVelRange(double av1, double av2, int num_angvel);
		void setControlRange(double c1, double c2, int num_controls);
		void setDt(double dt_);
		void setCostFuncVars(double Q, double Qf, double R);

		int getStateIdx(double val, int state_dim);
		int getNetIdx(int idx1, int idx2) const;

		struct state getStateAtNetIdx(int net_idx);
		double getControlAt(int c_idx);


	private:
		int num_pts, N1, N2, Nn, Nc;
		std::vector<double> t, td, control;
		double dt, Q, Qf, R;

		// helper functionality
		void getIndicies(int net_idx, int & idx1, int & idx2) const;
		void setRange(double start, double end, int N, std::vector<double> & v);
	};

}
