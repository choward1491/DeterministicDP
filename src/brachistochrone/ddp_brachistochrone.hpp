#pragma once

#include <vector>

namespace ddp {
	class brachistochrone {
	public:

		struct state {
			double y;
		};


		brachistochrone();
		~brachistochrone();

		// methods to be implemented
		void init();
		double finalCost(int state_idx);
		int getNextState(int state_idx, int control_idx);
		double cost(int current_iter, int max_iter, int state_idx, int control_idx);
		int numControls();
		int numStates();

		// extra methods to the problem
		void setStartAndEndPose(double x1, double y1, double x2, double y2);
		void setNumPointsCurve(int num_points);
		void setNumVerticlePositions(int num_vert_pos);
		void setControlRange(double u1, double u2, int Nc_);
		int getStateIdx(double val, int state_dim = 0);
		struct state getStateAtNetIdx(int net_idx);
		double getControlAt(int c_idx);



	private:
		int num_pts, Ns, Nc;
		std::vector<double> y, u;
		double p1[2], p2[2];
		double dx;

		// helper functionality
		
		
	};

}
