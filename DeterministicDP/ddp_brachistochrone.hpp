#pragma once

#include <vector>

namespace ddp {
	class brachistochrone{
	public:
		brachistochrone();
		~brachistochrone();

		void init();
		double finalCost(int state_idx);
		int getNextState(int state_idx, int control_idx);
		double cost(int current_iter, int max_iter, int state_idx, int control_idx);
		int numControls();
		int numStates();

		void setStartAndEndPose(double x1, double y1, double x2, double y2);
		void setNumPointsCurve(int num_points);
		void setNumVerticlePositions(int num_vert_pos);

		int getYStateIdx(double yv);
		int getSStateIdx(double sv);
		int computeNetIdx(int idx_y, int idx_s) const;

		double getYStateAtNetIdx(int net_idx);
		double getSStateAtNetIdx(int net_idx);


	private:
		int num_pts, Ns, Ns2, Nc;
		std::vector<double> y, s;
		double p1[2], p2[2];
		double dx;

		// helper functionality
		void getIndicies(int net_idx, int & idx_y, int & idx_s) const;
		
	};

}
