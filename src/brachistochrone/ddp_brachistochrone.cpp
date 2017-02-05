#include "ddp_brachistochrone.hpp"
#include <math.h>
namespace ddp {

	brachistochrone::brachistochrone(){
		num_pts = 2;
		p1[0] = 0.0; p1[1] = 1.0;
		p2[0] = 1.0; p2[1] = 0.0;
	}


	brachistochrone::~brachistochrone(){
	}

	void brachistochrone::setStartAndEndPose(double x1, double y1, double x2, double y2) {
		p1[0] = x1; p1[1] = y1;
		p2[0] = x2; p2[1] = y2;
	}
	void brachistochrone::setNumPointsCurve(int num_points) {
		num_pts = num_points;
		dx = (p2[0] - p1[0]) / static_cast<double>(num_points - 1);
	}

	void brachistochrone::setNumVerticlePositions(int num_vert_pos){
		Ns = Nc = num_vert_pos;
	}

	void brachistochrone::setControlRange(double u1, double u2, int Nc_){
		Nc = Nc_;
		const double du = (u2 - u1) / (static_cast<double>(Nc_-1));
		if (u.size() != Nc) { u.resize(Nc); }
		for (int i = 0; i < Nc; ++i) {
			u[i] = u1 + static_cast<double>(i)*du;
		}
	}

	brachistochrone::state brachistochrone::getStateAtNetIdx(int net_idx){
		return{ y[net_idx] };
	}

	double brachistochrone::getControlAt(int c_idx){
		return u[c_idx];
	}


	int brachistochrone::getStateIdx(double val, int state_dim) {
		std::vector<double> * sv = &y;
		const double invN = 1.0 / static_cast<double>(sv->size() - 1);
		const double lb = (*sv)[0], del = (*sv)[1] - (*sv)[0];
		int idx = round((val - lb) / del);
		if (idx >= static_cast<int>(sv->size()) ) { 
			idx = sv->size() - 1; 
		}
		if (idx < 0) { idx = 0; }
		return idx;
	}

	void brachistochrone::init() {
		if (y.size() != Ns) { y.resize(Ns); }

		double sh = 0.0, invN = 1.0/static_cast<double>(Ns - 1);
		double yb = p2[1];
		for (int i = 0; i < Ns; ++i) {
			sh = static_cast<double>(i)*invN;
			y[i] = yb*(1 - sh) + p1[1] * sh;
		}

	}
	double brachistochrone::finalCost(int state_idx) {
		const double Qf = 1.0e-1;
		double y_ = y[state_idx];
		double dy = (y_ - p2[1]);
		return Qf*dy*dy;
	}
	int brachistochrone::getNextState(int state_idx, int control_idx) {
		return getStateIdx(y[state_idx] + dx*u[control_idx]);
	}
	double brachistochrone::cost(int current_iter, int max_iter, int state_idx, int control_idx) {
		double u_ = u[control_idx];
		double y_ = y[state_idx];
		double gamma = 2.0*9.81;
		return dx*(sqrt((1.0 + u_*u_ )/( gamma*(p1[1] - y_) + 1e-6)) + 0.0*u_*u_);
	}
	int brachistochrone::numControls() {
		return Nc;
	}
	int brachistochrone::numStates() {
		return Ns;
	}

}