#include "ddp_brachistochrone.hpp"

namespace ddp {

	brachistochrone::brachistochrone()
	{
		num_pts = 2;
		p1[0] = 0.0; p1[1] = 1.0;
		p2[0] = 1.0; p2[1] = 0.0;
	}


	brachistochrone::~brachistochrone()
	{
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
		Ns2 = Ns*Ns;
	}

	int brachistochrone::getYStateIdx(double yv) {
		double sh = 0.0, invN = 1.0 / static_cast<double>(Ns - 1);
		double yb = p1[1] - 0.2*(p1[1] - p2[1]);
		double dy = (p1[1] - yb)*invN;
		int yi = (yv-yb)/dy;
		if (yi >= Ns) { yi = Ns-1; }
		if (yi < 0) { yi = 0; }
		return yi;
	}

	int brachistochrone::getSStateIdx(double sv) {
		return getYStateIdx(sv);
	}

	int brachistochrone::computeNetIdx(int idx_y, int idx_s) const {
		return idx_y + Ns*idx_s;
	}

	double brachistochrone::getYStateAtNetIdx(int net_idx) {
		int iy = 0, is = 0;
		getIndicies(net_idx, iy, is);
		return y[iy];
	}

	double brachistochrone::getSStateAtNetIdx(int net_idx) {
		int iy = 0, is = 0;
		getIndicies(net_idx, iy, is);
		return s[is];
	}

	void brachistochrone::getIndicies(int net_idx, int & idx_y, int & idx_s) const {
		idx_s = net_idx / Ns;
		idx_y = net_idx % Ns; 
	}

	void brachistochrone::init() {
		if (y.size() != Ns) { y.resize(Ns); }
		if (s.size() != Ns) { s.resize(Ns); }

		double sh = 0.0, invN = 1.0/static_cast<double>(Ns - 1);
		double yb = p2[1] - 0*0.2*(p1[1] - p2[1]);
		for (int i = 0; i < Ns; ++i) {
			sh = static_cast<double>(i)*invN;
			y[i] = yb*(1 - sh) + p1[1] * sh;
			s[i] = y[i];
		}

	}
	double brachistochrone::finalCost(int state_idx) {
		int iy = 0, is = 0;
		getIndicies(state_idx, iy, is);
		double y_ = y[iy], s_ = s[is];
		double dy = (y_ - p2[1]);
		double yp = (y_ - s_) / dx;
		return 10 * dy*dy + dx*sqrt((1.0 + yp*yp) / (y_ + 1));
	}
	int brachistochrone::getNextState(int state_idx, int control_idx) {
		int iy = 0, is = 0;
		getIndicies(state_idx, iy, is);
		return computeNetIdx(control_idx,iy);
	}
	double brachistochrone::cost(int current_iter, int max_iter, int state_idx, int control_idx) {
		int iy = 0, is = 0;
		getIndicies(state_idx, iy, is);
		double y_ = y[iy], s_ = s[is];
		double yp = (y_ - s_) / dx;
		return dx*sqrt((1.0 + yp*yp )/(y_ + 1e-6));
	}
	int brachistochrone::numControls() {
		return Ns;
	}
	int brachistochrone::numStates() {
		return Ns2;
	}

}