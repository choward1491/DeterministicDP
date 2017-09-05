#include "ddp_abstract_dyn.hpp"

void ddp::dynamics::getIndices(uint32_t net_idx) const
{
	uint32_t t = static_cast<uint32_t>(net_idx);
	uint32_t end = num_vars - 1;
	uint32_t N = 0;
	for (uint32_t i = 0; i < end; ++i) {
		N = states[i].size();
		indices[i] = t % N;
		t /= N;
	}
	indices[end] = t;
}

void ddp::dynamics::updateNumDiscretizedStates()
{
	num_discrete_states = 1;
	for (uint32_t i = 0; i < num_vars; ++i) {
		num_discrete_states *= states[i].size();
	}
}

void ddp::dynamics::setRange(double start, double end, uint32_t N, std::vector<double>& vec )
{
	if (vec.size() != N) { vec.resize(N); }
	double del = (end - start) / static_cast<double>(N - 1);
	for (uint32_t i = 0; i < N; ++i) {
		vec[i] = start + i*del;
	}
}

ddp::dynamics::dynamics()
{
}

uint32_t ddp::dynamics::numControls()
{
	return control.size();
}

uint32_t ddp::dynamics::numStates()
{
	return num_discrete_states;
}

void ddp::dynamics::setNumVars(uint32_t numvars)
{
	num_vars = numvars;
	states.resize(num_vars);
	state.resize(num_vars);
	indices.resize(num_vars);
}

void ddp::dynamics::setState(uint32_t idx, double start, double end, uint32_t num_vals)
{
	setRange(start, end, num_vals, states[idx]);
	updateNumDiscretizedStates();
}

void ddp::dynamics::setControlRange(double c1, double c2, uint32_t num_controls)
{
	setRange(c1, c2, num_controls, control);
}

void ddp::dynamics::setControlSet(const std::vector<double>& control_set)
{
	control = control_set;
}


uint32_t ddp::dynamics::getStateIdx(double val, uint32_t state_dim) const
{
	const std::vector<double> * sv = 0;
	sv = &states[state_dim];
	const double invN = 1.0 / static_cast<double>(sv->size() - 1);
	const double lb = (*sv)[0], del = (*sv)[1] - (*sv)[0];
	double idx_f = round((val - lb) / del);
	uint32_t idx = std::fmax(0.0,idx_f);
	uint32_t size = sv->size();
	if (idx >= size) { idx = size - 1; }
	return idx;
}

uint32_t ddp::dynamics::getNetIdx( const std::vector<uint32_t>& indices_) const
{
	uint32_t net_idx = 0;
	uint32_t end = num_vars - 1;
	for (uint32_t i = end; i != 0; --i) {
		net_idx += states[i - 1].size()*(indices_[i] + net_idx);
	}
	net_idx += indices_[0];
	return net_idx;
}

void ddp::dynamics::getStateAtNetIdx(uint32_t net_idx, std::vector<double>& out_state) const
{
	if (out_state.size() != state.size()) { out_state.resize(state.size()); }
	getIndices(net_idx);
	for (uint32_t i = 0; i < state.size(); ++i) { out_state[i] = states[i][indices[i]]; }
}

double ddp::dynamics::getControlAt(uint32_t c_idx) const
{
	return control[c_idx];
}

std::vector<double>& ddp::dynamics::getState(uint32_t net_idx)
{
	getStateAtNetIdx(net_idx, state);
	return state;
}

std::vector<uint32_t>& ddp::dynamics::getIndices()
{
	return indices;
}

