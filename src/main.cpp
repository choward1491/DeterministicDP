
//#include "src/pendulum/ddp_pendulum_test.hpp"
#include "src/alumni_challenge/ddp_glider_training.hpp"
#include "src/brachistochrone/ddp_brachistochrone_test.hpp"

int main(int argc, char** argv) {

	//ddp::trainGliderController("control_indices12.bin");
	ddp::brachistochroneTest("test.txt");

	return 0;
}