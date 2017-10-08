#include "Basic.h"

double myRand() {
	return double(double(rand()) / RAND_MAX);
}

//sort(openSet.begin(), openSet.end(),myfunc);