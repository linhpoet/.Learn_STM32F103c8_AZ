#include "stm32f10x.h"
#include "filter.h"

#define Pi	3.14159265
volatile float _random, _sin, random_LPF_HPF, random_HPF, random_kalman, signal_LPF, signal_HPF, signal_kalman;
void filter();

int  main()
{
	while(1)
	{
		filter();
	}
}

void filter()
{
	static float x = 0;
	_sin = sin(x)*1000;
	x+= 2*Pi/1000;
	_random = (float)rand()/1000;
	
	random_LPF_HPF = _random + _sin;
	signal_LPF = LPF(random_LPF_HPF, 1, 1000);
	signal_HPF = HPF(random_LPF_HPF, 10, 1000);
	
	random_kalman = (float)rand()/1000000;
	signal_kalman = kalman_single(random_kalman, 500, 10);
}