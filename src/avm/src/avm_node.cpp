/***********************************************
#
# Author: Sun Qinxuan
#
# Email: sunqinxuan@outlook.com
#
# Last modified:	2021-09-29 08:40
#
# Filename:		avm_node.cpp
#
# Description: 
#
************************************************/
#include "avm/avm.h"

using namespace std;
using namespace RESMAL;

int main(int argc, char** argv)
{
	ros::init(argc,argv,"avm",ros::init_options::AnonymousName);
	if(!ros::ok()) return 0;

	AVM avm;
	avm.run();

	ros::shutdown();
	return 0;
}


