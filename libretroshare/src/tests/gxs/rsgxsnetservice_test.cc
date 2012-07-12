/*
 * rsgxsnetservice_test.cc
 *
 *  Created on: 11 Jul 2012
 *      Author: crispy
 */

#include "util/utest.h"
#include "nxstesthub.h"
#include "nxstestscenario.h"

INITTEST();


int main()
{

	// first setup
	NxsMessageTest msgTest;
	NxsTestHub hub(&msgTest);

	// now get things started
	createThread(hub);

	// put this thread to sleep for 10 secs
	usleep(10000);

	hub.join();
	CHECK(hub.testsPassed());

    FINALREPORT("RsGxsNetService Tests");

    return TESTRESULT();
}
