/*
 * Copyright (c) 2019 UAlbertaBiomed Exoskeleton. All rights reserved.
 *
 * This software may be modified and distributed
 * under the terms of the BSD 3-Clause license.
 *
 * Refer to the LICENSE file for details.
 *
 * Author: Laura Petrich
 * Created on: June 6, 2019
 *
 */

#include <exoskeleton.h>

using namespace std;

static const char* const HEADER = "\nExoskeleton © 2019 UAlbertaBiomed\n\n";

int main(int argc, const char* argv[]) {
	Gains g;

	cout << HEADER;
	g.Kp = 10.0;
	g.Ki = 0.0;
	g.Kd = 0.0;

	Exoskeleton exo = Exoskeleton(g);
	try {
		cout << g.Kp << endl;
		exo.init();
		exo.loop();
	}
	catch (exception& e) {
		cout << e.what() << endl;
	}
	return 0;
}