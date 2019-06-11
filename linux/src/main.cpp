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

static const char *const HEADER = "\nExoskeleton © 2019 UAlbertaBiomed\n\n";
static const char *const USAGE = "Usage:\n\tdivider <numerator> <denominator>\n\nDescription:\n\tComputes the result of a fractional division,\n\tand reports both the result and the remainder.\n";

int main(int argc, const char *argv[]) {
	Fraction f;

	std::cout << HEADER;

	// ensure the correct number of parameters are used.
	if (argc < 3) {
		std::cout << USAGE;
		return 1;
	}

	f.numerator = atoll(argv[1]);
	f.denominator = atoll(argv[2]);

	Exoskeleton d = Exoskeleton(f);
	try {
		PIDResult r = d.pid();
		std::cout << "Division : " << f.numerator << " / " << f.denominator << " = " << r.division << "\n";
		std::cout << "Remainder: " << f.numerator << " % " << f.denominator << " = " << r.remainder << "\n";
	} catch (DivisionByZero) {
		std::cout << "Can not divide by zero, Homer. Sober up!\n";
	}
	return 0;
}
