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

#ifndef EXOSKELETON_H
#define EXOSKELETON_H

static const char *const DIVISION_BY_ZERO_MESSAGE = "Division by zero is illegal";

#include <iostream>
#include <stdexcept>

using namespace std;

class DivisionByZero : public exception {
public:
	virtual const char *what() const throw() {
		return DIVISION_BY_ZERO_MESSAGE;
	}
};

struct Fraction {
	long long numerator;
	long long denominator;
};

struct PIDResult {
	long long division;
	long long remainder;

	friend bool operator==(const PIDResult &lhs, const PIDResult &rhs) {
		return lhs.division == rhs.division ? lhs.remainder < rhs.remainder : lhs.division < rhs.division;
	}
};

class Exoskeleton {
public:
	explicit Exoskeleton(Fraction fraction) {
		this->fraction = fraction;
	}

	~Exoskeleton() {
	};

	PIDResult pid();

protected:
	Fraction fraction;
	PIDResult result;
};

#endif // EXOSKELETON_H
