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

#include "exoskeleton.h"

/**************************************************************************
 *
 * Calculate PID output
 *
 **************************************************************************/
PIDResult Exoskeleton::pid() {
	if (fraction.denominator == 0L) throw DivisionByZero();

	PIDResult result = PIDResult{
		fraction.numerator / fraction.denominator, 
		fraction.numerator % fraction.denominator
	};

	return result;
}
