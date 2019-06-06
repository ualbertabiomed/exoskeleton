//
// Created by Konstantin Gredeskoul on 5/14/17.
//

#include "exoskeleton.h"

ExoskeletonResult Exoskeleton::pid() {
  if (fraction.denominator == 0L) throw DivisionByZero();

  ExoskeletonResult result = ExoskeletonResult{
    fraction.numerator / fraction.denominator, 
    fraction.numerator % fraction.denominator
  };

  return result;
}
