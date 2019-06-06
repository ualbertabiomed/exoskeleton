//
// Created by Konstantin Gredeskoul on 5/14/17.
//

#ifndef CMAKE_EXOSKELETON_H
#define CMAKE_EXOSKELETON_H

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

struct ExoskeletonResult {
  long long division;
  long long remainder;

  friend bool operator==(const ExoskeletonResult &lhs, const ExoskeletonResult &rhs) {
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

  ExoskeletonResult pid();

protected:
  Fraction       fraction;
  ExoskeletonResult result;
};

#endif //CMAKE_EXOSKELETON_H
