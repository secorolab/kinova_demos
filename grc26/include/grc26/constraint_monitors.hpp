#ifndef CONSTRAINT_MONITORS_HPP
#define CONSTRAINTS_MONITORS_HPP

double evaluate_equality_constraint(double quantity, double reference);

double evaluate_less_than_constraint(double quantity, double threshold);

double evaluate_greater_than_constraint(double quantity, double threshold);

double evaluate_bilateral_constraint(double quantity, double lower, double upper);

#endif // CONSTRAINT_MONITORS_HPP