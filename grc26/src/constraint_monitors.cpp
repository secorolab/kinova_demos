#include "grc26/constraint_monitors.hpp"

double evaluate_equality_constraint(double quantity, double reference) {
    return reference - quantity;
}

double evaluate_less_than_constraint(double quantity, double threshold) {
    return (quantity <= threshold) ? 0.0 : quantity - threshold;
}

double evaluate_greater_than_constraint(double quantity, double threshold) {
    return (quantity >= threshold) ? 0.0 : threshold - quantity;
}

double evaluate_bilateral_constraint(double quantity, double lower, double upper) {
    if (quantity < lower)
        return lower - quantity;
    else if (quantity > upper)
        return quantity - upper;
    else
        return 0.0;
}