//
// Created by gorshkovna on 05/10/19.
//

#ifndef PID_HELPERS_H
#define PID_HELPERS_H

double normalize(double value, double min, double max) {
    return (value - min) / (max - min);
}

#endif //PID_HELPERS_H
