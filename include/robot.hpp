#pragma once

#include "pros/distance.hpp"
#include "pros/motors.hpp"
#include "pros/adi.hpp"
#include "pros/optical.hpp"

extern pros::Distance front_dist;
extern pros::Distance back_dist;
extern pros::Distance left_dist;
extern pros::Distance right_dist;
extern pros::Distance frontleft_dist;
extern pros::Distance frontright_dist;
extern pros::Distance backleft_dist;
extern pros::Distance backright_dist;
extern pros::Distance roller_dist;

extern pros::Distance long_goal_check;

extern pros::Motor lower;
extern pros::Motor upper;

extern pros::ADIDigitalOut level;
extern pros::ADIDigitalOut matchload;
extern pros::ADIDigitalOut descore;
extern pros::ADIDigitalOut odom;
extern pros::ADIDigitalOut mid;
extern pros::ADIDigitalOut intake_up;
extern pros::ADIDigitalOut parking;

extern bool odom_state;

extern pros::Optical roller_optical;