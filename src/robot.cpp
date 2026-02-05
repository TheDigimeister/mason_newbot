#include "main.h"
#include "pros/adi.hpp"
#include "pros/ai_vision.hpp"
#include "pros/optical.hpp"

pros::Distance front_dist(20);
pros::Distance back_dist(19);
pros::Distance left_dist(17);
pros::Distance right_dist(14);
pros::Distance frontleft_dist(16);
pros::Distance frontright_dist(15);
pros::Distance backleft_dist(18);
pros::Distance backright_dist(13);
pros::Distance roller_dist(21);


pros::Motor lower(7);
pros::Motor middle(-8);
pros::Motor upper(10);

pros::ADIDigitalOut level('B');
pros::ADIDigitalOut matchload('C');
pros::ADIDigitalOut descore('D');
pros::ADIDigitalOut mid('E');
pros::ADIDigitalOut intake_up('A');
pros::ADIDigitalOut parking('F');
pros::ADIDigitalOut odom('G');

pros::Optical roller_optical(18);
// pros::AIVision vision(21);