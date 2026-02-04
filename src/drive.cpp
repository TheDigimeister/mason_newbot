#include "lemlib/chassis/trackingWheel.hpp"
#include "main.h"

pros::Controller master(pros::E_CONTROLLER_MASTER);

pros::MotorGroup left_mg({-1, -5, -2});    // Creates a motor group with forwards ports 1 & 3 and reversed port 2
pros::MotorGroup right_mg({3, 6, 4});  // Creates a motor group with forwards port 5 and reversed ports 4 & 6

lemlib::Drivetrain drivetrain(&left_mg, // left motor group
                              &right_mg, // right motor group
                              11.5, // 10 inch track width
                              lemlib::Omniwheel::NEW_325, // using new 4" omnis 96.0/94.136
                              450, // drivetrain rpm is 360
                              2 // horizontal drift is 2 (for now)
);

pros::Rotation vert_odom(-12);
// pros::Rotation hor_odom(20);
pros::Imu inertial(11);

lemlib::TrackingWheel vertical_tracking_wheel(&vert_odom, lemlib::Omniwheel::NEW_2 * 96.0/100.982, 1.0);
// lemlib::TrackingWheel hor_tracking_wheel(&hor_odom, lemlib::Omniwheel::NEW_2 * 96.0/101.41, -2.866); // lemlib::Omniwheel::NEW_2 * (117.25/126.25)

lemlib::TrackingWheel left_wheel(&left_mg, lemlib::Omniwheel::NEW_325 * 96/94.578, -11.5/2, 450);
lemlib::TrackingWheel right_wheel(&right_mg, lemlib::Omniwheel::NEW_325 * 96/94.578, 11.5/2, 450);

lemlib::OdomSensors sensors(&left_wheel, // vertical tracking wheel 1, set to null
                            &right_wheel, // vertical tracking wheel 2, set to nullptr as we are using IMEs
                            nullptr, // horizontal tracking wheel 1
                            nullptr, // horizontal tracking wheel 2, set to nullptr as we don't have a second one
                            &inertial // inertial sensor
);

// PERFECT lateral PID controller
lemlib::ControllerSettings lateral_controller(10.583, // proportional gain (kP)
                                              0.0, // integral gain (kI) // 0.5
                                              46.41, // derivative gain (kD)
                                              2, // anti windup
                                              1, // small error range, in inches
                                              300, // small error range timeout, in milliseconds
                                              3, // large error range, in inches
                                              500, // large error range timeout, in milliseconds
                                              80 // maximum acceleration (slew)
);

// PERFECT angular PID controller
lemlib::ControllerSettings angular_controller(2.0, // proportional gain (kP)
                                              0.02, // integral gain (kI) // 0.04
                                              16.7, // derivative gain (kD)
                                              21, // anti windup
                                              1, // small error range, in degrees
                                              300, // small error range timeout, in milliseconds
                                              3, // large error range, in degrees
                                              500, // large error range timeout, in milliseconds
                                              0 // maximum acceleration (slew)
);

// // PERFECT angular PID controller
// lemlib::ControllerSettings angular_controller(3.5, // proportional gain (kP)
//                                               0.02, // integral gain (kI) // 0.04
//                                               30, // derivative gain (kD)
//                                               21, // anti windup
//                                               1, // small error range, in degrees
//                                               300, // small error range timeout, in milliseconds
//                                               3, // large error range, in degrees
//                                               700, // large error range timeout, in milliseconds
//                                               100 // maximum acceleration (slew)
// );


lemlib::Chassis chassis(drivetrain, // drivetrain settings
                        lateral_controller, // lateral PID settings
                        angular_controller, // angular PID settings
                        sensors // odometry sensors
);