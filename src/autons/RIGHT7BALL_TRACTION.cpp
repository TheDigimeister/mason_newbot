#include "lemlib/chassis/chassis.hpp"
#include "main.h"
#include "pros/rtos.hpp"
#include "robot.hpp"
#include "utils.hpp"

void right7BallTraction() {
    odom.set_value(false);
    level.set_value(true);
    descore.set_value(true);
    chassis.setPose(positionFromRaycast(back_dist.get() * MM_TO_IN, BACK_DIST_OFFSET, WEST), positionFromRaycast(right_dist.get() * MM_TO_IN, RIGHT_DIST_OFFSET, SOUTH),90);


    // 7-ball low goal side (31)

    lower.move(127);
    middle.move(127);
    pros::Task upper_roller_hold([] {
        while(roller_dist.get() > 50) {upper.move(50);}
        upper.move(0);
    });

    // pick up trio

    chassis.moveToPoint(-24, -24, 2000, {.maxSpeed = 127, .minSpeed=127, .earlyExitRange=8});
    pros::delay(500);
    matchload.set_value(true);
    chassis.turnToPoint(-50, -48,700, {.maxSpeed=127, .minSpeed=80}, false);
    matchload.set_value(true);

    // back up
    chassis.moveToPoint(-50, -48, 1500, {.maxSpeed=127});
    chassis.turnToHeading(270, 700, {}, false);
    chassis.setPose(positionFromRaycast(front_dist.get()*MM_TO_IN, FRONT_DIST_OFFSET, WEST), positionFromRaycast(left_dist.get()*MM_TO_IN, LEFT_DIST_OFFSET, SOUTH), chassis.getPose().theta);
    
    chassis.moveToPoint(-62, -48, 400, {.forwards=true, .maxSpeed=45, .minSpeed = 45});
    pros::delay(400);
    chassis.turnToHeading(270, 400);
    pros::delay(400);
    chassis.setPose(chassis.getPose().x, positionFromRaycast(left_dist.get()*MM_TO_IN, LEFT_DIST_OFFSET, SOUTH), chassis.getPose().theta);

    // move to goals
    upper.move(0);
    middle.move(127);
    lower.move(127);

    pros::Task align_score2{[=]{
        while(back_dist.get() > 100) { pros::delay(50);}
        upper.move(127);
    }}; 

    chassis.moveToPoint(-24, -49, 300, {.forwards=false, .minSpeed=127});
    chassis.moveToPoint(-24, -49, 2500, {.forwards=false, .maxSpeed=67});

    pros::delay(2500);

    // descore

    chassis.swingToHeading(330, lemlib::DriveSide::RIGHT, 1000);
    chassis.turnToHeading(90, 700, {.maxSpeed=100});
    descore.set_value(false);
    chassis.moveToPoint(-10, -38, 1000, {});
    matchload.set_value(false);

    odom.set_value(false);
    matchload.set_value(false);

    lower.move(0);
    middle.move(0);
    upper.move(0);




}