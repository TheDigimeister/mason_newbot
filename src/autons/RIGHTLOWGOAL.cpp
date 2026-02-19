#include "drive.hpp"
#include "lemlib/chassis/chassis.hpp"
#include "main.h"
#include "robot.hpp"
#include "utils.hpp"

void rightLowGoal() {
    odom.set_value(false);
    level.set_value(true);
    descore.set_value(true);
    chassis.setPose(positionFromRaycast(right_dist.get() * MM_TO_IN, RIGHT_DIST_OFFSET, WEST), positionFromRaycast(front_dist.get() * MM_TO_IN, FRONT_DIST_OFFSET, SOUTH),180);
    matchload.set_value(true);

    lower.move(127);
    middle.move(127);
    pros::Task upper_roller_hold([] {
        while(roller_dist.get() > 50) {upper.move(50);}
        upper.move(0);
    });

    chassis.moveToPoint(-48, -50, 1200, {.maxSpeed=127});
    chassis.turnToHeading(270, 800, {.minSpeed=20}, false);
    chassis.setPose(positionFromRaycast(front_dist.get()*MM_TO_IN, FRONT_DIST_OFFSET, WEST), positionFromRaycast(left_dist.get()*MM_TO_IN, LEFT_DIST_OFFSET, SOUTH), chassis.getPose().theta);
    
    chassis.moveToPoint(-62, -48, 800, {.forwards=true, .maxSpeed=45}, true);
    pros::delay(800);
    
    // move to goals
    upper.move(0);
    lower.move(127);

    pros::Task align_score2{[=]{
        while(back_dist.get() > 100) { pros::delay(50);}
        upper.move(127);
    }}; 



    quintic::moveToPoint(chassis, -25, -51, 2000, {.forwards=false, .async=false, .settleRange=2});
    matchload.set_value(false);
    upper.move(0);

    chassis.swingToHeading(0, lemlib::DriveSide::RIGHT, 1000, {.minSpeed=20}, false);
    chassis.setPose(positionFromRaycast(left_dist.get()*MM_TO_IN, LEFT_DIST_OFFSET, WEST), positionFromRaycast(back_dist.get()*MM_TO_IN, BACK_DIST_OFFSET, SOUTH), chassis.getPose().theta);
    middle.move(-50);
    chassis.moveToPoint(-24,-24,500, {.maxSpeed=80, .minSpeed=50}, false);
    quintic::moveToPoint(chassis, -16, -11, 2000, {.async=false});
    intake_up.set_value(true);
    chassis.turnToHeading(45, 200, {.minSpeed=20});
    lower.move(-57);
    middle.move(-57);
    upper.move(-57);
    pros::delay(1200);
    lower.move(127);
    middle.move(127);
    upper.move(0);
    level.set_value(true);

    chassis.moveToPose(-36,-32, 90, 3000, {.forwards=false, .minSpeed=50}, false);
    intake_up.set_value(false);
    descore.set_value(false);
    chassis.moveToPoint(-10, -32, 1700, {.maxSpeed=60});
    // chassis.turnToPoint(-24, 23, 2000, {.minSpeed=20});
    // chassis.moveToPoint(-24,23,1200, {.maxSpeed=80});

    // chassis.moveToPoint(-52, 40, 1000, {.maxSpeed=80});
    // pros::delay(200);
    // level.set_value(true);
    // chassis.turnToHeading(270, 700, {.minSpeed=20}, false);
    // level.set_value(true);
    // chassis.setPose(positionFromRaycast(front_dist.get() * MM_TO_IN, FRONT_DIST_OFFSET, WEST), positionFromRaycast(right_dist.get() * MM_TO_IN, RIGHT_DIST_OFFSET, NORTH), chassis.getPose().theta);
    // chassis.moveToPoint(-62, 47.5, 1200, {.forwards=true, .maxSpeed=60, .minSpeed=45}, true);
    // pros::delay(1000);

    // // long goal score
    //     pros::Task align_score3{[=]{
    //     while(back_dist.get() > 100) { pros::delay(50);}
    //     upper.move(127);
    //     lower.move(127);
    // }};
    
    // // chassis.turnToPoint(-25, 48, 500, {.forwards=false, .maxSpeed=80}, true);
    // chassis.moveToPoint(-25, 49, 1300, {.forwards=false, .maxSpeed=80}, false);
    // matchload.set_value(false);
    // lower.move(127);
    // middle.move(127);
    // upper.move(127);

}