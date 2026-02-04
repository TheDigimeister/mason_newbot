#include "drive.hpp"
#include "lemlib/chassis/chassis.hpp"
#include "main.h"
#include "pros/rtos.hpp"
#include "robot.hpp"
#include "utils.hpp"

void superMiddleGoal(){

    // Setup
    chassis.setPose(positionFromRaycast(back_dist.get() * MM_TO_IN, BACK_DIST_OFFSET, WEST), positionFromRaycast(left_dist.get() * MM_TO_IN, LEFT_DIST_OFFSET, NORTH),90);
    matchload.set_value(false);
    intake_up.set_value(false);
    lower.move(127);
    middle.move(127);
    upper.move(0);


    // pick up trio
    chassis.moveToPoint(-24, 24, 250, {.forwards=true, .maxSpeed=127, .minSpeed = 80, .earlyExitRange = 36});
    chassis.moveToPoint(-24, 24, 2000, {.forwards=true, .maxSpeed=80}, true);
    pros::delay(500);
    matchload.set_value(true);
    // // pick up long goal balls
    
    // middle goal score
    chassis.turnToHeading(330, 1000, {.maxSpeed=80});
    level.set_value(false);
    chassis.moveToPoint(-12, 8, 800, {.forwards=false, .maxSpeed=70}, false);
    lower.move(56);
    middle.move(56);
    upper.move(56);
    pros::delay(1000);
    lower.move(127);
    middle.move(127);
    upper.move(0);

    
    // matchload
    chassis.moveToPoint(-47.5, 44, 2000, {.forwards=true, .maxSpeed=127, .minSpeed=50}, true);
    
    matchload.set_value(true);
    chassis.turnToHeading(270, 700, {.maxSpeed=127}, false);
    level.set_value(true);
    chassis.setPose(positionFromRaycast(front_dist.get() * MM_TO_IN, FRONT_DIST_OFFSET, WEST), positionFromRaycast(right_dist.get() * MM_TO_IN, RIGHT_DIST_OFFSET, NORTH), chassis.getPose().theta);
    chassis.moveToPoint(-62, 49.3, 800, {.forwards=true, .maxSpeed=60, .minSpeed=45}, true);
    pros::delay(800);
    
    // long goal score
        pros::Task align_score2{[=]{
        while(back_dist.get() > 100) { pros::delay(50);}
        upper.move(127);
        middle.move(127);
        lower.move(127);
    }};
    
    chassis.moveToPoint(-25, 48, 300, {.forwards=false, .maxSpeed=127}, true);
    chassis.moveToPoint(-25, 48, 1300, {.forwards=false, .maxSpeed=80}, true);
    pros::delay(1300);
    matchload.set_value(false);


    // descore
    chassis.turnToHeading(270, 500, {}, false);
    chassis.setPose(chassis.getPose().x, positionFromRaycast(right_dist.get()*MM_TO_IN, RIGHT_DIST_OFFSET, NORTH), chassis.getPose().theta);

    chassis.swingToHeading(210, lemlib::DriveSide::LEFT, 1000, {}, false);
    left_mg.move(50);
    right_mg.move(50);
    pros::delay(250);
    chassis.turnToHeading(270, 900);
    chassis.moveToPoint(-10, 36, 1000, {.forwards=false});

    odom.set_value(false);
    matchload.set_value(false);

    upper.move(0);
    lower.move(0);

}