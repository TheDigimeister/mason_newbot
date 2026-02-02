#include "lemlib/chassis/chassis.hpp"
#include "main.h"
#include "pros/rtos.hpp"
#include "robot.hpp"
#include "utils.hpp"

void superLowGoal(){

    // Setup
    chassis.setPose(positionFromRaycast(back_dist.get() * MM_TO_IN, BACK_DIST_OFFSET, WEST), positionFromRaycast(right_dist.get() * MM_TO_IN, RIGHT_DIST_OFFSET, SOUTH),90);
    matchload.set_value(false);
    level.set_value(true);
    intake_up.set_value(false);
    lower.move(127);
    upper.move(0);

    // pros::Task upper_roller_hold([] {
    //     while(roller_dist.get() > 50) {upper.move(50);}
    //     upper.move(0);
    // });

    // pick up trio
    chassis.moveToPoint(-24, -23, 250, {.forwards=true, .maxSpeed=90, .minSpeed = 45, .earlyExitRange = 36});
    chassis.moveToPoint(-24, -23, 2000, {.forwards=true, .maxSpeed=70}, true);
    pros::delay(500);
    
    
    // middle goal score
    // chassis.moveToPoint(-24, 24, 2000, {.forwards=false, .maxSpeed=80}, true);
    chassis.turnToHeading(45, 1000, {.maxSpeed=70}, false);
    lower.move(0);
    upper.move(0);
    intake_up.set_value(true);
    chassis.moveToPoint(-11.5, -9.5, 1300, {.forwards=true, .maxSpeed=70}, true);
   
    pros::delay(500);
    chassis.turnToHeading(45, 200);
    lower.move(-50);
    upper.move(0);
    pros::delay(600);
    lower.move(127);
    upper.move(0);
    level.set_value(true);
    
    // matchload
    chassis.moveToPoint(-48, -49, 2000, {.forwards=false, .maxSpeed=80}, true);
    pros::delay(100);
    intake_up.set_value(false);
    matchload.set_value(true);
    chassis.turnToHeading(270, 1000, {.maxSpeed=80}, false);
    chassis.setPose(positionFromRaycast(front_dist.get() * MM_TO_IN, FRONT_DIST_OFFSET, WEST), positionFromRaycast(left_dist.get() * MM_TO_IN, LEFT_DIST_OFFSET, SOUTH), chassis.getPose().theta);
    chassis.moveToPoint(-62, -49, 1000, {.forwards=true, .maxSpeed=45, .minSpeed=45}, true);
    pros::delay(1150);
    
    // long goal score
        pros::Task align_score2{[=]{
        int timeout = 0;
        while(back_dist.get() > 70) { pros::delay(50);}
        upper.move(127);
        lower.move(127);
    }};
    
    // chassis.turnToPoint(-25, -48.25, 500, {.forwards=false, .maxSpeed=80}, true);
    chassis.moveToPoint(-25, -49.25, 2000, {.forwards=false, .maxSpeed=60}, true);
    pros::delay(2500);
    lower.move(0);
    upper.move(0);
    matchload.set_value(false);

    // descore
    chassis.turnToHeading(270, 500, {}, false);
    chassis.setPose(chassis.getPose().x, positionFromRaycast(left_dist.get()*MM_TO_IN, LEFT_DIST_OFFSET, SOUTH), chassis.getPose().theta);


    // upper.move(0);

    // descore

    // descore.set_value(true);
    // chassis.moveToPoint(-28, 48, 500);
    chassis.turnToHeading(270, 500, {}, false);
    chassis.setPose(chassis.getPose().x, positionFromRaycast(left_dist.get()*MM_TO_IN, LEFT_DIST_OFFSET, SOUTH), chassis.getPose().theta);


    // upper.move(0);

    // descore

    // descore.set_value(true);
    // chassis.moveToPoint(-28, 48, 500);
    left_mg.move(50);
    right_mg.move(50);
    pros::delay(150);
    chassis.turnToHeading(340, 500, {.minSpeed=5, .earlyExitRange=1}, false);
    left_mg.move(-50);
    right_mg.move(-50);
    pros::delay(330);
    descore.set_value(false);
    chassis.turnToHeading(290, 500, {.maxSpeed=90}, false);
    left_mg.move(-120);
    right_mg.move(-120);
    pros::delay(200);
    chassis.turnToHeading(250, 500, {.maxSpeed=90}, false);
    left_mg.move(-120);
    right_mg.move(-120);
    pros::delay(200);
    chassis.turnToHeading(270, 500, {}, false);
    left_mg.move(-120);
    right_mg.move(-120);
    pros::delay(230);
    chassis.turnToHeading(270, 500, {}, false);
    chassis.turnToHeading(300, 500, {}, false);
    odom.set_value(false);
    matchload.set_value(false);

    upper.move(0);
    lower.move(0);
}