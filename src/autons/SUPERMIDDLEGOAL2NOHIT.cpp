#include "autons.hpp"
#include "lemlib/chassis/chassis.hpp"
#include "main.h"
#include "utils.hpp"

void superMiddleGoal2NoHit(){
    // Setup
    chassis.setPose(positionFromRaycast(left_dist.get() * MM_TO_IN, LEFT_DIST_OFFSET, WEST), positionFromRaycast(front_dist.get() * MM_TO_IN, FRONT_DIST_OFFSET, NORTH),0);
    matchload.set_value(true);
    intake_up.set_value(false);
    lower.move(127);
    upper.move(0);
    level.set_value(true);

    chassis.moveToPoint(-51, 43, 2000, {.forwards=true, .maxSpeed=127, .minSpeed=5, .earlyExitRange=1}, true);
    chassis.turnToHeading(270, 700, {}, false);
    chassis.setPose(positionFromRaycast(front_dist.get() * MM_TO_IN, FRONT_DIST_OFFSET, WEST), positionFromRaycast(right_dist.get() * MM_TO_IN, RIGHT_DIST_OFFSET, NORTH), chassis.getPose().theta);
    chassis.moveToPoint(-64, 47, 850, {.forwards=true, .maxSpeed=127}, true);
    pros::delay(850);
    
    chassis.moveToPoint(-25, 49, 2000, {.forwards=false, .maxSpeed=80}, true);
    pros::Task align_score2{[=]{
        int timeout = 0;
        while(back_dist.get() > 80 && timeout < 100) { 
            pros::delay(50);
            timeout++;}
        upper.move(127);
        lower.move(127);
    }};
    pros::delay(2000);
    upper.move(0);
    matchload.set_value(false);
    
    chassis.turnToHeading(180,1000, {.maxSpeed=127}, false);
    chassis.setPose(positionFromRaycast(right_dist.get() * MM_TO_IN, RIGHT_DIST_OFFSET, WEST), positionFromRaycast(back_dist.get() * MM_TO_IN, BACK_DIST_OFFSET, NORTH), chassis.getPose().theta);
    chassis.moveToPoint(-23.5,24.5, 2000, {.forwards=true, .maxSpeed=127, .minSpeed=50}, true);
    chassis.turnToHeading(315, 1000, {}, false);
    lower.move(0);
    level.set_value(false);
    // chassis.moveToPose(-10, 10, 315, 600, {.forwards=false, .maxSpeed=127}, false);
    left_mg.move(-50);
    right_mg.move(-50);
    pros::delay(300);
    left_mg.move(0);
    right_mg.move(0);
    lower.move(127);
    upper.move(127);
    pros::delay(700);
    // descore.set_value(true);
    chassis.moveToPoint(-36, 36, 2000, {.forwards=true, .maxSpeed=127, .minSpeed=5, .earlyExitRange=1}, true);
    chassis.turnToHeading(270, 700, {}, false);
    chassis.setPose(chassis.getPose().x, positionFromRaycast(right_dist.get() * MM_TO_IN, RIGHT_DIST_OFFSET, NORTH), chassis.getPose().theta);

    descore.set_value(false);
    chassis.moveToPoint(-10, 36, 2000, {.forwards=false, .maxSpeed=127}, true);

}