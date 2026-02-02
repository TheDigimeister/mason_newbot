#include "lemlib/chassis/chassis.hpp"
#include "main.h"
#include "pros/rtos.hpp"
#include "robot.hpp"
#include "utils.hpp"

void rightslow7Ball() {
    odom.set_value(false);
    level.set_value(true);
    // chassis.setPose(-46.847,-14.278,90);
    chassis.setPose(positionFromRaycast(back_dist.get() * MM_TO_IN, BACK_DIST_OFFSET, WEST), positionFromRaycast(right_dist.get() * MM_TO_IN, RIGHT_DIST_OFFSET, SOUTH),90);
    // chassis.setPose()


    // 7-ball low goal side (31)

    lower.move(127);
    pros::Task upper_roller_hold([] {
        while(roller_dist.get() > 50) {upper.move(50);}
        upper.move(0);
    });

    // pick up trio

    chassis.moveToPoint(-24, -24, 250, {.maxSpeed=90, .minSpeed = 60, .earlyExitRange = 36});
    chassis.moveToPoint(-24, -24, 2000, {.maxSpeed=90, .minSpeed=5, .earlyExitRange=10});
    pros::delay(300);
    matchload.set_value(true);
    chassis.turnToPoint(-49.5, -49, 700, {.maxSpeed=80, .minSpeed=50, .earlyExitRange=5}, false);
    

    // back up
    chassis.moveToPoint(-53, -47, 1300, {.maxSpeed=80});
    chassis.turnToHeading(270, 500, {.maxSpeed=80}, false);
    chassis.setPose(positionFromRaycast(front_dist.get()*MM_TO_IN, FRONT_DIST_OFFSET, WEST), positionFromRaycast(left_dist.get()*MM_TO_IN, LEFT_DIST_OFFSET, SOUTH), chassis.getPose().theta);

    // chassis.moveToPoint(-61, -48.5, 900, {.forwards=true, .maxSpeed=127, .minSpeed = 45, .earlyExitRange=12});
    chassis.moveToPoint(-63, -48, 850, {.forwards=true, .maxSpeed=55, .minSpeed = 55});
    pros::delay(1050);
    
    
    // move to goals
    upper.move(0);
    lower.move(127);

    pros::Task align_score2{[=]{
        while(back_dist.get() > 85) { pros::delay(50);}
        upper.move(100);
    }}; 

    chassis.moveToPoint(-24, -49, 2500, {.forwards=false});
    // descore.set_value(false);

    pros::delay(2500);
    matchload.set_value(false);
    chassis.turnToHeading(270, 500, {.maxSpeed=80}, false);

    // upper.move(0);

    // descore

    descore.set_value(true);
    
    // left_mg.move(50);
    // right_mg.move(50);
    // pros::delay(1000);
    // left_mg.move(127);
    // right_mg.move(-127);
    // pros::delay(1000);
    // descore.set_value(false);
    // right_mg.move(127);
    // pros::delay(1000);
    // chassis.moveToPoint(-40, -36, 1500, {.minSpeed=5, .earlyExitRange=1.5});
    // // descore.set_value(true);
    // chassis.turnToHeading(90, 1000, {}, false);
    // chassis.setPose(positionFromRaycast(back_dist.get()*MM_TO_IN, BACK_DIST_OFFSET, WEST), positionFromRaycast(right_dist.get()*MM_TO_IN, RIGHT_DIST_OFFSET, SOUTH), chassis.getPose().theta);
    // chassis.moveToPose(-14, -36,90, 2000, {.forwards=true, .minSpeed = 30, .earlyExitRange = 3});
    // // chassis.waitUntil(15);
    // // descore.set_value(false);
    left_mg.move(50);
    right_mg.move(50);
    pros::delay(150);
    chassis.turnToHeading(340, 500, {.minSpeed=5, .earlyExitRange=2}, false);
    left_mg.move(-50);
    right_mg.move(-50);
    pros::delay(330);
    descore.set_value(false);
    chassis.turnToHeading(260, 500, {.maxSpeed=90}, false);
    left_mg.move(-120);
    right_mg.move(-120);
    pros::delay(200);
    chassis.turnToHeading(250, 500, {.maxSpeed=90}, false);
    left_mg.move(-120);
    right_mg.move(-120);
    pros::delay(200);
    chassis.turnToHeading(270, 500, {.earlyExitRange=1}, false);
    left_mg.move(-65);
    right_mg.move(-65);
    pros::delay(310);
    chassis.turnToHeading(270, 500, {.earlyExitRange=1}, false);
    chassis.turnToHeading(300, 500, {.earlyExitRange=1}, false);

    odom.set_value(false);
    matchload.set_value(false);

    upper.move(0);
    lower.move(0);


}