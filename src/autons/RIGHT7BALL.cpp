#include "lemlib/chassis/chassis.hpp"
#include "main.h"
#include "pros/rtos.hpp"
#include "robot.hpp"
#include "utils.hpp"

void right7ball() {
    odom.set_value(false);
    level.set_value(true);
    descore.set_value(true);
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

    chassis.moveToPoint(-24, -24, 250, {.maxSpeed=127, .minSpeed = 60, .earlyExitRange = 36});
    chassis.moveToPoint(-24, -24, 2000, {.maxSpeed = 60, .minSpeed=5, .earlyExitRange=1});
    pros::delay(500);
    matchload.set_value(true);
    chassis.turnToPoint(-48, -48, 700, {.maxSpeed=80}, false);
    matchload.set_value(true);

    // back up
    chassis.moveToPoint(-48, -48, 1500, {.maxSpeed=80});
    chassis.turnToHeading(270, 500, {.maxSpeed=127}, false);
    chassis.setPose(positionFromRaycast(front_dist.get()*MM_TO_IN, FRONT_DIST_OFFSET, WEST), positionFromRaycast(left_dist.get()*MM_TO_IN, LEFT_DIST_OFFSET, SOUTH), chassis.getPose().theta);

    // chassis.moveToPoint(-61, -48.5, 900, {.forwards=true, .maxSpeed=127, .minSpeed = 45, .earlyExitRange=12});
    chassis.moveToPoint(-62, -48, 1000, {.forwards=true, .maxSpeed=45, .minSpeed = 45});
    pros::delay(1000);
    
    
    // move to goals
    upper.move(0);
    lower.move(127);

    pros::Task align_score2{[=]{
        while(back_dist.get() > 70) { pros::delay(50);}
        upper.move(100);
    }}; 

    // chassis.turnToPoint(-24, -48, 500, {.forwards=false});
    chassis.moveToPoint(-24, -48, 2100, {.forwards=false, .maxSpeed=127});
    // descore.set_value(false);

    pros::delay(2500);
    matchload.set_value(false);
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
    pros::delay(300);
    chassis.turnToHeading(270, 500, {.maxSpeed=80, .minSpeed=5, .earlyExitRange=1}, false);
    left_mg.move(-100);
    right_mg.move(-100);
    pros::delay(450);
    chassis.turnToHeading(270, 500, {}, false);

    odom.set_value(false);
    matchload.set_value(false);

    upper.move(0);
    lower.move(0);



}