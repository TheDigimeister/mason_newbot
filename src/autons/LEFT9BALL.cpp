#include "main.h"
#include "robot.hpp"

void left9ball() {
    odom.set_value(false);
    chassis.setPose(positionFromRaycast(back_dist.get() * MM_TO_IN, BACK_DIST_OFFSET, WEST), positionFromRaycast(left_dist.get() * MM_TO_IN, LEFT_DIST_OFFSET, NORTH),90);


    // 9-ball low goal side (37)

    lower.move(127);
    pros::Task upper_roller_hold([] {
        while(roller_dist.get() > 50) {upper.move(50);}
        upper.move(0);
    });

    // pick up trio

    chassis.moveToPoint(-18, 28, 250, {.maxSpeed=127, .minSpeed = 60, .earlyExitRange = 36});
    chassis.moveToPoint(-18, 28, 2000, {.maxSpeed = 60, .minSpeed = 60, .earlyExitRange = 8});


    // pick up long goal balls
    
    chassis.moveToPoint(-7, 45, 1200, {.maxSpeed=80, .minSpeed = 5, .earlyExitRange = 3});
    chassis.waitUntil(18);
    matchload.set_value(true);
    pros::delay(100);

    // back up
    chassis.moveToPoint(-20, 36, 2000, {.forwards = false, .maxSpeed = 127, .minSpeed = 100, .earlyExitRange = 5}, false);
    // matchload.set_value(false);
    lower.move(0);

    level.set_value(false);


    // move to goals
    lower.move(127);
    chassis.moveToPoint(-26, 33, 2000, {.forwards = false, .minSpeed=5, .earlyExitRange = 1});
    chassis.turnToHeading(180, 500);
    chassis.moveToPoint(-26, 40, 500, {.forwards=false, .minSpeed=127}, false);
    right_mg.move(-127);
    // pros::Task align_score1{[=]{
    //     while(long_goal_check.get() > 70) { pros::delay(50);}
    //     upper.move(127);
    // }};
    pros::delay(700);
    left_mg.move(-127);
    pros::delay(1900);


  
    upper.move(0);
    matchload.set_value(true);
    chassis.moveToPoint(-61, 48, 900, {.forwards=true, .maxSpeed=127, .minSpeed = 45, .earlyExitRange=12});
    chassis.moveToPoint(-61, 48, 800, {.forwards=true, .maxSpeed=45, .minSpeed = 45});
    pros::delay(800);

    chassis.moveToPoint(-27, 48.5, 2000, {.forwards=false, .maxSpeed=127});
    chassis.waitUntil(5);
    lower.move(127);
    // pros::Task align_score2{[=]{
    //     while(long_goal_check.get() > 70) { pros::delay(50);}
    //     upper.move(127);
    // }};
    pros::delay(2000);
    upper.move(0);

    chassis.turnToHeading(270, 500, {}, false);
    chassis.setPose(chassis.getPose().x, positionFromRaycast(right_dist.get()*MM_TO_IN, RIGHT_DIST_OFFSET, NORTH), chassis.getPose().theta);


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