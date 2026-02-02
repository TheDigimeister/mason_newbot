#include "drive.hpp"
#include "main.h"
#include "pros/rtos.hpp"
#include "robot.hpp"

void leftmiddlegoal() {
    odom.set_value(false);
    chassis.setPose(positionFromRaycast(back_dist.get() * MM_TO_IN, BACK_DIST_OFFSET, WEST), positionFromRaycast(left_dist.get() * MM_TO_IN, LEFT_DIST_OFFSET, NORTH),90);


    // 9-ball low goal side (37)

    // // Score low goal

    lower.move(127);

    pros::Task upper_roller_hold([] {
        while(roller_dist.get() > 50) {upper.move(50);}
        upper.move(0);
    });


    // pick up trio

    chassis.moveToPoint(-22, 25, 250, {.maxSpeed=127, .minSpeed = 60, .earlyExitRange = 36});
    chassis.moveToPoint(-22, 25, 2000, {.maxSpeed = 60, .minSpeed = 60, .earlyExitRange = 3});

    // pick up long goal balls
    chassis.moveToPoint(-8.25, 44, 1000, {.maxSpeed=80, .minSpeed = 5, .earlyExitRange = 1});
    chassis.waitUntil(18);
    matchload.set_value(true);
    chassis.moveToPoint(-26.5, 33, 2000, {.forwards = false, .minSpeed=5, .earlyExitRange = 3});
    chassis.turnToHeading(180, 500);
    chassis.moveToPoint(-26.5, 40, 500, {.forwards=false, .minSpeed=127}, false);
    right_mg.move(-127);
    pros::delay(750);
    // pros::Task align_score1{[=]{
    //     while(long_goal_check.get() > 70) { pros::delay(50);}
    //     upper.move(127);
    // }};
    left_mg.move(-127);
    pros::delay(1000);


    // // move to goals
    chassis.turnToHeading(-90, 200, {.maxSpeed = 127, .minSpeed = 5, .earlyExitRange = 1});

    // score into long goal

    matchload.set_value(true);
    pros::delay(700);
    upper.move(0);

    chassis.moveToPoint(-59, 48, 900, {.forwards=true, .maxSpeed = 127, .minSpeed = 45, .earlyExitRange=12});
    chassis.moveToPoint(-59, 48, 800, {.forwards=true, .maxSpeed=45, .minSpeed = 45});
    pros::delay(800);

    // score into middle goal
    chassis.moveToPoint(-8, 8, 2000, {.forwards=false, .maxSpeed=127, .minSpeed = 5, .earlyExitRange = 3}); // 11.75, 8.5
    // chassis.waitUntil(5);
    lower.move(127);
    chassis.turnToHeading(-45, 2000);
    level.set_value(true);
    matchload.set_value(false);
    pros::delay(100);
    upper.move(76);
    pros::delay(1000);
    level.set_value(false);
    upper.move(0);

    // descore
    chassis.moveToPoint(-46.5, 35.5, 1500, {.minSpeed = 5, .earlyExitRange = 1.25});
    // descore.set_value(true);
    chassis.turnToHeading(90, 1000, {.minSpeed=5, .earlyExitRange=1.25});

    chassis.moveToPoint(-11, 36.75, 2000, {.minSpeed = 30, .earlyExitRange = 2});
    chassis.waitUntil(17);
    // descore.set_value(false);
    chassis.turnToHeading(120, 2000);


    // upper.move(127);
    odom.set_value(false);
    matchload.set_value(false);



}