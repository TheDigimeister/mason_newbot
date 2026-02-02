#include "drive.hpp"
#include "main.h"
#include "robot.hpp"
#include "utils.hpp"

void rightLowGoal() {
    odom.set_value(false);
    chassis.setPose(positionFromRaycast(back_dist.get() * MM_TO_IN, BACK_DIST_OFFSET, WEST), positionFromRaycast(right_dist.get() * MM_TO_IN, RIGHT_DIST_OFFSET, SOUTH),90);
    descore.set_value(true);
    level.set_value(true);
    // 9-ball low goal side (37)

    lower.move(127);
    pros::Task upper_roller_hold([] {
        while(roller_dist.get() > 50) {upper.move(50);}
        upper.move(0);
    });

    // pick up trio
    // chassis.moveToPose(-24, -24, 135, 1000, {.maxSpeed=127, .minSpeed=100, .earlyExitRange=12});
    // chassis.moveToPoint(-24, -24, 750, {.maxSpeed=50, .minSpeed=50, .earlyExitRange=3});

    // pick up trio
    chassis.moveToPoint(-18, -28, 500, {.maxSpeed=127, .minSpeed = 40, .earlyExitRange = 12});
    chassis.moveToPoint(-18, -28, 2000, {.maxSpeed = 40, .minSpeed = 40, .earlyExitRange = 5});

    // pick up long goal balls
    
    chassis.moveToPoint(-6, -46, 1200, {.maxSpeed=100, .minSpeed=5, .earlyExitRange=1});
    // chassis.moveToPoint(-6, -46, 400, {.maxSpeed=40});


    // back up
    chassis.moveToPoint(-24, -36, 2000, {.forwards = false, .maxSpeed = 127, .minSpeed = 100, .earlyExitRange = 5}, false);
    // matchload.set_value(false);
    // level.set_value(false);

    // move to goals
    lower.move(127);
    descore.set_value(false);
    chassis.moveToPoint(-27, -33, 2000, {.forwards = false, .minSpeed=5, .earlyExitRange=1});
    chassis.turnToHeading(0, 500);
    chassis.moveToPoint(-27, -48, 500, {.forwards=false}, false);
    left_mg.move(-127);
    right_mg.move(0);
    pros::Task align_score1{[=]{
        while(back_dist.get() > 70) {pros::delay(50);}
        upper.move(127);
    }};
    pros::delay(900);
    right_mg.move(-127);
    upper.move(127);
    pros::delay(1200);

    chassis.turnToHeading(270, 200, {}, false);
    chassis.setPose(chassis.getPose().x, positionFromRaycast(left_dist.get() * MM_TO_IN,LEFT_DIST_OFFSET, SOUTH),chassis.getPose().theta);
    upper.move(0);
    matchload.set_value(true);
    chassis.moveToPoint(-61, -48, 900, {.forwards=true, .maxSpeed=127, .minSpeed = 45, .earlyExitRange=24});
    chassis.moveToPoint(-61, -48, 800, {.forwards=true, .maxSpeed=45, .minSpeed = 45});
    pros::delay(1000);

    matchload.set_value(false);

    chassis.moveToPoint(-24, -24, 2000, {.forwards=false, .minSpeed=127, .earlyExitRange=5});
    chassis.turnToPoint(-10, -10, 1000, {.minSpeed=127, .earlyExitRange=1});
    intake_up.set_value(true);
    chassis.moveToPoint(-10, -10, 2000, {}, true);
    lower.move(-127);
    upper.move(-127);
    pros::delay(2000);
    intake_up.set_value(false);

    // descore
    chassis.moveToPoint(-39, -37.5, 1500, {.forwards=false, .minSpeed=50});
    descore.set_value(false);
    chassis.turnToHeading(90, 1000, {}, false);

    // left_mg.move(-80);
    // right_mg.move(-80);
    // pros::delay(700);

    chassis.setPose(positionFromRaycast(back_dist.get()*MM_TO_IN, BACK_DIST_OFFSET, WEST),positionFromRaycast(right_dist.get()*MM_TO_IN, RIGHT_DIST_OFFSET, SOUTH), chassis.getPose().theta);

    chassis.moveToPose(-12, -37.5, 90, 1000, {.forwards=true, .minSpeed=50});
    chassis.turnToHeading(60, 2000);


    odom.set_value(false);
    matchload.set_value(false);

}