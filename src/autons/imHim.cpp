#include "main.h"

void imHim(){

    descore.set_value(true);
    chassis.setPose(positionFromRaycast(left_dist.get() * MM_TO_IN, LEFT_DIST_OFFSET, WEST), positionFromRaycast(back_dist.get() * MM_TO_IN, BACK_DIST_OFFSET, SOUTH),0);
    lower.move(127);
    pros::Task roller_hold([&] {
        while(roller_dist.get() > 50) {
            upper.move(50);
            pros::delay(20);
        }
        upper.move(0);
    });

    level.set_value(true);
    #pragma region redParkingClear
    left_mg.move(60);
    right_mg.move(60);
    pros::delay(2000);

    #pragma region middleGoalScore

    chassis.turnToHeading(90, 1000, {}, false);
    chassis.setPose(positionFromRaycast(back_dist.get() * MM_TO_IN, BACK_DIST_OFFSET, WEST), positionFromRaycast(left_dist.get() * MM_TO_IN, LEFT_DIST_OFFSET, NORTH),chassis.getPose().theta);
    lower.move(127);

    chassis.moveToPose(-27.5, 30, 90, 3000, {.forwards=true, .maxSpeed=90}, false);
    chassis.setPose(positionFromRaycast(back_dist.get() * MM_TO_IN, BACK_DIST_OFFSET, WEST), positionFromRaycast(left_dist.get() * MM_TO_IN, LEFT_DIST_OFFSET, NORTH),chassis.getPose().theta);

    chassis.turnToPoint(-10,4.25, 1000, {.forwards=false, .maxSpeed=80}, false);
    // chassis.setPose(positionFromRaycast(front_dist.get() * MM_TO_IN, FRONT_DIST_OFFSET, WEST), positionFromRaycast(right_dist.get() * MM_TO_IN, RIGHT_DIST_OFFSET, NORTH),chassis.getPose().theta);

    chassis.moveToPose(-9, 6, 310, 1500, {.forwards=false, .maxSpeed=80});
    pros::delay(900);
    level.set_value(false);

    chassis.turnToHeading(315,500, {}, false);
    
    pros::Task middle_goal_score ([&] {
        int timeout = 0;
        bool redDetected = false;
        while (timeout <= 130 && redDetected == false) {
            if(timeout < 25) {
                lower.move(80);
                upper.move(76);
            } else if (timeout < 50) {
                lower.move(65);
                upper.move(65);
            } else if (timeout > 50) {
                lower.move(58);
                upper.move(58);
            }

            timeout++;
            pros::delay(20);
        }
        lower.move(0);
        upper.move(0);
    });

    pros::delay(2500);

    lower.move(127);
    upper.move(0);
    
    chassis.moveToPoint(-18, 18, 2000, {.forwards=true, .maxSpeed=127, .minSpeed=5, .earlyExitRange=1}, true);
    chassis.moveToPose(-52, -47, 270, 4000, {.forwards=true, .maxSpeed=127, .minSpeed=5, .earlyExitRange=1}, true);
    chassis.moveToPoint(-62, -47, 2000, {.forwards=true, .maxSpeed=127, .minSpeed=5, .earlyExitRange=1}, true);
    chassis.moveToPoint(24, -24, 2000, {.forwards=false, .maxSpeed=127, .minSpeed=5, .earlyExitRange=1}, true);
    chassis.moveToPose(36, -47, 90, 2000, {.forwards=false, .maxSpeed=127, .minSpeed=5, .earlyExitRange=1}, true);
    chassis.moveToPoint(26, -47, 2000, {.forwards=false, .maxSpeed=127, .minSpeed=5, .earlyExitRange=1}, true);
    chassis.moveToPoint(62, -47, 2000, {.forwards=true, .maxSpeed=127, .minSpeed=5, .earlyExitRange=1}, true);
    chassis.moveToPoint(26, -47, 2000, {.forwards=true, .maxSpeed=127, .minSpeed=5, .earlyExitRange=1}, true);
    chassis.moveToPoint(64, -12, 2000, {.forwards=true, .maxSpeed=127, .minSpeed=5, .earlyExitRange=1}, true);
    chassis.turnToHeading(0, 500);
    left_mg.move(80);
    right_mg.move(80);
    pros::delay(2500);

    chassis.moveToPoint(24, 30, 2000, {.forwards=true, .maxSpeed=127, .minSpeed=5, .earlyExitRange=1}, true);
    chassis.moveToPoint(8, 8, 2000, {.forwards=true, .maxSpeed=127, .minSpeed=5, .earlyExitRange=1}, true);
    chassis.moveToPoint(52, 47, 2000, {.forwards=false, .maxSpeed=127, .minSpeed=5, .earlyExitRange=1}, true);
    chassis.moveToPoint(62, 47, 2000, {.forwards=true, .maxSpeed=127, .minSpeed=5, .earlyExitRange=1}, true);
    chassis.moveToPoint(-24, 24, 2000, {.forwards=false, .maxSpeed=127, .minSpeed=5, .earlyExitRange=1}, true);
    chassis.moveToPoint(-36, 47, 2000, {.forwards=false, .maxSpeed=127, .minSpeed=5, .earlyExitRange=1}, true);
    chassis.moveToPoint(-24, 47, 2000, {.forwards=true, .maxSpeed=127, .minSpeed=5, .earlyExitRange=1}, true);
    chassis.moveToPoint(-62, 47, 2000, {.forwards=true, .maxSpeed=127, .minSpeed=5, .earlyExitRange=1}, true);
    chassis.moveToPoint(-24, 47, 2000, {.forwards=true, .maxSpeed=127, .minSpeed=5, .earlyExitRange=1}, true);
    chassis.moveToPose(-62, 12, 190, 2000, {.forwards=true, .maxSpeed=127, .minSpeed=5, .earlyExitRange=1}, true);

}