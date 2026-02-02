#include "drive.hpp"
#include "lemlib/chassis/chassis.hpp"
#include "main.h"
#include "pros/rtos.hpp"
#include "robot.hpp"

void superMiddleGoal(){

    // Setup
    chassis.setPose(positionFromRaycast(back_dist.get() * MM_TO_IN, BACK_DIST_OFFSET, WEST), positionFromRaycast(left_dist.get() * MM_TO_IN, LEFT_DIST_OFFSET, NORTH),90);
    matchload.set_value(false);
    intake_up.set_value(false);
    lower.move(127);
    upper.move(0);


    // pick up trio
    chassis.moveToPoint(-24, 24, 250, {.forwards=true, .maxSpeed=100, .minSpeed = 60, .earlyExitRange = 36});
    chassis.moveToPoint(-24, 24, 2000, {.forwards=true, .maxSpeed=80}, true);
    pros::delay(500);
    matchload.set_value(true);
    // // pick up long goal balls
    // chassis.moveToPoint(-4, 46, 2000, {.forwards=true, .maxSpeed=80}, true);
    
    // middle goal score
    // chassis.moveToPoint(-24, 24, 2000, {.forwards=false, .maxSpeed=80}, true);
    chassis.turnToHeading(330, 1000, {.maxSpeed=80});
    level.set_value(false);
    chassis.moveToPoint(-12, 8, 800, {.forwards=false, .maxSpeed=70}, false);
    lower.move(72);
    upper.move(72);
    pros::delay(1300);
    lower.move(127);
    upper.move(0);

    
    // matchload
    chassis.moveToPoint(-47.5, 48.5, 2000, {.forwards=true, .maxSpeed=80}, true);
    
    matchload.set_value(true);
    chassis.turnToHeading(270, 700, {.maxSpeed=80, .earlyExitRange=3}, false);
    level.set_value(true);
    chassis.setPose(positionFromRaycast(front_dist.get() * MM_TO_IN, FRONT_DIST_OFFSET, WEST), positionFromRaycast(right_dist.get() * MM_TO_IN, RIGHT_DIST_OFFSET, NORTH), chassis.getPose().theta);
    chassis.moveToPoint(-62, 49.3, 1200, {.forwards=true, .maxSpeed=60, .minSpeed=45}, true);
    pros::delay(600);

    // pros::Task align_score2{[=]{
    //     bool opponentColorDetected = false;
    //     int timeout = 0;
    //     while(back_dist.get() > 70) { pros::delay(50);}
    //     upper.move(100);
    //     lower.move(100);
    //     while(!opponentColorDetected && timeout < 150){
    //         std::printf("Detected hue: %3f\n", roller_optical.get_hue());
    //         if(fabs(roller_optical.get_hue() - OPPONENTCOLOR) < 20) {
    //             upper.move(-127);
    //             lower.move(0);
    //             opponentColorDetected = true;
    //             std::printf("Opponent color %d detected!\n", OPPONENTCOLOR);
    //         }
    //         timeout++;
    //         pros::delay(20);
    //     }
    // }};
    
    // long goal score
        pros::Task align_score2{[=]{
        while(back_dist.get() > 80) { pros::delay(50);}
        upper.move(127);
        lower.move(127);
    }};
    
    // chassis.turnToPoint(-25, 48, 500, {.forwards=false, .maxSpeed=80}, true);
    chassis.moveToPoint(-25, 48, 1300, {.forwards=false, .maxSpeed=80}, true);
    pros::delay(1300);
    matchload.set_value(false);


    // descore
    chassis.turnToHeading(270, 500, {}, false);
    chassis.setPose(chassis.getPose().x, positionFromRaycast(right_dist.get()*MM_TO_IN, RIGHT_DIST_OFFSET, NORTH), chassis.getPose().theta);


    // upper.move(0);

    // descore

    // descore.set_value(true);
    // chassis.moveToPoint(-28, 48, 500);
    // upper.move(0);

    // descore

    // descore.set_value(true);
    // chassis.moveToPoint(-28, 48, 500);

    chassis.swingToHeading(210, lemlib::DriveSide::LEFT, 1000, {}, false);
    left_mg.move(50);
    right_mg.move(50);
    pros::delay(250);
    chassis.turnToHeading(270, 700);
    chassis.moveToPoint(-10, 36, 1000, {.forwards=false});
    // left_mg.move(50);
    // right_mg.move(50);
    // pros::delay(150);
    // chassis.turnToHeading(340, 500, {.minSpeed=5, .earlyExitRange=1}, false);
    // left_mg.move(-60);
    // right_mg.move(-60);
    // pros::delay(260);
    // descore.set_value(false);
    // chassis.turnToHeading(260, 500, {.maxSpeed=90}, false);
    // left_mg.move(-120);
    // right_mg.move(-120);
    // pros::delay(200);
    // chassis.turnToHeading(270, 500, {}, false);
    // left_mg.move(-100);
    // right_mg.move(-100);
    // pros::delay(300);
    // chassis.turnToHeading(300, 500, {.maxSpeed=50}, false);
    // left_mg.move(0);
    // right_mg.move(0);


    odom.set_value(false);
    matchload.set_value(false);

    upper.move(0);
    lower.move(0);

}