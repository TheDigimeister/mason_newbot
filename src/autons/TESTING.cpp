#include "drive.hpp"
#include "lemlib/chassis/chassis.hpp"
#include "main.h"
#include "pros/rtos.hpp"
#include "robot.hpp"
#include "utils.hpp"

void testing(){

    #pragma region setup
    bool scoredSeven = false;

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
    left_mg.move(50);
    right_mg.move(50);
    pros::delay(2300);

    #pragma region middleGoalScore

    chassis.turnToHeading(90, 1000, {}, false);
    chassis.setPose(positionFromRaycast(back_dist.get() * MM_TO_IN, BACK_DIST_OFFSET, WEST), positionFromRaycast(left_dist.get() * MM_TO_IN, LEFT_DIST_OFFSET, NORTH),chassis.getPose().theta);
    lower.move(127);

    chassis.moveToPose(-27.5, 29, 90, 3000, {.forwards=true, .maxSpeed=90}, false);
    chassis.setPose(positionFromRaycast(back_dist.get() * MM_TO_IN, BACK_DIST_OFFSET, WEST), positionFromRaycast(left_dist.get() * MM_TO_IN, LEFT_DIST_OFFSET, NORTH),chassis.getPose().theta);

    chassis.turnToPoint(-10,4.25, 1000, {.forwards=false, .maxSpeed=80}, false);
    matchload.set_value(true);
    lower.move(0);
    // chassis.setPose(positionFromRaycast(front_dist.get() * MM_TO_IN, FRONT_DIST_OFFSET, WEST), positionFromRaycast(right_dist.get() * MM_TO_IN, RIGHT_DIST_OFFSET, NORTH),chassis.getPose().theta);

    chassis.moveToPose(-9, 7.5, 310, 1500, {.forwards=false, .maxSpeed=80});
    pros::delay(500);
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

    #pragma region NWMatchload
    chassis.turnToPoint(-46, 46, 500, {.minSpeed=5, .earlyExitRange=1});
    chassis.moveToPoint(-46, 46, 1800, {.forwards=true, .maxSpeed=100});
    chassis.waitUntil(12);
    level.set_value(true);
    descore.set_value(true);
    matchload.set_value(true);
    chassis.turnToHeading(270, 500, {}, false);
    upper.move(0);

    chassis.setPose(positionFromRaycast(front_dist.get()*MM_TO_IN, FRONT_DIST_OFFSET, WEST), positionFromRaycast(right_dist.get()*MM_TO_IN, RIGHT_DIST_OFFSET, NORTH), chassis.getPose().theta);

    lower.move(127);

    pros::Task roller_hold1([&] {
        while(roller_dist.get() > 50) {
            upper.move(50);
            pros::delay(20);
        }
        upper.move(0);
    });

    chassis.moveToPoint(-64, 48, 2000, {.forwards=true, .maxSpeed=55, .minSpeed=55});
    pros::delay(2000);

    lower.move(0);

    #pragma region NELongGoalScore1
    chassis.moveToPose(-24, 63, 260, 2000, {.forwards=false, .horizontalDrift=15, .minSpeed=50});
    chassis.moveToPoint(24, 63, 2000, {.forwards=false, .maxSpeed=127, .minSpeed=30, .earlyExitRange=3});
    lower.move(0);
    matchload.set_value(false);
    chassis.moveToPoint(36, 48, 1500, {.forwards=false, .maxSpeed=80, .minSpeed=5, .earlyExitRange=1});
    chassis.turnToHeading(90, 1000, {}, false);
    chassis.setPose(positionFromRaycast(front_dist.get()*MM_TO_IN, FRONT_DIST_OFFSET, EAST), positionFromRaycast(left_dist.get()*MM_TO_IN, LEFT_DIST_OFFSET, NORTH), chassis.getPose().theta);
    level.set_value(true);

    pros::Task align_score2{[=]{
        while(back_dist.get() > 90) { pros::delay(50);}
        lower.move(127);
        upper.move(127);
    }}; 
    chassis.moveToPoint(25, 48, 2000, {.forwards=false, .maxSpeed=80});
    matchload.set_value(true);
    pros::delay(2000);
    lower.move(127);
    upper.move(127);
    chassis.moveToPoint(48, 48, 1000, {.minSpeed=45, .earlyExitRange=5});
    chassis.turnToHeading(90, 1000, {}, false);
    chassis.setPose(positionFromRaycast(front_dist.get()*MM_TO_IN, FRONT_DIST_OFFSET, EAST), positionFromRaycast(left_dist.get()*MM_TO_IN, LEFT_DIST_OFFSET, NORTH), chassis.getPose().theta);
    
    lower.move(127);
    upper.move(127);

    pros::Task roller_hold1b([&] {
        while(roller_dist.get() > 50) {
            lower.move(127);
            upper.move(50);
            pros::delay(20);
        }
        upper.move(0);
    });
    
    #pragma region NEMatchload
    chassis.moveToPoint(64, 48.5, 2000, {.forwards=true, .maxSpeed=55, .minSpeed=55});
    pros::delay(2000);


    #pragma region NELongGoalScore2
    pros::Task align_score3{[=]{
        while(back_dist.get() > 90) { pros::delay(50);}
        lower.move(127);
        upper.move(127);
    }}; 
    chassis.moveToPoint(25, 48, 2500, {.forwards=false, .maxSpeed=80});
    pros::delay(2500);
    lower.move(127);
    upper.move(127);

    chassis.turnToHeading(90, 500, {}, false);

    chassis.setPose(positionFromRaycast(front_dist.get()*MM_TO_IN, FRONT_DIST_OFFSET, EAST),positionFromRaycast(left_dist.get()*MM_TO_IN, LEFT_DIST_OFFSET, NORTH),chassis.getPose().theta);

    matchload.set_value(false);

    pros::Task roller_hold2([&] {
        while(roller_dist.get() > 50) {
            upper.move(50);
            pros::delay(20);
        }
        upper.move(0);
    });

    #pragma region blueParkingClear

    chassis.moveToPose(64, 24, 170, 2000, {.horizontalDrift=15, .minSpeed=100}, false);

    left_mg.move(60);
    right_mg.move(60);
    pros::delay(3900);

    chassis.turnToHeading(90, 1000, {}, false);
    chassis.setPose(positionFromRaycast(front_dist.get()*MM_TO_IN, FRONT_DIST_OFFSET, EAST), positionFromRaycast(right_dist.get()*MM_TO_IN, RIGHT_DIST_OFFSET, SOUTH), chassis.getPose().theta);

    #pragma region SELongGoalScore
    pros::Task align_score4{[=]{
        while(back_dist.get() > 90) { pros::delay(50);}
        lower.move(127);
        upper.move(127);
    }}; 

    chassis.moveToPoint(42, -48, 2000, {.forwards=false}, true);
    chassis.turnToHeading(90, 500, {}, false);
    chassis.setPose(positionFromRaycast(front_dist.get()*MM_TO_IN, FRONT_DIST_OFFSET, EAST), positionFromRaycast(right_dist.get()*MM_TO_IN, RIGHT_DIST_OFFSET, SOUTH), chassis.getPose().theta);
    
    chassis.moveToPoint(25, -48, 2500, {.forwards=false, .maxSpeed=60});
    matchload.set_value(true);
    pros::delay(700);
    lower.move(127);
    upper.move(127);

    chassis.moveToPoint(48, -48, 1000, {.maxSpeed=45, .minSpeed=45, .earlyExitRange=5});


    chassis.turnToHeading(90, 1000, {}, false);


    chassis.setPose(positionFromRaycast(front_dist.get()*MM_TO_IN, FRONT_DIST_OFFSET, EAST), positionFromRaycast(right_dist.get()*MM_TO_IN, RIGHT_DIST_OFFSET, SOUTH),chassis.getPose().theta);
    lower.move(127);
    upper.move(0);

    pros::Task roller_hold3([&] {
        while(roller_dist.get() > 50) {
            upper.move(50);
            pros::delay(20);
        }
        upper.move(0);
    });

    #pragma region SEMatchload
    chassis.moveToPoint(64, -48.5, 2000, {.forwards=true, .maxSpeed=55, .minSpeed=55});
    pros::delay(2000);

    lower.move(0);

    #pragma region SELongGoalScore

    pros::Task align_score67{[=]{
        while(back_dist.get() > 90) { pros::delay(50);}
        lower.move(127);
        upper.move(127);
    }}; 

    chassis.turnToPoint(25, -48, 500, {.forwards=false});
    chassis.moveToPoint(25, -48, 1500, {.forwards=false, .maxSpeed=60});
    pros::delay(1500);

    #pragma region SWLongGoalScore1
    // // chassis.moveToPoint(56, -48, 1000, {.forwards=false, .minSpeed=50, .earlyExitRange=5});
    // left_mg.move(-50);
    // right_mg.move(-50);
    // pros::delay(300);
    matchload.set_value(false);

    // chassis.turnToHeading(190, 1000, {.minSpeed=50, .earlyExitRange=5});

    chassis.swingToHeading(180, lemlib::DriveSide::RIGHT, 700);
    chassis.swingToHeading(90, lemlib::DriveSide::LEFT, 700);

    // chassis.moveToPose(24, -68,90, 1000, {.forwards=false, .horizontalDrift=15, .maxSpeed=127, .minSpeed=50});
    matchload.set_value(false);
    lower.move(-127);
    upper.move(-127);

    

    chassis.moveToPoint(-24, -67.5, 3000, {.forwards=false, .minSpeed=20, .earlyExitRange=5});

    chassis.moveToPoint(-42, -53, 2000, {.forwards=false, .minSpeed=5, .earlyExitRange=1});
    matchload.set_value(true);
    
    level.set_value(true);

    chassis.turnToHeading(270, 1000, {}, false);
    chassis.setPose(positionFromRaycast(front_dist.get()*MM_TO_IN, FRONT_DIST_OFFSET, WEST), positionFromRaycast(left_dist.get()*MM_TO_IN, LEFT_DIST_OFFSET, SOUTH), chassis.getPose().theta);

    upper.move(0);

    lower.move(127);

    pros::Task roller_hold4([&] {
        while(roller_dist.get() > 50) {
            upper.move(50);
            pros::delay(20);
        }
        upper.move(0);
    });
    
    #pragma region SWMatchload
    chassis.moveToPoint(-64, -48.5, 2000, {.forwards=true, .maxSpeed=55, .minSpeed=55});
    pros::delay(2000);


    #pragma region SWLongGoalScore
    pros::Task align_score6{[=]{
        while(back_dist.get() > 90) { pros::delay(50);}
        lower.move(127);
        upper.move(127);
    }}; 

    chassis.turnToPoint(-25, -49.25, 500, {.forwards=false});
    chassis.moveToPoint(-25, -49.25, 3000, {.forwards=false, .maxSpeed=60});
    pros::delay(2000);

    chassis.turnToHeading(270, 500, {}, false);
    chassis.setPose(positionFromRaycast(front_dist.get()*MM_TO_IN, FRONT_DIST_OFFSET, WEST), positionFromRaycast(left_dist.get()*MM_TO_IN, LEFT_DIST_OFFSET, SOUTH), chassis.getPose().theta);
    upper.move(0);
    
    matchload.set_value(false);

    #pragma region finalParking
    chassis.moveToPose(-70, -7, 355, 2200, {.horizontalDrift=15, .maxSpeed=70}, false);
    lower.move(-127);
    left_mg.move(80);
    right_mg.move(80);
    pros::delay(600);
    left_mg.move(0);
    right_mg.move(0);

}