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
    // chassis.setPose(positionFromRaycast(front_dist.get() * MM_TO_IN, FRONT_DIST_OFFSET, EAST), positionFromRaycast(left_dist.get() * MM_TO_IN, LEFT_DIST_OFFSET, NORTH),90);

    lower.move(127);
    middle.move(127);
    level.set_value(true);

    pros::Task roller_hold([&] {
        while(roller_dist.get() > 50) {
            upper.move(50);
            pros::delay(20);
        }
        upper.move(0);
    });


    #pragma region redParkingClear
    left_mg.move(60);
    right_mg.move(60);
    while(front_disttwo.get() >= 1700 || front_disttwo.get() <= 600){pros::delay(20);}
    matchload.set_value(true);
    while(front_disttwo.get() >= 1075 || front_disttwo.get() <= 600){pros::delay(20);}
    matchload.set_value(false);

    #pragma region middleGoalScore

    chassis.turnToHeading(90, 1000, {}, false);
    chassis.setPose(positionFromRaycast(back_dist.get() * MM_TO_IN, BACK_DIST_OFFSET, WEST), positionFromRaycast(left_dist.get() * MM_TO_IN, LEFT_DIST_OFFSET, NORTH),chassis.getPose().theta);
    lower.move(127);

    chassis.moveToPoint(-24, 24, 1500, {.forwards=true, .maxSpeed=127}, true);
    pros::delay(700);
    matchload.set_value(true);
    middle.move(0);
    chassis.turnToHeading(270, 1000, {}, false);
    chassis.setPose(positionFromRaycast(front_dist.get() * MM_TO_IN, FRONT_DIST_OFFSET, WEST), positionFromRaycast(right_dist.get() * MM_TO_IN, RIGHT_DIST_OFFSET, NORTH),chassis.getPose().theta);

    chassis.turnToPoint(-10,4.25, 1000, {.forwards=false, .maxSpeed=80}, false);
    matchload.set_value(true);
    // lower.move(0);
    // middle.move(0);
    // chassis.setPose(positionFromRaycast(front_dist.get() * MM_TO_IN, FRONT_DIST_OFFSET, WEST), positionFromRaycast(right_dist.get() * MM_TO_IN, RIGHT_DIST_OFFSET, NORTH),chassis.getPose().theta);

    chassis.moveToPose(-12, 8.5, 310, 1400, {.forwards=false, .maxSpeed=80});
    pros::delay(500);
    level.set_value(false);

    chassis.turnToHeading(315,500, {}, false);
    
    int timeout = 0;
    bool redDetected = false;
    while (timeout <= 200 && redDetected == false) {
        if(fabs(roller_optical.get_hue() - 9.0) <= 2.0 && roller_optical.get_proximity() > 170){
            redDetected = true;
            master.print(0, 0, "Red Detected!");
        }

        if(timeout <= 40) {
            lower.move(127);
            middle.move(127);
            upper.move(36);
        } else if (timeout > 40) {
            lower.move(127);
            middle.move(127);
            upper.move(27);
        }

        timeout++;
        pros::delay(20);
    }

    upper.move(0);
    lower.move(127);
    middle.move(127);

    #pragma region NWMatchload
    chassis.turnToPoint(-48, 47, 500, {.minSpeed=5, .earlyExitRange=1});
    chassis.moveToPoint(-48, 47, 1800, {.forwards=true, .maxSpeed=100});
    chassis.waitUntil(12);
    level.set_value(true);
    descore.set_value(true);
    matchload.set_value(true);
    chassis.turnToHeading(270, 500, {}, false);
    upper.move(0);

    chassis.setPose(positionFromRaycast(fmax(front_dist.get(),front_disttwo.get())*MM_TO_IN, FRONT_DIST_OFFSET, WEST), positionFromRaycast(right_dist.get()*MM_TO_IN, RIGHT_DIST_OFFSET, NORTH), chassis.getPose().theta);

    lower.move(127);
    middle.move(127);

    pros::Task roller_hold1([&] {
        while(roller_dist.get() > 50) {
            upper.move(50);
            pros::delay(20);
        }
        upper.move(0);
    });

    chassis.moveToPoint(-64, 48, 2000, {.forwards=true, .maxSpeed=55, .minSpeed=55});
    pros::delay(2000);
    left_mg.move(-50);
    right_mg.move(-50);
    pros::delay(200);

    lower.move(127);
    middle.move(127);

    #pragma region NELongGoalScore1
    chassis.moveToPose(-24, 61, 260, 2000, {.forwards=false, .horizontalDrift=15, .minSpeed=50});
    chassis.moveToPoint(22, 57, 2000, {.forwards=false, .maxSpeed=127, .minSpeed=30, .earlyExitRange=3});
    lower.move(127);
    middle.move(127);
    matchload.set_value(false);
    chassis.moveToPoint(36, 46, 1500, {.forwards=false, .maxSpeed=80, .minSpeed=5, .earlyExitRange=1});
    chassis.turnToHeading(90, 1000, {}, false);
    chassis.setPose(positionFromRaycast(fmax(front_dist.get(),front_disttwo.get())*MM_TO_IN, FRONT_DIST_OFFSET, EAST), positionFromRaycast(left_dist.get()*MM_TO_IN, LEFT_DIST_OFFSET, NORTH), chassis.getPose().theta);
    level.set_value(true);

    pros::Task align_score2{[=]{
        while(back_dist.get() > 100) { pros::delay(50);}
        lower.move(127);
        middle.move(127);
        upper.move(127);
    }}; 
    chassis.moveToPoint(25, 50.75, 3300, {.forwards=false, .maxSpeed=50});
    matchload.set_value(true);
    pros::delay(3300);
    lower.move(127);
    middle.move(127);
    upper.move(127);
    
    #pragma region NEMatchload
    // chassis.moveToPoint(48, 47, 1000, {.minSpeed=45, .earlyExitRange=5});
    chassis.turnToHeading(90, 1000, {}, false);
    chassis.setPose(positionFromRaycast(fmax(front_dist.get(),front_disttwo.get())*MM_TO_IN, FRONT_DIST_OFFSET, EAST), positionFromRaycast(left_dist.get()*MM_TO_IN, LEFT_DIST_OFFSET, NORTH), chassis.getPose().theta);
    
    lower.move(127);
    middle.move(127);
    upper.move(127);

    pros::Task roller_hold1b([&] {
        while(roller_dist.get() > 50) {
            lower.move(127);
            middle.move(127);
            upper.move(50);
            pros::delay(20);
        }
        upper.move(0);
    });
    
    chassis.moveToPoint(64, 46, 3000, {.forwards=true, .maxSpeed=55, .minSpeed=55});
    pros::delay(3000);


    #pragma region NELongGoalScore2
    pros::Task align_score3{[=]{
        while(back_dist.get() > 100) { pros::delay(50);}
        lower.move(127);
        middle.move(127);
        upper.move(127);
    }}; 
    chassis.moveToPoint(25, 49.5, 2500, {.forwards=false, .maxSpeed=80});
    pros::delay(2500);
    lower.move(127);
    middle.move(127);
    upper.move(127);

    chassis.turnToHeading(90, 500, {}, false);

    chassis.setPose(positionFromRaycast(fmax(front_dist.get(),front_disttwo.get())*MM_TO_IN, FRONT_DIST_OFFSET, EAST),positionFromRaycast(left_dist.get()*MM_TO_IN, LEFT_DIST_OFFSET, NORTH),chassis.getPose().theta);

    matchload.set_value(false);

    pros::Task roller_hold2([&] {
        while(roller_dist.get() > 50) {
            upper.move(50);
            pros::delay(20);
        }
        upper.move(0);
    });

    #pragma region blueParkingClear
    chassis.moveToPose(70, 16, 178, 2500, {.horizontalDrift=15, .lead=0.3, .maxSpeed=80, .minSpeed=60}, false);
    level.set_value(true);

    left_mg.move(60);
    right_mg.move(60);
    pros::delay(2400);
    while(front_disttwo.get() >= 1300){pros::delay(20);}
    chassis.turnToHeading(220, 1000, {}, false);
    left_mg.move(80);
    right_mg.move(80);
    pros::delay(430);
    // chassis.setPose(positionFromRaycast(left_dist.get()*MM_TO_IN, LEFT_DIST_OFFSET, EAST), positionFromRaycast(fmax(front_dist.get(),front_disttwo.get())*MM_TO_IN, FRONT_DIST_OFFSET, SOUTH), chassis.getPose().theta);
    matchload.set_value(true);
    // chassis.moveToPoint(47, -47, 1000);
    chassis.turnToHeading(90, 1000, {}, false);
    chassis.setPose(positionFromRaycast(fmax(front_dist.get(),front_disttwo.get())*MM_TO_IN, FRONT_DIST_OFFSET, EAST), positionFromRaycast(right_dist.get()*MM_TO_IN, RIGHT_DIST_OFFSET, SOUTH), chassis.getPose().theta);

    #pragma region SEMatchload
    chassis.moveToPoint(65, -48.5, 2200, {.forwards=true, .maxSpeed=55, .minSpeed=55});
    pros::delay(2200);

    lower.move(0);
    middle.move(0);
    
    #pragma region SELongGoalScore
    pros::Task align_score4{[=]{
        while(back_dist.get() > 100) { pros::delay(50);}
        lower.move(127);
        middle.move(127);
        upper.move(127);
    }}; 

    // chassis.moveToPoint(42, -48, 2000, {.forwards=false}, true);
    // chassis.turnToHeading(90, 500, {}, false);
    // chassis.setPose(positionFromRaycast(front_dist.get()*MM_TO_IN, FRONT_DIST_OFFSET, EAST), positionFromRaycast(right_dist.get()*MM_TO_IN, RIGHT_DIST_OFFSET, SOUTH), chassis.getPose().theta);
    
    chassis.moveToPoint(25, -47.5, 2000, {.forwards=false, .maxSpeed=60});
    matchload.set_value(false);
    // pros::delay(700);
    // lower.move(127);
    // middle.move(127);
    // upper.move(127);
    pros::delay(4500);

    // chassis.moveToPoint(48, -48, 1000, {.maxSpeed=45, .minSpeed=45, .earlyExitRange=5});


    // chassis.turnToHeading(90, 1000, {}, false);


    // chassis.setPose(positionFromRaycast(front_dist.get()*MM_TO_IN, FRONT_DIST_OFFSET, EAST), positionFromRaycast(right_dist.get()*MM_TO_IN, RIGHT_DIST_OFFSET, SOUTH),chassis.getPose().theta);
    lower.move(127);
    middle.move(127);
    upper.move(0);

    pros::Task roller_hold3([&] {
        while(roller_dist.get() > 50) {
            upper.move(50);
            pros::delay(20);
        }
        upper.move(0);
    });

    // #pragma region SELongGoalScore

    // pros::Task align_score67{[=]{
    //     while(back_dist.get() > 100) { pros::delay(50);}
    //     lower.move(127);
    //     middle.move(127);
    //     upper.move(127);
    // }}; 

    // chassis.turnToPoint(25, -48, 500, {.forwards=false});
    // chassis.moveToPoint(25, -48, 1500, {.forwards=false, .maxSpeed=60});
    // pros::delay(1500);

    #pragma region newShih

    #pragma region trioBallGrabs
    chassis.swingToPoint(-24,-24, lemlib::DriveSide::LEFT, 2000, {.direction=lemlib::AngularDirection::CCW_COUNTERCLOCKWISE}, false);
    // chassis.setPose(positionFromRaycast(right_dist.get()*MM_TO_IN, RIGHT_DIST_OFFSET, EAST), chassis.getPose().y, chassis.getPose().theta);

    // chassis.moveToPoint(24, -24, 1000);
    // chassis.turnToPoint(-24, -24, 1000);
    chassis.moveToPoint(-20, -20, 3000);
    pros::delay(1000);
    matchload.set_value(true);

    #pragma region SWMatchload
    chassis.turnToPoint(-48, -43, 1000);
    chassis.moveToPoint(-48, -43, 1000, {});
    matchload.set_value(true);
    chassis.turnToHeading(270, 1000, {}, false);
    chassis.setPose(positionFromRaycast(fmax(front_dist.get(),front_disttwo.get())*MM_TO_IN, FRONT_DIST_OFFSET, WEST), positionFromRaycast(left_dist.get()*MM_TO_IN, LEFT_DIST_OFFSET, SOUTH), chassis.getPose().theta);

    upper.move(0);
    lower.move(127);
    middle.move(127);

    pros::Task roller_hold4([&] {
        while(roller_dist.get() > 50) {
            upper.move(50);
            pros::delay(20);
        }
        upper.move(0);
    });
    
    chassis.moveToPoint(-64, -46.5, 2000, {.forwards=true, .maxSpeed=55, .minSpeed=55});
    pros::delay(2000);

    #pragma region SWLongGoalScore

    pros::Task align_score6{[=]{
        while(back_dist.get() > 100) { pros::delay(50);}
        lower.move(127);
        middle.move(127);
        upper.move(127);
    }}; 

    chassis.turnToPoint(-25, -49.5, 500, {.forwards=false});
    chassis.moveToPoint(-25, -49.5, 1500, {.forwards=false, .maxSpeed=60});
    pros::delay(1400);

    chassis.turnToHeading(270, 500, {}, false);
    chassis.setPose(positionFromRaycast(fmax(front_dist.get(),front_disttwo.get())*MM_TO_IN, FRONT_DIST_OFFSET, WEST), positionFromRaycast(left_dist.get()*MM_TO_IN, LEFT_DIST_OFFSET, SOUTH), chassis.getPose().theta);
    upper.move(0);
    
    matchload.set_value(false);

    #pragma region lowGoalScore
    chassis.swingToPoint(-7, -10, lemlib::DriveSide::RIGHT, 2000, {}, false);
    lower.move(127);
    middle.move(0);
    upper.move(0);
    chassis.moveToPose(-7, -10, 45, 3000, {.maxSpeed=50}, false);
    lower.move(-60);
    middle.move(-60);
    upper.move(-60);
    pros::delay(2000);

    #pragma region finalParking
    chassis.moveToPose(-70, -24, 90, 2200, {.forwards=false, .horizontalDrift=15, .maxSpeed=70}, false);
    chassis.turnToHeading(355, 1000, {}, false);
    lower.move(-127);
    middle.move(-127);
    left_mg.move(80);
    right_mg.move(80);
    pros::delay(1500);
    // timeout = 0;
    // while((front_disttwo.get() >= 1800 || front_disttwo.get() <= 1500) && timeout <= 75){
    //     timeout++;
    //     pros::delay(20);
    // }
    left_mg.move(0);
    right_mg.move(0);

    // #pragma region SWMatchload
    // // // chassis.moveToPoint(56, -48, 1000, {.forwards=false, .minSpeed=50, .earlyExitRange=5});
    // // left_mg.move(-50);
    // // right_mg.move(-50);
    // // pros::delay(300);
    // matchload.set_value(false);

    // // chassis.turnToHeading(190, 1000, {.minSpeed=50, .earlyExitRange=5});

    // chassis.swingToHeading(190, lemlib::DriveSide::RIGHT, 700, {.direction=lemlib::AngularDirection::CW_CLOCKWISE});
    // chassis.swingToHeading(90, lemlib::DriveSide::LEFT, 700, {.direction=lemlib::AngularDirection::CCW_COUNTERCLOCKWISE});

    // // chassis.moveToPose(24, -68,90, 1000, {.forwards=false, .horizontalDrift=15, .maxSpeed=127, .minSpeed=50});
    // matchload.set_value(false);
    // lower.move(-127);
    // middle.move(-127);
    // upper.move(-127);

    

    // chassis.moveToPoint(-20, -58, 2700, {.forwards=false, .minSpeed=50});
    // // chassis.turnToHeading(90, 500, {}, false);
    // // chassis.setPose(positionFromRaycast(back_dist.get()*MM_TO_IN, BACK_DIST_OFFSET, WEST), positionFromRaycast(right_dist.get()*MM_TO_IN, RIGHT_DIST_OFFSET, SOUTH), chassis.getPose().theta);


    // chassis.moveToPoint(-36, -47, 2000, {.forwards=false}, false);
    // matchload.set_value(true);
    
    // level.set_value(true);

    // chassis.turnToHeading(270, 1000, {}, false);
    // chassis.setPose(positionFromRaycast(fmax(front_dist.get(),front_disttwo.get())*MM_TO_IN, FRONT_DIST_OFFSET, WEST), positionFromRaycast(left_dist.get()*MM_TO_IN, LEFT_DIST_OFFSET, SOUTH), chassis.getPose().theta);

    // upper.move(0);

    // lower.move(127);
    // middle.move(127);

    // pros::Task roller_hold4([&] {
    //     while(roller_dist.get() > 50) {
    //         upper.move(50);
    //         pros::delay(20);
    //     }
    //     upper.move(0);
    // });
    
    // chassis.moveToPoint(-64, -47.5, 2000, {.forwards=true, .maxSpeed=55, .minSpeed=55});
    // pros::delay(2000);


    // #pragma region SWLongGoalScore
    // pros::Task align_score6{[=]{
    //     while(back_dist.get() > 100) { pros::delay(50);}
    //     lower.move(127);
    //     middle.move(127);
    //     upper.move(127);
    // }}; 

    // chassis.turnToPoint(-25, -49.5, 500, {.forwards=false});
    // chassis.moveToPoint(-25, -49.5, 1500, {.forwards=false, .maxSpeed=60});
    // pros::delay(1500);

    // chassis.turnToHeading(270, 500, {}, false);
    // chassis.setPose(positionFromRaycast(fmax(front_dist.get(),front_disttwo.get())*MM_TO_IN, FRONT_DIST_OFFSET, WEST), positionFromRaycast(left_dist.get()*MM_TO_IN, LEFT_DIST_OFFSET, SOUTH), chassis.getPose().theta);
    // upper.move(0);
    
    // matchload.set_value(false);

    // #pragma region lowGoalScore
    // chassis.swingToHeading(0, lemlib::DriveSide::RIGHT, 1000, {}, false);
    // lower.move(127);
    // middle.move(0);
    // upper.move(0);
    // chassis.moveToPose(-7.75, -8, 45, 3000, {.maxSpeed=50}, false);
    // lower.move(-60);
    // middle.move(-60);
    // upper.move(-60);
    // pros::delay(2000);

    // #pragma region finalParking
    // chassis.moveToPose(-70, -24, 90, 2200, {.forwards=false, .horizontalDrift=15, .maxSpeed=70}, false);
    // chassis.turnToHeading(358, 1000, {}, false);
    // lower.move(-127);
    // middle.move(-127);
    // left_mg.move(80);
    // right_mg.move(80);
    // timeout = 0;
    // while((front_disttwo.get() >= 1800 || front_disttwo.get() <= 1500) && timeout <= 75){
    //     timeout++;
    //     pros::delay(20);
    // }
    // left_mg.move(0);
    // right_mg.move(0);

}