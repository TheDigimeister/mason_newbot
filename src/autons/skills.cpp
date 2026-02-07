#include "drive.hpp"
#include "lemlib/chassis/chassis.hpp"
#include "main.h"
#include "pros/rtos.hpp"
#include "robot.hpp"
#include "utils.hpp"

void skills(){

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
    left_mg.move(69);
    right_mg.move(69);
    while(front_disttwo.get() >= 1750 || front_disttwo.get() <= 600){pros::delay(20);}
    matchload.set_value(true);
    while(front_disttwo.get() >= 1100 || front_disttwo.get() <= 600){pros::delay(20);}

    #pragma region middleGoalScore

    chassis.turnToHeading(90, 1000, {}, false);
    chassis.setPose(positionFromRaycast(back_dist.get() * MM_TO_IN, BACK_DIST_OFFSET, WEST), positionFromRaycast(left_dist.get() * MM_TO_IN, LEFT_DIST_OFFSET, NORTH),chassis.getPose().theta);
    matchload.set_value(false);
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
    while (timeout <= 400 && redDetected == false) {
        if(fabs(roller_optical.get_hue() - 9.0) <= 2.0 && roller_optical.get_proximity() > 170){
            redDetected = true;
            upper.move(0);
            master.print(0, 0, "Red Detected!");
        }

        if(timeout <= 80) {
            lower.move(127);
            middle.move(127);
            upper.move(36);
        } else if (timeout > 80) {
            lower.move(127);
            middle.move(127);
            upper.move(27);
        }

        timeout++;
        pros::delay(10);
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

    chassis.moveToPoint(-64, 48, 2000, {.forwards=true, .maxSpeed=47, .minSpeed=47});
    pros::delay(2000);
    left_mg.move(-50);
    right_mg.move(-50);
    pros::delay(200);

    lower.move(127);
    middle.move(127);

    #pragma region NELongGoalScore1
    chassis.moveToPose(-24, 61, 260, 2000, {.forwards=false, .horizontalDrift=15, .minSpeed=127});
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
    chassis.moveToPoint(25, 50.75, 2300, {.forwards=false, .maxSpeed=50});
    matchload.set_value(true);
    pros::delay(2600);
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
    chassis.moveToPoint(25, 49.5, 3000, {.forwards=false, .maxSpeed=55});
    pros::delay(3000);
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
    chassis.moveToPoint(50,58,1000);
    chassis.turnToPoint(70, 24, 1000);
    chassis.moveToPoint(70, 24, 1000, {.maxSpeed=100, .minSpeed=20}, false);
    // chassis.moveToPose(69, 16, 178, 2500, {.horizontalDrift=15, .lead=0.4, .maxSpeed=80, .minSpeed=30}, false);
    level.set_value(true);
    left_mg.move(20);
    right_mg.move(20);
    pros::delay(200);
    left_mg.move(0);
    right_mg.move(0);
    pros::delay(100);
    left_mg.move(69);
    right_mg.move(69);
    pros::delay(1800);
    while(front_disttwo.get() >= 1400){pros::delay(20);}
    chassis.turnToHeading(180, 700, {}, false);
    chassis.setPose(positionFromRaycast(left_dist.get()*MM_TO_IN, LEFT_DIST_OFFSET, EAST), positionFromRaycast(fmax(front_dist.get(),front_disttwo.get())*MM_TO_IN, FRONT_DIST_OFFSET, SOUTH), 180);
    chassis.turnToPoint(47,-48, 1000, {});
    chassis.moveToPoint(47, -48, 1000, {}, false);
    matchload.set_value(true);

    // chassis.moveToPoint(47, -47, 1000);
    chassis.turnToPoint(65, -47, 1000, {}, false);
    // chassis.setPose(positionFromRaycast(fmax(front_dist.get(),front_disttwo.get())*MM_TO_IN, FRONT_DIST_OFFSET, EAST), positionFromRaycast(right_dist.get()*MM_TO_IN, RIGHT_DIST_OFFSET, SOUTH), chassis.getPose().theta);
    #pragma region SEMatchload
    chassis.moveToPoint(65, -47, 2200, {.forwards=true, .maxSpeed=55, .minSpeed=55});
    pros::delay(2200);

    left_mg.move(-50);
    right_mg.move(-50);
    pros::delay(200);
    // chassis.turnToHeading(90, 1000, {}, false);
    // chassis.setPose(positionFromRaycast(fmax(front_dist.get(),front_disttwo.get())*MM_TO_IN, FRONT_DIST_OFFSET, EAST), positionFromRaycast(right_dist.get()*MM_TO_IN, RIGHT_DIST_OFFSET, SOUTH), chassis.getPose().theta);

    chassis.turnToPoint(25, -46, 1000, {.forwards=false});
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
    
    chassis.moveToPoint(25, -46, 2000, {.forwards=false, .maxSpeed=60});
    matchload.set_value(false);
    // pros::delay(700);
    // lower.move(127);
    // middle.move(127);
    // upper.move(127);
    pros::delay(3500);

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

    #pragma region SWMatchload
    // // chassis.moveToPoint(56, -48, 1000, {.forwards=false, .minSpeed=50, .earlyExitRange=5});
    // left_mg.move(-50);
    // right_mg.move(-50);
    // pros::delay(300);
    matchload.set_value(false);

    // chassis.turnToHeading(190, 1000, {.minSpeed=50, .earlyExitRange=5});

    chassis.swingToHeading(190, lemlib::DriveSide::RIGHT, 700, {.direction=lemlib::AngularDirection::CW_CLOCKWISE});
    chassis.swingToHeading(90, lemlib::DriveSide::LEFT, 1200, {.direction=lemlib::AngularDirection::CCW_COUNTERCLOCKWISE}, false);
    chassis.setPose(positionFromRaycast(front_dist.get()*MM_TO_IN, FRONT_DIST_OFFSET, EAST), positionFromRaycast(right_dist.get()*MM_TO_IN, RIGHT_DIST_OFFSET, SOUTH), chassis.getPose().theta);
    // chassis.moveToPose(24, -68,90, 1000, {.forwards=false, .horizontalDrift=15, .maxSpeed=127, .minSpeed=50});
    matchload.set_value(false);
    lower.move(-127);
    middle.move(-127);
    upper.move(-127);

    

    chassis.moveToPoint(-23, -57, 2700, {.forwards=false, .minSpeed=10});
    // chassis.turnToHeading(90, 500, {}, false);
    // chassis.setPose(positionFromRaycast(back_dist.get()*MM_TO_IN, BACK_DIST_OFFSET, WEST), positionFromRaycast(right_dist.get()*MM_TO_IN, RIGHT_DIST_OFFSET, SOUTH), chassis.getPose().theta);


    chassis.moveToPoint(-36, -42, 2000, {.forwards=false}, false);
    matchload.set_value(true);
    
    level.set_value(true);

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
    
    chassis.moveToPoint(-64, -47.5, 2000, {.forwards=true, .maxSpeed=55, .minSpeed=55});
    pros::delay(2000);

    // left_mg.move(-50);
    // right_mg.move(-50);
    // pros::delay(200);
    // chassis.turnToHeading(270, 700, {}, false);
    // chassis.setPose(positionFromRaycast(fmax(front_dist.get(),front_disttwo.get())*MM_TO_IN, FRONT_DIST_OFFSET, WEST), positionFromRaycast(left_dist.get()*MM_TO_IN, LEFT_DIST_OFFSET, SOUTH), chassis.getPose().theta);

    #pragma region SWLongGoalScore
    pros::Task align_score6{[=]{
        while(back_dist.get() > 100) { pros::delay(50);}
        lower.move(127);
        middle.move(127);
        upper.move(127);
    }}; 

    chassis.turnToPoint(-25, -52, 500, {.forwards=false});
    chassis.moveToPoint(-25, -52, 1200, {.forwards=false, .maxSpeed=60});
    pros::delay(1200);

    chassis.turnToHeading(270, 500, {}, false);
    chassis.setPose(positionFromRaycast(fmax(front_dist.get(),front_disttwo.get())*MM_TO_IN, FRONT_DIST_OFFSET, WEST), positionFromRaycast(left_dist.get()*MM_TO_IN, LEFT_DIST_OFFSET, SOUTH), chassis.getPose().theta);
    upper.move(0);
    
    matchload.set_value(false);

    #pragma region lowGoalScore
    chassis.swingToPoint(-7, -9.5, lemlib::DriveSide::RIGHT, 1000, {.minSpeed=30}, false);
    lower.move(127);
    middle.move(0);
    upper.move(0);
    chassis.moveToPose(-7, -9.5, 45, 2200, {.maxSpeed=40}, false);
    intake_up.set_value(true);
    lower.move(-60);
    middle.move(-60);
    upper.move(-60);
    pros::delay(2000);

    #pragma region finalParking
    chassis.moveToPose(-70, -24, 90, 2200, {.forwards=false, .horizontalDrift=15, .maxSpeed=70}, false);
    chassis.turnToHeading(357, 1000, {}, false);
    lower.move(-127);
    middle.move(-127);
    left_mg.move(80);
    right_mg.move(80);
    pros::delay(400);
    timeout = 0;
    while((fmax(front_dist.get(),front_disttwo.get()) >= 1875 || fmax(front_dist.get(),front_disttwo.get()) <= 1500) && timeout <= 50){
        timeout++;
        pros::delay(20);
    }
    left_mg.move(0);
    right_mg.move(0);

}