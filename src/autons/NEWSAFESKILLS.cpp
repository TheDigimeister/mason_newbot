#include "drive.hpp"
#include "lemlib/chassis/chassis.hpp"
#include "main.h"
#include "pros/rtos.hpp"
#include "quintic_motion.hpp"
#include "robot.hpp"
#include "utils.hpp"

void newSafeSkills(){

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
            upper.move(38);
            pros::delay(20);
        }
        upper.move(0);
    });


    #pragma region redParkingClear
    left_mg.move(64);
    right_mg.move(64);
    while(front_disttwo.get() >= 1750 || front_disttwo.get() <= 600){pros::delay(20);}
    // matchload.set_value(true);
    while(front_disttwo.get() >= 1100 || front_disttwo.get() <= 600){pros::delay(20);}

    #pragma region middleGoalScore

    chassis.turnToHeading(90, 1100, {.minSpeed=14}, false);
    chassis.setPose(positionFromRaycast(back_dist.get() * MM_TO_IN, BACK_DIST_OFFSET, WEST), positionFromRaycast(left_dist.get() * MM_TO_IN, LEFT_DIST_OFFSET, NORTH),chassis.getPose().theta);
    matchload.set_value(false);
    lower.move(127);

    // quintic::moveToPoint(chassis,-24, 24, 1700, {.async=true});
    quintic::moveToPoint(chassis, -25, 24, 2000, {.async=true});
    pros::delay(800);
    // matchload.set_value(true);
    // middle.move(0);
    // chassis.turnToHeading(270, 1000, {}, false);
    // chassis.setPose(positionFromRaycast(front_dist.get() * MM_TO_IN, FRONT_DIST_OFFSET, WEST), positionFromRaycast(right_dist.get() * MM_TO_IN, RIGHT_DIST_OFFSET, NORTH),chassis.getPose().theta);

    pros::delay(1200);
    chassis.turnToPoint(-10, 12.5, 1500, {.forwards=false, .maxSpeed=50, .minSpeed=16}, false);
    quintic::moveToPoint(chassis,-10, 12.5, 1500, {.forwards=false, .async=false});
    matchload.set_value(true);
    // lower.move(0);
    // middle.move(0);
    // chassis.setPose(positionFromRaycast(front_dist.get() * MM_TO_IN, FRONT_DIST_OFFSET, WEST), positionFromRaycast(right_dist.get() * MM_TO_IN, RIGHT_DIST_OFFSET, NORTH),chassis.getPose().theta);

    // chassis.moveToPose(-12, 8.5, 310, 1400, {.forwards=false, .maxSpeed=80});
    // pros::delay(500);
    level.set_value(false);

    // chassis.turnToHeading(315,500, {}, false);
    // return;
    
    int timeout = 0;
    bool redDetected = false;
    while (timeout <= 300 && redDetected == false) {
        if(fabs(roller_optical.get_hue() - 9.0) <= 2.0 && roller_optical.get_proximity() > 170){
            redDetected = true;
            upper.move(0);
            master.print(0, 0, "Red Detected!");
            break;
        }

        if(timeout <= 70) {
            lower.move(127);
            middle.move(40);
            upper.move(40);
        } else if (timeout > 70) {
            lower.move(127);
            middle.move(30);
            upper.move(30);
        }

        timeout++;
        pros::delay(10);
    }

    // level.set_value(true);
    upper.move(0);
    lower.move(127);
    middle.move(127);

    #pragma region NWMatchload
    // chassis.turnToPoint(-48, 47, 500, {.minSpeed=5, .earlyExitRange=1});
    // chassis.moveToPoint(-48, 47, 1800, {.forwards=true, .maxSpeed=100});
    pros::Task roller_hold1([&] {
        while(roller_dist.get() > 50) {
            upper.move(38);
            pros::delay(20);
        }
        upper.move(0);
    });
    quintic::moveToPoint(chassis,-47, 46, 3000, {.async=false});
    level.set_value(true);
    chassis.turnToHeading(270, 1000, {.minSpeed=14}, false);
    quintic::moveToPoint(chassis, -64, 47, 2000, {.async=false});
    // chassis.moveToPose(-51, 47, 270, 3000, {.lead=0.3, .minSpeed=14}, false);
    level.set_value(true);
    // left_mg.move(50);
    // right_mg.move(50);
    pros::delay(200);
    left_mg.move(-50);
    right_mg.move(-50);
    pros::delay(150);
    // chassis.waitUntil(12);
    level.set_value(true);
    descore.set_value(true);
    matchload.set_value(true);
    chassis.turnToHeading(270, 1000, {.minSpeed=14}, false);
    upper.move(0);

    chassis.setPose(positionFromRaycast(fmax(front_dist.get(),front_disttwo.get())*MM_TO_IN, FRONT_DIST_OFFSET, WEST), positionFromRaycast(right_dist.get()*MM_TO_IN, RIGHT_DIST_OFFSET, NORTH), chassis.getPose().theta);

    lower.move(127);
    middle.move(127);

    // chassis.moveToPoint(-64, 48, 2000, {.forwards=true, .maxSpeed=47, .minSpeed=47});
    // pros::delay(2000);
    // left_mg.move(-50);
    // right_mg.move(-50);
    // pros::delay(100);

    lower.move(127);
    middle.move(127);

    #pragma region NELongGoalScore1
    chassis.turnToPoint(-30, 64, 700, {.forwards=false, .minSpeed=50});
    chassis.moveToPose(-24, 64, 270, 2000, {.forwards=false, .lead=0.4, .minSpeed=80});
    // quintic::moveToPoint(chassis, -24, 24, 1500, {.forwards=false, .async=false});
    matchload.set_value(false);
    // chassis.turnToPoint(24, 64, 600, {.forwards=false, .minSpeed=50});
    chassis.moveToPoint(26, 64, 1500, {.forwards=false, .minSpeed=50});
    chassis.moveToPoint(36, 51, 2000, {.forwards=false, .minSpeed=50, .earlyExitRange=1});
    // quintic::moveToPoint(chassis, 30, 35, 2200, {.forwards=false, .async=false});
    lower.move(127);
    middle.move(127);
    // matchload.set_value(false);
    level.set_value(true);

    pros::Task align_score2{[=]{
        while(back_dist.get() > 100) { pros::delay(50);}
        lower.move(127);
        middle.move(127);
        upper.move(127);
    }}; 

    chassis.turnToHeading(90, 1000, {.direction=lemlib::AngularDirection::CW_CLOCKWISE, .minSpeed=14}, false);
    chassis.setPose(positionFromRaycast(fmax(front_dist.get(),front_disttwo.get())*MM_TO_IN, FRONT_DIST_OFFSET, EAST), positionFromRaycast(left_dist.get()*MM_TO_IN, LEFT_DIST_OFFSET, NORTH), chassis.getPose().theta);
    
    quintic::moveToPoint(chassis, 26, 49, 2000, {.forwards=false, .async=false});
    pros::delay(200);
    // left_mg.move(-30);
    // right_mg.move(-30);
    // pros::delay(2000);
    // quintic::moveToPoint(chassis, 47, 48, 2000, {.forwards=false, .async=false});
    // chassis.turnToHeading(90, 1000, {.minSpeed=5}, false);    

    // quintic::moveToPoint(chassis, 24, 49, 2000, {.forwards=false, .async=false});
    
    // left_mg.move(50);
    // right_mg.move(50);
    // pros::delay(200);

    chassis.turnToHeading(90, 1000, {.minSpeed=14}, false);
    chassis.setPose(positionFromRaycast(fmax(front_dist.get(),front_disttwo.get())*MM_TO_IN, FRONT_DIST_OFFSET, EAST), positionFromRaycast(left_dist.get()*MM_TO_IN, LEFT_DIST_OFFSET, NORTH), chassis.getPose().theta);
    lower.move(127);
    middle.move(127);
    upper.move(127);
    matchload.set_value(true);

    #pragma region NEMatchload

    pros::Task roller_hold1b([&] {
        while(roller_dist.get() > 50) {
            lower.move(127);
            middle.move(127);
            upper.move(38);
            pros::delay(20);
        }
        upper.move(0);
    });
    
    quintic::moveToPoint(chassis, 62, 48.5, 2500, {.forwards=true, .async=false});
    pros::delay(300);

    // left_mg.move(-50);
    // right_mg.move(-50);
    // pros::delay(150);

    // chassis.turnToHeading(90, 1000, {.minSpeed=14}, false);
    // chassis.setPose(positionFromRaycast(fmax(front_dist.get(),front_disttwo.get())*MM_TO_IN, FRONT_DIST_OFFSET, EAST), positionFromRaycast(left_dist.get()*MM_TO_IN, LEFT_DIST_OFFSET, NORTH), chassis.getPose().theta);


    #pragma region NELongGoalScore2
    pros::Task align_score3{[=]{
        while(back_dist.get() > 100) { pros::delay(50);}
        lower.move(127);
        middle.move(127);
        upper.move(127);
    }}; 
    quintic::moveToPoint(chassis,26, 49, 3000, {.forwards=false, .async=false});
    // pros::delay(2000);

    #pragma region SEMatchload

    lower.move(127);
    middle.move(0);
    upper.move(127);

    chassis.turnToHeading(90, 1000, {.minSpeed=14}, false);

    chassis.setPose(positionFromRaycast(fmax(front_dist.get(),front_disttwo.get())*MM_TO_IN, FRONT_DIST_OFFSET, EAST),positionFromRaycast(left_dist.get()*MM_TO_IN, LEFT_DIST_OFFSET, NORTH),chassis.getPose().theta);

    matchload.set_value(false);

    pros::Task roller_hold2([&] {
        while(roller_dist.get() > 50) {
            upper.move(38);
            pros::delay(20);
        }
        upper.move(0);
    });

    // left_mg.move(50);
    // right_mg.move(50);
    // pros::delay(200);
    // chassis.swingToPoint(24, 24, lemlib::DriveSide::RIGHT, 1500, {.minSpeed=50}, false);
    chassis.turnToHeading(180,2000,{.direction=lemlib::AngularDirection::CW_CLOCKWISE, .minSpeed=30}, false);
    chassis.setPose(positionFromRaycast(left_dist.get()*MM_TO_IN, LEFT_DIST_OFFSET, EAST), positionFromRaycast(back_dist.get()*MM_TO_IN, BACK_DIST_OFFSET, NORTH), chassis.getPose().theta);

    // quintic::moveToPoint(chassis,24, 24, 1500, {.forwards=true, .async=true});
    // chassis.moveToPoint(24, 24, 1500, {.forwards=true, .minSpeed=14, .earlyExitRange=5}, false);
    
    // pros::delay(800);
    // matchload.set_value(true);
    // pros::delay(700);
    // intake_up.set_value(true);
    // lower.move(-64);
    // middle.move(-64);
    // upper.move(-64);
    // pros::delay(2000);
    // intake_up.set_value(false);
    // lower.move(127);
    // middle.move(127);
    // upper.move(127);

    // quintic::moveToPoint(chassis, 24, 24, 2000, {.forwards=false, .async=false});
    // pros::Task roller_hold6([&] {
    //     while(roller_dist.get() > 50) {
    //         upper.move(38);
    //         pros::delay(20);
    //     }
    //     upper.move(0);
    // });
    // chassis.turnToPoint(24, -26, 1000, {.forwards=true, .minSpeed=14, .earlyExitRange=5});
    // chassis.moveToPoint(24, -26, 2000, {.forwards=true, .minSpeed=14});
    // pros::delay(800);
    // matchload.set_value(true);
    // pros::delay(900);
    // chassis.turnToPoint(47, -47, 1000, {.minSpeed=14, .earlyExitRange=5});
    quintic::moveToPoint(chassis, 50, -49, 3000, {.forwards=true, .async=false});
    matchload.set_value(true);
    // quintic::moveToPoint(chassis,24, -48.5, 2000, {.forwards=false, .async=false});
    // pros::delay(1800);
    // left_mg.move(50);
    // right_mg.move(50);
    // pros::delay(200);
    lower.move(127);
    middle.move(127);
    upper.move(0);

    pros::Task roller_hold3([&] {
        while(roller_dist.get() > 50) {
            upper.move(38);
            pros::delay(20);
        }
        upper.move(0);
    });

    chassis.turnToHeading(90, 1000, {.minSpeed=14}, false);
    chassis.setPose(positionFromRaycast(fmax(front_dist.get(),front_disttwo.get())*MM_TO_IN, FRONT_DIST_OFFSET, EAST), positionFromRaycast(right_dist.get()*MM_TO_IN, RIGHT_DIST_OFFSET, SOUTH), chassis.getPose().theta);

    quintic::moveToPoint(chassis,62, -47, 2000, {.forwards=true, .async=false});
    pros::delay(200);
    level.set_value(true);

    left_mg.move(-50);
    right_mg.move(-50);
    pros::delay(225);
    chassis.turnToHeading(90, 1000, {.minSpeed=15}, false);
    chassis.setPose(positionFromRaycast(fmax(front_dist.get(),front_disttwo.get())*MM_TO_IN, FRONT_DIST_OFFSET, EAST), positionFromRaycast(right_dist.get()*MM_TO_IN, RIGHT_DIST_OFFSET, SOUTH), chassis.getPose().theta);
    
    matchload.set_value(false);
    // lower.move(-127);
    // middle.move(-127);
    // upper.move(-127);

    roller_optical.set_led_pwm(100);
    chassis.turnToPoint(36, -64, 1000, {.forwards=false, .minSpeed=14}, false);

    lower.move(0);
    middle.move(0);

    // pros::Task blue_eject([&] {
    // timeout = 0;
    // redDetected = false;
    // while (timeout <= 400 && redDetected == false) {
    //     if(fabs(roller_optical.get_hue() - 9.0) <= 3.0 && roller_optical.get_proximity() > 127){
    //         redDetected = true;
    //         upper.move(0);
    //         master.print(0, 0, "Red Detected again!");
    //         break;
    //     }

    //     if(timeout <= 80) {
    //         lower.move(127);
    //         middle.move(127);
    //         upper.move(36);
    //     } else if (timeout > 80) {
    //         lower.move(127);
    //         middle.move(127);
    //         upper.move(27);
    //     }

    //     timeout++;
    //     pros::delay(10);
    // }

    // upper.move(0);
    // middle.move(127);
    // lower.move(127);
    // });
   
    chassis.moveToPose(24, -64, 270, 2000, {.forwards=false, .minSpeed=80}, false);
    level.set_value(true);

    chassis.moveToPoint(-27, -64, 2700, {.forwards=false, .maxSpeed=80, .minSpeed=50}, false);
    // return;
    // chassis.turnToHeading(90, 500, {}, false);
    // chassis.setPose(positionFromRaycast(back_dist.get()*MM_TO_IN, BACK_DIST_OFFSET, WEST), positionFromRaycast(right_dist.get()*MM_TO_IN, RIGHT_DIST_OFFSET, SOUTH), chassis.getPose().theta);


    chassis.moveToPoint(-38, -51, 2000, {.forwards=false, .minSpeed=14}, false);
    matchload.set_value(true);
    
    level.set_value(true);

    chassis.turnToHeading(270, 1000, {.minSpeed=14}, false);
    chassis.setPose(positionFromRaycast(fmax(front_dist.get(),front_disttwo.get())*MM_TO_IN, FRONT_DIST_OFFSET, WEST), positionFromRaycast(left_dist.get()*MM_TO_IN, LEFT_DIST_OFFSET, SOUTH), chassis.getPose().theta);

    #pragma region SWLongGoalScore1
    pros::Task align_score6{[=]{
        while(back_dist.get() > 100) { pros::delay(50);}
        lower.move(127);
        middle.move(127);
        upper.move(127);
    }}; 

    // chassis.turnToPoint(-24, -50, 500, {.forwards=false}, false);
    // chassis.moveToPoint(-25, -52, 1200, {.forwards=false, .maxSpeed=60});
    quintic::moveToPoint(chassis,-24, -48.5, 2000, {.forwards=false, .async=false});
    // pros::delay(1800);

    chassis.turnToHeading(270, 1000, {.minSpeed=14}, false);
    chassis.setPose(positionFromRaycast(fmax(front_dist.get(),front_disttwo.get())*MM_TO_IN, FRONT_DIST_OFFSET, WEST), positionFromRaycast(left_dist.get()*MM_TO_IN, LEFT_DIST_OFFSET, SOUTH), chassis.getPose().theta);
    upper.move(0);
    
    // matchload.set_value(false);

    #pragma region SWMatchload
    upper.move(0);

    lower.move(127);
    middle.move(127);

    pros::Task roller_hold4([&] {
        while(roller_dist.get() > 50) {
            upper.move(38);
            pros::delay(20);
        }
        upper.move(0);
    });
    
    chassis.moveToPoint(-64, -47, 2000, {.forwards=true, .maxSpeed=55, .minSpeed=55});
    pros::delay(2500);

    #pragma region SWLongGoalScore2

    pros::Task align_score7{[=]{
        while(back_dist.get() > 100) { pros::delay(50);}
        lower.move(127);
        middle.move(127);
        upper.move(127);
    }}; 

    // chassis.turnToPoint(-24, -48, 500, {.forwards=false}, false);
    // chassis.moveToPoint(-25, -52, 1200, {.forwards=false, .maxSpeed=60});
    quintic::moveToPoint(chassis,-26, -48.5, 3000, {.forwards=false, .async=true});
    pros::delay(500);
    matchload.set_value(false);
    pros::delay(3100);

    // chassis.turnToHeading(270, 1000, {.minSpeed=14}, false);
    // chassis.setPose(positionFromRaycast(fmax(front_dist.get(),front_disttwo.get())*MM_TO_IN, FRONT_DIST_OFFSET, WEST), positionFromRaycast(left_dist.get()*MM_TO_IN, LEFT_DIST_OFFSET, SOUTH), chassis.getPose().theta);
    upper.move(0);
    
    left_mg.move(50);
    right_mg.move(50);
    pros::delay(200);
    chassis.turnToHeading(270, 700, {.minSpeed=14}, false);
    chassis.setPose(positionFromRaycast(fmax(front_dist.get(),front_disttwo.get())*MM_TO_IN, FRONT_DIST_OFFSET, WEST), positionFromRaycast(left_dist.get()*MM_TO_IN, LEFT_DIST_OFFSET, SOUTH), chassis.getPose().theta);

    #pragma region lowGoalScore
    chassis.swingToPoint(-15, -9.5, lemlib::DriveSide::RIGHT, 1000, {.minSpeed=30}, false);
    lower.move(127);
    middle.move(0);
    upper.move(0);
    // chassis.moveToPose(-7, -3, 45, 2000, {.maxSpeed=40}, false);
    quintic::moveToPoint(chassis, -15, -9.5, 3000, {.async=false});
    intake_up.set_value(true);
    lower.move(-60);
    middle.move(-60);
    upper.move(-60);
    pros::delay(2000);

    #pragma region finalParking
    chassis.moveToPose(-70, -19, 90, 1500, {.forwards=false, .horizontalDrift=15, .lead=0.3, .maxSpeed=70}, false);
    chassis.turnToHeading(350, 1100, {.minSpeed=14}, false);
    lower.move(-127);
    middle.move(-127);
    left_mg.move(80);
    right_mg.move(80);
    pros::delay(400);
    timeout = 0;
    while((fmax(front_dist.get(),front_disttwo.get()) >= 1925 || fmax(front_dist.get(),front_disttwo.get()) <= 1500) && timeout <= 60){
        timeout++;
        pros::delay(10);
    }
    left_mg.move(0);
    right_mg.move(0);

}