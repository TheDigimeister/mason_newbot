#include "drive.hpp"
#include "lemlib/chassis/chassis.hpp"
#include "main.h"
#include "pros/rtos.hpp"
#include "robot.hpp"
#include "utils.hpp"

void oldtesting(){
    bool scoredSeven = false;

    // level.set_value(true);

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

    // left_mg.move(45);
    // right_mg.move(45);
    // pros::delay(300);
    // left_mg.move(30);
    // right_mg.move(30);
    // pros::delay(200);
    // left_mg.move(40);
    // right_mg.move(60);
    // pros::delay(2500);

    left_mg.move(60);
    right_mg.move(60);
    pros::delay(2000);

    chassis.turnToHeading(90, 1000, {}, false);
    chassis.setPose(positionFromRaycast(back_dist.get() * MM_TO_IN, BACK_DIST_OFFSET, WEST), positionFromRaycast(left_dist.get() * MM_TO_IN, LEFT_DIST_OFFSET, NORTH),chassis.getPose().theta);
    lower.move(127);

    // alternate

    chassis.moveToPose(-28.5, 30, 90, 3000, {.forwards=true, .maxSpeed=90}, false);
    chassis.setPose(positionFromRaycast(back_dist.get() * MM_TO_IN, BACK_DIST_OFFSET, WEST), positionFromRaycast(left_dist.get() * MM_TO_IN, LEFT_DIST_OFFSET, NORTH),chassis.getPose().theta);

    chassis.turnToHeading(315, 2000, {.maxSpeed=80}, false);
    chassis.moveToPose(-8.5,5, 315, 1200, {.forwards=false, .maxSpeed=100}, true);
    pros::delay(500);
    level.set_value(false);
    chassis.turnToHeading(315,500, {}, false);
    // chassis.moveToPoint(-38, 40, 1000, {.forwards=true, .maxSpeed=80});
    // chassis.turnToHeading(135, 800, {.minSpeed=5, .earlyExitRange=1}, false);
    // // chassis.moveToPoint(-25, 25, 1000, {.forwards=true, .maxSpeed=50});
    // left_mg.move(70);
    // right_mg.move(70);
    // pros::delay(200);
    // lower.move(127);
    // pros::delay(200);
    // left_mg.move(0);
    // right_mg.move(0);
    // pros::delay(400);
    // // chassis.turnToPoint(-11.5, 11.5, 2000, {.forwards=false, .maxSpeed=70});
    // chassis.turnToHeading(315, 1300, {.direction=lemlib::AngularDirection::CW_CLOCKWISE, .maxSpeed=80, .minSpeed=5, .earlyExitRange=1});
    // lower.move(50);
    // chassis.moveToPoint(-11.5, 13.4, 2000, {.forwards=false, .maxSpeed=70});
    // chassis.moveToPoint(-37, 36, 2000, {.forwards=true, .maxSpeed=80});
    // chassis.turnToPoint(-26, 26, 500);
    // chassis.moveToPoint(-26, 26, 2000, {.minSpeed=50}, true);
    // chassis.waitUntil(24);
    // lower.move(127);
    // pros::delay(700);
    // lower.move(0);

    // chassis.turnToHeading(315, 1000);
    // chassis.turnToHeading(320, 1000, {}, false);
    // lower.move(76);
    // left_mg.move(-50);
    // right_mg.move(-50);
    // pros::delay(300);
    // lower.move(127);
    // upper.move(127);
    // pros::delay(300);
    // lower.move(100);
    // upper.move(100);
    // pros::delay(200);
    // lower.move(76);
    // upper.move(76);
    // pros::delay(500);
    
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

            // if(fabs(roller_optical.get_hue() - 15) < 15){
            //     redDetected = true;
            // }
            timeout++;
            pros::delay(20);
        }
        lower.move(0);
        upper.move(0);
    });

    pros::delay(2500);

    lower.move(127);
    upper.move(0);
    //67 41 :3 XD hi sean its allen >:(

    chassis.turnToPoint(-46, 46, 500, {.minSpeed=5, .earlyExitRange=1});
    chassis.moveToPoint(-46, 46, 1800, {.forwards=true, .maxSpeed=100});
    chassis.waitUntil(12);
    level.set_value(true);
    descore.set_value(true);
    matchload.set_value(true);
    chassis.turnToHeading(270, 500, {}, false);
    chassis.setPose(positionFromRaycast(front_dist.get()*MM_TO_IN, FRONT_DIST_OFFSET, WEST), positionFromRaycast(right_dist.get()*MM_TO_IN, RIGHT_DIST_OFFSET, NORTH), chassis.getPose().theta);

    lower.move(127);

    pros::Task roller_hold1([&] {
        while(roller_dist.get() > 50) {
            upper.move(50);
            pros::delay(20);
        }
        upper.move(0);
    });

    chassis.moveToPoint(-64, 48, 2000, {.forwards=true, .maxSpeed=55, .minSpeed=55}, false);
    pros::delay(400);
    left_mg.move(-50);
    right_mg.move(-50);
    pros::delay(100);
    left_mg.move(45);
    right_mg.move(45);
    pros::delay(300);

    lower.move(0);

    // descore.set_value(false);

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

    // // chassis.setPose(positionFromRaycast(front_dist.get()*MM_TO_IN, FRONT_DIST_OFFSET, EAST), positionFromRaycast(left_dist.get()*MM_TO_IN, LEFT_DIST_OFFSET, NORTH), 90);

    // chassis.moveToPoint(48, 48, 2000);
    // chassis.turnToHeading(225,2000);

    // chassis.moveToPoint(24,24,2000,{});
    // chassis.turnToHeading(225, 2000);
    // // pros::Task middle_goal_check([&] {
    // //     int timeout = 0;
    // //     scoredSeven = false;
    // //     while (timeout <= 100 || scoredSeven == true) {
    // //         int blueCount = 0;
    // //         auto objects = vision.get_all_objects();
	// // 		for (const auto& object: objects) {
	// // 			if(object.id == 0) {blueCount++;};
	// // 		}            
    // //         scoredSeven = blueCount >= 7;
    // //         timeout++;
    // //         if(scoredSeven) {descore.set_value(true);}
    // //         pros::delay(20);
    // //     }
    // // });
    // pros::delay(5000);
    // return;
    
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
    
    // chassis.turnToPoint(64, 49, 500, {}, true);
    chassis.moveToPoint(64, 48, 2000, {.forwards=true, .maxSpeed=55, .minSpeed=55}, false);
    pros::delay(400);
    left_mg.move(-50);
    right_mg.move(-50);
    pros::delay(100);
    left_mg.move(45);
    right_mg.move(45);
    pros::delay(300);

    pros::Task align_score3{[=]{
        while(back_dist.get() > 90) { pros::delay(50);}
        lower.move(127);
        upper.move(127);
    }}; 
    chassis.moveToPoint(25, 49, 2000, {.forwards=false, .maxSpeed=80});
    pros::delay(2000);
    lower.move(127);
    upper.move(127);

    chassis.turnToHeading(90, 500, {}, false);

    chassis.setPose(positionFromRaycast(front_dist.get()*MM_TO_IN, FRONT_DIST_OFFSET, EAST),positionFromRaycast(left_dist.get()*MM_TO_IN, LEFT_DIST_OFFSET, NORTH),chassis.getPose().theta);

    // lower.move(127);
    // upper.move(127);

    matchload.set_value(false);

    pros::Task roller_hold2([&] {
        while(roller_dist.get() > 50) {
            upper.move(50);
            pros::delay(20);
        }
        upper.move(0);
    });

    chassis.moveToPose(60, 24, 170, 2000, {.horizontalDrift=15, .minSpeed=100}, false);
    // chassis.turnToHeading(135, 500,{},false);

    left_mg.move(60);
    right_mg.move(60);
    pros::delay(3700);

    chassis.turnToHeading(90, 1000, {}, false);
    chassis.setPose(positionFromRaycast(front_dist.get()*MM_TO_IN, FRONT_DIST_OFFSET, EAST), positionFromRaycast(right_dist.get()*MM_TO_IN, RIGHT_DIST_OFFSET, SOUTH), chassis.getPose().theta);

    pros::Task align_score4{[=]{
        while(back_dist.get() > 90) { pros::delay(50);}
        lower.move(127);
        upper.move(127);
    }}; 

    chassis.moveToPoint(42, -48, 2000, {.forwards=false}, true);
    chassis.turnToHeading(90, 500, {}, false);
    chassis.setPose(positionFromRaycast(front_dist.get()*MM_TO_IN, FRONT_DIST_OFFSET, EAST), positionFromRaycast(right_dist.get()*MM_TO_IN, RIGHT_DIST_OFFSET, SOUTH), chassis.getPose().theta);
    
    chassis.moveToPoint(25, -49, 2500, {.forwards=false, .maxSpeed=60});
    matchload.set_value(true);
    // chassis.swingToHeading(90, lemlib::DriveSide::RIGHT, 500);
    pros::delay(700);
    lower.move(127);
    upper.move(127);

    chassis.moveToPoint(48, -48, 1000, {.maxSpeed=45, .minSpeed=45, .earlyExitRange=5});


    // chassis.turnToHeading(80, 500);
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

    chassis.moveToPoint(64, -49, 2000, {.forwards=true, .maxSpeed=55, .minSpeed=55},false);
    pros::delay(400);
    left_mg.move(-50);
    right_mg.move(-50);
    pros::delay(100);
    left_mg.move(45);
    right_mg.move(45);
    pros::delay(300);

    lower.move(0);
    // level.set_value(false);

    chassis.moveToPoint(56, -48, 1000, {.forwards=false, .minSpeed=50, .earlyExitRange=5});
    chassis.turnToHeading(190, 1000, {.minSpeed=50, .earlyExitRange=5});
    

    chassis.moveToPose(24, -68,270, 1000, {.forwards=true, .horizontalDrift=15, .maxSpeed=127, .minSpeed=50});
    matchload.set_value(false);
    // chassis.turnToHeading(270, 200);
    lower.move(127);
    upper.move(127);

    

    // lower.move(127);
    chassis.moveToPoint(-24, -72, 3000, {.forwards=true, .minSpeed=20, .earlyExitRange=5});

    // chassis.turnToHeading(135, 500);

    // chassis.moveToPoint(10, -10, 2500, {.forwards=false, .maxSpeed=70}, false);
    // lower.move(0);
    // upper.move(64);
    // matchload.set_value(false);
    // pros::delay(800);
    // lower.move(0);
    // upper.move(0);

    // chassis.moveToPoint(24, -24, 3000, {.forwards=true});
    // chassis.turnToHeading(270, 1000);
    chassis.moveToPoint(-42, -55, 2000, {.forwards=true, .minSpeed=5, .earlyExitRange=1});
    matchload.set_value(true);

    // chassis.turnToHeading(270,500);

    // pros::Task discard_blue ([&] {
    //     int timeout = 0;
    //     bool redDetected = false;
    //     while (timeout <= 50 && redDetected == false) {
    //         upper.move(100);
    //         lower.move(100);

    //         if(fabs(roller_optical.get_hue() - 15) < 15){
    //             redDetected = true;
    //         }
    //         timeout++;
    //         pros::delay(20);
    //     }
    //     lower.move(0);
    //     upper.move(0);
    // });

    // pros::delay(1000);
    
    level.set_value(true);

    // lower.move(127);

    chassis.turnToHeading(270, 1000, {}, false);
    chassis.setPose(positionFromRaycast(front_dist.get()*MM_TO_IN, FRONT_DIST_OFFSET, WEST), positionFromRaycast(left_dist.get()*MM_TO_IN, LEFT_DIST_OFFSET, SOUTH), chassis.getPose().theta);

    // pros::Task align_score5{[=]{
    //     while(back_dist.get() > 90) { pros::delay(50);}
    //     lower.move(127);
    //     upper.move(127);
    // }}; 
    
    // chassis.moveToPoint(-25, -48, 1500, {.forwards=false, .maxSpeed=60});
    // matchload.set_value(true);
    // pros::delay(1500);

    // chassis.turnToHeading(270, 500, {}, false);
    // chassis.setPose(positionFromRaycast(front_dist.get()*MM_TO_IN, FRONT_DIST_OFFSET, WEST), positionFromRaycast(left_dist.get()*MM_TO_IN, LEFT_DIST_OFFSET, SOUTH), chassis.getPose().theta);
    upper.move(0);

    lower.move(127);

    pros::Task roller_hold4([&] {
        while(roller_dist.get() > 50) {
            upper.move(50);
            pros::delay(20);
        }
        upper.move(0);
    });
    chassis.moveToPoint(-64, -49, 2000, {.forwards=true, .maxSpeed=55, .minSpeed=55}, false);
    pros::delay(400);
    left_mg.move(-50);
    right_mg.move(-50);
    pros::delay(100);
    left_mg.move(45);
    right_mg.move(45);
    pros::delay(300);

    pros::Task align_score6{[=]{
        while(back_dist.get() > 90) { pros::delay(50);}
        lower.move(127);
        upper.move(127);
    }}; 

    chassis.turnToPoint(-25, -48.75, 500, {.forwards=false});
    chassis.moveToPoint(-25, -48.75, 3000, {.forwards=false, .maxSpeed=60});
    pros::delay(2000);
    // pros::delay(700);
    // lower.move(127);
    // upper.move(127);

    
    chassis.turnToHeading(270, 500, {}, false);
    chassis.setPose(positionFromRaycast(front_dist.get()*MM_TO_IN, FRONT_DIST_OFFSET, WEST), positionFromRaycast(left_dist.get()*MM_TO_IN, LEFT_DIST_OFFSET, SOUTH), chassis.getPose().theta);
    upper.move(0);
    
    matchload.set_value(false);

    // chassis.moveToPoint(-48, -48, 2000, {.forwards=true, .minSpeed=5, .earlyExitRange=1});
    // matchload.set_value(false);

    chassis.moveToPose(-60, -12, 350, 2000, {.horizontalDrift=15, .maxSpeed=60}, false);
    left_mg.move(0);
    right_mg.move(0);
    // chassis.turnToHeading(135, 500,{},false);
    lower.move(-127);
    left_mg.move(80);
    right_mg.move(80);
    pros::delay(950);

    // chassis.moveToPoint(-42, 0, 3000, {.maxSpeed=127, .minSpeed=5, .earlyExitRange=1}, false);
    // chassis.turnToHeading(270, 1000, {}, false);

    // left_mg.move(70);
    // right_mg.move(70);
    // pros::delay(2000);

    left_mg.move(0);
    right_mg.move(0);

}