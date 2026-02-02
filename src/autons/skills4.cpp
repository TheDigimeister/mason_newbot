#include "drive.hpp"
#include "lemlib/chassis/chassis.hpp"
#include "main.h"
#include "pros/motors.h"
#include "robot.hpp"
#include "utils.hpp"

void skills_unsafe() {

    // odom.set_value(false);
    // odom_state = false;
    // chassis.setPose(positionFromRaycast(right_dist.get() * MM_TO_IN, RIGHT_DIST_OFFSET, WEST), positionFromRaycast(front_dist.get() * MM_TO_IN, FRONT_DIST_OFFSET, SOUTH),180);

    // lower.move(127);
    // level.set_value(false);
    // matchload.set_value(true);

    // // prep for 1st matchload
    // chassis.moveToPoint(-47, -47, 1500, {.forwards=true, .maxSpeed=127, .minSpeed=5, .earlyExitRange=3});
    // chassis.turnToHeading(270, 1000, {}, false);
    
    // // 1st matchload
    // chassis.moveToPoint(-59, -48, 1000, {.forwards=true, .maxSpeed=45, .minSpeed=45, .earlyExitRange=1}, false);
    // left_mg.move(-50);
    // right_mg.move(-50);
    // pros::delay(100);
    // left_mg.move(45);
    // right_mg.move(45);
    // pros::delay(900);
    // left_mg.move(-50);
    // right_mg.move(-50);
    // pros::delay(100);
    // left_mg.move(45);
    // right_mg.move(45);
    // pros::delay(900);

    // // prep for 1st long goal score
    // chassis.moveToPoint(-48, -55, 1000, {.forwards=false, .minSpeed=100, .earlyExitRange=5});
    // chassis.waitUntil(6);
    // matchload.set_value(false);
    // chassis.turnToHeading(110, 1000, {.direction=lemlib::AngularDirection::CW_CLOCKWISE});
    // chassis.moveToPoint(-36, -62, 2000, {.forwards=true, .maxSpeed=127, .minSpeed=100, .earlyExitRange=3});
    // lower.move(0);
    // chassis.moveToPoint(36, -59, 2000, {.forwards=true, .maxSpeed=127, .minSpeed=5, .earlyExitRange=5});
    // chassis.moveToPoint(48, -48, 1000, {.forwards=true, .maxSpeed=127, .minSpeed=5, .earlyExitRange=1});
    // chassis.turnToHeading(90, 1000, {}, false);
    // chassis.setPose(chassis.getPose().x, positionFromRaycast(right_dist.get()*MM_TO_IN,RIGHT_DIST_OFFSET,SOUTH),chassis.getPose().theta);
    
    // // 1st long goal score
    // pros::Task align_score1([&] {
    //     int timeout = 0;
    //     int count = 0;
    //     while (long_goal_check.get() > 120 && timeout <= 200) {
    //         pros::delay(30);
    //         timeout++;
    //     }
    //     while(count <= 100 && timeout <= 200) {
    //         upper.move(127);
    //         lower.move(127);
    //         count +=1;
    //         pros::delay(30);
    //         timeout++;
    //     }
    //     upper.move(-127);
    //     pros::delay(100);
    //     upper.move(0);
    // });

    // chassis.moveToPoint(25, -48.75, 3000, {.forwards=false, .maxSpeed=50, .minSpeed=50, .earlyExitRange=1});
    // pros::delay(3000);
    // lower.move(127);
    // chassis.turnToHeading(90, 1000, {}, false);
    // chassis.setPose(chassis.getPose().x, positionFromRaycast(right_dist.get()*MM_TO_IN,RIGHT_DIST_OFFSET,SOUTH),chassis.getPose().theta);

    // matchload.set_value(true);

    // // 2nd matchload
    // chassis.moveToPoint(59, -47.75, 2000, {.forwards=true, .maxSpeed=45, .minSpeed=45, .earlyExitRange=1}, false);
    // left_mg.move(-50);
    // right_mg.move(-50);
    // pros::delay(100);
    // left_mg.move(45);
    // right_mg.move(45);
    // pros::delay(900);
    // left_mg.move(-50);
    // right_mg.move(-50);
    // pros::delay(100);
    // left_mg.move(45);
    // right_mg.move(45);
    // pros::delay(900);
    // lower.move(0);
    
    // // 2nd long goal score
    // pros::Task align_score2([&] {
    //     int timeout = 0;
    //     int count = 0;
    //     while (long_goal_check.get() > 120 && timeout <= 200) {
    //         pros::delay(30);
    //         timeout++;
    //     }
    //     while(count <= 100 && timeout <= 200) {
    //         upper.move(127);
    //         lower.move(127);
    //         count +=1;
    //         pros::delay(30);
    //         timeout++;
    //     }
    //     upper.move(-127);
    //     pros::delay(100);
    //     upper.move(0);
    // });
    
    // chassis.moveToPoint(24, -48.5, 3000, {.forwards=false, .maxSpeed=60, .minSpeed=60, .earlyExitRange=1});
    // pros::delay(3000);
    // lower.move(127);
    // chassis.turnToHeading(90, 1000, {}, false);
    // chassis.setPose(chassis.getPose().x, positionFromRaycast(right_dist.get()*MM_TO_IN,RIGHT_DIST_OFFSET,SOUTH),chassis.getPose().theta);
    // // matchload.set_value(false);

    
    // // prep for 3rd matchload
    // chassis.moveToPoint(47, 46, 2000, {.forwards=true, .maxSpeed=127, .minSpeed=5, .earlyExitRange=3});
    // chassis.waitUntil(72);
    // matchload.set_value(true);
    // chassis.turnToHeading(90, 1000);
    
    // // 3rd matchload
    // chassis.moveToPoint(59, 48, 1000, {.forwards=true, .maxSpeed=45, .minSpeed=45, .earlyExitRange=1}, false);
    // left_mg.move(-50);
    // right_mg.move(-50);
    // pros::delay(100);
    // left_mg.move(45);
    // right_mg.move(45);
    // pros::delay(900);
    // left_mg.move(-50);
    // right_mg.move(-50);
    // pros::delay(100);
    // left_mg.move(45);
    // right_mg.move(45);
    // pros::delay(900);    
    // lower.move(0);
    // matchload.set_value(false);

    // // prep for 3rd long goal score
    // chassis.moveToPoint(36, 59, 2000, {.forwards=false, .maxSpeed=127, .minSpeed=100, .earlyExitRange=5});
    // chassis.turnToHeading(90, 1000);
    // chassis.moveToPoint(-36, 60, 2000, {.forwards=false, .maxSpeed=127, .minSpeed=5, .earlyExitRange=3});
    // chassis.moveToPoint(-44, 48, 1000, {.forwards=false, .maxSpeed=127, .minSpeed=5, .earlyExitRange=1});
    // chassis.turnToHeading(270, 1000, {}, false);
    // chassis.setPose(chassis.getPose().x, positionFromRaycast(right_dist.get()*MM_TO_IN,RIGHT_DIST_OFFSET,NORTH),chassis.getPose().theta);

    
    // // 3rd long goal score
    // pros::Task align_score3([&] {
    //     int timeout = 0;
    //     int count = 0;
    //     while (long_goal_check.get() > 120 && timeout <= 200) {
    //         pros::delay(30);
    //         timeout++;
    //     }
    //     while(count <= 100 && timeout <= 200) {
    //         upper.move(127);
    //         lower.move(127);
    //         count +=1;
    //         pros::delay(30);
    //         timeout++;
    //     }
    //     upper.move(-127);
    //     pros::delay(100);
    //     upper.move(0);
    // });
    
    // chassis.moveToPoint(-25, 49, 3000, {.forwards=false, .maxSpeed=50, .minSpeed=50, .earlyExitRange=1});
    // pros::delay(3000);
    // lower.move(127);
    // chassis.turnToHeading(270, 1000, {}, false);
    // matchload.set_value(true);
    
    // // 4th matchload
    // chassis.moveToPoint(-59, 48.8, 2000, {.forwards=true, .maxSpeed=45, .minSpeed=45, .earlyExitRange=1}, false);
    // left_mg.move(-50);
    // right_mg.move(-50);
    // pros::delay(100);
    // left_mg.move(45);
    // right_mg.move(45);
    // pros::delay(900);
    // left_mg.move(-50);
    // right_mg.move(-50);
    // pros::delay(100);
    // left_mg.move(45);
    // right_mg.move(45);
    // pros::delay(900);
    // lower.move(0);
    
    // // 4th long goal score
    // pros::Task align_score4([&] {
    //     int timeout = 0;
    //     int count = 0;
    //     while (long_goal_check.get() > 120 && timeout <= 200) {
    //         pros::delay(30);
    //         timeout++;
    //     }
    //     while(count <= 100 && timeout <= 200) {
    //         upper.move(127);
    //         lower.move(127);
    //         count +=1;
    //         pros::delay(30);
    //         timeout++;
    //     }
    //     upper.move(-127);
    //     pros::delay(100);
    //     upper.move(0);
    // });
    
    // chassis.moveToPoint(-25, 49, 3000, {.forwards=false, .maxSpeed=50, .minSpeed=50, .earlyExitRange=1});
    // pros::delay(3000);
    // lower.move(127);
    // matchload.set_value(false);
    // chassis.turnToHeading(270, 1000, {}, false);
    // chassis.setPose(chassis.getPose().x, positionFromRaycast(right_dist.get()*MM_TO_IN,RIGHT_DIST_OFFSET,NORTH),chassis.getPose().theta);
    chassis.setPose(positionFromRaycast(front_dist.get()*MM_TO_IN,FRONT_DIST_OFFSET,WEST), positionFromRaycast(right_dist.get()*MM_TO_IN,RIGHT_DIST_OFFSET,NORTH),270);

    lower.move(127);

    chassis.moveToPoint(-48, 36, 2000, {.forwards=true, .maxSpeed=127, .minSpeed=50, .earlyExitRange=5}, true);
    chassis.turnToPoint(-24, 24, 1000, {.maxSpeed=127, .minSpeed=5, .earlyExitRange=3}, false);
    chassis.moveToPoint(-24, 24, 2000, {.forwards=true, .maxSpeed=127, .minSpeed=127, .earlyExitRange=5}, true);
    chassis.moveToPoint(0, 22, 2000, {.forwards=true, .maxSpeed=127, .minSpeed=127, .earlyExitRange=5}, true);
    // chassis.turnToPoint(24, 24, 500);
    chassis.moveToPoint(24, 22, 2000, {.forwards=true, .maxSpeed=127, .minSpeed=127, .earlyExitRange=5}, true);
    chassis.moveToPoint(48, 36, 2000, {.forwards=true, .maxSpeed=127, .minSpeed=50, .earlyExitRange=5}, true);
    chassis.moveToPoint(63, 16, 2000, {.forwards=true, .maxSpeed=127, .minSpeed=5, .earlyExitRange=1}, false);
    
    // chassis.turnToHeading(150, 500, {}, false);

    odom.set_value(true);
    matchload.set_value(true);
    pros::delay(700);
    left_mg.move(80);
    right_mg.move(80);
    pros::delay(1300);
    matchload.set_value(false);
    pros::delay(1000);
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_COAST);
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_COAST);
    // left_mg.move(-20);
    // right_mg.move(-20);
    // pros::delay(1000);
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_BRAKE);
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_BRAKE);
    chassis.turnToHeading(270, 1000, {}, false);
    chassis.setPose(positionFromRaycast(back_dist.get()*MM_TO_IN, BACK_DIST_OFFSET, EAST), positionFromRaycast(left_dist.get()*MM_TO_IN,LEFT_DIST_OFFSET,SOUTH),chassis.getPose().theta);
    odom.set_value(false);
    pros::delay(100);

    chassis.moveToPoint(24, -24, 2000, {.forwards=true, .maxSpeed=127, .minSpeed=5, .earlyExitRange=1}, true);
    chassis.turnToPoint(8, -8, 500, {.forwards=false});
    chassis.moveToPoint(8, -8, 2000, {.forwards=false, .maxSpeed=127, .minSpeed=5, .earlyExitRange=1}, false);
    
    level.set_value(true);
    pros::delay(200);
    upper.move(64);
    pros::delay(1000);
    upper.move(0);
    lower.move(-127);
    level.set_value(false);
    
    chassis.moveToPoint(24, -24, 2000, {.forwards=true, .maxSpeed=127, .minSpeed=5, .earlyExitRange=5}, true);
    chassis.moveToPoint(-24, -24, 2000, {.forwards=true, .maxSpeed=127, .minSpeed=127, .earlyExitRange=5}, true);
    chassis.moveToPoint(-57, -24, 2000, {.forwards=true, .maxSpeed=127, .minSpeed=5, .earlyExitRange=5}, true);
    chassis.moveToPoint(-63, -16, 2000, {.forwards=true, .maxSpeed=127, .minSpeed=5, .earlyExitRange=1}, false);
    // chassis.turnToHeading(-30, 500, {}, false);

    odom.set_value(true);
    matchload.set_value(true);
    pros::delay(700);
    left_mg.move(80);
    right_mg.move(80);
    pros::delay(600);
    matchload.set_value(false);
    pros::delay(700);
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_COAST);
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_COAST);
    // left_mg.move(-20);
    // right_mg.move(-20);
    // pros::delay(1000);
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_BRAKE);
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_BRAKE);
    chassis.turnToHeading(90, 1000, {}, false);



    // back up
    // lower.move(-127);
    // chassis.moveToPoint(-50, 46, 2000, {.forwards=true, .maxSpeed=127, .minSpeed=5, .earlyExitRange=5});
    // chassis.moveToPoint(-62, 17, 2000, {.forwards=true, .maxSpeed=127, .minSpeed=5, .earlyExitRange=5});
    // chassis.turnToHeading(190, 200, {}, false);
    // odom.set_value(true);
    // matchload.set_value(true);
    // pros::delay(200);
    // left_mg.move(80);
    // right_mg.move(80);
    // pros::delay(500);
    // matchload.set_value(false);
    // pros::delay(700);
    // chassis.setBrakeMode(pros::E_MOTOR_BRAKE_COAST);
    // chassis.setBrakeMode(pros::E_MOTOR_BRAKE_COAST);
    // left_mg.move(-20);
    // right_mg.move(-20);
    // pros::delay(1000);
    // chassis.setBrakeMode(pros::E_MOTOR_BRAKE_BRAKE);
    // chassis.setBrakeMode(pros::E_MOTOR_BRAKE_BRAKE);
    // chassis.turnToHeading(90, 1000, {}, false);
    // pros::delay(500);
    // return;
    // left_mg.move(80);
    // right_mg.move(80);
    // pros::delay(700);
    // left_mg.move(0);
    // right_mg.move(0);
    // odom.set_value(false);
    // pros::delay(200);
    // chassis.turnToHeading(90, 200, {}, false);
    // chassis.setPose(positionFromRaycast(back_dist.get()*MM_TO_IN, BACK_DIST_OFFSET, WEST), positionFromRaycast(right_dist.get()*MM_TO_IN,RIGHT_DIST_OFFSET,SOUTH),chassis.getPose().theta);
    // lower.move(127);

    // chassis.turnToPoint(-24, -24, 500);
    // chassis.moveToPoint(-24, -24, 2000, {.forwards=true, .maxSpeed=127, .minSpeed=50, .earlyExitRange=5});
    // chassis.moveToPoint(0, -36, 2000, {.forwards=true, .maxSpeed=127, .minSpeed=100, .earlyExitRange=5});
    // chassis.moveToPoint(24, -24, 2000, {.forwards=true, .maxSpeed=127, .minSpeed=5, .earlyExitRange=1});

    // chassis.turnToPoint(8, -8, 1000, {.forwards=false});
    // chassis.moveToPoint(8, -8, 2000, {.forwards=false, .maxSpeed=50, .minSpeed=50, .earlyExitRange=5}, false);
    // level.set_value(true);
    // pros::delay(200);
    // upper.move(64);
    // pros::delay(1000);
    // upper.move(0);
    // lower.move(-127);
    // level.set_value(false);
    // // descore.set_value(true);
    
    // // parking
    // chassis.moveToPoint(42, -42, 2000, {.forwards=true, .maxSpeed=127, .minSpeed=30, .earlyExitRange=5});
    // chassis.moveToPoint(63, -16, 2000, {.forwards=true, .maxSpeed=127, .minSpeed=5, .earlyExitRange=5});
    // chassis.turnToHeading(10, 200, {}, false);
    // odom.set_value(true);
    // matchload.set_value(true);
    // pros::delay(200);
    // left_mg.move(80);
    // right_mg.move(80);
    // pros::delay(500);
    // matchload.set_value(false);
    // pros::delay(500);
    // chassis.setBrakeMode(pros::E_MOTOR_BRAKE_COAST);
    // chassis.setBrakeMode(pros::E_MOTOR_BRAKE_COAST);
    // left_mg.move(-20);
    // right_mg.move(-20);
    // pros::delay(1000);
    // chassis.setBrakeMode(pros::E_MOTOR_BRAKE_BRAKE);
    // chassis.setBrakeMode(pros::E_MOTOR_BRAKE_BRAKE);
    // chassis.turnToHeading(270, 1000, {.direction=lemlib::AngularDirection::CCW_COUNTERCLOCKWISE}, false);
    // // left_mg.move(-100);
    // // right_mg.move(-100);
    // // pros::delay(1000);
    // left_mg.move(-20);
    // right_mg.move(-20);
    // pros::delay(1000);
    // left_mg.move(0);
    // right_mg.move(0);

    // // // back up
    // // upper.move(0);
    // // lower.move(127);
    // // chassis.moveToPoint(-38, 38, 2000, {.forwards=true, .maxSpeed=127, .minSpeed=5, .earlyExitRange=5});
    
    // // // take NW trio
    // // chassis.turnToPoint(-20, 20, 1000);
    // // chassis.moveToPoint(-20, 20, 2000, {.forwards=true, .maxSpeed=127, .minSpeed=30, .earlyExitRange=5});
    
    // // // take NE trio
    // // chassis.moveToPoint(0, 28, 2000, {.forwards=true, .maxSpeed=127, .minSpeed=100, .earlyExitRange=5});
    // // chassis.moveToPoint(18, 28, 2000, {.forwards=true, .maxSpeed=127, .minSpeed=50, .earlyExitRange=5});
    // // chassis.moveToPoint(28, 18, 2000, {.forwards=true, .maxSpeed=127, .minSpeed=127, .earlyExitRange=5});
    
    // // // take SE trio
    // // chassis.moveToPoint(24, -24, 2000, {.forwards=true, .maxSpeed=127, .minSpeed=5, .earlyExitRange=1});
    // // chassis.turnToPoint(8, -8, 1000, {.forwards=false});
    // // chassis.moveToPoint(8, -8, 2000, {.forwards=false, .maxSpeed=50, .minSpeed=50, .earlyExitRange=5}, false);
    // // level.set_value(true);
    // // pros::delay(200);
    // // upper.move(64);
    // // pros::delay(2500);
    // // upper.move(0);
    // // lower.move(127);
    // // level.set_value(false);
    // // descore.set_value(true);
    
    // // // parking
    // // // chassis.moveToPoint(36, 0, 2000, {.forwards=true, .maxSpeed=127, .minSpeed=30, .earlyExitRange=5});
    // // // chassis.turnToHeading(90, 1000, {}, false);
    // // chassis.moveToPoint(42, -42, 2000, {.forwards=true, .maxSpeed=127, .minSpeed=30, .earlyExitRange=5});
    // // chassis.moveToPoint(63, -21, 2000, {.forwards=true, .maxSpeed=127, .minSpeed=5, .earlyExitRange=5});
    // // chassis.turnToHeading(20, 1000, {}, false);
    // // odom.set_value(true);
    // // pros::delay(200);
    // // left_mg.move(127);
    // // right_mg.move(127);
    // // pros::delay(2000);
    // // left_mg.move(0);
    // // right_mg.move(0);
    // // chassis.turnToHeading(-20, 1000, {.direction=lemlib::AngularDirection::CCW_COUNTERCLOCKWISE}, false);
    // // left_mg.move(-100);
    // // right_mg.move(-100);
    // // pros::delay(1000);
    // // left_mg.move(0);
    // // right_mg.move(0);

}