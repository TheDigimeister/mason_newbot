#include "drive.hpp"
#include "main.h"
#include "robot.hpp"
#include "utils.hpp"

void skills3(){
    chassis.setPose(positionFromRaycast(left_dist.get()*MM_TO_IN, LEFT_DIST_OFFSET, WEST), positionFromRaycast(front_dist.get()*MM_TO_IN, FRONT_DIST_OFFSET, NORTH), 0);

    lower.move(127);

    // Path

    chassis.moveToPoint(-48, 48, 1000, {.maxSpeed=127, .minSpeed=5, .earlyExitRange=3}); // matchload 90 setup
    matchload.set_value(true);
    chassis.turnToHeading(270, 500);
    chassis.moveToPoint(-63, 48, 1000, {.minSpeed=60}); // matchload
    pros::delay(2000);
    chassis.moveToPoint(-48, 48, 1000, {.forwards=false, .minSpeed=127, .earlyExitRange=10}); // back up
    chassis.moveToPoint(-8, 8, 1500, {.forwards=false, .maxSpeed=127}, false); // score middle
    level.set_value(true);
    upper.move(60);
    pros::delay(700);
    upper.move(0);
    pros::delay(200);
    chassis.moveToPoint(-26, 24, 1000, {.minSpeed=127}); // back up
    level.set_value(false);
    matchload.set_value(false);
    chassis.turnToHeading(270, 500);
    chassis.moveToPoint(26, 32, 2000, {.forwards=false, .maxSpeed=127, .minSpeed=5, .earlyExitRange=2}); // prepare to long goal score
    chassis.turnToHeading(180, 500);
    chassis.moveToPoint(26, 40, 500, {.forwards=false, .minSpeed=127}, false);
    left_mg.move(-127);
    pros::delay(1000);
    right_mg.move(-127);
    pros::delay(500);
    chassis.turnToHeading(90, 500);
    chassis.setPose(chassis.getPose().x, positionFromRaycast(left_dist.get()*MM_TO_IN, LEFT_DIST_OFFSET, NORTH),chassis.getPose().theta);
    upper.move(127);
    pros::delay(1000);
    upper.move(0);
    matchload.set_value(true);
    chassis.moveToPoint(63, 48, 2000, {.maxSpeed=60, .minSpeed=60}); // matchload
    pros::delay(3500);
    lower.move(0);
    chassis.moveToPoint(26, 48, 1000, {.forwards=false}, false); // long goal score
    lower.move(127);
    upper.move(127);
    pros::delay(1100);
    upper.move(-127);
    pros::delay(200);
    upper.move(0);
    chassis.moveToPoint(38, 48, 2000, {.forwards=true, .maxSpeed=127}); // back up
    matchload.set_value(false);
    chassis.moveToPoint(38, -24, 2000, {.forwards=true, .maxSpeed=127}); // prep for middle goal score
    chassis.turnToPoint(7, -7, 500, {.forwards=false});
    chassis.moveToPoint(7, -7, 2000, {.forwards=false, .maxSpeed=127}, false); // middle goal score
    chassis.turnToHeading(135, 500);
    level.set_value(true);
    pros::delay(100);
    upper.move(88);
    pros::delay(2000);
    chassis.moveToPoint(40, -48, 2000, {.maxSpeed=127, .minSpeed=5, .earlyExitRange=2}); // prep for matchload
    upper.move(0);
    level.set_value(false);
    matchload.set_value(true);
    chassis.turnToHeading(90, 500);
    chassis.moveToPoint(63, -48, 2000, {.maxSpeed=80}); //  matchload
    chassis.moveToPoint(50, -48, 500);
    // chassis.moveToPoint(24, -24, 2000, {.forwards=false, .maxSpeed=127, .minSpeed=100, .earlyExitRange=5}); // prep for middle goal score
    chassis.turnToPoint(7, -7, 500, {.forwards=false});
    chassis.moveToPoint(7, -7, 2000, {.forwards=false, .maxSpeed=80}, false); // middle goal score
    chassis.turnToHeading(135, 500);
    level.set_value(true);
    pros::delay(100);
    upper.move(20);
    pros::delay(1000);
    upper.move(0);
    level.set_value(false);
    matchload.set_value(false);
    chassis.moveToPoint(48, -36, 2000, {.maxSpeed=80}, false); // back up
    chassis.turnToHeading(90, 1000);
    chassis.moveToPoint(-26, -24, 2000, {.forwards=false, .maxSpeed=80}); // prep for red left long goal score
    chassis.turnToHeading(0, 1000);
    // chassis.moveToPoint(-26, -40, 500, {.forwards=false, .maxSpeed=80}); // long goal anchor
    right_mg.move(-127);
    pros::delay(1000);
    left_mg.move(-127);
    pros::delay(500);
    chassis.turnToHeading(270, 500);
    chassis.setPose(chassis.getPose().x, positionFromRaycast(left_dist.get()*MM_TO_IN, LEFT_DIST_OFFSET, SOUTH),chassis.getPose().theta);
    upper.move(127);
    pros::delay(1000);
    upper.move(0);
    matchload.set_value(true);
    chassis.moveToPoint(-63, -48, 2000, {.maxSpeed=80}); // matchload
    chassis.moveToPoint(-26, -48, 2000, {.forwards=false, .minSpeed=100}, false); // long goal score
    upper.move(127);
    pros::delay(1000);
    upper.move(0);
    chassis.moveToPoint(-48, -48, 2000, {.maxSpeed=80}); // back up
    matchload.set_value(false);
    chassis.moveToPoint(-24, 24, 2000);
    chassis.turnToPoint(-7, 7, 500, {.forwards=false});
    chassis.moveToPoint(-7, 7, 2000, {.forwards=false},false);
    level.set_value(true);
    upper.move(30);
    pros::delay(2000);
    upper.move(0);
    level.set_value(false);
    chassis.moveToPoint(-57, 32, 2000);
    chassis.turnToHeading(215, 1000, {}, false);
    odom.set_value(true);
    pros::delay(100);
    left_mg.move(127);
    right_mg.move(127);
    pros::delay(1000);
    left_mg.move(0);
    right_mg.move(0);

}