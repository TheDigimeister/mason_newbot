#include "main.h"
#include "quintic_motion.hpp"

void pidTune(){
    chassis.setPose(0,0,0);

    // chassis.moveToPoint(0, 24, 5000);
    // chassis.turnToPoint(0, 0, 5000);
    // chassis.moveToPoint(0, 0, 5000);
    // chassis.turnToPoint(0, 24, 5000);
    // chassis.moveToPoint(0, 24, 5000);
    // chassis.turnToPoint(0, 0, 5000);
    // chassis.moveToPoint(0, 0, 5000);
    // chassis.turnToPoint(0, 24, 5000);
    // chassis.moveToPoint(0, 24, 5000);
    // chassis.turnToPoint(0, 0, 5000);
    // chassis.moveToPoint(0, 0, 5000);
    // chassis.turnToPoint(0, 24, 5000);

    // left_mg.move(64);
    // right_mg.move(64);
    // pros::delay(200);
    // float startY = chassis.getPose().y;
    // left_mg.move(127);
    // right_mg.move(127);
    // pros::delay(1000);
    // master.print(0,0,"S: %.3f, E: %.3f", startY, chassis.getPose().y);
    // left_mg.move(64);
    // right_mg.move(64);
    // pros::delay(200);
    // left_mg.move(0);
    // right_mg.move(0);
    // quintic::moveToPose(chassis,-24,66,270, 2000, {.async=false});
    // quintic::moveToPoint(chassis,48,48,1750, {.async=false});
    // quintic::moveToPoint(chassis,48,0,1750, {.async=false});
    // quintic::moveToPoint(chassis,0,0,1750, {.async=false});

    quintic::moveToPoint(chassis,0,48,3000, {.async=false, .settleRange = 5, .settleTimeout = 0});
    quintic::moveToPoint(chassis,48,48,3000, {.async=false, .settleRange = 5, .settleTimeout = 0});
    quintic::moveToPoint(chassis,48,0,3000, {.async=false, .settleRange = 5, .settleTimeout = 0});
    quintic::moveToPoint(chassis,0,0,3000, {.async=false, .settleRange = 5, .settleTimeout = 0});

    quintic::moveToPoint(chassis,0,48,3000, {.async=false, .settleRange = 5, .settleTimeout = 0});
    quintic::moveToPoint(chassis,48,48,3000, {.async=false, .settleRange = 5, .settleTimeout = 0});
    quintic::moveToPoint(chassis,48,0,3000, {.async=false, .settleRange = 5, .settleTimeout = 0});
    quintic::moveToPoint(chassis,0,0,3000, {.async=false});

    // quintic::moveToPose(chassis,-24,48,270,4000,{.async=false});
    // quintic::moveToPoint(chassis,0,24,5000);
    // quintic::moveToPoint(chassis,24,24,5000);
    // quintic::moveToPoint(chassis,24,0,5000);
    // quintic::moveToPoint(chassis,0,0,5000);

    // quintic::moveToPoint(chassis,0,24,5000);
    // quintic::moveToPoint(chassis,24,24,5000);
    // quintic::moveToPoint(chassis,24,0,5000);
    // quintic::moveToPoint(chassis,0,0,5000);

}