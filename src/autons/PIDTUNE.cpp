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

    // left_mg.move(12);
    // right_mg.move(12);
    // pros::delay(5000);
    // left_mg.move(0);
    // right_mg.move(0);
    quintic::moveToPose(chassis,-24,66,270, 2000, {.async=false});
    // quintic::moveToPoint(chassis,48,48,1750, {.async=false});
    // quintic::moveToPoint(chassis,48,0,1750, {.async=false});
    // quintic::moveToPoint(chassis,0,0,1750, {.async=false});

    // quintic::moveToPoint(chassis,0,48,1750, {.async=false});
    // quintic::moveToPoint(chassis,48,48,1750, {.async=false});
    // quintic::moveToPoint(chassis,48,0,1750, {.async=false});
    // quintic::moveToPoint(chassis,0,0,1750, {.async=false});

    // quintic::moveToPoint(chassis,0,48,1750, {.async=false});
    // quintic::moveToPoint(chassis,48,48,1750, {.async=false});
    // quintic::moveToPoint(chassis,48,0,1750, {.async=false});
    // quintic::moveToPoint(chassis,0,0,1750, {.async=false});

    // quintic::moveToPose(chassis,-24,48,270,3000,{.async=false, .lead=0.9});
    // quintic::moveToPoint(chassis,0,24,5000);
    // quintic::moveToPoint(chassis,24,24,5000);
    // quintic::moveToPoint(chassis,24,0,5000);
    // quintic::moveToPoint(chassis,0,0,5000);

    // quintic::moveToPoint(chassis,0,24,5000);
    // quintic::moveToPoint(chassis,24,24,5000);
    // quintic::moveToPoint(chassis,24,0,5000);
    // quintic::moveToPoint(chassis,0,0,5000);

}