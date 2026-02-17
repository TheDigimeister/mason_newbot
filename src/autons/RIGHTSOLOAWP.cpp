#include "drive.hpp"
#include "main.h"
#include "robot.hpp"
#include "utils.hpp"

void rightSoloAWP() {
    odom.set_value(false);
    level.set_value(true);
    descore.set_value(true);
    chassis.setPose(positionFromRaycast(right_dist.get() * MM_TO_IN, RIGHT_DIST_OFFSET, WEST), positionFromRaycast(front_dist.get() * MM_TO_IN, FRONT_DIST_OFFSET, SOUTH),180);
    matchload.set_value(true);

    lower.move(127);
    middle.move(127);
    pros::Task upper_roller_hold([] {
        while(roller_dist.get() > 50) {upper.move(50);}
        upper.move(0);
    });

    chassis.moveToPoint(-50, -50, 1200, {.maxSpeed=127});
    chassis.turnToHeading(270, 700, {.minSpeed=20}, false);
    chassis.setPose(positionFromRaycast(front_dist.get()*MM_TO_IN, FRONT_DIST_OFFSET, WEST), positionFromRaycast(left_dist.get()*MM_TO_IN, LEFT_DIST_OFFSET, SOUTH), chassis.getPose().theta);
    
    quintic::moveToPoint(chassis, -62, -48, 1000, {.forwards=true, .async=false});
    
    // move to goals
    upper.move(0);
    lower.move(127);

    pros::Task align_score2{[=]{
        while(back_dist.get() > 100) { pros::delay(50);}
        upper.move(127);
    }}; 



    quintic::moveToPoint(chassis, -24, -50, 1900, {.forwards=false, .async=false});
    matchload.set_value(false);
    pros::delay(400);
    upper.move(0);

    chassis.turnToHeading(0, 1000, {.minSpeed=20}, false);
    chassis.setPose(positionFromRaycast(left_dist.get()*MM_TO_IN, LEFT_DIST_OFFSET, WEST), positionFromRaycast(back_dist.get()*MM_TO_IN, BACK_DIST_OFFSET, SOUTH), chassis.getPose().theta);
    
    chassis.moveToPoint(-23,-24,500, {.maxSpeed=80, .minSpeed=80});
    chassis.moveToPoint(-23,15,1000, {.maxSpeed=80});
    pros::delay(1000);
    matchload.set_value(true);
    chassis.turnToHeading(305, 700, {}, false);
    level.set_value(false);
    left_mg.move(-50);
    right_mg.move(-50);
    pros::delay(500);
    left_mg.move(0);
    right_mg.move(0);
    pros::delay(200);

    // chassis.turnToPoint(0,0,700, {.forwards=false});
    // chassis.turnToPoint(-8, 6, 700, {.forwards=false});

    // chassis.moveToPoint(-8, 6, 950, {.forwards=false}, false);
    lower.move(127);
    middle.move(41);
    upper.move(41);
    pros::delay(950);
    upper.move(0);
    lower.move(127);
    middle.move(127);

    chassis.moveToPoint(-52, 40, 1000, {.maxSpeed=80});
    chassis.turnToHeading(270, 700, {.minSpeed=20}, false);
    level.set_value(true);
    chassis.setPose(positionFromRaycast(front_dist.get() * MM_TO_IN, FRONT_DIST_OFFSET, WEST), positionFromRaycast(right_dist.get() * MM_TO_IN, RIGHT_DIST_OFFSET, NORTH), chassis.getPose().theta);
    chassis.moveToPoint(-62, 47.5, 1200, {.forwards=true, .maxSpeed=60, .minSpeed=45}, true);
    pros::delay(1000);

    // long goal score
        pros::Task align_score3{[=]{
        while(back_dist.get() > 100) { pros::delay(50);}
        upper.move(127);
        lower.move(127);
    }};
    
    // chassis.turnToPoint(-25, 48, 500, {.forwards=false, .maxSpeed=80}, true);
    chassis.moveToPoint(-25, 49, 1300, {.forwards=false, .maxSpeed=80}, false);
    matchload.set_value(false);
    lower.move(127);
    middle.move(127);
    upper.move(127);
}