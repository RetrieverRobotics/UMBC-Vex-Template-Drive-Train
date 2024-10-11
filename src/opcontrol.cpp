/**
 * \file opcontrol.cpp
 *
 * Contains user implemented opcontrol. User must use the
 * parameters to the opcontrol function when referencing
 * the master V5 controller or partner V5 controller.
 */

#include "api.h"
#include "pros/misc.h"
#include "pros/motors.h"
#include "pros/motors.hpp"
#include "umbc.h"

#include <cstdint>
#include <vector>

using namespace pros;
using namespace umbc;
using namespace std;

#define DRIVE_MOTOR_COUNT 6 // both 6 and 8 are valid numbers
#define DRIVE_L1 0
#define DRIVE_L2 1
#define DRIVE_L3 2
#define DRIVE_L4 3
#define DRIVE_R1 11
#define DRIVE_R2 12
#define DRIVE_R3 13
#define DRIVE_R4 14

#define GEARBOX_COLOR 'B' // set to the color that all gearboxes will be (R, G, or B)
#define RED_MOTOR_MULT 100
#define GREEN_MOTOR_MULT 400
#define BLUE_MOTOR_MULT 600

#define ANALOG_ABS_MAX 127.0

void umbc::Robot::opcontrol() {

    // nice names for controllers (do not edit)
    umbc::Controller* controller_master = this->controller_master;
    umbc::Controller* controller_partner = this->controller_partner; // do NOT get input from this controller

    // initialize motors and sensors
    std::vector<int8_t> leftTrainPorts;
    leftTrainPorts.assign({DRIVE_L1, DRIVE_L2, DRIVE_L3, DRIVE_L4});
    std::vector<int8_t> rightTrainPorts;
    rightTrainPorts.assign({DRIVE_R1, DRIVE_R2, DRIVE_R3, DRIVE_R4});
    if (DRIVE_MOTOR_COUNT == 6) {
        leftTrainPorts.pop_back();
        rightTrainPorts.pop_back();
    }
    Motor_Group leftTrain(leftTrainPorts);
    Motor_Group rightTrain(rightTrainPorts);
    int32_t gearMult;
    switch (GEARBOX_COLOR) {
        case 'R':
        case 'r':
            leftTrain.set_gearing(pros::E_MOTOR_GEAR_RED);
            rightTrain.set_gearing(pros::E_MOTOR_GEAR_RED);
            gearMult = RED_MOTOR_MULT;
            break;
        case 'G':
        case 'g':
            leftTrain.set_gearing(pros::E_MOTOR_GEAR_GREEN);
            rightTrain.set_gearing(pros::E_MOTOR_GEAR_GREEN);
            gearMult = GREEN_MOTOR_MULT;
            break;
        case 'B':
        case 'b':
        default:
            leftTrain.set_gearing(pros::E_MOTOR_GEAR_BLUE);
            rightTrain.set_gearing(pros::E_MOTOR_GEAR_BLUE);
            gearMult = BLUE_MOTOR_MULT;
    }

    while(1) {

        // implement opcontrols
        double steer = controller_master->get_analog(pros::E_CONTROLLER_ANALOG_LEFT_X) / ANALOG_ABS_MAX;
        double power = controller_master->get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y) / ANALOG_ABS_MAX;
        leftTrain.move_velocity((steer - power) * gearMult);
        rightTrain.move_velocity((power + steer) * gearMult);

        // required loop delay (do not edit)
        pros::Task::delay(this->opcontrol_delay_ms);
    }
}