#include "main.h"
#pragma once
#include "pros/adi.hpp"


#ifndef ROBOTH
#define ROBOTH

extern pros::Motor LF;
extern pros::Motor LM;
extern pros::Motor LB;
extern pros::Motor RF;
extern pros::Motor RM;
extern pros::Motor RB;
extern pros::Motor INTAKE;
extern pros::Motor INTAKE1;
extern pros::Motor INTAKE2;
extern pros::Rotation roto;
extern pros::Rotation roto2;
extern pros::MotorGroup intakes;

extern pros::Controller con;

extern pros::Imu imu;

extern pros::ADIDigitalOut descore;
extern pros::ADIDigitalOut descore2;
extern bool descoreToggle;
extern bool descore2Toggle;

extern pros::ADIDigitalOut dp1;
extern pros::ADIDigitalOut dp2;
extern bool dp1Toggle;
extern bool dp2Toggle;

extern pros::ADIDigitalOut scrapper;
extern bool scrapperToggle;


extern pros::Optical OpticalC;
extern pros::Distance DistanceLeft;
extern pros::Distance DistanceRight;

extern pros::ADIDigitalIn Autonselect;


#endif 
