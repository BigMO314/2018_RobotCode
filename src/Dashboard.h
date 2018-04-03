#pragma once

#include "MOLib.h"

struct {
	struct {
		MOLib::Dashboard::Number		LeftScale{"Drivetrain/LeftScale"};
		struct {

			MOLib::Dashboard::Number	P{"Drivetrain/Distance/P"};
			MOLib::Dashboard::Number	I{"Drivetrain/Distance/I"};
			MOLib::Dashboard::Number	D{"Drivetrain/Distance/D"};
			MOLib::Dashboard::Number	Measurement{"Drivetrain/Distance/Measurement"};
			MOLib::Dashboard::Indicator	Enabled{"Drivetrain/Distance/Enabled"};
			MOLib::Dashboard::Indicator	OnTarget{"Drivetrain/Distance/OnTarget"};


		} Distance;

		struct {
			MOLib::Dashboard::Number	P{"Drivetrain/Angle/P"};
			MOLib::Dashboard::Number	I{"Drivetrain/Angle/I"};
			MOLib::Dashboard::Number	D{"Drivetrain/Angle/D"};
			MOLib::Dashboard::Number	Angle{"Drivetrain/Angle/Angle"};
			MOLib::Dashboard::Indicator	Enabled{"Drivetrain/Angle/Enabled"};
			MOLib::Dashboard::Indicator OnTarget{"Drivetrain/Angle/OnTarget"};
			MOLib::Dashboard::Number	MaxError{"Drivetrain/Angle/Max Error"};
		} Angle;

		struct {
			MOLib::Dashboard::Number	P{"Drivetrain/LimeLight/P"};
			MOLib::Dashboard::Number	I{"Drivetrain/LimeLight/I"};
			MOLib::Dashboard::Number	D{"Drivetrain/LimeLight/D"};
			MOLib::Dashboard::Number	Offset{"Drivetrain/LimeLight/Offset"};
			MOLib::Dashboard::Indicator	Enabled{"Drivetrain/LimeLight/Enabled"};
			MOLib::Dashboard::Indicator	OnTarget{"Drivetrain/LimeLight/OnTarget"};
		} LimeLight;
	} Drivetrain;

	struct {
		//If we have any
	} Autonomous;
	struct {
		struct {
			MOLib::Dashboard::Number	P{"Arm/Angle/P"};
			MOLib::Dashboard::Number	I{"Arm/Angle/I"};
			MOLib::Dashboard::Number	D{"Arm/Angle/D"};
			MOLib::Dashboard::Number	Angle{"Arm/Angle/Angle"};
			MOLib::Dashboard::Indicator Enabled{"Arm/Angle/Enabled"};
			MOLib::Dashboard::Indicator OnTarget{"Arm/Angle/OnTarget"};
		} Angle;
		MOLib::Dashboard::Indicator		ShooterLim{"Arm/ShooterLim"};

	} Arm;

	struct {
		MOLib::Dashboard::Checkbox		LightEnabled{"Vision/LightEnabled"};
	} Vision;

	struct {
		MOLib::Dashboard::Checkbox		TuningMode{"Misc/TuningMode"};
		MOLib::Dashboard::Checkbox		TestingMode{"Misc/TestingMode"};
		MOLib::Dashboard::Boolean		RunDistance{"Misc/RunDistance"};
		MOLib::Dashboard::Indicator		IsCompBot{"Misc/IsCompBot"};
	} Misc;
} Dashboard;
