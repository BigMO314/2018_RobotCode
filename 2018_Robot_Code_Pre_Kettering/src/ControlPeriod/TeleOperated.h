#pragma once
#include "MOLib.h"
#include "Dashboard.h"
#include "RobotMechanism/Arm.h"

#include "RobotMechanism/Drivetrain.h"
using namespace MOLib::Utilities::Measurements;

namespace ControlPeriod{
	class TeleOperated{
	public:
		//who tucha my code;
		TeleOperated(RobotMechanism::Drivetrain *rbt_Drivetrain,RobotMechanism::Arm *rbt_Arm,MOLib::XboxController *ctl_Driver,MOLib::XboxController *ctl_Operator){
			this->rbt_Drivetrain		= rbt_Drivetrain;
			this->rbt_Arm				= rbt_Arm;
			this->ctl_Driver			= ctl_Driver;
			this->ctl_Operator			= ctl_Operator;
		};
		~TeleOperated(){};


		//Update the values and handle driving + Arm
		void Update(){

			if(!rbt_Drivetrain->IsAnglePIDEnabled() && !rbt_Drivetrain->IsDistancePIDEnabled()){
				CheezyDrive(ctl_Driver->GetY(XboxController::kLeftHand), (Dashboard.Drivetrain.Inverted.Get() ? -ctl_Driver->GetX(XboxController::kRightHand) : ctl_Driver->GetX(XboxController::kRightHand)));
			}
			else if((rbt_Drivetrain->IsDistancePIDEnabled() && rbt_Drivetrain->IsAtDistance()) || (rbt_Drivetrain->IsAnglePIDEnabled() && rbt_Drivetrain->IsAtDistance())){
				rbt_Drivetrain->StopDrive();
			}

			if(ctl_Driver->GetBumperPressed(XboxController::kLeftHand)) { rbt_Arm->Shoot(); }
			if (ctl_Driver->GetStartButton() && ctl_Driver->GetBackButton()) { rbt_Drivetrain->DisableDistancePID(); rbt_Drivetrain->DisableAnglePID(); }

			(ctl_Operator->GetBumper(XboxController::kRightHand) ? rbt_Arm->EnableIntake() : (ctl_Operator->GetTriggerAxis(XboxController::kRightHand) ? rbt_Arm->ReverseIntake() : rbt_Arm->DisableIntake()));

			if(ctl_Operator->GetTriggerAxis(XboxController::kLeftHand)){
				if(ctl_Operator->GetAButtonPressed()) rbt_Arm->Point(RobotMechanism::Arm::PointPosition::kForwardCollect);
				else if(ctl_Operator->GetXButtonPressed()) rbt_Arm->Point(RobotMechanism::Arm::PointPosition::kForwardShoot);
			}
			else if(ctl_Operator->GetBumper(XboxController::kLeftHand)){
				if(ctl_Operator->GetAButtonPressed()) rbt_Arm->Point(RobotMechanism::Arm::PointPosition::kReverseCollect);
				else if(ctl_Operator->GetXButtonPressed()) rbt_Arm->Point(RobotMechanism::Arm::PointPosition::kReverseShoot);
			}
			else{
				if(ctl_Operator->GetStickButton(XboxController::kLeftHand)) rbt_Arm->SetArmPower(ctl_Operator->GetY(XboxController::kLeftHand) * 0.25);
				else rbt_Arm->Point(RobotMechanism::Arm::PointPosition::kUp);
			}

			if(ctl_Operator->GetStartButton() && ctl_Operator->GetBackButton()) rbt_Arm->ResetAngle();


		}
	private:
		RobotMechanism::Drivetrain		*rbt_Drivetrain;
		RobotMechanism::Arm				*rbt_Arm;

		MOLib::XboxController			*ctl_Driver;
		MOLib::XboxController			*ctl_Operator;

		/**
		 * @brief Simple steering; individual params control each side
		 * @param lPower Power for left side
		 * @param rPower Power for right side
		 */
		//Simple steering
		void TankDrive(float lPower, float rPower)		{ rbt_Drivetrain->SetDrive(lPower,rPower);										};


		/**
		 * @brief Steering with throttle
		 * @param throttle Throttle power
		 * @param steering Steering power
		 */
		//Easier steering
		void CheezyDrive(float throttle,float steering)	{ rbt_Drivetrain->SetDrive((throttle - steering), (throttle + steering));		};

	};
}
