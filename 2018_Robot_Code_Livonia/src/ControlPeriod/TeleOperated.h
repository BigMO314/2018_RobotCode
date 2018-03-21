#pragma once
#include "MOLib.h"
#include "Dashboard.h"
#include "RobotMechanism/Arm.h"

#include "RobotMechanism/Drivetrain.h"
using namespace MOLib::Utilities::Measurements;
using namespace MOLib::Utilities::Math;

namespace ControlPeriod{
	class TeleOperated{
	public:
		//Constructor
		TeleOperated(RobotMechanism::Drivetrain *rbt_Drivetrain,RobotMechanism::Arm *rbt_Arm,MOLib::XboxController *ctl_Driver,MOLib::XboxController *ctl_Operator){
			this->rbt_Drivetrain		= rbt_Drivetrain;
			this->rbt_Arm				= rbt_Arm;
			this->ctl_Driver			= ctl_Driver;
			this->ctl_Operator			= ctl_Operator;
		};
		~TeleOperated(){};


		//Update the values and handle driving, Arm,
		void Update(){

			if(ctl_Driver->GetYButtonPressed()) rbt_Drivetrain->GoToDistance(350.0);

			if(!rbt_Drivetrain->IsAnglePIDEnabled() && !rbt_Drivetrain->IsDistancePIDEnabled()){
				if (ctl_Driver->GetTriggerAxis(XboxController::kLeftHand)) {
					CheezyDrive(Sign(ctl_Driver->GetY(XboxController::kLeftHand)) * 0.4, Sign(ctl_Driver->GetX(XboxController::kRightHand)) * 0.2);
				}
				else { CheezyDrive(ctl_Driver->GetY(XboxController::kLeftHand), ctl_Driver->GetX(XboxController::kRightHand)); }
			}
			else if((rbt_Drivetrain->IsDistancePIDEnabled() && rbt_Drivetrain->IsAtDistance()) || (rbt_Drivetrain->IsAnglePIDEnabled() && rbt_Drivetrain->IsAtDistance())){
				rbt_Drivetrain->StopDrive();
			}

			if(!ctl_Driver->GetBumper(XboxController::kLeftHand) || !ctl_Driver->GetBumper(XboxController::kRightHand)) m_CanShoot = true;
			if(ctl_Driver->GetBumper(XboxController::kLeftHand) && ctl_Driver->GetBumper(XboxController::kRightHand) && m_CanShoot) {
				rbt_Arm->Shoot();
				m_CanShoot = false;
			}

			if (ctl_Driver->GetStartButton() && ctl_Driver->GetBackButton()) { rbt_Drivetrain->DisableDistancePID(); rbt_Drivetrain->DisableAnglePID(); }

			if(ctl_Operator->GetBumper(XboxController::kRightHand)) rbt_Arm->EnableIntake();
			else if(ctl_Operator->GetTriggerAxis(XboxController::kRightHand)) rbt_Arm->ReverseIntake();
			else if(ctl_Operator->GetStickButton(XboxController::kRightHand)) rbt_Arm->ShootIntake();
			else rbt_Arm->DisableIntake();

			if(ctl_Operator->GetBumper(XboxController::kLeftHand)){
		  		if(ctl_Operator->GetAButton()) rbt_Arm->Point(RobotMechanism::Arm::PointPosition::kForwardCollect);
				else if(ctl_Operator->GetXButton()) rbt_Arm->Point(RobotMechanism::Arm::PointPosition::kForwardSwitch);
				else if(ctl_Operator->GetBButton()) rbt_Arm->Point(RobotMechanism::Arm::PointPosition::kForwardMidCollect);
				else if(ctl_Operator->GetYButton()) rbt_Arm->Point(RobotMechanism::Arm::PointPosition::kForwardScale);
			}
			else if(ctl_Operator->GetTriggerAxis(XboxController::kLeftHand)){
				if(ctl_Operator->GetAButton()) rbt_Arm->Point(RobotMechanism::Arm::PointPosition::kReverseCollect);
				else if(ctl_Operator->GetXButton()) rbt_Arm->Point(RobotMechanism::Arm::PointPosition::kReverseSwitch);
				else if(ctl_Operator->GetBButton()) rbt_Arm->Point(RobotMechanism::Arm::PointPosition::kReverseMidCollect);
				else if(ctl_Operator->GetYButton()) rbt_Arm->Point(RobotMechanism::Arm::PointPosition::kReverseScale);
			}
			else{
				if(ctl_Operator->GetStickButton(XboxController::kLeftHand)) rbt_Arm->SetArmPower(ctl_Operator->GetY(XboxController::kLeftHand) * 0.6);
				else rbt_Arm->Point(RobotMechanism::Arm::PointPosition::kUp);
			}

			if(ctl_Operator->GetStartButton() && ctl_Operator->GetBackButton()) rbt_Arm->ResetAngle();


		}
	private:
		RobotMechanism::Drivetrain		*rbt_Drivetrain;
		RobotMechanism::Arm				*rbt_Arm;

		MOLib::XboxController			*ctl_Driver;
		MOLib::XboxController			*ctl_Operator;

		bool m_CanShoot = true;

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
