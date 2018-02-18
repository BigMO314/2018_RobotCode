#pragma once
#include "MOLib.h"
#include "RobotMechanism/Drivetrain.h"
#include "RobotMechanism/Elevator.h"
#include "RobotMechanism/Intake.h"
namespace ControlPeriod{
	class TeleOperated{
	public:
		TeleOperated(RobotMechanism::Drivetrain *rbt_Drivetrain,RobotMechanism::Elevator *rbt_Elevator,RobotMechanism::Intake *rbt_Intake,MOLib::XboxController *ctl_Driver,MOLib::XboxController *ctl_Operator){
			this->rbt_Drivetrain		= rbt_Drivetrain;
			this->rbt_Elevator			= rbt_Elevator;
			this->rbt_Intake			= rbt_Intake;
			this->ctl_Driver			= ctl_Driver;
			this->ctl_Operator			= ctl_Operator;
		};
		~TeleOperated(){};


		//Update the values and handle driving + Elevator
		void Update(){
			float m_steerscale=SmartDashboard::GetNumber("Steer Scale",0.6);
			float m_drivescale=SmartDashboard::GetNumber("Drive Scale",0.6);
			bool m_inverted=SmartDashboard::GetBoolean("Inverted",false);
			CheezyDrive(ctl_Driver->GetY(XboxController::kLeftHand)*m_drivescale,(m_inverted ? 0-ctl_Driver->GetX(XboxController::kRightHand)*m_steerscale : ctl_Driver->GetX(XboxController::kRightHand)*m_steerscale));

			(ctl_Operator->GetBumper(XboxController::kLeftHand) ? rbt_Intake->EnableIntake() : (ctl_Operator->GetBumper(XboxController::kRightHand) ? rbt_Intake->ReverseIntake() : rbt_Intake->DisableIntake()));
			(ctl_Operator->GetXButton() ? rbt_Intake->Extend() : rbt_Intake->Retract());
			(ctl_Operator->GetYButton() ? rbt_Intake->Lift() : rbt_Intake->Lower());
			(ctl_Operator->GetStartButton() ? rbt_Intake->Engage() : rbt_Intake->Disengage()); //TODO Work on Intake automation?

			MoveElevator((ctl_Operator->GetAButton() ? SmartDashboard::GetNumber("Elevator Speed",0.0) : (ctl_Operator->GetBButton() ? -SmartDashboard::GetNumber("Elevator Speed",0.0) : 0.0)));
		}
	private:
		RobotMechanism::Drivetrain		*rbt_Drivetrain;
		RobotMechanism::Elevator		*rbt_Elevator;
		RobotMechanism::Intake			*rbt_Intake;

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

		//make the elevator go nyoom
		void MoveElevator(float ref_power)				{ rbt_Elevator->SetElevatorPower(ref_power);									}
		//TODO: Finish function here ^
		//One button for each stage?
		//a for top, b for bottom maybe

	};
}
