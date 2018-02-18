#pragma once
#include "MOLib.h"
#include "Configuration/Goal.h"
#include "RobotMechanism/Drivetrain.h"
#include "RobotMechanism/Elevator.h"
#include "RobotMechanism/Intake.h"

namespace ControlPeriod{
	class Autonomous{
	public:
		Autonomous(RobotMechanism::Drivetrain *rbt_Drivetrain,RobotMechanism::Elevator *rbt_Elevator,RobotMechanism::Intake *rbt_Intake){
			this->rbt_Drivetrain	= rbt_Drivetrain;
			this->rbt_Elevator		= rbt_Elevator;
			this->rbt_Intake		= rbt_Intake;
		}
		~Autonomous();

		void CrossLine(){
			rbt_Drivetrain->SetDrive(0.5,0.5);
		}
		void DoNothing(){
			rbt_Drivetrain->SetDrive(0.0,0.0);
		}
		void Update(){
			AutonStates m_SelectedAuton=chs_Auton->GetSelected();
			if (m_SelectedAuton==AutonStates::kDoNothing){
				DoNothing();
			} else if(m_SelectedAuton==AutonStates::kCrossLine){
				CrossLine();
			} else {
				DoNothing();
			}

			rbt_Drivetrain->Update();
		}
	private:
		RobotMechanism::Drivetrain				*rbt_Drivetrain;
		RobotMechanism::Elevator				*rbt_Elevator;
		RobotMechanism::Intake					*rbt_Intake;

		enum class AutonStates {kDoNothing,kCrossLine};

		WPILib::SendableChooser<AutonStates> *chs_Auton=new WPILib::SendableChooser<AutonStates>();
	};
}
