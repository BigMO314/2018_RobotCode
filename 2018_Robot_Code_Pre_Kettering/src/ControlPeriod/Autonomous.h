#pragma once
#include "MOLib.h"
#include "Dashboard.h"
#include "Configuration/Goal.h"
#include "RobotMechanism/Arm.h"
#include "RobotMechanism/Drivetrain.h"

using namespace MOLib::Utilities::Measurements;

namespace ControlPeriod{
	class Autonomous{
	public:
		Autonomous(RobotMechanism::Drivetrain *rbt_Drivetrain,RobotMechanism::Arm *rbt_Arm){
			this->rbt_Drivetrain	= rbt_Drivetrain;
			this->rbt_Arm			= rbt_Arm;

			chs_Auton.AddDefault("Do Nothing",AutonStates::kDoNothing);
			chs_Auton.AddObject("Cross Line",AutonStates::kCrossLine);
			chs_Auton.AddObject("Switch",AutonStates::kSwitch);
			chs_Auton.AddObject("SwitchScale",AutonStates::kSwitchScale);
			chs_Auton.AddObject("SwitchExchange",AutonStates::kSwitchExchange);

			chs_Position.AddDefault("Nothing", StartingPosition::kLeft);
			chs_Position.AddObject("Right", StartingPosition::kRight);
			chs_Position.AddObject("Middle", StartingPosition::kCenter);
			chs_Position.AddObject("Left", StartingPosition::kLeft);

		}
		~Autonomous();

		void AutonInit(){
			AutonStage			= 0;
			m_ChosenState		= chs_Auton.GetSelected();
			m_SelectedPosition 	= chs_Position.GetSelected();
			rbt_Drivetrain->ResetDistance();
			rbt_Drivetrain->ResetAngle();

			m_SwitchSide=Configuration::Goal::GetSwitch();
			m_ScaleSide=Configuration::Goal::GetScale();


		}

		void CrossLine(){
			switch(AutonStage){
			case 0:
				rbt_Drivetrain->GoToDistance(114.0);
				AutonStage++;
				break;
			default:
				Dashboard.Autonomous.AutonStage.Set("Doing nothing.");
				rbt_Drivetrain->SetDrive(0.0,0.0);
				break;
			}
		}

		void LSt_LSw(){
			switch(AutonStage) {
				case 0:
					tmr_TimeOut.Reset();
					tmr_TimeOut.Start();
					rbt_Drivetrain->GoToDistance(150.0);
					AutonStage++;
					break;
				case 1:
					if (rbt_Drivetrain->IsAtDistance() || tmr_TimeOut.Get() > 3.0) AutonStage++;
					break;
				case 2:
					tmr_TimeOut.Reset();
					rbt_Drivetrain->DisableDistancePID();
					rbt_Drivetrain->GoToAngle(-90.0);
					rbt_Arm->Point(RobotMechanism::Arm::PointPosition::kReverseShoot);
					AutonStage++;
					break;
				case 3:
					if (rbt_Drivetrain->IsAtAngle() || tmr_TimeOut.Get() > 1.0) AutonStage++;
					break;
				case 4:
					tmr_TimeOut.Reset();
					rbt_Drivetrain->GoToDistance(-6.0);
					AutonStage++;
					break;
				case 5:
					if(rbt_Drivetrain->IsAtDistance() || tmr_TimeOut.Get() > 2.0) AutonStage++;
					break;
				case 6:
					tmr_TimeOut.Reset();
					rbt_Arm->ReverseIntake();
					AutonStage++;
					break;
				case 7:
					if(tmr_TimeOut.Get() > 0.5) AutonStage++;
					break;
				default:
					DoNothing();
					break;
			}
		}

		void LSt_RSw() {
			switch(AutonStage) {
				case 0:
					tmr_TimeOut.Reset();
					tmr_TimeOut.Start();
					rbt_Drivetrain->GoToDistance(210.0);
					AutonStage++;
					break;
				case 1:
					if (rbt_Drivetrain->IsAtDistance() || tmr_TimeOut.Get() > 3.0) AutonStage++;
					break;
				case 2:
					tmr_TimeOut.Reset();
					rbt_Drivetrain->DisableDistancePID();
					rbt_Drivetrain->GoToAngle(-90.0);
					AutonStage++;
					break;
				case 3:
					if (rbt_Drivetrain->IsAtAngle() || tmr_TimeOut.Get() > 1.0) AutonStage++;
					break;
				case 4:
					tmr_TimeOut.Reset();
					rbt_Drivetrain->GoToDistance(-205.0);
					AutonStage++;
					break;
				case 5:
					if (rbt_Drivetrain->IsAtDistance() || tmr_TimeOut.Get() > 3) AutonStage++;
					break;
				case 6:
					tmr_TimeOut.Reset();
					rbt_Drivetrain->DisableDistancePID();
					rbt_Drivetrain->GoToAngle(115.0);
					AutonStage++;
					break;
				case 7:
					if (rbt_Drivetrain->IsAtAngle() || tmr_TimeOut.Get() > 3.0) AutonStage++;
					break;
				case 8:
					tmr_TimeOut.Reset();
					rbt_Arm->Point(RobotMechanism::Arm::PointPosition::kReverseShoot);
					rbt_Drivetrain->GoToDistance(-10.0);
					AutonStage++;
					break;
				case 9:
					if (rbt_Drivetrain->IsAtDistance() || tmr_TimeOut.Get() > 2.0) AutonStage++;
					break;
				case 10:
					tmr_TimeOut.Reset();
					rbt_Arm->ReverseIntake();
					AutonStage++;
					break;
				case 11:
					if (tmr_TimeOut.Get() > 2.0) AutonStage++;
					break;
				default:
					DoNothing();
					break;
			}
		}
		void RSt_RSw(){
			switch(AutonStage) {
			case 0:
				tmr_TimeOut.Reset();
				tmr_TimeOut.Start();
				rbt_Drivetrain->GoToDistance(150.0);
				AutonStage++;
				break;
			case 1:
				if (rbt_Drivetrain->IsAtDistance() || tmr_TimeOut.Get() > 3.0) AutonStage++;
				break;
			case 2:
				tmr_TimeOut.Reset();
				rbt_Drivetrain->DisableDistancePID();
				rbt_Drivetrain->GoToAngle(90.0);
				rbt_Arm->Point(RobotMechanism::Arm::PointPosition::kReverseShoot);
				AutonStage++;
				break;
			case 3:
				if (rbt_Drivetrain->IsAtAngle() || tmr_TimeOut.Get() > 1.0) AutonStage++;
				break;
			case 4:
				tmr_TimeOut.Reset();
				rbt_Drivetrain->GoToDistance(-6.0);
				AutonStage++;
				break;
			case 5:
				if(rbt_Drivetrain->IsAtDistance() || tmr_TimeOut.Get() > 2.0) AutonStage++;
				break;
			case 6:
				tmr_TimeOut.Reset();
				rbt_Arm->ReverseIntake();
				AutonStage++;
				break;
			case 7:
				if(tmr_TimeOut.Get() > 0.5) AutonStage++;
				break;
			default:
				DoNothing();
				break;
		}
		}

		void RSt_LSw() {
			switch(AutonStage) {
			case 0:
				tmr_TimeOut.Reset();
				tmr_TimeOut.Start();
				rbt_Drivetrain->GoToDistance(210.0);
				AutonStage++;
				break;
			case 1:
				if (rbt_Drivetrain->IsAtDistance() || tmr_TimeOut.Get() > 3.0) AutonStage++;
				break;
			case 2:
				tmr_TimeOut.Reset();
				rbt_Drivetrain->DisableDistancePID();
				rbt_Drivetrain->GoToAngle(90.0);
				AutonStage++;
				break;
			case 3:
				if (rbt_Drivetrain->IsAtAngle() || tmr_TimeOut.Get() > 1.0) AutonStage++;
				break;
			case 4:
				tmr_TimeOut.Reset();
				rbt_Drivetrain->GoToDistance(-200.0);
				AutonStage++;
				break;
			case 5:
				if (rbt_Drivetrain->IsAtDistance() || tmr_TimeOut.Get() > 3) AutonStage++;
				break;
			case 6:
				tmr_TimeOut.Reset();
				rbt_Drivetrain->DisableDistancePID();
				rbt_Drivetrain->GoToAngle(-115.0);
				AutonStage++;
				break;
			case 7:
				if (rbt_Drivetrain->IsAtAngle() || tmr_TimeOut.Get() > 3.0) AutonStage++;
				break;
			case 8:
				tmr_TimeOut.Reset();
				rbt_Arm->Point(RobotMechanism::Arm::PointPosition::kReverseShoot);
				rbt_Drivetrain->GoToDistance(-10.0);
				AutonStage++;
				break;
			case 9:
				if (rbt_Drivetrain->IsAtDistance() || tmr_TimeOut.Get() > 2.0) AutonStage++;
				break;
			case 10:
				tmr_TimeOut.Reset();
				rbt_Arm->ReverseIntake();
				AutonStage++;
				break;
			case 11:
				if (tmr_TimeOut.Get() > 2.0) AutonStage++;
				break;
			default:
				DoNothing();
				break;
			}
		}

		void CSt_LSw(){
			switch(AutonStage){
				case 1:
					tmr_TimeOut.Reset();
					tmr_TimeOut.Start();
					rbt_Drivetrain->GoToDistance(36.0);
					AutonStage++;
					break;
				case 2:
					if (rbt_Drivetrain->IsAtDistance() || tmr_TimeOut.Get() > 1.0) AutonStage++;
					break;
				case 3:
					tmr_TimeOut.Reset();
					rbt_Drivetrain->GoToAngle(-90.0);
					AutonStage++;
					break;
				case 4:
					if(rbt_Drivetrain->IsAtAngle() || tmr_TimeOut.Get() > 1.0) AutonStage++;
					break;
				case 5:
					tmr_TimeOut.Reset();
					rbt_Drivetrain->GoToDistance(130);
					AutonStage++;
					break;
				case 6:
					if(rbt_Drivetrain->IsAtDistance() || tmr_TimeOut.Get() > 2.0) AutonStage++;
					break;
				case 7:
					tmr_TimeOut.Reset();
					rbt_Drivetrain->GoToAngle(90.0);
					AutonStage++;
					break;
				case 8:
					if(rbt_Drivetrain->IsAtAngle() || tmr_TimeOut.Get() > 1.0) AutonStage++;
					break;
				case 9:
					tmr_TimeOut.Reset();
					rbt_Arm->Point(RobotMechanism::Arm::PointPosition::kForwardShoot);
					rbt_Arm->ReverseIntake();
					AutonStage++;
					break;
				case 10:
					if(tmr_TimeOut.Get() > 2.0) AutonStage++;
					break;
				default:
					DoNothing();
					break;
			}
		}

		void CSt_RSw(){
			switch(AutonStage){
				case 0:
					tmr_TimeOut.Reset();
					tmr_TimeOut.Start();
					rbt_Drivetrain->GoToDistance(24.0);
					AutonStage++;
					break;
				case 1:
					if (rbt_Drivetrain->IsAtDistance() || tmr_TimeOut.Get() > 2.0) AutonStage++;
					break;
				case 2:
					tmr_TimeOut.Reset();
					rbt_Drivetrain->GoToAngle(30.0);
					AutonStage++;
					break;
				case 3:
					if(rbt_Drivetrain->IsAtAngle() || tmr_TimeOut.Get() > 2.0) AutonStage++;
					break;
				case 4:
					tmr_TimeOut.Reset();
					rbt_Drivetrain->GoToDistance(74); //go a little further
					AutonStage++;
					break;
				case 5:
					if(rbt_Drivetrain->IsAtDistance() || tmr_TimeOut.Get() > 3.0) AutonStage++;
					break;
				case 6:
					tmr_TimeOut.Reset();
					rbt_Arm->Point(RobotMechanism::Arm::PointPosition::kForwardShoot);
					AutonStage++;
					break;
				case 7:
					if(rbt_Arm->IsAtAngle() || tmr_TimeOut.Get() > 2.0) AutonStage++;
					break;
				case 8:
					tmr_TimeOut.Reset();
					rbt_Arm->ReverseIntake();
					AutonStage++;
					break;
				case 9:
					if (tmr_TimeOut.Get() > 2.0) AutonStage++;
					break;
				default:
					DoNothing();
					break;
			}
		}

		void LSt_LSwEx() {
			switch(AutonStage) {
				case 0:
					tmr_TimeOut.Reset();
					tmr_TimeOut.Start();
					rbt_Drivetrain->GoToDistance(156.0);
					AutonStage++;
					break;
				case 1:
					if (rbt_Drivetrain->IsAtDistance() || tmr_TimeOut.Get() > 3.0) AutonStage++;
					break;
				case 2:
					tmr_TimeOut.Reset();
					Dashboard.Autonomous.AutonStage.Set("Turning...");
					rbt_Drivetrain->GoToAngle(90.0);
					AutonStage++;
					break;
				case 3:
					if (rbt_Drivetrain->IsAtAngle() || tmr_TimeOut.Get() > 2.0) AutonStage++;
					break;
				case 4:
					tmr_TimeOut.Reset();
					Dashboard.Autonomous.AutonStage.Set("Going two feet...");
					rbt_Drivetrain->GoToDistance(36.0);
					rbt_Arm->Point(rbt_Arm->PointPosition::kForwardShoot);
					AutonStage++;
					break;
				case 5:
					if(rbt_Drivetrain->IsAtDistance() || tmr_TimeOut.Get() > 2.0) AutonStage++;
					break;
				case 6:
					tmr_TimeOut.Reset();
					rbt_Arm->ReverseIntake();
					AutonStage++;
					break;
				case 7:
					if ( tmr_TimeOut.Get() > 1.5) {
						rbt_Arm->DisableIntake();
						AutonStage++;
					}
					break;
				case 8:
					tmr_TimeOut.Reset();
					rbt_Drivetrain->GoToDistance(-24.0);
					AutonStage++;
					break;
				case 9:
					if (rbt_Drivetrain->IsAtDistance() || tmr_TimeOut.Get() > 2.0) AutonStage++;
					break;
				case 10:
					tmr_TimeOut.Reset();
					rbt_Drivetrain->GoToAngle(-90.0);
					AutonStage++;
					break;
				case 11:
					if (rbt_Drivetrain->IsAtAngle() || tmr_TimeOut.Get() > 2.0) AutonStage++;
					break;
				case 12:
					tmr_TimeOut.Reset();
					rbt_Drivetrain->GoToDistance(80.0);
					AutonStage++;
					break;
				case 13:
					if (rbt_Drivetrain->IsAtDistance() || tmr_TimeOut.Get() > 3.0) AutonStage++;
					break;
				case 14:
					tmr_TimeOut.Reset();
					rbt_Drivetrain->GoToAngle(90);
					AutonStage++;
					break;
				case 15:
					if (rbt_Drivetrain->IsAtAngle() || tmr_TimeOut.Get() > 2.0) AutonStage++;
					break;
				case 16:
					tmr_TimeOut.Reset();
					rbt_Drivetrain->GoToDistance(56);
					AutonStage++;
					break;
				case 17:
					if (rbt_Drivetrain->IsAtDistance() || tmr_TimeOut.Get() > 2.0) AutonStage++;
					break;
				case 18:
					tmr_TimeOut.Reset();
					rbt_Arm->Point(rbt_Arm->PointPosition::kForwardCollect);
					rbt_Drivetrain->GoToAngle(90);
					AutonStage++;
					break;
				case 19:
					if (rbt_Drivetrain->IsAtAngle() || tmr_TimeOut.Get() > 3.0) AutonStage++;
					break;
				case 20:
					tmr_TimeOut.Reset();
					rbt_Arm->EnableIntake();
					rbt_Drivetrain->SetDrive(0.5, 0.5);
					AutonStage++;
					break;
				case 21:
					if ( tmr_TimeOut.Get() > 3.0) {
						rbt_Drivetrain->SetDrive(0.0, 0.0);
						rbt_Arm->DisableIntake();
						AutonStage++;
					}
					break;
				case 22:
					tmr_TimeOut.Reset();
					rbt_Drivetrain->GoToAngle(90.0);
					break;
				case 23:
					if (rbt_Drivetrain->IsAtAngle() || tmr_TimeOut.Get() > 2.0) AutonStage++;
					break;
				case 24:
					tmr_TimeOut.Reset();
					rbt_Drivetrain->GoToDistance(56.0);
					AutonStage++;
					break;
				case 25:
					if (rbt_Drivetrain->IsAtDistance() || tmr_TimeOut.Get() > 2.0) AutonStage++;
					break;
				case 26:
					tmr_TimeOut.Reset();
					rbt_Drivetrain->GoToAngle(-90.0);
					AutonStage++;
					break;
				case 27:
					if (rbt_Drivetrain->IsAtAngle() || tmr_TimeOut.Get() > 2.0) AutonStage++;
					break;
				case 28:
					tmr_TimeOut.Reset();
					rbt_Drivetrain->GoToDistance(159.0);
					AutonStage++;
					break;
				case 29:
					if (rbt_Drivetrain->IsAtDistance() || tmr_TimeOut.Get() > 2.0) AutonStage++;
					break;
				case 30:
					tmr_TimeOut.Reset();
					rbt_Drivetrain->GoToAngle(-90.0);
					AutonStage++;
					break;
				case 31:
					if (rbt_Drivetrain->IsAtAngle() || tmr_TimeOut.Get() > 2.0) AutonStage++;
					break;
				case 32:
					tmr_TimeOut.Reset();
					rbt_Drivetrain->GoToDistance(75.0);
					AutonStage++;
					break;
				case 33:
					if (rbt_Drivetrain->IsAtDistance() || tmr_TimeOut.Get() > 2.0) AutonStage++;
					break;
				case 34:
					tmr_TimeOut.Reset();
					rbt_Drivetrain->GoToDistance(75);
					AutonStage++;
					break;
				case 35:
					if (rbt_Drivetrain->IsAtDistance() || tmr_TimeOut.Get() > 2.0) AutonStage++;
					break;
				case 36:
					tmr_TimeOut.Reset();
					rbt_Arm->ReverseIntake();
					AutonStage++;
					break;
				case 37:
					if ( tmr_TimeOut.Get() > 2.0) AutonStage++;
					break;
				default:
					rbt_Drivetrain->SetDrive(0, 0);
					break;
				}
		}

		void LSt_RSwEx() {
			switch(AutonStage) {
			case 0:
				tmr_TimeOut.Reset();
				rbt_Drivetrain->GoToDistance(41.0);
				break;
			case 1:
				if (rbt_Drivetrain->IsAtDistance() || tmr_TimeOut.Get() > 3.0) AutonStage++;
				break;
			case 2:
				tmr_TimeOut.Reset();
				rbt_Drivetrain->GoToAngle(90);
				AutonStage++;
				break;
			case 3:
				if (rbt_Drivetrain->IsAtAngle() || tmr_TimeOut.Get() > 2.0) AutonStage++;
				break;
			case 4:
				tmr_TimeOut.Reset();
				rbt_Drivetrain->GoToDistance(219.0);
				AutonStage++;
				break;
			case 5:
				if (rbt_Drivetrain->IsAtDistance() || tmr_TimeOut.Get() > 3.0) AutonStage++;
				break;
			case 6:
				tmr_TimeOut.Reset();
				rbt_Drivetrain->GoToAngle(-90.0);
				AutonStage++;
				break;
			case 7:
				if (rbt_Drivetrain->IsAtAngle() || tmr_TimeOut.Get() > 2.0) AutonStage++;
				break;
			case 8:
				tmr_TimeOut.Reset();
				rbt_Drivetrain->GoToDistance(115.0);
				AutonStage++;
				break;
			case 9:
				if (rbt_Drivetrain->IsAtDistance() || tmr_TimeOut.Get() > 3.0) AutonStage++;
				break;
			case 10:
				tmr_TimeOut.Reset();
				rbt_Drivetrain->GoToAngle(-90.0);
				AutonStage++;
				break;
			case 11:
				if (rbt_Drivetrain->IsAtAngle() || tmr_TimeOut.Get() > 2.0) AutonStage++;
				break;
			case 12:
				tmr_TimeOut.Reset();
				rbt_Drivetrain->GoToDistance(36.0);
				AutonStage++;
				break;
			case 13:
				if (rbt_Drivetrain->IsAtDistance() || tmr_TimeOut.Get() > 2.0) AutonStage++;
				break;
			case 14:
				tmr_TimeOut.Reset();
				rbt_Arm->ReverseIntake();
				AutonStage++;
				break;
			case 15:
				if ( tmr_TimeOut.Get() > 1.5) {
					rbt_Arm->DisableIntake();
					AutonStage++;
				}
				break;
			case 16:
				tmr_TimeOut.Reset();
				rbt_Drivetrain->GoToDistance(-24.0);
				AutonStage++;
				break;
			case 17:
				if (rbt_Drivetrain->IsAtDistance() || tmr_TimeOut.Get() > 2.0) AutonStage++;
				break;
			case 18:
				tmr_TimeOut.Reset();
				rbt_Drivetrain->GoToAngle(90.0);
				AutonStage++;
				break;
			case 19:
				if (rbt_Drivetrain->IsAtAngle() || tmr_TimeOut.Get() > 2.0) AutonStage++;
				break;
			case 20:
				tmr_TimeOut.Reset();
				rbt_Drivetrain->GoToDistance(80.0);
				AutonStage++;
				break;
			case 21:
				if (rbt_Drivetrain->IsAtDistance() || tmr_TimeOut.Get() > 3.0) AutonStage++;
				break;
			case 22:
				tmr_TimeOut.Reset();
				rbt_Drivetrain->GoToAngle(-90);
				AutonStage++;
				break;
			case 23:
				if (rbt_Drivetrain->IsAtAngle() || tmr_TimeOut.Get() > 2.0) AutonStage++;
				break;
			case 24:
				tmr_TimeOut.Reset();
				rbt_Drivetrain->GoToDistance(56);
				AutonStage++;
				break;
			case 25:
				if (rbt_Drivetrain->IsAtDistance() || tmr_TimeOut.Get() > 2.0) AutonStage++;
				break;
			case 26:
				tmr_TimeOut.Reset();
				rbt_Arm->Point(rbt_Arm->PointPosition::kForwardCollect);
				rbt_Drivetrain->GoToAngle(-90);
				AutonStage++;
				break;
			case 27:
				if (rbt_Drivetrain->IsAtAngle() || tmr_TimeOut.Get() > 3.0) AutonStage++;
				break;
			case 28:
				tmr_TimeOut.Reset();
				rbt_Arm->EnableIntake();
				rbt_Drivetrain->SetDrive(0.5, 0.5);
				AutonStage++;
				break;
			case 29:
				if ( tmr_TimeOut.Get() > 3.0) {
					rbt_Drivetrain->SetDrive(0.0, 0.0);
					rbt_Arm->DisableIntake();
					AutonStage++;
				}
				break;
			case 30:
				tmr_TimeOut.Reset();
				rbt_Drivetrain->GoToAngle(-90.0);
				break;
			case 31:
				if (rbt_Drivetrain->IsAtAngle() || tmr_TimeOut.Get() > 2.0) AutonStage++;
				break;
			case 32:
				tmr_TimeOut.Reset();
				rbt_Drivetrain->GoToDistance(56.0);
				AutonStage++;
				break;
			case 33:
				if (rbt_Drivetrain->IsAtDistance() || tmr_TimeOut.Get() > 2.0) AutonStage++;
				break;
			case 34:
				tmr_TimeOut.Reset();
				rbt_Drivetrain->GoToAngle(90.0);
				AutonStage++;
				break;
			case 35:
				if (rbt_Drivetrain->IsAtAngle() || tmr_TimeOut.Get() > 2.0) AutonStage++;
				break;
			case 36:
				tmr_TimeOut.Reset();
				rbt_Drivetrain->GoToDistance(159.0);
				AutonStage++;
				break;
			case 37:
				if (rbt_Drivetrain->IsAtDistance() || tmr_TimeOut.Get() > 2.0) AutonStage++;
				break;
			case 38:
				tmr_TimeOut.Reset();
				rbt_Drivetrain->GoToAngle(90.0);
				AutonStage++;
				break;
			case 39:
				if (rbt_Drivetrain->IsAtAngle() || tmr_TimeOut.Get() > 2.0) AutonStage++;
				break;
			case 40:
				tmr_TimeOut.Reset();
				rbt_Drivetrain->GoToDistance(105.0);
				AutonStage++;
				break;
			case 41:
				if (rbt_Drivetrain->IsAtDistance() || tmr_TimeOut.Get() > 2.0) AutonStage++;
				break;
			case 42:
				tmr_TimeOut.Reset();
				rbt_Drivetrain->GoToAngle(-90);
				AutonStage++;
				break;
			case 43:
				if (rbt_Drivetrain->IsAtAngle() || tmr_TimeOut.Get() > 2.0) AutonStage++;
				break;
			case 44:
				tmr_TimeOut.Reset();
				rbt_Drivetrain->GoToDistance(75);
				AutonStage++;
				break;
			case 45:
				if (rbt_Drivetrain->IsAtDistance() || tmr_TimeOut.Get() > 2.0) AutonStage++;
				break;
			case 46:
				tmr_TimeOut.Reset();
				rbt_Arm->ReverseIntake();
				AutonStage++;
				break;
			case 47:
				if ( tmr_TimeOut.Get() > 2.0) AutonStage++;
				break;
			}
		}

		void CSt_LSwEx() {
			switch(AutonStage) {
			case 0:
				tmr_TimeOut.Reset();
				tmr_TimeOut.Start();
				rbt_Drivetrain->GoToDistance(41.0);
				AutonStage++;
				break;
			case 1:
				if (rbt_Drivetrain->IsAtDistance() || tmr_TimeOut.Get() > 2.0) AutonStage++;
				break;
			case 2:
				tmr_TimeOut.Reset();
				rbt_Drivetrain->GoToAngle(-90.0);
				AutonStage++;
				break;
			case 3:
				if (rbt_Drivetrain->IsAtAngle() || tmr_TimeOut.Get() > 2.0) AutonStage++;
				break;
			case 4:
				tmr_TimeOut.Reset();
				rbt_Drivetrain->GoToDistance(88.0);
				AutonStage++;
				break;
			case 5:
				if (rbt_Drivetrain->IsAtDistance() || tmr_TimeOut.Get() > 2.0) AutonStage++;
				break;
			case 6:
				tmr_TimeOut.Reset();
				rbt_Drivetrain->GoToAngle(90.0);
				AutonStage++;
				break;
			case 7:
				if (rbt_Drivetrain->IsAtAngle() || tmr_TimeOut.Get() > 2.0) AutonStage++;
				break;
			case 8:
				tmr_TimeOut.Reset();
				rbt_Drivetrain->GoToDistance(115.0);
				rbt_Arm->Point(rbt_Arm->PointPosition::kForwardCollect);
				AutonStage++;
				break;
			case 9:
				if (rbt_Drivetrain->IsAtDistance() || tmr_TimeOut.Get() > 3.0) AutonStage++;
				break;
			case 10:
				tmr_TimeOut.Reset();
				rbt_Drivetrain->GoToAngle(90.0);
				AutonStage++;
				break;
			case 11:
				if (rbt_Drivetrain->IsAtAngle() || tmr_TimeOut.Get() > 2.0) AutonStage++;
				break;
			case 12:
				tmr_TimeOut.Reset();
				rbt_Drivetrain->GoToDistance(36.0);
				AutonStage++;
				break;
			case 13:
				if (rbt_Drivetrain->IsAtDistance() || tmr_TimeOut.Get() > 2.0) AutonStage++;
				break;
			case 14:
				tmr_TimeOut.Reset();
				rbt_Arm->ReverseIntake();
				AutonStage++;
				break;
			case 15:
				if ( tmr_TimeOut.Get() > 1.5) {
					rbt_Arm->DisableIntake();
					AutonStage++;
				}
				break;
			case 16:
				tmr_TimeOut.Reset();
				rbt_Drivetrain->GoToDistance(-24.0);
				AutonStage++;
				break;
			case 17:
				if (rbt_Drivetrain->IsAtDistance() || tmr_TimeOut.Get() > 2.0) AutonStage++;
				break;
			case 18:
				tmr_TimeOut.Reset();
				rbt_Drivetrain->GoToAngle(-90.0);
				AutonStage++;
				break;
			case 19:
				if (rbt_Drivetrain->IsAtAngle() || tmr_TimeOut.Get() > 2.0) AutonStage++;
				break;
			case 20:
				tmr_TimeOut.Reset();
				rbt_Drivetrain->GoToDistance(80.0);
				AutonStage++;
				break;
			case 21:
				if (rbt_Drivetrain->IsAtDistance() || tmr_TimeOut.Get() > 3.0) AutonStage++;
				break;
			case 22:
				tmr_TimeOut.Reset();
				rbt_Drivetrain->GoToAngle(90);
				AutonStage++;
				break;
			case 23:
				if (rbt_Drivetrain->IsAtAngle() || tmr_TimeOut.Get() > 2.0) AutonStage++;
				break;
			case 24:
				tmr_TimeOut.Reset();
				rbt_Drivetrain->GoToDistance(56);
				AutonStage++;
				break;
			case 25:
				if (rbt_Drivetrain->IsAtDistance() || tmr_TimeOut.Get() > 2.0) AutonStage++;
				break;
			case 26:
				tmr_TimeOut.Reset();
				rbt_Arm->Point(rbt_Arm->PointPosition::kForwardCollect);
				rbt_Drivetrain->GoToAngle(90);
				AutonStage++;
				break;
			case 27:
				if (rbt_Drivetrain->IsAtAngle() || tmr_TimeOut.Get() > 3.0) AutonStage++;
				break;
			case 28:
				tmr_TimeOut.Reset();
				rbt_Arm->EnableIntake();
				rbt_Drivetrain->SetDrive(0.5, 0.5);
				AutonStage++;
				break;
			case 29:
				if ( tmr_TimeOut.Get() > 3.0) {
					rbt_Drivetrain->SetDrive(0.0, 0.0);
					rbt_Arm->DisableIntake();
					AutonStage++;
				}
				break;
			case 30:
				tmr_TimeOut.Reset();
				rbt_Drivetrain->GoToAngle(90.0);
				break;
			case 31:
				if (rbt_Drivetrain->IsAtAngle() || tmr_TimeOut.Get() > 2.0) AutonStage++;
				break;
			case 32:
				tmr_TimeOut.Reset();
				rbt_Drivetrain->GoToDistance(56.0);
				AutonStage++;
				break;
			case 33:
				if (rbt_Drivetrain->IsAtDistance() || tmr_TimeOut.Get() > 2.0) AutonStage++;
				break;
			case 34:
				tmr_TimeOut.Reset();
				rbt_Drivetrain->GoToAngle(-90.0);
				AutonStage++;
				break;
			case 35:
				if (rbt_Drivetrain->IsAtAngle() || tmr_TimeOut.Get() > 2.0) AutonStage++;
				break;
			case 36:
				tmr_TimeOut.Reset();
				rbt_Drivetrain->GoToDistance(159.0);
				AutonStage++;
				break;
			case 37:
				if (rbt_Drivetrain->IsAtDistance() || tmr_TimeOut.Get() > 2.0) AutonStage++;
				break;
			case 38:
				tmr_TimeOut.Reset();
				rbt_Drivetrain->GoToAngle(-90.0);
				AutonStage++;
				break;
			case 39:
				if (rbt_Drivetrain->IsAtAngle() || tmr_TimeOut.Get() > 2.0) AutonStage++;
				break;
			case 40:
				tmr_TimeOut.Reset();
				rbt_Drivetrain->GoToDistance(75.0);
				AutonStage++;
				break;
			case 41:
				if (rbt_Drivetrain->IsAtDistance() || tmr_TimeOut.Get() > 2.0) AutonStage++;
				break;
			case 42:
				tmr_TimeOut.Reset();
				rbt_Drivetrain->GoToAngle(90);
				AutonStage++;
				break;
			case 43:
				if (rbt_Drivetrain->IsAtAngle() || tmr_TimeOut.Get() > 2.0) AutonStage++;
				break;
			case 44:
				tmr_TimeOut.Reset();
				rbt_Drivetrain->GoToDistance(75);
				AutonStage++;
				break;
			case 45:
				if (rbt_Drivetrain->IsAtDistance() || tmr_TimeOut.Get() > 2.0) AutonStage++;
				break;
			case 46:
				tmr_TimeOut.Reset();
				rbt_Arm->ReverseIntake();
				AutonStage++;
				break;
			case 47:
				if ( tmr_TimeOut.Get() > 2.0) AutonStage++;
				break;
			default:
				break;
			}
		}

		void CSt_RSwEx() {
			switch(AutonStage) {
			case 0:
				tmr_TimeOut.Reset();
				tmr_TimeOut.Start();
				rbt_Drivetrain->GoToDistance(41.0);
				AutonStage++;
				break;
			case 1:
				if (rbt_Drivetrain->IsAtDistance() || tmr_TimeOut.Get() > 3.0) AutonStage++;
				break;
			case 2:
				tmr_TimeOut.Reset();
				rbt_Drivetrain->GoToAngle(90.0);
				AutonStage++;
				break;
			case 3:
				if (rbt_Drivetrain->IsAtAngle() || tmr_TimeOut.Get() > 2.0) AutonStage++;
				break;
			case 4:
				tmr_TimeOut.Reset();
				rbt_Drivetrain->GoToDistance(56.0);
				AutonStage++;
				break;
			case 5:
				if (rbt_Drivetrain->IsAtDistance() || tmr_TimeOut.Get() > 3.0) AutonStage++;
				break;
			case 6:
				tmr_TimeOut.Reset();
				rbt_Drivetrain->GoToAngle(-90.0);
				AutonStage++;
				break;
			case 7:
				if (rbt_Drivetrain->IsAtAngle() || tmr_TimeOut.Get() > 2.0) AutonStage++;
				break;
			case 8:
				tmr_TimeOut.Reset();
				rbt_Drivetrain->GoToDistance(115.0);
				AutonStage++;
				break;
			case 9:
				if (rbt_Drivetrain->IsAtDistance() || tmr_TimeOut.Get() > 3.0) AutonStage++;
				break;
			case 10:
				tmr_TimeOut.Reset();
				rbt_Drivetrain->GoToAngle(-90.0);
				AutonStage++;
				break;
			case 11:
				if (rbt_Drivetrain->IsAtAngle() || tmr_TimeOut.Get() > 2.0) AutonStage++;
				break;
			case 12:
				tmr_TimeOut.Reset();
				rbt_Drivetrain->GoToDistance(36.0);
				AutonStage++;
				break;
			case 13:
				if (rbt_Drivetrain->IsAtDistance() || tmr_TimeOut.Get() > 2.0) AutonStage++;
				break;
			case 14:
				tmr_TimeOut.Reset();
				rbt_Arm->ReverseIntake();
				AutonStage++;
				break;
			case 15:
				if ( tmr_TimeOut.Get() > 1.5) {
					rbt_Arm->DisableIntake();
					AutonStage++;
				}
				break;
			case 16:
				tmr_TimeOut.Reset();
				rbt_Drivetrain->GoToDistance(-24.0);
				AutonStage++;
				break;
			case 17:
				if (rbt_Drivetrain->IsAtDistance() || tmr_TimeOut.Get() > 2.0) AutonStage++;
				break;
			case 18:
				tmr_TimeOut.Reset();
				rbt_Drivetrain->GoToAngle(90.0);
				AutonStage++;
				break;
			case 19:
				if (rbt_Drivetrain->IsAtAngle() || tmr_TimeOut.Get() > 2.0) AutonStage++;
				break;
			case 20:
				tmr_TimeOut.Reset();
				rbt_Drivetrain->GoToDistance(80.0);
				AutonStage++;
				break;
			case 21:
				if (rbt_Drivetrain->IsAtDistance() || tmr_TimeOut.Get() > 3.0) AutonStage++;
				break;
			case 22:
				tmr_TimeOut.Reset();
				rbt_Drivetrain->GoToAngle(-90);
				AutonStage++;
				break;
			case 23:
				if (rbt_Drivetrain->IsAtAngle() || tmr_TimeOut.Get() > 2.0) AutonStage++;
				break;
			case 24:
				tmr_TimeOut.Reset();
				rbt_Drivetrain->GoToDistance(56);
				AutonStage++;
				break;
			case 25:
				if (rbt_Drivetrain->IsAtDistance() || tmr_TimeOut.Get() > 2.0) AutonStage++;
				break;
			case 26:
				tmr_TimeOut.Reset();
				rbt_Arm->Point(rbt_Arm->PointPosition::kForwardCollect);
				rbt_Drivetrain->GoToAngle(-90);
				AutonStage++;
				break;
			case 27:
				if (rbt_Drivetrain->IsAtAngle() || tmr_TimeOut.Get() > 3.0) AutonStage++;
				break;
			case 28:
				tmr_TimeOut.Reset();
				rbt_Arm->EnableIntake();
				rbt_Drivetrain->SetDrive(0.5, 0.5);
				AutonStage++;
				break;
			case 29:
				if ( tmr_TimeOut.Get() > 3.0) {
					rbt_Drivetrain->SetDrive(0.0, 0.0);
					rbt_Arm->DisableIntake();
					AutonStage++;
				}
				break;
			case 30:
				tmr_TimeOut.Reset();
				rbt_Drivetrain->GoToAngle(-90.0);
				break;
			case 31:
				if (rbt_Drivetrain->IsAtAngle() || tmr_TimeOut.Get() > 2.0) AutonStage++;
				break;
			case 32:
				tmr_TimeOut.Reset();
				rbt_Drivetrain->GoToDistance(56.0);
				AutonStage++;
				break;
			case 33:
				if (rbt_Drivetrain->IsAtDistance() || tmr_TimeOut.Get() > 2.0) AutonStage++;
				break;
			case 34:
				tmr_TimeOut.Reset();
				rbt_Drivetrain->GoToAngle(90.0);
				AutonStage++;
				break;
			case 35:
				if (rbt_Drivetrain->IsAtAngle() || tmr_TimeOut.Get() > 2.0) AutonStage++;
				break;
			case 36:
				tmr_TimeOut.Reset();
				rbt_Drivetrain->GoToDistance(159.0);
				AutonStage++;
				break;
			case 37:
				if (rbt_Drivetrain->IsAtDistance() || tmr_TimeOut.Get() > 2.0) AutonStage++;
				break;
			case 38:
				tmr_TimeOut.Reset();
				rbt_Drivetrain->GoToAngle(90.0);
				AutonStage++;
				break;
			case 39:
				if (rbt_Drivetrain->IsAtAngle() || tmr_TimeOut.Get() > 2.0) AutonStage++;
				break;
			case 40:
				tmr_TimeOut.Reset();
				rbt_Drivetrain->GoToDistance(105.0);
				AutonStage++;
				break;
			case 41:
				if (rbt_Drivetrain->IsAtDistance() || tmr_TimeOut.Get() > 2.0) AutonStage++;
				break;
			case 42:
				tmr_TimeOut.Reset();
				rbt_Drivetrain->GoToAngle(-90);
				AutonStage++;
				break;
			case 43:
				if (rbt_Drivetrain->IsAtAngle() || tmr_TimeOut.Get() > 2.0) AutonStage++;
				break;
			case 44:
				tmr_TimeOut.Reset();
				rbt_Drivetrain->GoToDistance(75);
				AutonStage++;
				break;
			case 45:
				if (rbt_Drivetrain->IsAtDistance() || tmr_TimeOut.Get() > 2.0) AutonStage++;
				break;
			case 46:
				tmr_TimeOut.Reset();
				rbt_Arm->ReverseIntake();
				AutonStage++;
				break;
			case 47:
				if (tmr_TimeOut.Get() > 2.0) AutonStage++;
				break;
			default:
				break;
			}
		}

		void RSt_LSwEx() {
			switch(AutonStage) {
			case 0:
				tmr_TimeOut.Reset();
				rbt_Drivetrain->GoToDistance(41.0);
				break;
			case 1:
				if (rbt_Drivetrain->IsAtDistance() || tmr_TimeOut.Get() > 3.0) AutonStage++;
				break;
			case 2:
				tmr_TimeOut.Reset();
				rbt_Drivetrain->GoToAngle(-90);
				AutonStage++;
				break;
			case 3:
				if (rbt_Drivetrain->IsAtAngle() || tmr_TimeOut.Get() > 2.0) AutonStage++;
				break;
			case 4:
				tmr_TimeOut.Reset();
				rbt_Drivetrain->GoToDistance(219.0);
				AutonStage++;
				break;
			case 5:
				if (rbt_Drivetrain->IsAtDistance() || tmr_TimeOut.Get() > 3.0) AutonStage++;
				break;
			case 6:
				tmr_TimeOut.Reset();
				rbt_Drivetrain->GoToAngle(90.0);
				AutonStage++;
				break;
			case 7:
				if (rbt_Drivetrain->IsAtAngle() || tmr_TimeOut.Get() > 2.0) AutonStage++;
				break;
			case 8:
				tmr_TimeOut.Reset();
				rbt_Drivetrain->GoToDistance(115.0);
				AutonStage++;
				break;
			case 9:
				if (rbt_Drivetrain->IsAtDistance() || tmr_TimeOut.Get() > 3.0) AutonStage++;
				break;
			case 10:
				tmr_TimeOut.Reset();
				rbt_Drivetrain->GoToAngle(90.0);
				AutonStage++;
				break;
			case 11:
				if (rbt_Drivetrain->IsAtAngle() || tmr_TimeOut.Get() > 2.0) AutonStage++;
				break;
			case 12:
				tmr_TimeOut.Reset();
				rbt_Drivetrain->GoToDistance(36.0);
				AutonStage++;
				break;
			case 13:
				if (rbt_Drivetrain->IsAtDistance() || tmr_TimeOut.Get() > 2.0) AutonStage++;
				break;
			case 14:
				tmr_TimeOut.Reset();
				rbt_Arm->ReverseIntake();
				AutonStage++;
				break;
			case 15:
				if ( tmr_TimeOut.Get() > 1.5) {
					rbt_Arm->DisableIntake();
					AutonStage++;
				}
				break;
			case 16:
				tmr_TimeOut.Reset();
				rbt_Drivetrain->GoToDistance(-24.0);
				AutonStage++;
				break;
			case 17:
				if (rbt_Drivetrain->IsAtDistance() || tmr_TimeOut.Get() > 2.0) AutonStage++;
				break;
			case 18:
				tmr_TimeOut.Reset();
				rbt_Drivetrain->GoToAngle(-90.0);
				AutonStage++;
				break;
			case 19:
				if (rbt_Drivetrain->IsAtAngle() || tmr_TimeOut.Get() > 2.0) AutonStage++;
				break;
			case 20:
				tmr_TimeOut.Reset();
				rbt_Drivetrain->GoToDistance(80.0);
				AutonStage++;
				break;
			case 21:
				if (rbt_Drivetrain->IsAtDistance() || tmr_TimeOut.Get() > 3.0) AutonStage++;
				break;
			case 22:
				tmr_TimeOut.Reset();
				rbt_Drivetrain->GoToAngle(90);
				AutonStage++;
				break;
			case 23:
				if (rbt_Drivetrain->IsAtAngle() || tmr_TimeOut.Get() > 2.0) AutonStage++;
				break;
			case 24:
				tmr_TimeOut.Reset();
				rbt_Drivetrain->GoToDistance(56);
				AutonStage++;
				break;
			case 25:
				if (rbt_Drivetrain->IsAtDistance() || tmr_TimeOut.Get() > 2.0) AutonStage++;
				break;
			case 26:
				tmr_TimeOut.Reset();
				rbt_Arm->Point(rbt_Arm->PointPosition::kForwardCollect);
				rbt_Drivetrain->GoToAngle(90);
				AutonStage++;
				break;
			case 27:
				if (rbt_Drivetrain->IsAtAngle() || tmr_TimeOut.Get() > 3.0) AutonStage++;
				break;
			case 28:
				tmr_TimeOut.Reset();
				rbt_Arm->EnableIntake();
				rbt_Drivetrain->SetDrive(0.5, 0.5);
				AutonStage++;
				break;
			case 29:
				if ( tmr_TimeOut.Get() > 3.0) {
					rbt_Drivetrain->SetDrive(0.0, 0.0);
					rbt_Arm->DisableIntake();
					AutonStage++;
				}
				break;
			case 30:
				tmr_TimeOut.Reset();
				rbt_Drivetrain->GoToAngle(-90.0);
				break;
			case 31:
				if (rbt_Drivetrain->IsAtAngle() || tmr_TimeOut.Get() > 2.0) AutonStage++;
				break;
			case 32:
				tmr_TimeOut.Reset();
				rbt_Drivetrain->GoToDistance(56.0);
				AutonStage++;
				break;
			case 33:
				if (rbt_Drivetrain->IsAtDistance() || tmr_TimeOut.Get() > 2.0) AutonStage++;
				break;
			case 34:
				tmr_TimeOut.Reset();
				rbt_Drivetrain->GoToAngle(-90.0);
				AutonStage++;
				break;
			case 35:
				if (rbt_Drivetrain->IsAtAngle() || tmr_TimeOut.Get() > 2.0) AutonStage++;
				break;
			case 36:
				tmr_TimeOut.Reset();
				rbt_Drivetrain->GoToDistance(159.0);
				AutonStage++;
				break;
			case 37:
				if (rbt_Drivetrain->IsAtDistance() || tmr_TimeOut.Get() > 2.0) AutonStage++;
				break;
			case 38:
				tmr_TimeOut.Reset();
				rbt_Drivetrain->GoToAngle(-90.0);
				AutonStage++;
				break;
			case 39:
				if (rbt_Drivetrain->IsAtAngle() || tmr_TimeOut.Get() > 2.0) AutonStage++;
				break;
			case 40:
				tmr_TimeOut.Reset();
				rbt_Drivetrain->GoToDistance(105.0);
				AutonStage++;
				break;
			case 41:
				if (rbt_Drivetrain->IsAtDistance() || tmr_TimeOut.Get() > 2.0) AutonStage++;
				break;
			case 42:
				tmr_TimeOut.Reset();
				rbt_Drivetrain->GoToAngle(90);
				AutonStage++;
				break;
			case 43:
				if (rbt_Drivetrain->IsAtAngle() || tmr_TimeOut.Get() > 2.0) AutonStage++;
				break;
			case 44:
				tmr_TimeOut.Reset();
				rbt_Drivetrain->GoToDistance(75);
				AutonStage++;
				break;
			case 45:
				if (rbt_Drivetrain->IsAtDistance() || tmr_TimeOut.Get() > 2.0) AutonStage++;
				break;
			case 46:
				tmr_TimeOut.Reset();
				rbt_Arm->ReverseIntake();
				AutonStage++;
				break;
			case 47:
				if ( tmr_TimeOut.Get() > 2.0) AutonStage++;
				break;
			}
		}

		void RSt_RSwEx() {
			switch(AutonStage) {
				case 0:
					tmr_TimeOut.Reset();
					tmr_TimeOut.Start();
					rbt_Drivetrain->GoToDistance(156.0);
					AutonStage++;
					break;
				case 1:
					if (rbt_Drivetrain->IsAtDistance() || tmr_TimeOut.Get() > 3.0) AutonStage++;
					break;
				case 2:
					tmr_TimeOut.Reset();
					Dashboard.Autonomous.AutonStage.Set("Turning...");
					rbt_Drivetrain->GoToAngle(-90.0);
					AutonStage++;
					break;
				case 3:
					if (rbt_Drivetrain->IsAtAngle() || tmr_TimeOut.Get() > 2.0) AutonStage++;
					break;
				case 4:
					tmr_TimeOut.Reset();
					Dashboard.Autonomous.AutonStage.Set("Going two feet...");
					rbt_Drivetrain->GoToDistance(36.0);
					rbt_Arm->Point(rbt_Arm->PointPosition::kForwardShoot);
					AutonStage++;
					break;
				case 5:
					if(rbt_Drivetrain->IsAtDistance() || tmr_TimeOut.Get() > 2.0) AutonStage++;
					break;
				case 6:
					tmr_TimeOut.Reset();
					rbt_Arm->ReverseIntake();
					AutonStage++;
					break;
				case 7:
					if (tmr_TimeOut.Get() > 1.5) {
						rbt_Arm->DisableIntake();
						AutonStage++;
					}
					break;
				case 8:
					tmr_TimeOut.Reset();
					rbt_Drivetrain->GoToDistance(-24.0);
					AutonStage++;
					break;
				case 9:
					if (rbt_Drivetrain->IsAtDistance() || tmr_TimeOut.Get() > 2.0) AutonStage++;
					break;
				case 10:
					tmr_TimeOut.Reset();
					rbt_Drivetrain->GoToAngle(90.0);
					AutonStage++;
					break;
				case 11:
					if (rbt_Drivetrain->IsAtAngle() || tmr_TimeOut.Get() > 2.0) AutonStage++;
					break;
				case 12:
					tmr_TimeOut.Reset();
					rbt_Drivetrain->GoToDistance(80.0);
					AutonStage++;
					break;
				case 13:
					if (rbt_Drivetrain->IsAtDistance() || tmr_TimeOut.Get() > 3.0) AutonStage++;
					break;
				case 14:
					tmr_TimeOut.Reset();
					rbt_Drivetrain->GoToAngle(-90);
					AutonStage++;
					break;
				case 15:
					if (rbt_Drivetrain->IsAtAngle() || tmr_TimeOut.Get() > 2.0) AutonStage++;
					break;
				case 16:
					tmr_TimeOut.Reset();
					rbt_Drivetrain->GoToDistance(56);
					AutonStage++;
					break;
				case 17:
					if (rbt_Drivetrain->IsAtDistance() || tmr_TimeOut.Get() > 2.0) AutonStage++;
					break;
				case 18:
					tmr_TimeOut.Reset();
					rbt_Arm->Point(rbt_Arm->PointPosition::kForwardCollect);
					rbt_Drivetrain->GoToAngle(-90);
					AutonStage++;
					break;
				case 19:
					if (rbt_Drivetrain->IsAtAngle() || tmr_TimeOut.Get() > 3.0) AutonStage++;
					break;
				case 20:
					tmr_TimeOut.Reset();
					rbt_Arm->EnableIntake();
					rbt_Drivetrain->SetDrive(0.5, 0.5);
					AutonStage++;
					break;
				case 21:
					if ( tmr_TimeOut.Get() > 3.0) {
						rbt_Drivetrain->SetDrive(0.0, 0.0);
						rbt_Arm->DisableIntake();
						AutonStage++;
					}
					break;
				case 22:
					tmr_TimeOut.Reset();
					rbt_Drivetrain->GoToAngle(-90.0);
					break;
				case 23:
					if (rbt_Drivetrain->IsAtAngle() || tmr_TimeOut.Get() > 2.0) AutonStage++;
					break;
				case 24:
					tmr_TimeOut.Reset();
					rbt_Drivetrain->GoToDistance(56.0);
					AutonStage++;
					break;
				case 25:
					if (rbt_Drivetrain->IsAtDistance() || tmr_TimeOut.Get() > 2.0) AutonStage++;
					break;
				case 26:
					tmr_TimeOut.Reset();
					rbt_Drivetrain->GoToAngle(90.0);
					AutonStage++;
					break;
				case 27:
					if (rbt_Drivetrain->IsAtAngle() || tmr_TimeOut.Get() > 2.0) AutonStage++;
					break;
				case 28:
					tmr_TimeOut.Reset();
					rbt_Drivetrain->GoToDistance(159.0);
					AutonStage++;
					break;
				case 29:
					if (rbt_Drivetrain->IsAtDistance() || tmr_TimeOut.Get() > 2.0) AutonStage++;
					break;
				case 30:
					tmr_TimeOut.Reset();
					rbt_Drivetrain->GoToAngle(90.0);
					AutonStage++;
					break;
				case 31:
					if (rbt_Drivetrain->IsAtAngle() || tmr_TimeOut.Get() > 2.0) AutonStage++;
					break;
				case 32:
					tmr_TimeOut.Reset();
					rbt_Drivetrain->GoToDistance(75.0);
					AutonStage++;
					break;
				case 33:
					if (rbt_Drivetrain->IsAtDistance() || tmr_TimeOut.Get() > 2.0) AutonStage++;
					break;
				case 34:
					tmr_TimeOut.Reset();
					rbt_Drivetrain->GoToDistance(105);
					AutonStage++;
					break;
				case 35:
					if (rbt_Drivetrain->IsAtDistance() || tmr_TimeOut.Get() > 2.0) AutonStage++;
					break;
				case 36:
					tmr_TimeOut.Reset();
					rbt_Arm->ReverseIntake();
					AutonStage++;
					break;
				case 37:
					if (tmr_TimeOut.Get() > 2.0) AutonStage++;
					break;
				default:
					rbt_Drivetrain->SetDrive(0, 0);
					break;
				}
		}

 		void LSt_LSc_LSw(){
			switch(AutonStage){
			case 0:
				tmr_TimeOut.Reset();
				tmr_TimeOut.Start();
				Dashboard.Autonomous.AutonStage.Set("Going Forward 320 inches...");
				rbt_Drivetrain->GoToDistance(320);
				rbt_Arm->Point(rbt_Arm->PointPosition::kForwardShoot);
				AutonStage++;
				break;
			case 1:
				Dashboard.Autonomous.AutonStage.Set("Waiting...");
				if (rbt_Drivetrain->IsAtDistance() || tmr_TimeOut.Get() > 3.0) AutonStage++;
				break;
			case 2:
				tmr_TimeOut.Reset();
				rbt_Drivetrain->GoToAngle(90);
				AutonStage++;
				break;
			case 3:
				if (rbt_Drivetrain->IsAtAngle() || tmr_TimeOut.Get() > 2.0) AutonStage++;
				break;
			case 4:
				tmr_TimeOut.Reset();
				rbt_Arm->Shoot();
				break;
			case 5:

				break;
			default:
				Dashboard.Autonomous.AutonStage.Set("Doing Nothing");
				break;
			}
		}

		void LSt_LSw_RSc() {
			switch(AutonStage){

			}
		}

		void LSt_LSc_RSw() {
			switch(AutonStage) {

			}
		}

		void LSt_RSc_RSw(){
			switch(AutonStage){

			}
		}

		void CSt_LSc_LSw() {

		}

		void CSt_LSw_RSc() {

		}

		void CSt_LSc_RSw() {

		}

		void CSt_RSc_RSw() {

		}

		void RSt_RSc_RSw() {

		}

		void RSt_RSw_LSc() {

		}

		void RSt_RSc_LSw() {

		}

		void RSt_LSc_RSw() {

		}

		void DoNothing(){
			rbt_Drivetrain->ResetAngle();
			rbt_Drivetrain->DisableDistancePID();
			rbt_Drivetrain->DisableAnglePID();
			rbt_Arm->DisableIntake();
			rbt_Arm->Point(RobotMechanism::Arm::PointPosition::kUp);
			rbt_Drivetrain->SetDrive(0.0,0.0);
		}

		void Update(){
			switch(m_ChosenState){
				case AutonStates::kDoNothing: 				DoNothing(); 				break;
				case AutonStates::kCrossLine: 				CrossLine(); 				break;
				case AutonStates::kSwitch:
					switch(m_SelectedPosition) {
						case StartingPosition::kLeft:
							switch(m_SwitchSide) {
								case Configuration::Goal::GoalOrientation::kLeft: LSt_LSw(); break;
								case Configuration::Goal::GoalOrientation::kRight: LSt_RSw(); break;
							}
							break;
						case StartingPosition::kRight:
							switch(m_SwitchSide) {
								case Configuration::Goal::GoalOrientation::kLeft:
									RSt_LSw();
									break;
								case Configuration::Goal::GoalOrientation::kRight:
									RSt_RSw();
									break;
							}
							break;
						case StartingPosition::kCenter:
							switch(m_SwitchSide) {
								case Configuration::Goal::GoalOrientation::kLeft:
									CSt_LSw();
									break;
								case Configuration::Goal::GoalOrientation::kRight:
									CSt_RSw();
									break;
							}
							break;
					}
					break;
				default: DoNothing(); break;
			}
		}
	private:
		RobotMechanism::Drivetrain				*rbt_Drivetrain;
		RobotMechanism::Arm						*rbt_Arm;
		WPILib::Timer							tmr_TimeOut;

		enum class AutonStates {
			kDoNothing,
			kCrossLine,
			kSwitch,
			kSwitchScale,
			kSwitchExchange
		};

		enum class StartingPosition {
			kRight,
			kCenter,
			kLeft
		};

		AutonStates m_ChosenState=AutonStates::kDoNothing;

		StartingPosition m_SelectedPosition = StartingPosition::kLeft;

		Configuration::Goal::GoalOrientation m_ScaleSide = Configuration::Goal::GoalOrientation::kLeft;
		Configuration::Goal::GoalOrientation m_SwitchSide = Configuration::Goal::GoalOrientation::kLeft;

		int AutonStage	=	0;

	public:
		WPILib::SendableChooser<AutonStates> chs_Auton;
		WPILib::SendableChooser<StartingPosition> chs_Position;
	};
}
