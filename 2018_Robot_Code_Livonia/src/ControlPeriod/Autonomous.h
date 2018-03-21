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
			chs_Auton.AddObject("1XScale",AutonStates::k1XScale);
			chs_Auton.AddObject("2XScale",AutonStates::k2XScale);
			chs_Auton.AddObject("3XScale",AutonStates::k3XScale);
			chs_Auton.AddObject("4XScale",AutonStates::k4XScale);

			chs_Position.AddObject("Right", StartingPosition::kRight);
			chs_Position.AddObject("Center", StartingPosition::kCenter);
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

			rbt_Arm->Point(RobotMechanism::Arm::PointPosition::kUp);


		}

		void CrossLine(){
			switch(AutonStage){
			case 0:
				tmr_TimeOut.Reset();
				tmr_TimeOut.Start();
				rbt_Drivetrain->GoToDistance(103.0);
				AutonStage++;
				break;
			case 1:
				if(rbt_Drivetrain->IsAtDistance() || tmr_TimeOut.Get() > 3.0) AutonStage++;
				break;
			default:
				DoNothing();
				break;
			}
		}

		void LSt_LSw(){
			switch(AutonStage) {
				case 0:
					tmr_TimeOut.Reset();
					tmr_TimeOut.Start();
					rbt_Drivetrain->GoToDistance(151.5);
					AutonStage++;
					break;
				case 1:
					if (rbt_Drivetrain->IsAtDistance() || tmr_TimeOut.Get() > 3.0) AutonStage++;
					break;
				case 2:
					tmr_TimeOut.Reset();
					rbt_Drivetrain->DisableDistancePID();
					rbt_Drivetrain->GoToAngle(-90.0);
					rbt_Arm->Point(RobotMechanism::Arm::PointPosition::kReverseSwitch);
					AutonStage++;
					break;
				case 3:
					if (rbt_Drivetrain->IsAtAngle() || tmr_TimeOut.Get() > 1.0) AutonStage++;
					break;
				case 4:
					tmr_TimeOut.Reset();
					rbt_Drivetrain->GoToDistance(-18.0);
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
					rbt_Drivetrain->GoToDistance(216.5);
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
					rbt_Drivetrain->GoToDistance(-237.0);
					AutonStage++;
					break;
				case 5:
					if (rbt_Drivetrain->IsAtDistance() || tmr_TimeOut.Get() > 3) AutonStage++;
					break;
				case 6:
					tmr_TimeOut.Reset();
					rbt_Drivetrain->DisableDistancePID();
					rbt_Drivetrain->GoToAngle(90.0);
					AutonStage++;
					break;
				case 7:
					if (rbt_Drivetrain->IsAtAngle() || tmr_TimeOut.Get() > 3.0) AutonStage++;
					break;
				case 8:
					tmr_TimeOut.Reset();
					rbt_Drivetrain->GoToDistance(-6.0);
					AutonStage++;
					break;
				case 9:
					if(rbt_Drivetrain->IsAtDistance() || tmr_TimeOut.Get() > 2.0) AutonStage++;
					break;
				case 10:
					tmr_TimeOut.Reset();
					rbt_Arm->Point(RobotMechanism::Arm::PointPosition::kReverseSwitch);
					rbt_Drivetrain->GoToAngle(90.0);
					AutonStage++;
					break;
				case 11:
					if (rbt_Drivetrain->IsAtAngle() || tmr_TimeOut.Get() > 2.0) AutonStage++;
					break;
				case 12:
					tmr_TimeOut.Reset();
					rbt_Arm->ReverseIntake();
					AutonStage++;
					break;
				case 13:
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
				rbt_Drivetrain->GoToDistance(148.5);
				AutonStage++;
				break;
			case 1:
				if (rbt_Drivetrain->IsAtDistance() || tmr_TimeOut.Get() > 3.0) AutonStage++;
				break;
			case 2:
				tmr_TimeOut.Reset();
				rbt_Drivetrain->DisableDistancePID();
				rbt_Drivetrain->GoToAngle(90.0);
				rbt_Arm->Point(RobotMechanism::Arm::PointPosition::kReverseSwitch);
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
				rbt_Drivetrain->GoToDistance(210.5);
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
				if (rbt_Drivetrain->IsAtAngle() || tmr_TimeOut.Get() > 2.0) AutonStage++;
				break;
			case 4:
				tmr_TimeOut.Reset();
				rbt_Drivetrain->ConfigScale(0.5, 0.5, -0.5, -0.5);
				rbt_Drivetrain->GoToDistance(-237.0);
				AutonStage++;
				break;
			case 5:
				if (rbt_Drivetrain->IsAtDistance() || tmr_TimeOut.Get() > 3) AutonStage++;
				break;
			case 6:
				tmr_TimeOut.Reset();
				rbt_Drivetrain->ConfigScale(0.66, 0.66, -0.66, -0.66);
				rbt_Drivetrain->DisableDistancePID();
				rbt_Drivetrain->GoToAngle(90.0);
				AutonStage++;
				break;
			case 7:
				if (rbt_Drivetrain->IsAtAngle() || tmr_TimeOut.Get() > 3.0) AutonStage++;
				break;
			case 8:
				tmr_TimeOut.Reset();
				rbt_Drivetrain->GoToDistance(16.0);
				AutonStage++;
				break;
			case 9:
				if(rbt_Drivetrain->IsAtDistance() || tmr_TimeOut.Get() > 2.0) AutonStage++;
				break;
			case 10:
				tmr_TimeOut.Reset();
				rbt_Arm->Point(RobotMechanism::Arm::PointPosition::kReverseSwitch);
				rbt_Drivetrain->GoToAngle(-90.0);
				AutonStage++;
				break;
			case 11:
				if (rbt_Drivetrain->IsAtDistance() || tmr_TimeOut.Get() > 2.0) AutonStage++;
				break;
			case 12:
				tmr_TimeOut.Reset();
				rbt_Arm->ReverseIntake();
				AutonStage++;
				break;
			case 13:
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
					rbt_Drivetrain->GoToDistance(37.5);
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
					rbt_Arm->Point(RobotMechanism::Arm::PointPosition::kForwardSwitch);
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
					rbt_Drivetrain->GoToDistance(22.5);
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
					rbt_Drivetrain->GoToDistance(80);
					AutonStage++;
					break;
				case 5:
					if(rbt_Drivetrain->IsAtDistance() || tmr_TimeOut.Get() > 3.0) AutonStage++;
					break;
				case 6:
					tmr_TimeOut.Reset();
					rbt_Arm->Point(RobotMechanism::Arm::PointPosition::kForwardSwitch);
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
					rbt_Drivetrain->GoToDistance(157.5);
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
					rbt_Drivetrain->GoToDistance(36.0);
					rbt_Arm->Point(rbt_Arm->PointPosition::kForwardSwitch);
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
				rbt_Drivetrain->GoToDistance(42.5);
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
				rbt_Drivetrain->GoToDistance(42.5);
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
				rbt_Drivetrain->GoToDistance(39.5);
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
				rbt_Drivetrain->GoToDistance(42.5);
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
					rbt_Drivetrain->GoToDistance(154.5);
					AutonStage++;
					break;
				case 1:
					if (rbt_Drivetrain->IsAtDistance() || tmr_TimeOut.Get() > 3.0) AutonStage++;
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
					rbt_Drivetrain->GoToDistance(36.0);
					rbt_Arm->Point(rbt_Arm->PointPosition::kForwardSwitch);
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

		void LSt_LSc_LSw() {
		  switch(AutonStage) {
		    case 0:
		      tmr_TimeOut.Reset();
		      tmr_TimeOut.Start();
		      rbt_Arm->Point(rbt_Arm->PointPosition::kForwardSwitch);
		      rbt_Drivetrain->GoToDistance(320);
		      AutonStage++;
		      break;
		    case 1:
		      if (rbt_Drivetrain->IsAtDistance() || tmr_TimeOut.Get() > 4.0) AutonStage++;
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
		      AutonStage++;
		      break;
		    case 5:
		      if (tmr_TimeOut.Get() > 2.0) AutonStage++;
		      break;
		    case 6:
		      tmr_TimeOut.Reset();
		      rbt_Drivetrain->GoToAngle(90);
		      break;
		    case 7:
		      if (rbt_Drivetrain->IsAtAngle() || tmr_TimeOut.Get() > 2.0) AutonStage++;
		      break;
		    case 8:
		      tmr_TimeOut.Reset();
		      rbt_Drivetrain->GoToDistance(107);
		      break;
		    case 9:
		      if (rbt_Drivetrain->IsAtDistance() || tmr_TimeOut.Get() > 3.0) AutonStage++;
		      break;
		    case 10:
		      tmr_TimeOut.Reset();
		      rbt_Arm->Point(rbt_Arm->PointPosition::kForwardCollect);
		      rbt_Drivetrain->GoToAngle(-90);
		      break;
		    case 11:
		      if (rbt_Drivetrain->IsAtAngle() || tmr_TimeOut.Get() > 2.0) AutonStage++;
		      break;
		    case 12:
		      tmr_TimeOut.Reset();
		      rbt_Drivetrain->GoToDistance(56);
		      break;
		    case 13:
		      if (rbt_Drivetrain->IsAtDistance() || tmr_TimeOut.Get() > 2.0) AutonStage++;
		      break;
		    case 14:
		      tmr_TimeOut.Reset();
		      rbt_Drivetrain->GoToAngle(90);
		      break;
		    case 15:
		      if (rbt_Drivetrain->IsAtAngle() || tmr_TimeOut.Get() > 2.0) AutonStage++;
		      break;
		    case 16:
		      tmr_TimeOut.Reset();
		      rbt_Arm->EnableIntake();
		      rbt_Drivetrain->SetDrive(0.5, 0.5);
		      break;
		    case 17:
		      if (tmr_TimeOut.Get() > 2.0) AutonStage++;
		      break;
		    case 18:
		      tmr_TimeOut.Reset();
		      rbt_Arm->DisableIntake();
		      rbt_Arm->Point(rbt_Arm->PointPosition::kForwardSwitch);
		      break;
		    case 19:
		      if (rbt_Arm->IsAtAngle() || tmr_TimeOut.Get() > 3.0) AutonStage++;
		      break;
		    case 20:
		      tmr_TimeOut.Reset();
		      rbt_Arm->ReverseIntake();
		      break;
		    case 21:
		      if (tmr_TimeOut.Get() > 2.0) AutonStage++;
		      break;
		  }
		}

		void LSt_LSc_RSw() {
		  switch(AutonStage) {
		  case 0:
		    tmr_TimeOut.Reset();
		    tmr_TimeOut.Start();
		    rbt_Arm->Point(rbt_Arm->PointPosition::kForwardSwitch);
		    rbt_Drivetrain->GoToDistance(320);
		    AutonStage++;
		    break;
		  case 1:
		    if (rbt_Drivetrain->IsAtDistance() || tmr_TimeOut.Get() > 4.0) AutonStage++;
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
		    AutonStage++;
		    break;
		  case 5:
		    if (tmr_TimeOut.Get() > 2.0) AutonStage++;
		    break;
		  case 6:
		    tmr_TimeOut.Reset();
		    rbt_Drivetrain->GoToAngle(90);
		    break;
		  case 7:
		    if (rbt_Drivetrain->IsAtAngle() || tmr_TimeOut.Get() > 2.0) AutonStage++;
		    break;
		  case 8:
		    tmr_TimeOut.Reset();
		    rbt_Drivetrain->GoToDistance(107);
		    break;
		  case 9:
		    if (rbt_Drivetrain->IsAtDistance() || tmr_TimeOut.Get() > 3.0) AutonStage++;
		    break;
		  case 10:
		    tmr_TimeOut.Reset();
		    rbt_Arm->Point(rbt_Arm->PointPosition::kForwardCollect);
		    rbt_Drivetrain->GoToAngle(-90);
		    break;
		  case 11:
		    if (rbt_Drivetrain->IsAtAngle() || tmr_TimeOut.Get() > 2.0) AutonStage++;
		    break;
		  case 12:
		    tmr_TimeOut.Reset();
		    rbt_Drivetrain->GoToDistance(56);
		    AutonStage++;
		    break;
		  case 13:
		    if (rbt_Drivetrain->IsAtDistance() || tmr_TimeOut.Get() > 2.0) AutonStage++;
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
		    rbt_Arm->EnableIntake();
		    rbt_Drivetrain->SetDrive(0.5, 0.5);
		    AutonStage++;
		    break;
		  case 17:
		    if (tmr_TimeOut.Get() > 2.0) {
		      rbt_Arm->DisableIntake();
		      rbt_Drivetrain->SetDrive(0, 0);
		      AutonStage++;
		    }
		    break;
		  case 18:
		    tmr_TimeOut.Reset();
		    rbt_Arm->Point(rbt_Arm->PointPosition::kForwardSwitch);
		    rbt_Drivetrain->GoToDistance(-5);
		    AutonStage++;
		    break;
		  case 19:
		    if (rbt_Drivetrain->IsAtDistance() || tmr_TimeOut.Get() > 1.0) AutonStage++;
		    break;
		  case 20:
		    tmr_TimeOut.Reset();
		    rbt_Arm->ReverseIntake();
		    AutonStage++;
		    break;
		  case 21:
		    if (tmr_TimeOut.Get() > 1.0) {
		      rbt_Arm->DisableIntake();
		      AutonStage++;
		    }
		    break;
		  }
		}

		void LSt_LSw_RSc() {
		  switch(AutonStage) {
		    case 0:
		      tmr_TimeOut.Reset();
		      rbt_Drivetrain->GoToDistance(169.5);
		      rbt_Arm->Point(rbt_Arm->PointPosition::kForwardSwitch);
		      AutonStage++;
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
		      rbt_Drivetrain->GoToDistance(18);
		      break;
		    case 5:
		      if (rbt_Drivetrain->IsAtDistance() || tmr_TimeOut.Get() > 2.0) AutonStage++;
		      break;
		    case 6:
		      tmr_TimeOut.Reset();
		      rbt_Arm->ReverseIntake();
		      AutonStage++;
		      break;
		    case 7:
		      if (tmr_TimeOut.Get() > 1.0) AutonStage++;
		      break;
		    case 8:
		      tmr_TimeOut.Reset();
		      rbt_Drivetrain->GoToAngle(-90);
		      AutonStage++;
		      break;
		    case 9:
		      if (rbt_Drivetrain->IsAtAngle() || tmr_TimeOut.Get() > 2.0) AutonStage++;
		      break;
		    case 10:
		      tmr_TimeOut.Reset();
		      rbt_Drivetrain->GoToDistance(56);
		      AutonStage++;
		      break;
		    case 11:
		      if (rbt_Drivetrain->IsAtDistance() || tmr_TimeOut.Get() > 2.0) AutonStage++;
		      break;
		    case 12:
		      tmr_TimeOut.Reset();
		      rbt_Drivetrain->GoToAngle(90);
		      rbt_Arm->Point(rbt_Arm->PointPosition::kForwardCollect);
		      AutonStage++;
		      break;
		    case 13:
		      if (rbt_Drivetrain->IsAtDistance() || tmr_TimeOut.Get() > 2.0) AutonStage++;
		      break;
		    case 14:
		      tmr_TimeOut.Reset();
		      rbt_Drivetrain->GoToDistance(29);
		      AutonStage++;
		      break;
		    case 15:
		      if (rbt_Drivetrain->IsAtDistance() || tmr_TimeOut.Get() > 2.0) AutonStage++;
		      break;
		    case 16:
		      tmr_TimeOut.Reset();
		      rbt_Drivetrain->GoToAngle(90);
		      AutonStage++;
		      break;
		    case 17:
		      if (rbt_Drivetrain->IsAtAngle() || tmr_TimeOut.Get() > 1.0) AutonStage++;
		      break;
		    case 18:
		      tmr_TimeOut.Reset();
		      rbt_Arm->EnableIntake();
		      rbt_Drivetrain->SetDrive(0.5, 0.5);
		      AutonStage++;
		      break;
		    case 19:
		      if (tmr_TimeOut.Get() > 2.0) {
		        rbt_Arm->DisableIntake();
		        rbt_Drivetrain->SetDrive(0, 0);
		        AutonStage++;
		      }
		      break;
		    case 20:
		      tmr_TimeOut.Reset();
		      rbt_Drivetrain->GoToAngle(-90);
		      rbt_Arm->Point(rbt_Arm->PointPosition::kForwardSwitch);
		      AutonStage++;
		      break;
		    case 21:
		      if (rbt_Drivetrain->IsAtAngle() || tmr_TimeOut.Get() > 1.0) AutonStage++;
		      break;
		    case 22:
		      tmr_TimeOut.Reset();
		      rbt_Drivetrain->GoToDistance(207);
		      AutonStage++;
		      break;
		    case 23:
		      if (rbt_Drivetrain->IsAtDistance() || tmr_TimeOut.Get() > 3.0) AutonStage++;
		      break;
		    case 24:
		      tmr_TimeOut.Reset();
		      rbt_Drivetrain->GoToAngle(-90);
		      AutonStage++;
		      break;
		    case 25:
		      if (rbt_Drivetrain->IsAtAngle() || tmr_TimeOut.Get() > 1.0) AutonStage++;
		      break;
		    case 26:
		      tmr_TimeOut.Reset();
		      rbt_Drivetrain->GoToDistance(36);
		      AutonStage++;
		      break;
		    case 27:
		      if (rbt_Drivetrain->IsAtDistance() || tmr_TimeOut.Get() > 2.0) AutonStage++;
		      break;
		    case 28:
		      tmr_TimeOut.Reset();
		      rbt_Drivetrain->GoToAngle(-90);
		      AutonStage++;
		      break;
		    case 29:
		      if (rbt_Drivetrain->IsAtAngle() || tmr_TimeOut.Get() > 1.0) AutonStage++;
		      break;
		    case 30:
		      tmr_TimeOut.Reset();
		      rbt_Arm->ReverseIntake();
		      AutonStage++;
		      break;
		    case 31:
		      if (tmr_TimeOut.Get() > 1.5) {
		        rbt_Arm->DisableIntake();
		        AutonStage++;
		      }
		      break;
		  }
		}

		void LSt_RSc_RSw() {
		  switch(AutonStage) {
		    case 0:
		      tmr_TimeOut.Reset();
		      rbt_Drivetrain->GoToDistance(230);
		      AutonStage++;
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
		      if (rbt_Drivetrain->IsAtAngle() || tmr_TimeOut.Get() > 1.0) AutonStage++;
		      break;
		    case 4:
		      tmr_TimeOut.Reset();
		      rbt_Drivetrain->GoToDistance(250);
		      AutonStage++;
		      break;
		    case 5:
		      if (rbt_Drivetrain->IsAtDistance() || tmr_TimeOut.Get() > 2.0) AutonStage++;
		      break;
		    case 6:
		      tmr_TimeOut.Reset();
		      rbt_Arm->Point(rbt_Arm->PointPosition::kForwardSwitch);
		      rbt_Drivetrain->GoToAngle(-90);
		      AutonStage++;
		      break;
		    case 7:
		      if (rbt_Drivetrain->IsAtAngle() || tmr_TimeOut.Get() > 1.0) AutonStage++;
		      break;
		    case 8:
		      tmr_TimeOut.Reset();
		      rbt_Drivetrain->GoToDistance(36);
		      AutonStage++;
		      break;
		    case 9:
		      if (rbt_Drivetrain->IsAtDistance() || tmr_TimeOut.Get() > 2.0) AutonStage++;
		      break;
		    case 10:
		      tmr_TimeOut.Reset();
		      rbt_Drivetrain->GoToAngle(-90);
		      AutonStage++;
		      break;
		    case 11:
		      if (rbt_Drivetrain->IsAtAngle() || tmr_TimeOut.Get() > 1.0) AutonStage++;
		      break;
		    case 12:
		      tmr_TimeOut.Reset();
		      rbt_Arm->ReverseIntake();
		      AutonStage++;
		      break;
		    case 13:
		      if (tmr_TimeOut.Get() > 1.5) {
		        rbt_Arm->DisableIntake();
		        AutonStage++;
		      }
		      break;
		    case 14:
		      tmr_TimeOut.Reset();
		      rbt_Arm->Point(rbt_Arm->PointPosition::kForwardCollect);
		      rbt_Drivetrain->GoToAngle(-90);
		      AutonStage++;
		      break;
		    case 15:
		      if (rbt_Drivetrain->IsAtAngle() > 1.0) AutonStage++;
		      break;
		    case 16:
		      tmr_TimeOut.Reset();
		      rbt_Drivetrain->GoToDistance(87);
		      AutonStage++;
		      break;
		    case 17:
		      if (rbt_Drivetrain->IsAtDistance() || tmr_TimeOut.Get() > 2.0) AutonStage++;
		      break;
		    case 18:
		      tmr_TimeOut.Reset();
		      rbt_Drivetrain->GoToAngle(90);
		      AutonStage++;
		      break;
		    case 19:
		      if (rbt_Drivetrain->IsAtAngle() || tmr_TimeOut.Get() > 1.0) AutonStage++;
		      break;
		    case 20:
		      tmr_TimeOut.Reset();
		      rbt_Drivetrain->GoToDistance(47);
		      AutonStage++;
		      break;
		    case 21:
		      if (rbt_Drivetrain->IsAtDistance() || tmr_TimeOut.Get() > 2.0) AutonStage++;
		      break;
		    case 22:
		      tmr_TimeOut.Reset();
		      rbt_Drivetrain->GoToAngle(-90);
		      AutonStage++;
		      break;
		    case 23:
		      if (rbt_Drivetrain->IsAtAngle() || tmr_TimeOut.Get() > 1.0) AutonStage++;
		      break;
		    case 24:
		      tmr_TimeOut.Reset();
		      rbt_Arm->EnableIntake();
		      rbt_Drivetrain->SetDrive(0.5, 0.5);
		      AutonStage++;
		      break;
		    case 25:
		      if (tmr_TimeOut.Get() > 2.0) {
		        rbt_Drivetrain->SetDrive(0, 0);
		        rbt_Arm->DisableIntake();
		        AutonStage++;
		      }
		      break;
		    case 26:
		      tmr_TimeOut.Reset();
		      rbt_Arm->Point(rbt_Arm->PointPosition::kForwardSwitch);
		      AutonStage++;
		      break;
		    case 27:
		      if (rbt_Arm->IsAtAngle() || tmr_TimeOut.Get() > 2.0) AutonStage++;
		      break;
		    case 28:
		      tmr_TimeOut.Reset();
		      rbt_Arm->ReverseIntake();
		      AutonStage++;
		      break;
		    case 29:
		      if (tmr_TimeOut.Get() > 1.5) {
		        rbt_Arm->DisableIntake();
		        AutonStage++;
		      }
		      break;
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

		void RSt_LSc_LSw() {

		}

		void LSt_LSc_1x() {
			switch(AutonStage) {
			case 0:
				tmr_TimeOut.Reset();
				tmr_TimeOut.Start();
				rbt_Drivetrain->GoToDistance(309.552);
				rbt_Arm->Point(rbt_Arm->PointPosition::kForwardScale);
				AutonStage++;
				break;
			case 1:
				if (rbt_Drivetrain->IsAtDistance() || tmr_TimeOut.Get() > 4.0) AutonStage++;
				break;
			case 2:
				tmr_TimeOut.Reset();
				rbt_Drivetrain->GoToAngle(90.0);
				AutonStage++;
				break;
			case 3:
				if (rbt_Drivetrain->IsAtAngle() || tmr_TimeOut.Get() > 1.0) AutonStage++;
				break;
			case 4:
				tmr_TimeOut.Reset();
				rbt_Drivetrain->GoToDistance(24.0);
				AutonStage++;
				break;
			case 5:
				if (rbt_Drivetrain->IsAtDistance() || tmr_TimeOut.Get() > 2.0) AutonStage++;
				break;
			case 6:
				tmr_TimeOut.Reset();
				rbt_Arm->Shoot();
				AutonStage++;4
				break;
			}
		}

		void LSt_RSc_1x() {
			switch(AutonStage) {
			case 0:
				tmr_TimeOut.Reset();
				tmr_TimeOut.Start();
				rbt_Drivetrain->GoToDistance(219.408);
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
				if (rbt_Drivetrain->IsAtAngle() || tmr_TimeOut.Get() > 1.0) AutonStage++;
				break;
			case 4:
				tmr_TimeOut.Reset();
				rbt_Arm->Point(rbt_Arm->PointPosition::kReverseScale);
				rbt_Drivetrain->GoToDistance(238.998);
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
				if (rbt_Drivetrain->IsAtAngle() || tmr_TimeOut.Get() > 1.0) AutonStage++;
				break;
			case 8:
				tmr_TimeOut.Reset();
				rbt_Drivetrain->GoToDistance(90.144);
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
				if (rbt_Drivetrain->IsAtAngle() || tmr_TimeOut.Get() > 1.0) AutonStage++;
				break;
			case 12:
				tmr_TimeOut.Reset();
				rbt_Arm->Shoot();
				AutonStage++;
				break;
			case 13:
				if (tmr_TimeOut.Get() > 1.5) {
					rbt_Arm->DisableIntake();
					AutonStage++;
				}
				break;
			}
		}

		void RSt_LSc_1x() {

		}

		void RSt_RSc_1x() {

		}

		void LSt_LSc_2x() {
			switch(AutonStage) {
			case 0:
				tmr_TimeOut.Reset();
				tmr_TimeOut.Start();
				rbt_Drivetrain->GoToDistance(309.552);
				rbt_Arm->Point(rbt_Arm->PointPosition::kReverseScale);
				AutonStage++;
				break;
			case 1:
				if (rbt_Drivetrain->IsAtDistance() || tmr_TimeOut.Get() > 4.0) AutonStage++;
				break;
			case 2:
				tmr_TimeOut.Reset();
				rbt_Drivetrain->GoToAngle(-90.0);
				AutonStage++;
				break;
			case 3:
				if (rbt_Drivetrain->IsAtAngle() || tmr_TimeOut.Get() > 1.0) AutonStage++;
				break;
			case 4:
				tmr_TimeOut.Reset();
				rbt_Arm->Shoot();
				break;
			case 5:
				if (tmr_TimeOut.Get() > 1.5) {
					rbt_Arm->DisableIntake();
					AutonStage++;
				}
				break;
			case 6:
				tmr_TimeOut.Reset();
				rbt_Arm->Point(rbt_Arm->PointPosition::kForwardCollect);
				rbt_Drivetrain->GoToAngle(-90.0);
				AutonStage++;
				break;
			case 7:
				if (rbt_Drivetrain->IsAtAngle() || tmr_TimeOut.Get() > 1.0) AutonStage++;
				break;
			case 8:
				tmr_TimeOut.Reset();
				rbt_Drivetrain->GoToDistance(86.196);
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
				if (rbt_Drivetrain->IsAtDistance() || tmr_TimeOut.Get() > 1.0) AutonStage++;
				break;
			case 12:
				tmr_TimeOut.Reset();
				rbt_Drivetrain->GoToDistance(47.016);
				AutonStage++;
				break;
			case 13:
				if (rbt_Drivetrain->IsAtDistance() || tmr_TimeOut.Get() > 1.0) AutonStage++;
				break;
			case 14:
				tmr_TimeOut.Reset();
				rbt_Drivetrain->GoToAngle(90.0);
				AutonStage++;
				break;
			case 15:
				if (rbt_Drivetrain->IsAtAngle() || tmr_TimeOut.Get() > 1.0) AutonStage++;
				break;
			case 16:
				tmr_TimeOut.Reset();
				rbt_Arm->EnableIntake();
				rbt_Drivetrain->SetDrive(0.5, 0.5);
				break;
			case 17:
				if (tmr_TimeOut.Get() > 1.5) {
					rbt_Drivetrain->SetDrive(0, 0);
					rbt_Arm->DisableIntake();
					AutonStage++;
				}
				break;
			case 18:
				tmr_TimeOut.Reset();
				rbt_Arm->Point(rbt_Arm->PointPosition::kReverseScale);
				rbt_Drivetrain->GoToAngle(90.0);
				break;
			case 19:
				if (rbt_Drivetrain->IsAtAngle() || tmr_TimeOut.Get() > 1.0) AutonStage++;
				break;
			case 20:
				tmr_TimeOut.Reset();
				rbt_Drivetrain->GoToDistance(27.426);
				AutonStage++;
				break;
			case 21:
				if (rbt_Drivetrain->IsAtDistance() || tmr_TimeOut.Get() > 2.0) AutonStage++;
				break;
			case 22:
				tmr_TimeOut.Reset();
				rbt_Drivetrain->GoToAngle(90.0);
				AutonStage++;
				break;
			case 23:
				if (rbt_Drivetrain->IsAtAngle() || tmr_TimeOut.Get() > 1.0) AutonStage++;
				break;
			case 24:
				tmr_TimeOut.Reset();
				rbt_Drivetrain->GoToDistance(110);
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
				if (rbt_Drivetrain->IsAtAngle() || tmr_TimeOut.Get() > 1.0) AutonStage++;
				break;
			case 28:
				tmr_TimeOut.Reset();
				rbt_Arm->Shoot();
				AutonStage++;
				break;
			case 29:
				if (tmr_TimeOut.Get() > 1.5) {
					rbt_Arm->DisableIntake();
					AutonStage++;
				}
				break;
			}
		}

		void LSt_RSc_2x() {
			switch(AutonStage) {
			case 0:
				tmr_TimeOut.Reset();
				rbt_Arm->Point(rbt_Arm->PointPosition::kReverseScale);
				rbt_Drivetrain->GoToDistance(228.735);
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
				if (rbt_Drivetrain->IsAtAngle() || tmr_TimeOut.Get() > 1.0) AutonStage++;
				break;
			case 4:
				tmr_TimeOut.Reset();
				rbt_Drivetrain->GoToDistance(230);
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
				if (rbt_Drivetrain->IsAtDistance() || tmr_TimeOut.Get()) AutonStage++;
				break;
			case 8:
				tmr_TimeOut.Reset();
				rbt_Drivetrain->GoToDistance(70.915);
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
				if (rbt_Drivetrain->IsAtAngle() || tmr_TimeOut.Get() > 1.0) AutonStage++;
				break;
			case 12:
				tmr_TimeOut.Reset();
				rbt_Arm->Shoot();
				AutonStage++;
				break;
			case 13:
				if (tmr_TimeOut.Get() > 1.5) {
					rbt_Arm->DisableIntake();
					AutonStage++;
				}
				break;
			case 14:
				tmr_TimeOut.Reset();
				rbt_Drivetrain->GoToAngle(-90.0);
				AutonStage++;
				break;
			case 15:
				if (rbt_Drivetrain->IsAtAngle() || tmr_TimeOut.Get() > 1.0) AutonStage++;
				break;
			case 16:
				tmr_TimeOut.Reset();
				rbt_Drivetrain->GoToDistance(70.915);
				AutonStage++;
				break;
			case 17:
				if (rbt_Drivetrain->IsAtDistance() || tmr_TimeOut.Get() > 2.0) AutonStage++;
				break;
			case 18:
				tmr_TimeOut.Reset();
				rbt_Arm->Point(rbt_Arm->PointPosition::kReverseCollect);
				rbt_Drivetrain->GoToAngle(90.0);
				AutonStage++;
				break;
			case 19:
				if (rbt_Drivetrain->IsAtAngle() || tmr_TimeOut.Get() > 1.0) AutonStage++;
				break;
			case 20:
				tmr_TimeOut.Reset();
				rbt_Drivetrain->GoToDistance(40.75);
				AutonStage++;
				break;
			case 21:
				if (rbt_Drivetrain->IsAtDistance() || tmr_TimeOut.Get() > 2.0) AutonStage++;
				break;
			case 22:
				tmr_TimeOut.Reset();
				rbt_Drivetrain->GoToAngle(90.0);
				AutonStage++;
				break;
			case 23:
				if (rbt_Drivetrain->IsAtAngle() || tmr_TimeOut.Get() > 1.0) AutonStage++;
				break;
			case 24:
				tmr_TimeOut.Reset();
				rbt_Arm->EnableIntake();
				rbt_Drivetrain->SetDrive(0.5, 0.5);
				AutonStage++;
				break;
			case 25:
				if (tmr_TimeOut.Get() > 1.5) {
					rbt_Drivetrain->SetDrive(0, 0);
					rbt_Arm->DisableIntake();
					AutonStage++;
				}
				break;
			case 26:
				tmr_TimeOut.Reset();
				rbt_Arm->Point(rbt_Arm->PointPosition::kReverseScale);
				rbt_Drivetrain->GoToAngle(90.0);
				AutonStage++;
				break;
			case 27:
				if (rbt_Drivetrain->IsAtAngle() || tmr_TimeOut.Get() > 1.0) AutonStage++;
				break;
			case 29:
				tmr_TimeOut.Reset();
				rbt_Drivetrain->GoToDistance(40.75);
				AutonStage++;
				break;
			case 30:
				if (rbt_Drivetrain->IsAtDistance() || tmr_TimeOut.Get() > 2.0) AutonStage++;
				break;
			case 31:
				tmr_TimeOut.Reset();
				rbt_Drivetrain->GoToAngle(-90.0);
				AutonStage++;
				break;
			case 32:
				if (rbt_Drivetrain->IsAtAngle() || tmr_TimeOut.Get() > 1.0) AutonStage++;
				break;
			case 33:
				tmr_TimeOut.Reset();
				rbt_Drivetrain->GoToDistance(70.915);
				AutonStage++;
				break;
			case 34:
				if (rbt_Drivetrain->IsAtDistance() || tmr_TimeOut.Get() > 2.0) AutonStage++;
				break;
			case 35:
				tmr_TimeOut.Reset();
				rbt_Drivetrain->GoToAngle(-90.0);
				AutonStage++;
				break;
			case 36:
				if (rbt_Drivetrain->IsAtAngle() || tmr_TimeOut.Get() > 2.0) AutonStage++;
				break;
			case 37:
				tmr_TimeOut.Reset();
				rbt_Arm->Shoot();
				AutonStage++;
				break;
			case 38:
				if (tmr_TimeOut.Get() > 1.5) {
					rbt_Arm->DisableIntake();
					AutonStage++;
				}
				break;
			}
		}

		void RSt_LSc_2x() {

		}

		void RSt_RSc_2x() {

		}

		void LSt_LSc_3x() {
			switch(AutonStage) {
			case 0:
				tmr_TimeOut.Reset();
				tmr_TimeOut.Start();
				rbt_Drivetrain->GoToDistance(309.552);
				rbt_Arm->Point(rbt_Arm->PointPosition::kReverseScale);
				AutonStage++;
				break;
			case 1:
				if (rbt_Drivetrain->IsAtDistance() || tmr_TimeOut.Get() > 4.0) AutonStage++;
				break;
			case 2:
				tmr_TimeOut.Reset();
				rbt_Drivetrain->GoToAngle(-90.0);
				AutonStage++;
				break;
			case 3:
				if (rbt_Drivetrain->IsAtAngle() || tmr_TimeOut.Get() > 1.0) AutonStage++;
				break;
			case 4:
				tmr_TimeOut.Reset();
				rbt_Arm->Shoot();
				break;
			case 5:
				if (tmr_TimeOut.Get() > 1.5) {
					rbt_Arm->DisableIntake();
					AutonStage++;
				}
				break;
			case 6:
				tmr_TimeOut.Reset();
				rbt_Arm->Point(rbt_Arm->PointPosition::kForwardCollect);
				rbt_Drivetrain->GoToAngle(-90.0);
				AutonStage++;
				break;
			case 7:
				if (rbt_Drivetrain->IsAtAngle() || tmr_TimeOut.Get() > 1.0) AutonStage++;
				break;
			case 8:
				tmr_TimeOut.Reset();
				rbt_Drivetrain->GoToDistance(86.196);
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
				if (rbt_Drivetrain->IsAtDistance() || tmr_TimeOut.Get() > 1.0) AutonStage++;
				break;
			case 12:
				tmr_TimeOut.Reset();
				rbt_Drivetrain->GoToDistance(47.016);
				AutonStage++;
				break;
			case 13:
				if (rbt_Drivetrain->IsAtDistance() || tmr_TimeOut.Get() > 1.0) AutonStage++;
				break;
			case 14:
				tmr_TimeOut.Reset();
				rbt_Drivetrain->GoToAngle(90.0);
				AutonStage++;
				break;
			case 15:
				if (rbt_Drivetrain->IsAtAngle() || tmr_TimeOut.Get() > 1.0) AutonStage++;
				break;
			case 16:
				tmr_TimeOut.Reset();
				rbt_Arm->EnableIntake();
				rbt_Drivetrain->SetDrive(0.5, 0.5);
				break;
			case 17:
				if (tmr_TimeOut.Get() > 1.5) {
					rbt_Drivetrain->SetDrive(0, 0);
					rbt_Arm->DisableIntake();
					AutonStage++;
				}
				break;
			case 18:
				tmr_TimeOut.Reset();
				rbt_Arm->Point(rbt_Arm->PointPosition::kReverseScale);
				rbt_Drivetrain->GoToAngle(90.0);
				break;
			case 19:
				if (rbt_Drivetrain->IsAtAngle() || tmr_TimeOut.Get() > 1.0) AutonStage++;
				break;
			case 20:
				tmr_TimeOut.Reset();
				rbt_Drivetrain->GoToDistance(27.426);
				AutonStage++;
				break;
			case 21:
				if (rbt_Drivetrain->IsAtDistance() || tmr_TimeOut.Get() > 2.0) AutonStage++;
				break;
			case 22:
				tmr_TimeOut.Reset();
				rbt_Drivetrain->GoToAngle(90.0);
				AutonStage++;
				break;
			case 23:
				if (rbt_Drivetrain->IsAtAngle() || tmr_TimeOut.Get() > 1.0) AutonStage++;
				break;
			case 24:
				tmr_TimeOut.Reset();
				rbt_Drivetrain->GoToDistance(110);
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
				if (rbt_Drivetrain->IsAtAngle() || tmr_TimeOut.Get() > 1.0) AutonStage++;
				break;
			case 28:
				tmr_TimeOut.Reset();
				rbt_Arm->Shoot();
				AutonStage++;
				break;
			case 29:
				if (tmr_TimeOut.Get() > 1.5) {
					rbt_Arm->DisableIntake();
					AutonStage++;
				}
				break;
			case 30:
				tmr_TimeOut.Reset();
				rbt_Arm->Point(rbt_Arm->PointPosition::kForwardCollect);
				rbt_Drivetrain->GoToAngle(-90.0);
				AutonStage++;
				break;
			case 31:
				if (rbt_Drivetrain->IsAtAngle() || tmr_TimeOut.Get() > 1.0) AutonStage++;
				break;
			case 32:
				tmr_TimeOut.Reset();
				rbt_Drivetrain->GoToDistance(86.196);
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
				if (rbt_Drivetrain->IsAtDistance() || tmr_TimeOut.Get() > 1.0) AutonStage++;
				break;
			case 36:
				tmr_TimeOut.Reset();
				rbt_Drivetrain->GoToDistance(66.75);
				AutonStage++;
				break;
			case 37:
				if (rbt_Drivetrain->IsAtDistance() || tmr_TimeOut.Get() > 1.0) AutonStage++;
				break;
			case 38:
				tmr_TimeOut.Reset();
				rbt_Drivetrain->GoToAngle(90.0);
				AutonStage++;
				break;
			case 39:
				if (rbt_Drivetrain->IsAtAngle() || tmr_TimeOut.Get() > 1.0) AutonStage++;
				break;
			case 40:
				tmr_TimeOut.Reset();
				rbt_Arm->EnableIntake();
				rbt_Drivetrain->SetDrive(0.5, 0.5);
				break;
			case 41:
				if (tmr_TimeOut.Get() > 1.5) {
					rbt_Drivetrain->SetDrive(0, 0);
					rbt_Arm->DisableIntake();
					AutonStage++;
				}
				break;
			case 42:
				tmr_TimeOut.Reset();
				rbt_Arm->Point(rbt_Arm->PointPosition::kReverseScale);
				rbt_Drivetrain->GoToAngle(90.0);
				break;
			case 43:
				if (rbt_Drivetrain->IsAtAngle() || tmr_TimeOut.Get() > 1.0) AutonStage++;
				break;
			case 44:
				tmr_TimeOut.Reset();
				rbt_Drivetrain->GoToDistance(66.75);
				AutonStage++;
				break;
			case 45:
				if (rbt_Drivetrain->IsAtDistance() || tmr_TimeOut.Get() > 2.0) AutonStage++;
				break;
			case 46:
				tmr_TimeOut.Reset();
				rbt_Drivetrain->GoToAngle(90.0);
				AutonStage++;
				break;
			case 47:
				if (rbt_Drivetrain->IsAtAngle() || tmr_TimeOut.Get() > 1.0) AutonStage++;
				break;
			case 48:
				tmr_TimeOut.Reset();
				rbt_Drivetrain->GoToDistance(110);
				AutonStage++;
				break;
			case 49:
				if (rbt_Drivetrain->IsAtDistance() || tmr_TimeOut.Get() > 2.0) AutonStage++;
				break;
			case 50:
				tmr_TimeOut.Reset();
				rbt_Drivetrain->GoToAngle(-90.0);
				AutonStage++;
				break;
			case 51:
				if (rbt_Drivetrain->IsAtAngle() || tmr_TimeOut.Get() > 1.0) AutonStage++;
				break;
			case 52:
				tmr_TimeOut.Reset();
				rbt_Arm->Shoot();
				AutonStage++;
				break;
			case 53:
				if (tmr_TimeOut.Get() > 1.5) {
					rbt_Arm->DisableIntake();
					AutonStage++;
				}
				break;
			}
		}

		void LSt_RSc_3x() {
			switch(AutonStage) {
			case 0:
				tmr_TimeOut.Reset();
				rbt_Arm->Point(rbt_Arm->PointPosition::kReverseScale);
				rbt_Drivetrain->GoToDistance(228.735);
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
				if (rbt_Drivetrain->IsAtAngle() || tmr_TimeOut.Get() > 1.0) AutonStage++;
				break;
			case 4:
				tmr_TimeOut.Reset();
				rbt_Drivetrain->GoToDistance(230);
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
				if (rbt_Drivetrain->IsAtDistance() || tmr_TimeOut.Get()) AutonStage++;
				break;
			case 8:
				tmr_TimeOut.Reset();
				rbt_Drivetrain->GoToDistance(70.915);
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
				if (rbt_Drivetrain->IsAtAngle() || tmr_TimeOut.Get() > 1.0) AutonStage++;
				break;
			case 12:
				tmr_TimeOut.Reset();
				rbt_Arm->Shoot();
				AutonStage++;
				break;
			case 13:
				if (tmr_TimeOut.Get() > 1.5) {
					rbt_Arm->DisableIntake();
					AutonStage++;
				}
				break;
			case 14:
				tmr_TimeOut.Reset();
				rbt_Drivetrain->GoToAngle(-90.0);
				AutonStage++;
				break;
			case 15:
				if (rbt_Drivetrain->IsAtAngle() || tmr_TimeOut.Get() > 1.0) AutonStage++;
				break;
			case 16:
				tmr_TimeOut.Reset();
				rbt_Drivetrain->GoToDistance(70.915);
				AutonStage++;
				break;
			case 17:
				if (rbt_Drivetrain->IsAtDistance() || tmr_TimeOut.Get() > 2.0) AutonStage++;
				break;
			case 18:
				tmr_TimeOut.Reset();
				rbt_Arm->Point(rbt_Arm->PointPosition::kReverseCollect);
				rbt_Drivetrain->GoToAngle(90.0);
				AutonStage++;
				break;
			case 19:
				if (rbt_Drivetrain->IsAtAngle() || tmr_TimeOut.Get() > 1.0) AutonStage++;
				break;
			case 20:
				tmr_TimeOut.Reset();
				rbt_Drivetrain->GoToDistance(40.75);
				AutonStage++;
				break;
			case 21:
				if (rbt_Drivetrain->IsAtDistance() || tmr_TimeOut.Get() > 2.0) AutonStage++;
				break;
			case 22:
				tmr_TimeOut.Reset();
				rbt_Drivetrain->GoToAngle(90.0);
				AutonStage++;
				break;
			case 23:
				if (rbt_Drivetrain->IsAtAngle() || tmr_TimeOut.Get() > 1.0) AutonStage++;
				break;
			case 24:
				tmr_TimeOut.Reset();
				rbt_Arm->EnableIntake();
				rbt_Drivetrain->SetDrive(0.5, 0.5);
				AutonStage++;
				break;
			case 25:
				if (tmr_TimeOut.Get() > 1.5) {
					rbt_Drivetrain->SetDrive(0, 0);
					rbt_Arm->DisableIntake();
					AutonStage++;
				}
				break;
			case 26:
				tmr_TimeOut.Reset();
				rbt_Arm->Point(rbt_Arm->PointPosition::kReverseScale);
				rbt_Drivetrain->GoToAngle(90.0);
				AutonStage++;
				break;
			case 27:
				if (rbt_Drivetrain->IsAtAngle() || tmr_TimeOut.Get() > 1.0) AutonStage++;
				break;
			case 29:
				tmr_TimeOut.Reset();
				rbt_Drivetrain->GoToDistance(40.75);
				AutonStage++;
				break;
			case 30:
				if (rbt_Drivetrain->IsAtDistance() || tmr_TimeOut.Get() > 2.0) AutonStage++;
				break;
			case 31:
				tmr_TimeOut.Reset();
				rbt_Drivetrain->GoToAngle(-90.0);
				AutonStage++;
				break;
			case 32:
				if (rbt_Drivetrain->IsAtAngle() || tmr_TimeOut.Get() > 1.0) AutonStage++;
				break;
			case 33:
				tmr_TimeOut.Reset();
				rbt_Drivetrain->GoToDistance(70.915);
				AutonStage++;
				break;
			case 34:
				if (rbt_Drivetrain->IsAtDistance() || tmr_TimeOut.Get() > 2.0) AutonStage++;
				break;
			case 35:
				tmr_TimeOut.Reset();
				rbt_Drivetrain->GoToAngle(-90.0);
				AutonStage++;
				break;
			case 36:
				if (rbt_Drivetrain->IsAtAngle() || tmr_TimeOut.Get() > 2.0) AutonStage++;
				break;
			case 37:
				tmr_TimeOut.Reset();
				rbt_Arm->Shoot();
				AutonStage++;
				break;
			case 38:
				if (tmr_TimeOut.Get() > 1.5) {
					rbt_Arm->DisableIntake();
					AutonStage++;
				}
				break;
			case 39:
				tmr_TimeOut.Reset();
				rbt_Drivetrain->GoToAngle(-90.0);
				AutonStage++;
				break;
			case 40:
				if (rbt_Drivetrain->IsAtAngle() || tmr_TimeOut.Get() > 1.0) AutonStage++;
				break;
			case 41:
				tmr_TimeOut.Reset();
				rbt_Drivetrain->GoToDistance(70.915);
				AutonStage++;
				break;
			case 42:
				if (rbt_Drivetrain->IsAtDistance() || tmr_TimeOut.Get() > 2.0) AutonStage++;
				break;
			case 43:
				tmr_TimeOut.Reset();
				rbt_Arm->Point(rbt_Arm->PointPosition::kReverseCollect);
				rbt_Drivetrain->GoToAngle(90.0);
				AutonStage++;
				break;
			case 44:
				if (rbt_Drivetrain->IsAtAngle() || tmr_TimeOut.Get() > 1.0) AutonStage++;
				break;
			case 45:
				tmr_TimeOut.Reset();
				rbt_Drivetrain->GoToDistance(66.75);
				AutonStage++;
				break;
			case 46:
				if (rbt_Drivetrain->IsAtDistance() || tmr_TimeOut.Get() > 2.0) AutonStage++;
				break;
			case 47:
				tmr_TimeOut.Reset();
				rbt_Drivetrain->GoToAngle(90.0);
				AutonStage++;
				break;
			case 48:
				if (rbt_Drivetrain->IsAtAngle() || tmr_TimeOut.Get() > 1.0) AutonStage++;
				break;
			case 49:
				tmr_TimeOut.Reset();
				rbt_Arm->EnableIntake();
				rbt_Drivetrain->SetDrive(0.5, 0.5);
				AutonStage++;
				break;
			case 50:
				if (tmr_TimeOut.Get() > 1.5) {
					rbt_Drivetrain->SetDrive(0, 0);
					rbt_Arm->DisableIntake();
					AutonStage++;
				}
				break;
			case 51:
				tmr_TimeOut.Reset();
				rbt_Arm->Point(rbt_Arm->PointPosition::kReverseScale);
				rbt_Drivetrain->GoToAngle(90.0);
				AutonStage++;
				break;
			case 52:
				if (rbt_Drivetrain->IsAtAngle() || tmr_TimeOut.Get() > 1.0) AutonStage++;
				break;
			case 53:
				tmr_TimeOut.Reset();
				rbt_Drivetrain->GoToDistance(66.75);
				AutonStage++;
				break;
			case 54:
				if (rbt_Drivetrain->IsAtDistance() || tmr_TimeOut.Get() > 2.0) AutonStage++;
				break;
			case 55:
				tmr_TimeOut.Reset();
				rbt_Drivetrain->GoToAngle(-90.0);
				AutonStage++;
				break;
			case 56:
				if (rbt_Drivetrain->IsAtAngle() || tmr_TimeOut.Get() > 1.0) AutonStage++;
				break;
			case 57:
				tmr_TimeOut.Reset();
				rbt_Drivetrain->GoToDistance(70.915);
				AutonStage++;
				break;
			case 58:
				if (rbt_Drivetrain->IsAtDistance() || tmr_TimeOut.Get() > 2.0) AutonStage++;
				break;
			case 59:
				tmr_TimeOut.Reset();
				rbt_Drivetrain->GoToAngle(-90.0);
				AutonStage++;
				break;
			case 60:
				if (rbt_Drivetrain->IsAtAngle() || tmr_TimeOut.Get() > 2.0) AutonStage++;
				break;
			case 61:
				tmr_TimeOut.Reset();
				rbt_Arm->Shoot();
				AutonStage++;
				break;
			case 62:
				if (tmr_TimeOut.Get() > 1.5) {
					rbt_Arm->DisableIntake();
					AutonStage++;
				}
				break;
			}
		}

		void RSt_LSc_3x() {

		}

		void RSt_RSc_3x() {

		}

		void LSt_LSc_4x() {
			switch(AutonStage) {
			case 0:
				tmr_TimeOut.Reset();
				tmr_TimeOut.Start();
				rbt_Drivetrain->GoToDistance(309.552);
				rbt_Arm->Point(rbt_Arm->PointPosition::kReverseScale);
				AutonStage++;
				break;
			case 1:
				if (rbt_Drivetrain->IsAtDistance() || tmr_TimeOut.Get() > 4.0) AutonStage++;
				break;
			case 2:
				tmr_TimeOut.Reset();
				rbt_Drivetrain->GoToAngle(-90.0);
				AutonStage++;
				break;
			case 3:
				if (rbt_Drivetrain->IsAtAngle() || tmr_TimeOut.Get() > 1.0) AutonStage++;
				break;
			case 4:
				tmr_TimeOut.Reset();
				rbt_Arm->Shoot();
				break;
			case 5:
				if (tmr_TimeOut.Get() > 1.5) {
					rbt_Arm->DisableIntake();
					AutonStage++;
				}
				break;
			case 6:
				tmr_TimeOut.Reset();
				rbt_Arm->Point(rbt_Arm->PointPosition::kForwardCollect);
				rbt_Drivetrain->GoToAngle(-90.0);
				AutonStage++;
				break;
			case 7:
				if (rbt_Drivetrain->IsAtAngle() || tmr_TimeOut.Get() > 1.0) AutonStage++;
				break;
			case 8:
				tmr_TimeOut.Reset();
				rbt_Drivetrain->GoToDistance(86.196);
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
				if (rbt_Drivetrain->IsAtDistance() || tmr_TimeOut.Get() > 1.0) AutonStage++;
				break;
			case 12:
				tmr_TimeOut.Reset();
				rbt_Drivetrain->GoToDistance(47.016);
				AutonStage++;
				break;
			case 13:
				if (rbt_Drivetrain->IsAtDistance() || tmr_TimeOut.Get() > 1.0) AutonStage++;
				break;
			case 14:
				tmr_TimeOut.Reset();
				rbt_Drivetrain->GoToAngle(90.0);
				AutonStage++;
				break;
			case 15:
				if (rbt_Drivetrain->IsAtAngle() || tmr_TimeOut.Get() > 1.0) AutonStage++;
				break;
			case 16:
				tmr_TimeOut.Reset();
				rbt_Arm->EnableIntake();
				rbt_Drivetrain->SetDrive(0.5, 0.5);
				break;
			case 17:
				if (tmr_TimeOut.Get() > 1.5) {
					rbt_Drivetrain->SetDrive(0, 0);
					rbt_Arm->DisableIntake();
					AutonStage++;
				}
				break;
			case 18:
				tmr_TimeOut.Reset();
				rbt_Arm->Point(rbt_Arm->PointPosition::kReverseScale);
				rbt_Drivetrain->GoToAngle(90.0);
				break;
			case 19:
				if (rbt_Drivetrain->IsAtAngle() || tmr_TimeOut.Get() > 1.0) AutonStage++;
				break;
			case 20:
				tmr_TimeOut.Reset();
				rbt_Drivetrain->GoToDistance(27.426);
				AutonStage++;
				break;
			case 21:
				if (rbt_Drivetrain->IsAtDistance() || tmr_TimeOut.Get() > 2.0) AutonStage++;
				break;
			case 22:
				tmr_TimeOut.Reset();
				rbt_Drivetrain->GoToAngle(90.0);
				AutonStage++;
				break;
			case 23:
				if (rbt_Drivetrain->IsAtAngle() || tmr_TimeOut.Get() > 1.0) AutonStage++;
				break;
			case 24:
				tmr_TimeOut.Reset();
				rbt_Drivetrain->GoToDistance(110);
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
				if (rbt_Drivetrain->IsAtAngle() || tmr_TimeOut.Get() > 1.0) AutonStage++;
				break;
			case 28:
				tmr_TimeOut.Reset();
				rbt_Arm->Shoot();
				AutonStage++;
				break;
			case 29:
				if (tmr_TimeOut.Get() > 1.5) {
					rbt_Arm->DisableIntake();
					AutonStage++;
				}
				break;
			case 30:
				tmr_TimeOut.Reset();
				rbt_Arm->Point(rbt_Arm->PointPosition::kForwardCollect);
				rbt_Drivetrain->GoToAngle(-90.0);
				AutonStage++;
				break;
			case 31:
				if (rbt_Drivetrain->IsAtAngle() || tmr_TimeOut.Get() > 1.0) AutonStage++;
				break;
			case 32:
				tmr_TimeOut.Reset();
				rbt_Drivetrain->GoToDistance(86.196);
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
				if (rbt_Drivetrain->IsAtDistance() || tmr_TimeOut.Get() > 1.0) AutonStage++;
				break;
			case 36:
				tmr_TimeOut.Reset();
				rbt_Drivetrain->GoToDistance(66.75);
				AutonStage++;
				break;
			case 37:
				if (rbt_Drivetrain->IsAtDistance() || tmr_TimeOut.Get() > 1.0) AutonStage++;
				break;
			case 38:
				tmr_TimeOut.Reset();
				rbt_Drivetrain->GoToAngle(90.0);
				AutonStage++;
				break;
			case 39:
				if (rbt_Drivetrain->IsAtAngle() || tmr_TimeOut.Get() > 1.0) AutonStage++;
				break;
			case 40:
				tmr_TimeOut.Reset();
				rbt_Arm->EnableIntake();
				rbt_Drivetrain->SetDrive(0.5, 0.5);
				break;
			case 41:
				if (tmr_TimeOut.Get() > 1.5) {
					rbt_Drivetrain->SetDrive(0, 0);
					rbt_Arm->DisableIntake();
					AutonStage++;
				}
				break;
			case 42:
				tmr_TimeOut.Reset();
				rbt_Arm->Point(rbt_Arm->PointPosition::kReverseScale);
				rbt_Drivetrain->GoToAngle(90.0);
				break;
			case 43:
				if (rbt_Drivetrain->IsAtAngle() || tmr_TimeOut.Get() > 1.0) AutonStage++;
				break;
			case 44:
				tmr_TimeOut.Reset();
				rbt_Drivetrain->GoToDistance(66.75);
				AutonStage++;
				break;
			case 45:
				if (rbt_Drivetrain->IsAtDistance() || tmr_TimeOut.Get() > 2.0) AutonStage++;
				break;
			case 46:
				tmr_TimeOut.Reset();
				rbt_Drivetrain->GoToAngle(90.0);
				AutonStage++;
				break;
			case 47:
				if (rbt_Drivetrain->IsAtAngle() || tmr_TimeOut.Get() > 1.0) AutonStage++;
				break;
			case 48:
				tmr_TimeOut.Reset();
				rbt_Drivetrain->GoToDistance(110);
				AutonStage++;
				break;
			case 49:
				if (rbt_Drivetrain->IsAtDistance() || tmr_TimeOut.Get() > 2.0) AutonStage++;
				break;
			case 50:
				tmr_TimeOut.Reset();
				rbt_Drivetrain->GoToAngle(-90.0);
				AutonStage++;
				break;
			case 51:
				if (rbt_Drivetrain->IsAtAngle() || tmr_TimeOut.Get() > 1.0) AutonStage++;
				break;
			case 52:
				tmr_TimeOut.Reset();
				rbt_Arm->Shoot();
				AutonStage++;
				break;
			case 53:
				if (tmr_TimeOut.Get() > 1.5) {
					rbt_Arm->DisableIntake();
					AutonStage++;
				}
				break;
			case 54:
				tmr_TimeOut.Reset();
				rbt_Arm->Point(rbt_Arm->PointPosition::kForwardCollect);
				rbt_Drivetrain->GoToAngle(-90.0);
				AutonStage++;
				break;
			case 55:
				if (rbt_Drivetrain->IsAtAngle() || tmr_TimeOut.Get() > 1.0) AutonStage++;
				break;
			case 56:
				tmr_TimeOut.Reset();
				rbt_Drivetrain->GoToDistance(86.196);
				AutonStage++;
				break;
			case 57:
				if (rbt_Drivetrain->IsAtDistance() || tmr_TimeOut.Get() > 2.0) AutonStage++;
				break;
			case 58:
				tmr_TimeOut.Reset();
				rbt_Drivetrain->GoToAngle(-90.0);
				AutonStage++;
				break;
			case 59:
				if (rbt_Drivetrain->IsAtDistance() || tmr_TimeOut.Get() > 1.0) AutonStage++;
				break;
			case 60:
				tmr_TimeOut.Reset();
				rbt_Drivetrain->GoToDistance(92.75);
				AutonStage++;
				break;
			case 61:
				if (rbt_Drivetrain->IsAtDistance() || tmr_TimeOut.Get() > 1.0) AutonStage++;
				break;
			case 62:
				tmr_TimeOut.Reset();
				rbt_Drivetrain->GoToAngle(90.0);
				AutonStage++;
				break;
			case 63:
				if (rbt_Drivetrain->IsAtAngle() || tmr_TimeOut.Get() > 1.0) AutonStage++;
				break;
			case 64:
				tmr_TimeOut.Reset();
				rbt_Arm->EnableIntake();
				rbt_Drivetrain->SetDrive(0.5, 0.5);
				break;
			case 65:
				if (tmr_TimeOut.Get() > 1.5) {
					rbt_Drivetrain->SetDrive(0, 0);
					rbt_Arm->DisableIntake();
					AutonStage++;
				}
				break;
			case 66:
				tmr_TimeOut.Reset();
				rbt_Arm->Point(rbt_Arm->PointPosition::kReverseScale);
				rbt_Drivetrain->GoToAngle(90.0);
				break;
			case 67:
				if (rbt_Drivetrain->IsAtAngle() || tmr_TimeOut.Get() > 1.0) AutonStage++;
				break;
			case 68:
				tmr_TimeOut.Reset();
				rbt_Drivetrain->GoToDistance(92.75);
				AutonStage++;
				break;
			case 69:
				if (rbt_Drivetrain->IsAtDistance() || tmr_TimeOut.Get() > 2.0) AutonStage++;
				break;
			case 70:
				tmr_TimeOut.Reset();
				rbt_Drivetrain->GoToAngle(90.0);
				AutonStage++;
				break;
			case 71:
				if (rbt_Drivetrain->IsAtAngle() || tmr_TimeOut.Get() > 1.0) AutonStage++;
				break;
			case 72:
				tmr_TimeOut.Reset();
				rbt_Drivetrain->GoToDistance(110);
				AutonStage++;
				break;
			case 73:
				if (rbt_Drivetrain->IsAtDistance() || tmr_TimeOut.Get() > 2.0) AutonStage++;
				break;
			case 74:
				tmr_TimeOut.Reset();
				rbt_Drivetrain->GoToAngle(-90.0);
				AutonStage++;
				break;
			case 75:
				if (rbt_Drivetrain->IsAtAngle() || tmr_TimeOut.Get() > 1.0) AutonStage++;
				break;
			case 76:
				tmr_TimeOut.Reset();
				rbt_Arm->Shoot();
				AutonStage++;
				break;
			case 77:
				if (tmr_TimeOut.Get() > 1.5) {
					rbt_Arm->DisableIntake();
					AutonStage++;
				}
				break;
			}
		}

		void LSt_RSc_4x() {
			switch(AutonStage) {
			case 0:
				tmr_TimeOut.Reset();
				rbt_Arm->Point(rbt_Arm->PointPosition::kReverseScale);
				rbt_Drivetrain->GoToDistance(228.735);
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
				if (rbt_Drivetrain->IsAtAngle() || tmr_TimeOut.Get() > 1.0) AutonStage++;
				break;
			case 4:
				tmr_TimeOut.Reset();
				rbt_Drivetrain->GoToDistance(230);
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
				if (rbt_Drivetrain->IsAtDistance() || tmr_TimeOut.Get()) AutonStage++;
				break;
			case 8:
				tmr_TimeOut.Reset();
				rbt_Drivetrain->GoToDistance(70.915);
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
				if (rbt_Drivetrain->IsAtAngle() || tmr_TimeOut.Get() > 1.0) AutonStage++;
				break;
			case 12:
				tmr_TimeOut.Reset();
				rbt_Arm->Shoot();
				AutonStage++;
				break;
			case 13:
				if (tmr_TimeOut.Get() > 1.5) {
					rbt_Arm->DisableIntake();
					AutonStage++;
				}
				break;
			case 14:
				tmr_TimeOut.Reset();
				rbt_Drivetrain->GoToAngle(-90.0);
				AutonStage++;
				break;
			case 15:
				if (rbt_Drivetrain->IsAtAngle() || tmr_TimeOut.Get() > 1.0) AutonStage++;
				break;
			case 16:
				tmr_TimeOut.Reset();
				rbt_Drivetrain->GoToDistance(70.915);
				AutonStage++;
				break;
			case 17:
				if (rbt_Drivetrain->IsAtDistance() || tmr_TimeOut.Get() > 2.0) AutonStage++;
				break;
			case 18:
				tmr_TimeOut.Reset();
				rbt_Arm->Point(rbt_Arm->PointPosition::kReverseCollect);
				rbt_Drivetrain->GoToAngle(90.0);
				AutonStage++;
				break;
			case 19:
				if (rbt_Drivetrain->IsAtAngle() || tmr_TimeOut.Get() > 1.0) AutonStage++;
				break;
			case 20:
				tmr_TimeOut.Reset();
				rbt_Drivetrain->GoToDistance(40.75);
				AutonStage++;
				break;
			case 21:
				if (rbt_Drivetrain->IsAtDistance() || tmr_TimeOut.Get() > 2.0) AutonStage++;
				break;
			case 22:
				tmr_TimeOut.Reset();
				rbt_Drivetrain->GoToAngle(90.0);
				AutonStage++;
				break;
			case 23:
				if (rbt_Drivetrain->IsAtAngle() || tmr_TimeOut.Get() > 1.0) AutonStage++;
				break;
			case 24:
				tmr_TimeOut.Reset();
				rbt_Arm->EnableIntake();
				rbt_Drivetrain->SetDrive(0.5, 0.5);
				AutonStage++;
				break;
			case 25:
				if (tmr_TimeOut.Get() > 1.5) {
					rbt_Drivetrain->SetDrive(0, 0);
					rbt_Arm->DisableIntake();
					AutonStage++;
				}
				break;
			case 26:
				tmr_TimeOut.Reset();
				rbt_Arm->Point(rbt_Arm->PointPosition::kReverseScale);
				rbt_Drivetrain->GoToAngle(90.0);
				AutonStage++;
				break;
			case 27:
				if (rbt_Drivetrain->IsAtAngle() || tmr_TimeOut.Get() > 1.0) AutonStage++;
				break;
			case 29:
				tmr_TimeOut.Reset();
				rbt_Drivetrain->GoToDistance(40.75);
				AutonStage++;
				break;
			case 30:
				if (rbt_Drivetrain->IsAtDistance() || tmr_TimeOut.Get() > 2.0) AutonStage++;
				break;
			case 31:
				tmr_TimeOut.Reset();
				rbt_Drivetrain->GoToAngle(-90.0);
				AutonStage++;
				break;
			case 32:
				if (rbt_Drivetrain->IsAtAngle() || tmr_TimeOut.Get() > 1.0) AutonStage++;
				break;
			case 33:
				tmr_TimeOut.Reset();
				rbt_Drivetrain->GoToDistance(70.915);
				AutonStage++;
				break;
			case 34:
				if (rbt_Drivetrain->IsAtDistance() || tmr_TimeOut.Get() > 2.0) AutonStage++;
				break;
			case 35:
				tmr_TimeOut.Reset();
				rbt_Drivetrain->GoToAngle(-90.0);
				AutonStage++;
				break;
			case 36:
				if (rbt_Drivetrain->IsAtAngle() || tmr_TimeOut.Get() > 2.0) AutonStage++;
				break;
			case 37:
				tmr_TimeOut.Reset();
				rbt_Arm->Shoot();
				AutonStage++;
				break;
			case 38:
				if (tmr_TimeOut.Get() > 1.5) {
					rbt_Arm->DisableIntake();
					AutonStage++;
				}
				break;
			case 39:
				tmr_TimeOut.Reset();
				rbt_Drivetrain->GoToAngle(-90.0);
				AutonStage++;
				break;
			case 40:
				if (rbt_Drivetrain->IsAtAngle() || tmr_TimeOut.Get() > 1.0) AutonStage++;
				break;
			case 41:
				tmr_TimeOut.Reset();
				rbt_Drivetrain->GoToDistance(70.915);
				AutonStage++;
				break;
			case 42:
				if (rbt_Drivetrain->IsAtDistance() || tmr_TimeOut.Get() > 2.0) AutonStage++;
				break;
			case 43:
				tmr_TimeOut.Reset();
				rbt_Arm->Point(rbt_Arm->PointPosition::kReverseCollect);
				rbt_Drivetrain->GoToAngle(90.0);
				AutonStage++;
				break;
			case 44:
				if (rbt_Drivetrain->IsAtAngle() || tmr_TimeOut.Get() > 1.0) AutonStage++;
				break;
			case 45:
				tmr_TimeOut.Reset();
				rbt_Drivetrain->GoToDistance(66.75);
				AutonStage++;
				break;
			case 46:
				if (rbt_Drivetrain->IsAtDistance() || tmr_TimeOut.Get() > 2.0) AutonStage++;
				break;
			case 47:
				tmr_TimeOut.Reset();
				rbt_Drivetrain->GoToAngle(90.0);
				AutonStage++;
				break;
			case 48:
				if (rbt_Drivetrain->IsAtAngle() || tmr_TimeOut.Get() > 1.0) AutonStage++;
				break;
			case 49:
				tmr_TimeOut.Reset();
				rbt_Arm->EnableIntake();
				rbt_Drivetrain->SetDrive(0.5, 0.5);
				AutonStage++;
				break;
			case 50:
				if (tmr_TimeOut.Get() > 1.5) {
					rbt_Drivetrain->SetDrive(0, 0);
					rbt_Arm->DisableIntake();
					AutonStage++;
				}
				break;
			case 51:
				tmr_TimeOut.Reset();
				rbt_Arm->Point(rbt_Arm->PointPosition::kReverseScale);
				rbt_Drivetrain->GoToAngle(90.0);
				AutonStage++;
				break;
			case 52:
				if (rbt_Drivetrain->IsAtAngle() || tmr_TimeOut.Get() > 1.0) AutonStage++;
				break;
			case 53:
				tmr_TimeOut.Reset();
				rbt_Drivetrain->GoToDistance(66.75);
				AutonStage++;
				break;
			case 54:
				if (rbt_Drivetrain->IsAtDistance() || tmr_TimeOut.Get() > 2.0) AutonStage++;
				break;
			case 55:
				tmr_TimeOut.Reset();
				rbt_Drivetrain->GoToAngle(-90.0);
				AutonStage++;
				break;
			case 56:
				if (rbt_Drivetrain->IsAtAngle() || tmr_TimeOut.Get() > 1.0) AutonStage++;
				break;
			case 57:
				tmr_TimeOut.Reset();
				rbt_Drivetrain->GoToDistance(70.915);
				AutonStage++;
				break;
			case 58:
				if (rbt_Drivetrain->IsAtDistance() || tmr_TimeOut.Get() > 2.0) AutonStage++;
				break;
			case 59:
				tmr_TimeOut.Reset();
				rbt_Drivetrain->GoToAngle(-90.0);
				AutonStage++;
				break;
			case 60:
				if (rbt_Drivetrain->IsAtAngle() || tmr_TimeOut.Get() > 2.0) AutonStage++;
				break;
			case 61:
				tmr_TimeOut.Reset();
				rbt_Arm->Shoot();
				AutonStage++;
				break;
			case 62:
				if (tmr_TimeOut.Get() > 1.5) {
					rbt_Arm->DisableIntake();
					AutonStage++;
				}
				break;
			case 63:
				tmr_TimeOut.Reset();
				rbt_Drivetrain->GoToAngle(-90.0);
				AutonStage++;
				break;
			case 64:
				if (rbt_Drivetrain->IsAtAngle() || tmr_TimeOut.Get() > 1.0) AutonStage++;
				break;
			case 65:
				tmr_TimeOut.Reset();
				rbt_Drivetrain->GoToDistance(70.915);
				AutonStage++;
				break;
			case 66:
				if (rbt_Drivetrain->IsAtDistance() || tmr_TimeOut.Get() > 2.0) AutonStage++;
				break;
			case 67:
				tmr_TimeOut.Reset();
				rbt_Arm->Point(rbt_Arm->PointPosition::kReverseCollect);
				rbt_Drivetrain->GoToAngle(90.0);
				AutonStage++;
				break;
			case 68:
				if (rbt_Drivetrain->IsAtAngle() || tmr_TimeOut.Get() > 1.0) AutonStage++;
				break;
			case 69:
				tmr_TimeOut.Reset();
				rbt_Drivetrain->GoToDistance(92.75);
				AutonStage++;
				break;
			case 70:
				if (rbt_Drivetrain->IsAtDistance() || tmr_TimeOut.Get() > 2.0) AutonStage++;
				break;
			case 71:
				tmr_TimeOut.Reset();
				rbt_Drivetrain->GoToAngle(90.0);
				AutonStage++;
				break;
			case 72:
				if (rbt_Drivetrain->IsAtAngle() || tmr_TimeOut.Get() > 1.0) AutonStage++;
				break;
			case 73:
				tmr_TimeOut.Reset();
				rbt_Arm->EnableIntake();
				rbt_Drivetrain->SetDrive(0.5, 0.5);
				AutonStage++;
				break;
			case 74:
				if (tmr_TimeOut.Get() > 1.5) {
					rbt_Drivetrain->SetDrive(0, 0);
					rbt_Arm->DisableIntake();
					AutonStage++;
				}
				break;
			case 75:
				tmr_TimeOut.Reset();
				rbt_Arm->Point(rbt_Arm->PointPosition::kReverseScale);
				rbt_Drivetrain->GoToAngle(90.0);
				AutonStage++;
				break;
			case 76:
				if (rbt_Drivetrain->IsAtAngle() || tmr_TimeOut.Get() > 1.0) AutonStage++;
				break;
			case 77:
				tmr_TimeOut.Reset();
				rbt_Drivetrain->GoToDistance(92.75);
				AutonStage++;
				break;
			case 78:
				if (rbt_Drivetrain->IsAtDistance() || tmr_TimeOut.Get() > 2.0) AutonStage++;
				break;
			case 79:
				tmr_TimeOut.Reset();
				rbt_Drivetrain->GoToAngle(-90.0);
				AutonStage++;
				break;
			case 80:
				if (rbt_Drivetrain->IsAtAngle() || tmr_TimeOut.Get() > 1.0) AutonStage++;
				break;
			case 81:
				tmr_TimeOut.Reset();
				rbt_Drivetrain->GoToDistance(70.915);
				AutonStage++;
				break;
			case 82:
				if (rbt_Drivetrain->IsAtDistance() || tmr_TimeOut.Get() > 2.0) AutonStage++;
				break;
			case 83:
				tmr_TimeOut.Reset();
				rbt_Drivetrain->GoToAngle(-90.0);
				AutonStage++;
				break;
			case 84:
				if (rbt_Drivetrain->IsAtAngle() || tmr_TimeOut.Get() > 2.0) AutonStage++;
				break;
			case 85:
				tmr_TimeOut.Reset();
				rbt_Arm->Shoot();
				AutonStage++;
				break;
			case 86:
				if (tmr_TimeOut.Get() > 1.5) {
					rbt_Arm->DisableIntake();
					AutonStage++;
				}
				break;
			}
		}

		void RSt_LSc_4x() {

		}

		void RSt_RSc_4x() {

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
								case Configuration::Goal::GoalOrientation::kLeft: 		LSt_LSw(); 	break; //Left start left switch
								case Configuration::Goal::GoalOrientation::kRight: 		LSt_RSw(); 	break; //Left start right switch
							}
							break;
						case StartingPosition::kRight:
							switch(m_SwitchSide) {
								case Configuration::Goal::GoalOrientation::kLeft: 		RSt_LSw(); 	break; //Center start left switch
								case Configuration::Goal::GoalOrientation::kRight: 		RSt_RSw(); 	break; //Center start right switch
							}
							break;
						case StartingPosition::kCenter:
							switch(m_SwitchSide) {
								case Configuration::Goal::GoalOrientation::kLeft: 		CSt_LSw(); 	break; //Right start left switch
								case Configuration::Goal::GoalOrientation::kRight: 		CSt_RSw(); 	break; //Right start right switch
							}
							break;
					}
					break;
					case AutonStates::kSwitchScale:
						switch(m_SelectedPosition) {
						case StartingPosition::kLeft:
							switch(m_SwitchSide) {
							case Configuration::Goal::GoalOrientation::kLeft:
								switch(m_ScaleSide) {
								case Configuration::Goal::GoalOrientation::kLeft: 	LSt_LSc_LSw(); 	break; //Left start left scale left switch
								case Configuration::Goal::GoalOrientation::kRight: 	LSt_LSw_RSc(); 	break; //Left start left switch right scale
								}
								break;
							case Configuration::Goal::GoalOrientation::kRight:
								switch(m_ScaleSide) {
								case Configuration::Goal::GoalOrientation::kLeft: 	LSt_LSc_RSw(); 	break; //Left start left scale right switch
								case Configuration::Goal::GoalOrientation::kRight: 	LSt_RSc_RSw(); 	break; //Left start right scale right switch
								}
								break;
							}
							break;
						case StartingPosition::kCenter:
							switch(m_SwitchSide) {
							case Configuration::Goal::GoalOrientation::kLeft:
								switch(m_ScaleSide) {
								case Configuration::Goal::GoalOrientation::kLeft: 	CSt_LSc_LSw(); 	break; //Center start left scale left switch
								case Configuration::Goal::GoalOrientation::kRight: 	CSt_LSw_RSc(); 	break; //Center start left switch right scale
								}
								break;
							case Configuration::Goal::GoalOrientation::kRight:
								switch(m_ScaleSide) {
								case Configuration::Goal::GoalOrientation::kLeft: 	CSt_LSc_RSw(); 	break; //Center start left scale right switch
								case Configuration::Goal::GoalOrientation::kRight: 	CSt_RSc_RSw(); 	break; //Center start right scale right switch
								}
								break;
							}
							break;
						case StartingPosition::kRight:
							switch(m_SwitchSide) {
							case Configuration::Goal::GoalOrientation::kLeft:
								switch(m_ScaleSide) {
								case Configuration::Goal::GoalOrientation::kLeft: 	RSt_LSc_LSw(); 	break; //Right start left scale left switch
								case Configuration::Goal::GoalOrientation::kRight: 	RSt_RSc_LSw(); 	break; //Right start right scale left switch
								}
								break;
							case Configuration::Goal::GoalOrientation::kRight:
								switch(m_ScaleSide) {
								case Configuration::Goal::GoalOrientation::kLeft: 	RSt_RSw_LSc(); 	break; //Right start right switch left scale
								case Configuration::Goal::GoalOrientation::kRight: 	RSt_RSc_RSw(); 	break; //Right start right scale right switch
								}
								break;
							}
							break;
						}
						break;
				case AutonStates::kSwitchExchange:
					switch(m_SelectedPosition) {
					case StartingPosition::kLeft:
						switch(m_SwitchSide) {
						case Configuration::Goal::GoalOrientation::kLeft: 			LSt_LSwEx(); 	break; //Left start left switch exchange
						case Configuration::Goal::GoalOrientation::kRight: 			LSt_RSwEx(); 	break; //Left start right switch exchange
						}
						break;
					case StartingPosition::kCenter:
						switch(m_SwitchSide) {
						case Configuration::Goal::GoalOrientation::kLeft: 			CSt_LSwEx(); 	break; //Center start left switch exchange
						case Configuration::Goal::GoalOrientation::kRight: 			CSt_RSwEx(); 	break; //Center start right switch exchange
						}
						break;
					case StartingPosition::kRight:
						switch(m_SwitchSide) {
						case Configuration::Goal::GoalOrientation::kLeft: 			RSt_LSwEx(); 	break; //Right start left switch exchange
						case Configuration::Goal::GoalOrientation::kRight: 			RSt_RSwEx(); 	break; //Right start right switch exchange
						}
						break;
					}
					break;
				case AutonStates::k1XScale:
					switch(m_SelectedPosition) {
					case StartingPosition::kLeft:
						switch (m_ScaleSide) {
						case Configuration::Goal::GoalOrientation::kLeft:
							LSt_LSc_1x();
							break;
						case Configuration::Goal::GoalOrientation::kRight:
							LSt_RSc_1x();
							break;
						}
						break;
					case StartingPosition::kCenter:
						CrossLine();
						break;
					case StartingPosition::kRight:
						switch (m_ScaleSide) {
						case Configuration::Goal::GoalOrientation::kLeft:
							RSt_LSc_1x();
							break;
						case Configuration::Goal::GoalOrientation::kRight:
							RSt_RSc_1x();
							break;
						}
						break;
					}
					break;
				case AutonStates::k2XScale:
					switch(m_SelectedPosition) {
					case StartingPosition::kLeft:
						switch(m_ScaleSide) {
						case Configuration::Goal::GoalOrientation::kLeft:
							LSt_LSc_2x();
							break;
						case Configuration::Goal::GoalOrientation::kRight:
							LSt_RSc_2x();
							break;
						}
						break;
					case StartingPosition::kCenter:
						CrossLine();
						break;
					case StartingPosition::kRight:
						switch(m_ScaleSide) {
						case Configuration::Goal::GoalOrientation::kLeft:
							RSt_LSc_2x();
							break;
						case Configuration::Goal::GoalOrientation::kRight:
							RSt_RSc_2x();
							break;
						}
						break;
					}
					break;
				case AutonStates::k3XScale:
					switch(m_SelectedPosition) {
					case StartingPosition::kLeft:
						switch(m_ScaleSide) {
						case Configuration::Goal::GoalOrientation::kLeft:
							LSt_LSc_3x();
							break;
						case Configuration::Goal::GoalOrientation::kRight:
							LSt_RSc_3x();
							break;
						}
						break;
					case StartingPosition::kCenter:
						CrossLine();
						break;
					case StartingPosition::kRight:
						switch(m_ScaleSide) {
						case Configuration::Goal::GoalOrientation::kLeft:
							RSt_LSc_3x();
							break;
						case Configuration::Goal::GoalOrientation::kRight:
							RSt_RSc_3x();
							break;
						}
						break;
					}
					break;
				case AutonStates::k4XScale:
					switch(m_SelectedPosition) {
					case StartingPosition::kLeft:
						switch(m_ScaleSide) {
						case Configuration::Goal::GoalOrientation::kLeft:
							LSt_LSc_4x();
							break;
						case Configuration::Goal::GoalOrientation::kRight:
							LSt_RSc_4x();
							break;
						}
						break;
					case StartingPosition::kCenter:
						CrossLine();
						break;
					case StartingPosition::kRight:
						switch(m_ScaleSide) {
						case Configuration::Goal::GoalOrientation::kLeft:
							RSt_LSc_4x();
							break;
						case Configuration::Goal::GoalOrientation::kRight:
							RSt_RSc_4x();
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
			kSwitchExchange,
			k1XScale,
			k2XScale,
			k3XScale,
			k4XScale
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
