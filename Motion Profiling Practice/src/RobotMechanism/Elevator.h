#pragma once
#include "MOLib.h"
#include "Intake.h"
extern "C" {
#include "pathfinder.h"
}

namespace RobotMechanism{
	class Elevator{
	public:
		Elevator(TalonSRX *mtr_L_Elevator, VictorSPX *mtr_R_Elevator, Intake *rbt_Intake){
			this->mtr_L_Elevator	= mtr_L_Elevator;
			this->mtr_R_Elevator	= mtr_R_Elevator;
			this->rbt_Intake		= rbt_Intake;
		}
		~Elevator();

		double ElevatorConvertToFeet(float input){
					return((input / 4096) * M_PI);

				}
				void ResetSensors(){
					mtr_L_Elevator->SetSelectedSensorPosition(0.0, 0.0, 0.0);
				}

		/**
		 * @brief Sets the max speed for motors
		 * @param peakoutputforward Max speed going forward
		 * @param peakoutputreverse Max speed going backward
		 */
		void ConfigPeakOutput(float peakoutputforward, float peakoutputreverse){
			mtr_L_Elevator->ConfigPeakOutputForward(peakoutputforward, 0);
			mtr_L_Elevator->ConfigPeakOutputReverse(peakoutputreverse, 0);
			mtr_R_Elevator->ConfigPeakOutputForward(peakoutputforward, 0);
			mtr_R_Elevator->ConfigPeakOutputReverse(peakoutputreverse, 0);
		}

		/**
		 * @brief Sets the power for the elevator motors
		 * @param power The power to set it to
		 */
		void SetElevatorPower(float power)	{ m_power	= power;									}

		void ConfigureElevatorPID(double p, double i, double d){
			mtr_L_Elevator->Config_kP(0, p, 0.0);
			mtr_L_Elevator->Config_kI(0, i, 0.0);
			mtr_L_Elevator->Config_kD(0, d, 0.0);

		}

		void SetPosition(double dist){
				 dist = ElevatorConvertToFeet(dist);

				mtr_L_Elevator->Set(ControlMode::Position, dist);


		}

		void SetStage(int s){
			switch(s){

			case 0: SetPosition(0);		break;
			case 1: SetPosition(12); 	break;
			case 2:	SetPosition(24); 	break;
			}

		}

		double GetElevatorVelocity(){
			return mtr_L_Elevator->GetSelectedSensorVelocity(0);
		}

		void SetLimitCurrent(int temp){
			mtr_L_Elevator->ConfigContinuousCurrentLimit(temp, 0);
		}

		double GetElevatorVoltageDraw(){
			return mtr_L_Elevator->GetBusVoltage();
		}

		void	GoToTop()				{ /*TODO: go to top of elevator*/							}
		void	GoToBottom()			{ /*TODO: go to bottom of elevator*/						}


		double	GetPosition()			{ return mtr_L_Elevator->GetSelectedSensorPosition(0);		}
		int		GetStage()				{ /*return stage based on position outputed */ return 0;	};

		void	Update()				{ mtr_L_Elevator->Set(ControlMode::PercentOutput,m_power);	}

	private:
		Intake		*rbt_Intake;
		TalonSRX	*mtr_L_Elevator;
		VictorSPX	*mtr_R_Elevator;

		float		m_power		= 0;
		//int m_stage=0;
		//double m_position
		//int m_stageLimit=3;
	};
}
