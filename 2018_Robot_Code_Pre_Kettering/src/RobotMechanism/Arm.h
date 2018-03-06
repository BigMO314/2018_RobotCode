#pragma once
#include "MOLib.h"
#include "Dashboard.h"

namespace RobotMechanism{
	class Arm{
	public:
		Arm(TalonSRX *mtr_Angle_1, VictorSPX *mtr_Angle_2, VictorSP *mtr_L_Intake, VictorSP *mtr_R_Intake, VictorSP *mtr_Winch1, VictorSP *mtr_Winch2, Solenoid *sol_Winch, WPILib::DigitalInput *lim_Shooter){
			this->mtr_Angle_1			= mtr_Angle_1;
			this->mtr_Angle_2			= mtr_Angle_2;
			this->mtr_L_Intake			= mtr_L_Intake;
			this->mtr_R_Intake			= mtr_R_Intake;

			this->tmr_AnglePID.Reset();
			this->tmr_AnglePID.Start();

			this->mtr_Winch_1			= mtr_Winch1;
			this->mtr_Winch_2			= mtr_Winch2;
			this->sol_Winch				= sol_Winch;

			this->lim_Shooter			= lim_Shooter;
		}
		~Arm();

		void ResetAngle(){
			mtr_Angle_1->SetSelectedSensorPosition(0.0, 0.0, 0.0);
		}

		/**
		 * @brief Sets the max speed for motors
		 * @param peakoutputforward Max speed going forward
		 * @param peakoutputreverse Max speed going backward
		 */
		void ConfigPeakOutput(float peakoutputforward, float peakoutputreverse){
			mtr_Angle_1->ConfigPeakOutputForward(peakoutputforward, 0);
			mtr_Angle_1->ConfigPeakOutputReverse(peakoutputreverse, 0);
			mtr_Angle_2->ConfigPeakOutputForward(peakoutputforward, 0);
			mtr_Angle_2->ConfigPeakOutputReverse(peakoutputreverse, 0);
		}

		/**
		 * @brief Sets the power for the Arm motors
		 * @param power The power to set it to
		 */
		void SetArmPower(float power)	{ mtr_Angle_1->Set(ControlMode::PercentOutput, power); }

		void SetWinchPower(float power){
			mtr_Winch_1->Set(power);
			mtr_Winch_2->Set(power);
		};
		void ConfigureArmPID(double p, double i, double d){
			mtr_Angle_1->Config_kP(0, p, 0.0);
			mtr_Angle_1->Config_kI(0, i, 0.0);
			mtr_Angle_1->Config_kD(0, d, 0.0);

		}

		bool IsAtAngle() {
			return tmr_AnglePID.Get() >= m_TargetTime;
		}

		bool IsPIDEnabled() {
			return mtr_Angle_1->GetControlMode() == ControlMode::Position;
		}

		void GoToAngle(double angle){
			m_Target = angle;
			angle /= Configuration::Robot::Arm::DegreesPerCount;
			mtr_Angle_1->Set(ControlMode::Position, angle);
			tmr_AnglePID.Reset();
		}

		enum class PointPosition{
			kForwardCollect,
			kForwardShoot,
			kUp,
			kReverseShoot,
			kReverseCollect
		};

		void Point(PointPosition pos){
			switch(pos){
				case PointPosition::kForwardCollect:	GoToAngle(95.0); 	break;
				case PointPosition::kForwardShoot:		GoToAngle(45.0); 	break;
				case PointPosition::kUp:				GoToAngle(0.0); 	break;
				case PointPosition::kReverseShoot:		GoToAngle(-45.0); 	break;
				case PointPosition::kReverseCollect:	GoToAngle(-95.0); 	break;
			}
		}

		void SetIntakePower(float power) { m_IntakePower = power; }

		void EnableIntake()		{ SetIntakePower(-0.75);	} //start it
		void ReverseIntake()	{ SetIntakePower(0.5);	} //shoot things out
		void DisableIntake()	{ SetIntakePower(0.0);	} //stop intake

		void EnableWinch() { SetWinchPower(1.0); }
		void DisableWinch() { SetWinchPower(0.0); }

		void EngageWinch() { sol_Winch->Set(true); }

		void ReleaseWinch() { sol_Winch->Set(false); }

		void Shoot() {
			tmr_Winch.Reset();
			tmr_Winch.Start();
			m_Shooting = true;
		}

		void Update() {
			ConfigureArmPID(Dashboard.Arm.Angle.P.Get(), Dashboard.Arm.Angle.I.Get(), Dashboard.Arm.Angle.D.Get());
			if(fabs((mtr_Angle_1->GetSelectedSensorPosition(0) * Configuration::Robot::Arm::DegreesPerCount) - m_Target) > m_Tolerance) { tmr_AnglePID.Reset(); tmr_AnglePID.Start(); }

			if (tmr_Winch.Get() < 1.0 && m_Shooting) {
				ReleaseWinch();
				DisableWinch();
				ReverseIntake();
			}
			else {
				m_Shooting = false;
				if(lim_Shooter->Get()) DisableWinch();
				else EnableWinch();
			}

			mtr_L_Intake->Set(m_IntakePower);
			mtr_R_Intake->Set(m_IntakePower);
			Dashboard.Arm.Angle.Angle.Set(mtr_Angle_1->GetSelectedSensorPosition(0)  * Configuration::Robot::Arm::DegreesPerCount);
			Dashboard.Arm.Angle.OnTarget.Set(IsAtAngle());
			Dashboard.Arm.Angle.Enabled.Set(IsPIDEnabled());
		}

	private:
		TalonSRX	*mtr_Angle_1;
		VictorSPX	*mtr_Angle_2;
		VictorSP	*mtr_L_Intake;
		VictorSP	*mtr_R_Intake;

		float		m_IntakePower = 0;

		double		m_Target 	= 0.0;
		double 		m_Tolerance	= 1.0;
		double		m_TargetTime = 0.2;

		WPILib::Timer	tmr_AnglePID;

		WPILib::Timer	tmr_Winch;

		VictorSP	*mtr_Winch_1;
		VictorSP	*mtr_Winch_2;
		Solenoid	*sol_Winch;

		WPILib::DigitalInput *lim_Shooter;

		bool m_Shooting = false;
	};
}
