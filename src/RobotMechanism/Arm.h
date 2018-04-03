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

			ConfigureArmPID(0.75, 0.0, 20.0);
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

		void Disable() {
			SetArmPower(0.0);
			SetWinchPower(0.0);
			EngageWinch();
			SetIntakePower(0.0);
		}

		/**
		 * @brief Sets the power for the Arm motors
		 * @param power The power to set it to
		 */
		void SetArmPower(float power)	{ mtr_Angle_1->Set(ControlMode::PercentOutput, power); }

		void SetWinchPower(float power){
			if(lim_Shooter->Get()) power = 0.0;
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
			if(!Dashboard.Misc.TuningMode.Get()) ConfigureArmPID(0.75, 0.0, 20.0);
			m_Target = angle;
			angle /= Configuration::Robot::Arm::DegreesPerCount;
			mtr_Angle_1->Set(ControlMode::Position, angle);
			tmr_AnglePID.Reset();
		}

		enum class PointPosition{
			kForwardCollect,
			kForwardMidCollect,
			kForwardSwitch,
			kForwardPortal,
			kForwardScale,
			kUp,
			kReverseScale,
			kReversePortal,
			kReverseSwitch,
			kReverseMidCollect,
			kReverseCollect
		};

		void Point(PointPosition pos){
			switch(pos){
				case PointPosition::kForwardCollect:	GoToAngle(99.0); 	break;
				case PointPosition::kForwardMidCollect:	GoToAngle(75.0); 	break;
				case PointPosition::kForwardSwitch:		GoToAngle(45.0); 	break;
				case PointPosition::kForwardPortal:		GoToAngle(55.0);	break;
				case PointPosition::kForwardScale:		GoToAngle(10.0);	break;
				case PointPosition::kUp:				GoToAngle(0.0); 	break;
				case PointPosition::kReverseScale:		GoToAngle(-10.0);	break;
				case PointPosition::kReversePortal:		GoToAngle(-55.0);	break;
				case PointPosition::kReverseSwitch:		GoToAngle(-45.0); 	break;
				case PointPosition::kReverseMidCollect:	GoToAngle(-75.0); 	break;
				case PointPosition::kReverseCollect:	GoToAngle(-99.0); 	break;

			}
		}

		void SetIntakePower(float power) { m_IntakePower = power; }

		void EnableIntake()		{ SetIntakePower(-0.5);	} //start it
		void ShootIntake()		{ SetIntakePower(1.0); }
		void ReverseIntake()	{ SetIntakePower(0.3);	} //shoot things out
		void DisableIntake()	{ SetIntakePower(0.0);	} //stop intake

		void EnableWinch() { SetWinchPower(1.0); }
		void DisableWinch() { SetWinchPower(0.0); }

		void EngageWinch() { sol_Winch->Set(false); }
		void DisengageWinch() { sol_Winch->Set(true); }

		void Shoot() {
			tmr_Winch.Reset();
			tmr_Winch.Start();
			m_Shooting = true;
		}

		void Update() {
			if(fabs((mtr_Angle_1->GetSelectedSensorPosition(0) * Configuration::Robot::Arm::DegreesPerCount) - m_Target) > m_Tolerance) { tmr_AnglePID.Reset(); tmr_AnglePID.Start(); }
			if (tmr_Winch.Get() < 0.25 && m_Shooting) {
				DisengageWinch();
				EnableWinch();

				if (tmr_Winch.Get() > 0.125) ShootIntake();
			}
			else {
				if(m_Shooting) DisableIntake();

				if(lim_Shooter->Get()) DisableWinch();
				else EnableWinch();
				EngageWinch();

				m_Shooting = false;
			}

			mtr_L_Intake->Set(m_IntakePower);
			mtr_R_Intake->Set(m_IntakePower);

			Dashboard.Arm.Angle.Angle.Set(mtr_Angle_1->GetSelectedSensorPosition(0)  * Configuration::Robot::Arm::DegreesPerCount);
			Dashboard.Arm.Angle.OnTarget.Set(IsAtAngle());
			Dashboard.Arm.Angle.Enabled.Set(IsPIDEnabled());

			Dashboard.Arm.ShooterLim.Set(lim_Shooter->Get());
		}

	private:
		CTRLib::TalonSRX	*mtr_Angle_1;
		CTRLib::VictorSPX	*mtr_Angle_2;
		WPILib::VictorSP	*mtr_L_Intake;
		WPILib::VictorSP	*mtr_R_Intake;

		float				 m_IntakePower	= 0;

		double				 m_Target		= 0.0;
		double 				 m_Tolerance	= 1.0;
		double				 m_TargetTime	= 0.2;

		WPILib::Timer		 tmr_AnglePID;

		WPILib::Timer		 tmr_Winch;

		WPILib::VictorSP	*mtr_Winch_1;
		WPILib::VictorSP	*mtr_Winch_2;
		WPILib::Solenoid	*sol_Winch;

		WPILib::DigitalInput *lim_Shooter;

		bool m_Shooting = false;
	};
}
