#pragma once
#include "MOLib.h"
#include "Dashboard.h"
#include "Configuration/Robot.h"
#include <iostream>

namespace RobotMechanism{
	class Drivetrain{
	public:
		Drivetrain(TalonSRX *mtr_L_Drive_1, TalonSRX *mtr_R_Drive_1, WPILib::ADXRS450_Gyro *gyr_DriveAngle){
			this->mtr_L_Drive_1		= mtr_L_Drive_1;
			this->mtr_R_Drive_1		= mtr_R_Drive_1;
			this->gyr_DriveAngle	= gyr_DriveAngle;

			this->pid_DriveAngle	= new MOLib::PID::GyrLoop(gyr_DriveAngle);
			pid_DriveAngle->SetOutputRange(-1.0, 1.0);
			pid_DriveAngle->SetTargetTime(0.1);
			pid_DriveAngle->SetAbsoluteTolerance(0.0);

			this->pid_DriveDistance = new MOLib::PID::MagEncLoop(new MOLib::Sensors::MagEncoder(mtr_L_Drive_1));
			pid_DriveDistance->SetOutputRange(-1.0, 1.0);
			pid_DriveDistance->SetTargetTime(0.2);
			pid_DriveDistance->SetAbsoluteTolerance(200);

			pid_DriveAngle->Disable();
			pid_DriveDistance->Disable();
		};
		~Drivetrain(){};

		void ConfigScale(double lf, double rf, double lr, double rr){
			mtr_L_Drive_1->ConfigPeakOutputForward(lf, 0);
			mtr_R_Drive_1->ConfigPeakOutputForward(rf, 0);
			mtr_L_Drive_1->ConfigPeakOutputReverse(lr, 0);
			mtr_R_Drive_1->ConfigPeakOutputReverse(rr, 0);
		}

		void ConfigWheelDiameter(double diameter) 			{
			m_Diameter				= diameter;
			m_Circumference 		= M_PI * m_Diameter;
			m_DistancePerPulse 		= m_Circumference / 4096.0;
		}

		void ConfigPIDConst(int pid_const)					{ m_PIDConst		= pid_const;	}
		void ConfigPIDTimeout(int pid_timeout)				{ m_PIDTimeOut		= pid_timeout;	}

		/**
		 * @brief Sets power for left and right motors.
		 *
		 * @param lPower Power for left motor.
		 * @param rPower Power for right motor.
		 */

		//set the power level for the update function
		void SetDrive(float lPower, float rPower){
			m_L_Power	= lPower;
			m_R_Power	= rPower;
		};

		void StopDrive() {
			m_L_Power = 0.0;
			m_R_Power = 0.0;
		}

		void ConfigureAnglePID(double P, double I, double D) { pid_DriveAngle->SetPID(P, I, D); }
		void GoToAngle(double angle, bool reset = true) {
			pid_DriveAngle->SetPID(0.0215, 0.0, 0.024);
			pid_DriveAngle->SetSetpoint(angle);
			pid_DriveAngle->SetAbsoluteTolerance(0.5);
			pid_DriveAngle->Enable();
			if(reset) pid_DriveAngle->ResetSource();
		}

		void ResetAngle() { pid_DriveAngle->ResetSource(); }
		bool IsAnglePIDEnabled() { return pid_DriveAngle->IsEnabled(); }
		bool IsAtAngle() { return pid_DriveAngle->OnTarget(); }
		double GetAngle() { return gyr_DriveAngle->GetAngle(); }
		void DisableAnglePID() { pid_DriveAngle->Disable(); }

		void ConfigureDistancePID(double P, double I, double D) { pid_DriveDistance->SetPID(P, I, D);}
		void GoToDistance(double distance, bool reset = true) {
			pid_DriveAngle->SetPID(0.024, 0.0, 0.03);
			pid_DriveDistance->SetPID(0.00015, 0.0, 0.00025);
			pid_DriveDistance->SetSetpoint(distance / m_DistancePerPulse);
			pid_DriveAngle->SetSetpoint(0.0);
			pid_DriveAngle->SetAbsoluteTolerance(0.0);
			pid_DriveDistance->Enable();
			pid_DriveAngle->Enable();
			if(reset) {
				pid_DriveDistance->ResetSource();
				pid_DriveAngle->ResetSource();
				m_MaxError = 0.0;
			}
		}
		void ResetDistance() { pid_DriveDistance->ResetSource(); }
		bool IsDistancePIDEnabled() { return pid_DriveDistance->IsEnabled(); }
		bool IsAtDistance() { return pid_DriveDistance->OnTarget(); }
		double GetDistance() { return mtr_L_Drive_1->GetSelectedSensorPosition(0) * m_DistancePerPulse;}
		void DisableDistancePID() { pid_DriveDistance->Disable(); pid_DriveAngle->Disable(); }
		void ConfigureDistanceOutputRange(float minimum,float maximum) {pid_DriveDistance->SetOutputRange(minimum,maximum); }

		void Update() {
			//ConfigureDistancePID(Dashboard.Drivetrain.Distance.P.Get(), Dashboard.Drivetrain.Distance.I.Get(), Dashboard.Drivetrain.Distance.D.Get());

			Dashboard.Drivetrain.Angle.Angle.Set(GetAngle());
			Dashboard.Drivetrain.Angle.Enabled.Set(IsAnglePIDEnabled());
			Dashboard.Drivetrain.Angle.OnTarget.Set(IsAtAngle());
			Dashboard.Drivetrain.Angle.MaxError.Set(m_MaxError);

			Dashboard.Drivetrain.Distance.Measurement.Set(GetDistance());
			Dashboard.Drivetrain.Distance.Enabled.Set(IsDistancePIDEnabled());
			Dashboard.Drivetrain.Distance.OnTarget.Set(IsAtDistance());

			if(IsDistancePIDEnabled()){
				ConfigureAnglePID(Dashboard.Drivetrain.Angle.P.Get(), Dashboard.Drivetrain.Angle.I.Get(), Dashboard.Drivetrain.Angle.D.Get());
				if(fabs(GetAngle()) > m_MaxError) m_MaxError = fabs(GetAngle());
				if(IsAtDistance()) { pid_DriveDistance->Disable(); pid_DriveAngle->Disable(); }
				SetDrive(pid_DriveDistance->Get() + pid_DriveAngle->Get(), pid_DriveDistance->Get() - pid_DriveAngle->Get());
			}
			else if(IsAnglePIDEnabled()){
				if(IsAtAngle()) pid_DriveAngle->Disable();
				SetDrive(pid_DriveAngle->Get(), -pid_DriveAngle->Get());
			}

			mtr_L_Drive_1->Set(ControlMode::PercentOutput, m_L_Power);
			mtr_R_Drive_1->Set(ControlMode::PercentOutput, m_R_Power);

		}

	private:
		TalonSRX		*mtr_L_Drive_1;
		TalonSRX		*mtr_R_Drive_1;

		WPILib::ADXRS450_Gyro		*gyr_DriveAngle;

		MOLib::PIDLoop	*pid_DriveAngle;
		MOLib::PIDLoop	*pid_DriveDistance;


		WPILib::Timer	tmr_AnglePID;
		WPILib::Timer	tmr_DistancePID;


		float m_Gyro_Tolerance = 1.0;

		float			m_L_Power			= 0.0;
		float			m_R_Power			= 0.0;

		double			m_Diameter 			= 0.0;
		double			m_Circumference		= 0.0;
		double			m_DistancePerPulse		= 0.0;

		int				m_PIDTimeOut		= 0;
		int				m_PIDConst			= 0;

		double			ref_distance		= 0;
		double			error_tolerance		= 0.5;
		double 			m_DistanceTargetTime = 0.2;

		float 			m_LForwardScale		= 1.0;
		float 			m_RForwardScale		= 1.0;
		float 			m_LReverseScale		= 1.0;
		float 			m_RReverseScale		= 1.0;

		float			m_MaxError			= 0.0;

//		float			m_LStraightScale	= 0.899;
//		float			m_RStraightScale	= 0.899;
	};
}
