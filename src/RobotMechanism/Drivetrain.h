#pragma once
#include "MOLib.h"
#include "Dashboard.h"
#include "Configuration/Robot.h"
#include <iostream>

namespace RobotMechanism{
	class Drivetrain{
	public:
		Drivetrain(TalonSRX *mtr_L_Drive_1, TalonSRX *mtr_R_Drive_1, WPILib::ADXRS450_Gyro *gyr_DriveAngle, MOLib::Vision::LimeLight *lml_DriveLimeLight){
			this->mtr_L_Drive_1			= mtr_L_Drive_1;
			this->mtr_R_Drive_1			= mtr_R_Drive_1;
			this->gyr_DriveAngle		= gyr_DriveAngle;
			this->lml_DriveLimeLight	= lml_DriveLimeLight;
			this->enc_L_DriveDistance	= new MOLib::Sensors::MagEncoder(mtr_L_Drive_1);
			this->enc_R_DriveDistance	= new MOLib::Sensors::MagEncoder(mtr_R_Drive_1);

			this->pid_DriveAngle	= new MOLib::PID::GyrLoop(gyr_DriveAngle);
			pid_DriveAngle->SetOutputRange(-0.75, 0.75);
			pid_DriveAngle->SetTargetTime(0.1);
			pid_DriveAngle->SetAbsoluteTolerance(0.0);

			this->pid_DriveDistance = new MOLib::PID::MagEncLoop(enc_L_DriveDistance);
			pid_DriveDistance->SetPID(0.038, 0.0, 0.07);
			pid_DriveDistance->SetOutputRange(-8.0, 8.0);
			pid_DriveDistance->SetTargetTime(0.3);
			pid_DriveDistance->SetAbsoluteTolerance(1.0);

			this->pid_DriveLimeLight = new MOLib::PID::LimeLoop(lml_DriveLimeLight);
			pid_DriveLimeLight->SetPID(0.01, 0.0, 0.0);
			pid_DriveLimeLight->SetOutputRange(-0.75, 0.75);
			pid_DriveLimeLight->SetTargetTime(0.1);
			pid_DriveLimeLight->SetAbsoluteTolerance(0.0);

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

		void ConfigScale(double l, double r) {
			mtr_L_Drive_1->ConfigPeakOutputForward(l, 0);
			mtr_R_Drive_1->ConfigPeakOutputForward(r, 0);
			mtr_L_Drive_1->ConfigPeakOutputReverse(-l, 0);
			mtr_R_Drive_1->ConfigPeakOutputReverse(-r, 0);
		}

		void ConfigScale(double scale) {
			mtr_L_Drive_1->ConfigPeakOutputForward(scale, 0);
			mtr_R_Drive_1->ConfigPeakOutputForward(scale, 0);
			mtr_L_Drive_1->ConfigPeakOutputReverse(-scale, 0);
			mtr_R_Drive_1->ConfigPeakOutputReverse(-scale, 0);
		}

		void ConfigWheelDiameter(double diameter) {
			m_WheelDiameter = diameter;
			enc_L_DriveDistance->ConfigDistancePerPulse(((M_PI * m_WheelDiameter) / 4096.0) * m_GearRatio);
		}

		void ConfigGearRatio(double ratio) {
			m_GearRatio = ratio;
			enc_L_DriveDistance->ConfigDistancePerPulse(((M_PI * m_WheelDiameter) / 4096.0) * m_GearRatio);
		}

		void Disable() {
			DisableLimeLightPID();
			DisableAnglePID();
			DisableDistancePID();
			SetDrive(0.0, 0.0);
		}
		/**
		 * @brief Sets power for left and right motors.
		 *
		 * @param lPower Power for left motor.
		 * @param rPower Power for right motor.
		 */
		void SetDrive(float lPower, float rPower){
			m_L_Power	= lPower;
			m_R_Power	= rPower;
		};

		void StopDrive() {
			m_L_Power = 0.0;
			m_R_Power = 0.0;
		}

		void ConfigureAnglePID(double P, double I, double D) { pid_DriveAngle->SetPID(P, I, D); }
		void ConfigAngleOutputRange(double min, double max) { pid_DriveAngle->SetOutputRange(min, max); }
		void GoToAngle(double angle, bool reset = true) {
			DisableLimeLightPID();
			DisableDistancePID();
			if(!Dashboard.Misc.TuningMode.Get()) pid_DriveAngle->SetPID(0.024, 0.0, 0.0245);
			pid_DriveAngle->SetSetpoint(angle);
			pid_DriveAngle->SetAbsoluteTolerance(0.5);
			pid_DriveAngle->Enable();
			if(reset) pid_DriveAngle->ResetSource();
		}


		void GoToAbsoluteAngle(double angle, bool reset = true) {
			DisableLimeLightPID();
			DisableDistancePID();
			if(!Dashboard.Misc.TuningMode.Get()) pid_DriveAngle->SetPID(0.024, 0.0, 0.0245);
			pid_DriveAngle->SetSetpoint(angle);
			pid_DriveAngle->SetAbsoluteTolerance(0.5);
			pid_DriveAngle->Enable();
		}

		void ResetAngle() { pid_DriveAngle->ResetSource(); }
		bool IsAnglePIDEnabled() { return pid_DriveAngle->IsEnabled(); }
		bool IsAtAngle() { return pid_DriveAngle->OnTarget(); }
		double GetAngle() { return gyr_DriveAngle->GetAngle(); }
		void DisableAnglePID() { pid_DriveAngle->Disable(); }

		void ConfigureLimeLightPID(double P, double I, double D) { pid_DriveLimeLight->SetPID(P, I, D); }
		void GoToCube(double distance, bool reset = true) {
			DisableAnglePID();
			pid_DriveDistance->SetOutputRange(-0.6, 0.6);
			pid_DriveLimeLight->Enable();
			pid_DriveDistance->Enable();
			pid_DriveLimeLight->SetSetpoint(0);
			pid_DriveDistance->SetSetpoint(distance);
			if (reset) pid_DriveDistance->ResetSource();
		}

		bool IsLimeLightPIDEnabled() { return pid_DriveLimeLight->IsEnabled(); }
		bool IsAtCube() { return pid_DriveLimeLight->OnTarget(); }
		void DisableLimeLightPID() { pid_DriveLimeLight->Disable(); }

		void ConfigureDistancePID(double P, double I, double D) { pid_DriveDistance->SetPID(P, I, D); }
		void ConfigureDistanceOutputRange(double min, double max) {pid_DriveDistance->SetOutputRange(min, max); }
		void GoToDistance(double distance, bool reset = true) {
			DisableLimeLightPID();
			if(!Dashboard.Misc.TuningMode.Get()){
				pid_DriveAngle->SetPID(0.015, 0.0, 0.0);
				pid_DriveDistance->SetPID(0.038, 0.0, 0.065);
			}
			if(reset) {
				pid_DriveDistance->ResetSource();
				pid_DriveAngle->ResetSource();
				m_MaxError = 0.0;
			}
			pid_DriveDistance->SetOutputRange(-1.0, 1.0);
			pid_DriveDistance->SetSetpoint(distance);
			pid_DriveAngle->SetSetpoint(0.0);
			pid_DriveAngle->SetTargetTime(0.2);
			pid_DriveAngle->SetAbsoluteTolerance(0.0);
			pid_DriveDistance->Enable();
			pid_DriveAngle->Enable();
		}
		void ResetDistance() { pid_DriveDistance->ResetSource(); }
		bool IsDistancePIDEnabled() { return pid_DriveDistance->IsEnabled(); }
		bool IsAtDistance() { return pid_DriveDistance->OnTarget(); }
		double GetDistance() { return enc_L_DriveDistance->GetDistance();}
		void DisableDistancePID() { pid_DriveDistance->Disable(); pid_DriveAngle->Disable(); }

		void EnableLight() { lml_DriveLimeLight->EnableLight(); }
		void DisableLight() { lml_DriveLimeLight->DisableLight(); }

		void Update() {
			if(!Dashboard.Misc.TuningMode.Get()){
				Dashboard.Drivetrain.LimeLight.P.Set(pid_DriveLimeLight->GetP());
				Dashboard.Drivetrain.LimeLight.I.Set(pid_DriveLimeLight->GetI());
				Dashboard.Drivetrain.LimeLight.D.Set(pid_DriveLimeLight->GetD());

				Dashboard.Drivetrain.Angle.P.Set(pid_DriveAngle->GetP());
				Dashboard.Drivetrain.Angle.I.Set(pid_DriveAngle->GetI());
				Dashboard.Drivetrain.Angle.D.Set(pid_DriveAngle->GetD());

				Dashboard.Drivetrain.Distance.P.Set(pid_DriveDistance->GetP());
				Dashboard.Drivetrain.Distance.I.Set(pid_DriveDistance->GetI());
				Dashboard.Drivetrain.Distance.D.Set(pid_DriveDistance->GetD());

			}

			if(Dashboard.Misc.TuningMode.Get()) {
				ConfigureLimeLightPID(Dashboard.Drivetrain.LimeLight.P.Get(), Dashboard.Drivetrain.LimeLight.I.Get(), Dashboard.Drivetrain.LimeLight.D.Get());
				ConfigureAnglePID(Dashboard.Drivetrain.Angle.P.Get(), Dashboard.Drivetrain.Angle.I.Get(), Dashboard.Drivetrain.Angle.D.Get());
				ConfigureDistancePID(Dashboard.Drivetrain.Distance.P.Get(), Dashboard.Drivetrain.Distance.I.Get(), Dashboard.Drivetrain.Distance.D.Get());
			}

			Dashboard.Drivetrain.LimeLight.Offset.Set(GetDistance()); //TODO DO THE PROPER VALUE
			Dashboard.Drivetrain.LimeLight.Enabled.Set(IsLimeLightPIDEnabled());
			Dashboard.Drivetrain.LimeLight.OnTarget.Set(IsAtCube());

			Dashboard.Drivetrain.Angle.Angle.Set(GetAngle());
			Dashboard.Drivetrain.Angle.Enabled.Set(IsAnglePIDEnabled());
			Dashboard.Drivetrain.Angle.OnTarget.Set(IsAtAngle());
			Dashboard.Drivetrain.Angle.MaxError.Set(m_MaxError);

			Dashboard.Drivetrain.Distance.Measurement.Set(GetDistance());
			Dashboard.Drivetrain.Distance.Enabled.Set(IsDistancePIDEnabled());
			Dashboard.Drivetrain.Distance.OnTarget.Set(IsAtDistance());

			if(IsLimeLightPIDEnabled()) {
				if (IsAtCube()) pid_DriveLimeLight->Disable();
				SetDrive(pid_DriveDistance->Get() + pid_DriveLimeLight->Get(), pid_DriveDistance->Get() - pid_DriveLimeLight->Get());
			}
			else if(IsDistancePIDEnabled()){
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

			m_IsTuningMode = Dashboard.Misc.TuningMode.Get();

		}

	private:
		CTRLib::TalonSRX		*mtr_L_Drive_1;
		CTRLib::TalonSRX		*mtr_R_Drive_1;

		//TODO Consider incorporating one pid for each side, just lots of control
		MOLib::Sensors::MagEncoder *enc_L_DriveDistance;
		MOLib::Sensors::MagEncoder *enc_R_DriveDistance;

		WPILib::ADXRS450_Gyro		*gyr_DriveAngle;

		MOLib::Vision::LimeLight	*lml_DriveLimeLight;

		//TODO Add pid for drivestraight, consider Turn/Angle, Distance, Straight, Cubes rather than conventional <system><sensor> naming scheme
		MOLib::PIDLoop	*pid_DriveAngle;
		MOLib::PIDLoop	*pid_DriveDistance;
		MOLib::PIDLoop	*pid_DriveLimeLight;

		float			m_L_Power			= 0.0;
		float			m_R_Power			= 0.0;

		double			m_WheelDiameter 	= 0.0;
		double			m_GearRatio			= 1.0;

		float			m_MaxError			= 0.0;

		bool m_IsTuningMode					= false;
	};
}
