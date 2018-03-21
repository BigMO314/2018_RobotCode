#include "MOLib.h"
#include "Dashboard.h"
#include "Configuration/Goal.h"
#include "Configuration/Robot.h"
#include "ControlPeriod/Autonomous.h"
#include "ControlPeriod/TeleOperated.h"
#include "RobotMechanism/Arm.h"
#include "RobotMechanism/Drivetrain.h"

class Lockdown : public frc::SampleRobot {
public:
	Lockdown() {
		//delete everything in SmartDashboard, clears clutter
		nt::DeleteAllEntries(nt::GetDefaultInstance());

		Dashboard.Drivetrain.Distance.P.Set(0.00015);	//0.00015
		Dashboard.Drivetrain.Distance.I.Set(0.0);
		Dashboard.Drivetrain.Distance.D.Set(0.00027);	//0.00027

		Dashboard.Drivetrain.LeftScale.Set(0.76);

		Dashboard.Drivetrain.Angle.P.Set(0.0215); //0.024 Potential new values 0.0215
		Dashboard.Drivetrain.Angle.I.Set(0.0);
		Dashboard.Drivetrain.Angle.D.Set(0.0262); //0.0201 Potential new values 0.0262

		Dashboard.Arm.Angle.P.Set(0.75); // 1.0 Potential new values 0.75
		Dashboard.Arm.Angle.I.Set(0.0); // 0.0
		Dashboard.Arm.Angle.D.Set(20.0); // 0.0 Potential new values

		Dashboard.Misc.TuningMode.Set(false);
		Dashboard.Misc.RunDistance.Set(false);

		Dashboard.Vision.LightEnabled.Set(false);

		rbt_Drivetrain->ResetAngle();
		rbt_Drivetrain->ResetDistance();

		//Drivetrain
		mtr_L_Drive_1->SetInverted(true);
		mtr_L_Drive_2->SetInverted(true);
		mtr_L_Drive_3->SetInverted(true);

		mtr_L_Drive_2->Follow(*mtr_L_Drive_1);
		mtr_L_Drive_3->Follow(*mtr_L_Drive_1);
		mtr_L_Drive_1->ConfigSelectedFeedbackSensor(FeedbackDevice::CTRE_MagEncoder_Absolute, 0, 0);
		mtr_R_Drive_1->SetInverted(false);
		mtr_R_Drive_2->SetInverted(false);
		mtr_R_Drive_3->SetInverted(false);
		mtr_R_Drive_2->Follow(*mtr_R_Drive_1);
		mtr_R_Drive_3->Follow(*mtr_R_Drive_1);
		mtr_R_Drive_1->ConfigSelectedFeedbackSensor(FeedbackDevice::CTRE_MagEncoder_Absolute, 0, 0);


		//Arm
		mtr_Angle_1->SetInverted(false);
		mtr_Angle_2->SetInverted(false);
		mtr_Shooter1->SetInverted(true);
		mtr_Shooter2->SetInverted(true);
		mtr_Angle_2->Follow(*mtr_Angle_1);
		mtr_Angle_1->ConfigSelectedFeedbackSensor(FeedbackDevice::CTRE_MagEncoder_Absolute, 0, 0);

		mtr_Angle_1->ConfigForwardSoftLimitThreshold(97.0 / Configuration::Robot::Arm::DegreesPerCount, 0);
		mtr_Angle_1->ConfigReverseSoftLimitThreshold(-97.0 / Configuration::Robot::Arm::DegreesPerCount, 0);

		mtr_Angle_1->ConfigForwardSoftLimitEnable(true, 0);
		mtr_Angle_1->ConfigReverseSoftLimitEnable(true, 0);

		rbt_Arm->ConfigPeakOutput(0.6, -0.6);
		rbt_Arm->ResetAngle();

		//Intake
		mtr_L_Intake->SetInverted(!jmp_CompRobot->Get());
		mtr_R_Intake->SetInverted(jmp_CompRobot->Get());

		std::cout << ">>Calibrating Gyro..." << std::endl;
		gyr_DriveAngle->Calibrate();
		std::cout << ">>Gyro Calibrated" << std::endl;

		SmartDashboard::PutData("Autonomous/PathChooser", &prd_Autonomous->chs_Auton);
		SmartDashboard::PutData("Autonomous/PositionChooser", &prd_Autonomous->chs_Position);

		cs::UsbCamera camera = CameraServer::GetInstance()->StartAutomaticCapture();

		camera.SetResolution(160, 120);
		camera.SetFPS(20);

		//Limelight
		lml_Vision->SetVisionMode(0);
		lml_Vision->DisableLight();

	}

	void RobotInit() {

		rbt_Drivetrain->ConfigWheelDiameter(Configuration::Robot::Drivetrain::WheelDiameter);
	}

	void Autonomous() {
		prd_Autonomous->AutonInit();
		gyr_DriveAngle->Reset();
		rbt_Drivetrain->ConfigScale(0.75, 0.75, -0.75, -0.75);

		while(IsAutonomous() && IsEnabled()){
			prd_Autonomous->Update();
			rbt_Drivetrain->Update();
			rbt_Arm->Update();
			frc::Wait(0.005);
		}
	}

	void OperatorControl() override {
		rbt_Drivetrain->DisableDistancePID();
		rbt_Drivetrain->DisableLimeLightPID();
		if(jmp_CompRobot->Get()) rbt_Drivetrain->ConfigScale(0.75, 0.75, -0.75, -0.75);
		else rbt_Drivetrain->ConfigScale(Dashboard.Drivetrain.LeftScale.Get(), 0.75, -0.75, -0.75);
		while (IsOperatorControl() && IsEnabled()) {
			//Update inputs
			prd_TeleOperated->Update();
			rbt_Drivetrain->Update();
			rbt_Arm->Update();

			Dashboard.Arm.ShooterLim.Set(lim_Shooter->Get());

			frc::Wait(0.005);
		}
	}

	void Test() override {}

private:
	//Talons and Victors for Drivetrain																												_
	CTRLib::TalonSRX				*mtr_L_Drive_1		= new CTRLib::TalonSRX(0);
	CTRLib::VictorSPX				*mtr_L_Drive_2		= new CTRLib::VictorSPX(1);
	CTRLib::VictorSPX				*mtr_L_Drive_3		= new CTRLib::VictorSPX(2);
	CTRLib::TalonSRX				*mtr_R_Drive_1		= new CTRLib::TalonSRX(3);
	CTRLib::VictorSPX				*mtr_R_Drive_2		= new CTRLib::VictorSPX(4);
	CTRLib::VictorSPX				*mtr_R_Drive_3		= new CTRLib::VictorSPX(5);

	WPILib::ADXRS450_Gyro			*gyr_DriveAngle		= new ADXRS450_Gyro(SPI::kOnboardCS0);
	MOLib::Sensors::MagEncoder		*enc_DriveDistance	= new MOLib::Sensors::MagEncoder(mtr_L_Drive_1);

	//Create Drivetrain object																														_
	RobotMechanism::Drivetrain		*rbt_Drivetrain		= new RobotMechanism::Drivetrain(mtr_L_Drive_1,mtr_R_Drive_1,gyr_DriveAngle,lml_Vision);

	//Talon and Victor for Arm																													_
	CTRLib::TalonSRX				*mtr_Angle_1		= new CTRLib::TalonSRX(6);
	CTRLib::VictorSPX				*mtr_Angle_2		= new CTRLib::VictorSPX(7);

	WPILib::Solenoid				*sol_Winch			= new WPILib::Solenoid(0);


	//Victors for Intake																															_
	WPILib::VictorSP				*mtr_L_Intake		= new WPILib::VictorSP(0);
	WPILib::VictorSP				*mtr_R_Intake		= new WPILib::VictorSP(1);
	VictorSP						*mtr_Shooter1		= new		  VictorSP(2);
	VictorSP						*mtr_Shooter2		= new		  VictorSP(3);
	//Intake sensors																																_
	WPILib::DigitalInput			*lim_Shooter		= new WPILib::DigitalInput(9);

	//Create Arm object
	RobotMechanism::Arm				*rbt_Arm			= new RobotMechanism::Arm(mtr_Angle_1, mtr_Angle_2, mtr_L_Intake, mtr_R_Intake, mtr_Shooter1, mtr_Shooter2, sol_Winch, lim_Shooter);

	//Controllers for TeleOperated																													_
	MOLib::XboxController			*ctl_Driver			= new MOLib::XboxController(0);
	MOLib::XboxController			*ctl_Operator		= new MOLib::XboxController(1);

	//Create Control Periods																														_
	ControlPeriod::TeleOperated		*prd_TeleOperated	= new ControlPeriod::TeleOperated(rbt_Drivetrain,rbt_Arm,ctl_Driver,ctl_Operator);
	ControlPeriod::Autonomous		*prd_Autonomous		= new ControlPeriod::Autonomous(rbt_Drivetrain,rbt_Arm);

	MOLib::Sensors::Jumper			*jmp_CompRobot		= new MOLib::Sensors::Jumper(0);

	//Vision Stuff
	MOLib::Vision::LimeLight		*lml_Vision			= new MOLib::Vision::LimeLight(0);


};

START_ROBOT_CLASS(Lockdown);
