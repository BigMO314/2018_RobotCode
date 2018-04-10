#include "MOLib.h"
#include "Dashboard.h"
#include "Configuration/Goal.h"
#include "Configuration/Robot.h"
#include "ControlPeriod/Autonomous.h"
#include "ControlPeriod/TeleOperated.h"
#include "RobotMechanism/Arm.h"
#include "RobotMechanism/Drivetrain.h"
#include "RobotMechanism/Climber.h"

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc_c.h>
#include <opencv2/imgproc.hpp>

class Robot : public frc::SampleRobot {
public:
	Robot() {
		//delete everything in SmartDashboard, clears clutter
		nt::DeleteAllEntries(nt::GetDefaultInstance());

		Dashboard.Drivetrain.LeftScale.Set(0.75);

		Dashboard.Misc.TuningMode.Set(false);
		Dashboard.Misc.TestingMode.Set(false);
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
		rbt_Drivetrain->ConfigureAnglePID(0.024, 0.0, 0.03);
		rbt_Drivetrain->ConfigureDistancePID(0.00015, 0.0, 0.03);


		//Arm
		mtr_Angle_1->SetInverted(false);
		mtr_Angle_2->SetInverted(false);
		mtr_Shooter1->SetInverted(true);
		mtr_Shooter2->SetInverted(true);
		mtr_Angle_2->Follow(*mtr_Angle_1);
		mtr_Angle_1->ConfigSelectedFeedbackSensor(FeedbackDevice::CTRE_MagEncoder_Absolute, 0, 0);

		mtr_Angle_1->ConfigForwardSoftLimitThreshold(99.0 / Configuration::Robot::Arm::DegreesPerCount, 0);
		mtr_Angle_1->ConfigReverseSoftLimitThreshold(-99.0 / Configuration::Robot::Arm::DegreesPerCount, 0);

		mtr_Angle_1->ConfigForwardSoftLimitEnable(true, 0);
		mtr_Angle_1->ConfigReverseSoftLimitEnable(true, 0);

		rbt_Arm->ConfigPeakOutput(0.6, -0.6);
		rbt_Arm->ResetAngle();

		//Intake
		mtr_L_Intake->SetInverted(!jmp_CompRobot->Get());
		mtr_R_Intake->SetInverted(jmp_CompRobot->Get());

		lml_Vision->DisableLight();

		std::cout << ">>Calibrating Gyro..." << std::endl;
		gyr_DriveAngle->Calibrate();
		std::cout << ">>Gyro Calibrated" << std::endl;

		SmartDashboard::PutData("Autonomous/PathChooser", &prd_Autonomous->chs_Path);
		SmartDashboard::PutData("Autonomous/PositionChooser", &prd_Autonomous->chs_Position);

		//Limelight
		lml_Vision->SetVisionMode(0);

		cs::UsbCamera camera = CameraServer::GetInstance()->StartAutomaticCapture();
		camera.SetResolution(120, 80);
		camera.SetFPS(20);


	}

	static void VisionThread() {

		cs::UsbCamera camera = CameraServer::GetInstance()->StartAutomaticCapture();
		camera.SetResolution(160, 120);
		camera.SetFPS(20);
		cs::CvSink cvSink = CameraServer::GetInstance()->GetVideo();
		cs::CvSource outputStreamStd = CameraServer::GetInstance()->PutVideo("Driver Camera", 160, 120);
		cv::Mat source;
		cv::Mat output;
		while(true) {
			cvSink.GrabFrame(source);
			line(source, cv::Point(80, 15), cv::Point(80, 77), CV_RGB(0, 255, 0));
			line(source, cv::Point(60, 77), cv::Point(100, 77), CV_RGB(0, 255, 0));
			outputStreamStd.PutFrame(source);
		}
	}

	void RobotInit() {
		rbt_Drivetrain->ConfigWheelDiameter(Configuration::Robot::Drivetrain::WheelDiameter);
//		std::thread visionThread(VisionThread);
//		visionThread.detach();
	}

	void Autonomous() {
		prd_Autonomous->AutonInit();
		gyr_DriveAngle->Reset();

		if(jmp_CompRobot->Get()) rbt_Drivetrain->ConfigScale(0.66);
		else rbt_Drivetrain->ConfigScale(0.65032, 0.66, -0.66, -0.66); //First should be 0.7125

		while(IsAutonomous() && IsEnabled()){
			if (jmp_CompRobot->Get()) rbt_Drivetrain->ConfigScale(0.75, 0.75, -0.75, -0.75);
			else rbt_Drivetrain->ConfigScale(0.68, 0.72, -0.72, -0.72);
			prd_Autonomous->Update();
			rbt_Drivetrain->Update();
			rbt_Arm->Update();
			frc::Wait(0.005);
		}
	}

	void OperatorControl() override {
		bkn_Decoration->SetMode(MOLib::Lights::Blinkin::ColorMode::Pattern::Color1::LightChase);

		rbt_Drivetrain->DisableDistancePID();
		rbt_Drivetrain->DisableLimeLightPID();

		rbt_Climber->Reset();

		if(jmp_CompRobot->Get()) rbt_Drivetrain->ConfigScale(0.75);
		else rbt_Drivetrain->ConfigScale(Dashboard.Drivetrain.LeftScale.Get(), 0.72, -0.72, -0.72); //First should be 0.7125

		while (IsOperatorControl() && IsEnabled()) {


			//Update inputs
			Dashboard.Misc.IsCompBot.Set(jmp_CompRobot->Get());
			prd_TeleOperated->Update();
			rbt_Drivetrain->Update();
			rbt_Arm->Update();
			rbt_Climber->Update();
			if(jmp_CompRobot->Get()) rbt_Drivetrain->ConfigScale(Dashboard.Drivetrain.LeftScale.Get(), 0.75, -0.75, -0.75); //First should be 0.766
			else if(!Dashboard.Misc.TuningMode.Get()) rbt_Drivetrain->ConfigScale(0.68, 0.72, -0.72, -0.72);
			else if(Dashboard.Misc.TuningMode.Get())rbt_Drivetrain->ConfigScale(Dashboard.Drivetrain.LeftScale.Get(), 0.72, -0.72, -0.72); //First should be 0.7125



			frc::Wait(0.005);
		}
	}

	void Test() override {

	}

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

	//Vision Stuff
	MOLib::Vision::LimeLight		*lml_Vision			= new MOLib::Vision::LimeLight(0);

	//Create Drivetrain object																														_
	RobotMechanism::Drivetrain		*rbt_Drivetrain		= new RobotMechanism::Drivetrain(mtr_L_Drive_1,mtr_R_Drive_1,gyr_DriveAngle,lml_Vision);

	//Talon and Victor for Arm																													_
	CTRLib::TalonSRX				*mtr_Angle_1		= new CTRLib::TalonSRX(6);
	CTRLib::VictorSPX				*mtr_Angle_2		= new CTRLib::VictorSPX(7);

	WPILib::Solenoid				*sol_Winch			= new WPILib::Solenoid(0);


	//Victors for Intake																															_
	WPILib::VictorSP				*mtr_L_Intake		= new WPILib::VictorSP(0);
	WPILib::VictorSP				*mtr_R_Intake		= new WPILib::VictorSP(1);
	WPILib::VictorSP				*mtr_Shooter1		= new WPILib::VictorSP(2);
	WPILib::VictorSP				*mtr_Shooter2		= new WPILib::VictorSP(3);
	//Intake sensors																																_
	WPILib::DigitalInput			*lim_Shooter		= new WPILib::DigitalInput(9);

	//Create Arm object
	RobotMechanism::Arm				*rbt_Arm			= new RobotMechanism::Arm(mtr_Angle_1, mtr_Angle_2, mtr_L_Intake, mtr_R_Intake, mtr_Shooter1, mtr_Shooter2, sol_Winch, lim_Shooter);

	//Create Climber motor and solenoid for Climber
	WPILib::VictorSP				*mtr_Climb			= new WPILib::VictorSP(5);
	WPILib::Solenoid				*sol_Climb			= new WPILib::Solenoid(1);
	WPILib::Solenoid				*sol_Lock			= new WPILib::Solenoid(7);

	//Create Climber object
	RobotMechanism::Climber			*rbt_Climber		= new RobotMechanism::Climber(mtr_Climb, sol_Climb, sol_Lock);

	//Controllers for TeleOperated																													_
	MOLib::XboxController			*ctl_Driver			= new MOLib::XboxController(0);
	MOLib::XboxController			*ctl_Operator		= new MOLib::XboxController(1);

	//Create Control Periods																														_
	ControlPeriod::TeleOperated		*prd_TeleOperated	= new ControlPeriod::TeleOperated(rbt_Drivetrain,rbt_Arm,rbt_Climber,ctl_Driver,ctl_Operator);
	ControlPeriod::Autonomous		*prd_Autonomous		= new ControlPeriod::Autonomous(rbt_Drivetrain,rbt_Arm);

	MOLib::Sensors::Jumper			*jmp_CompRobot		= new MOLib::Sensors::Jumper(0);

	MOLib::Lights::Blinkin			*bkn_Decoration		= new MOLib::Lights::Blinkin(4);

	std::string m_CamName = "Grey";


};

START_ROBOT_CLASS(Robot);
