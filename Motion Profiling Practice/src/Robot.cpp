#include "MOLib.h"
#include "RobotMechanism/Drivetrain.h"
#include "RobotMechanism/Elevator.h"
#include "RobotMechanism/Intake.h"
#include "Configuration/Goal.h"
#include "Configuration/Robot.h"
#include "ControlPeriod/Autonomous.h"
#include "ControlPeriod/TeleOperated.h"

extern "C" {
#include <pathfinder.h>
}

class Lockdown : public frc::SampleRobot {
public:
	Lockdown() {
		//delete everything in SmartDashboard, clears clutter
		nt::DeleteAllEntries(nt::GetDefaultInstance());

		SmartDashboard::PutNumber("Drive Scale",0.6);
		SmartDashboard::PutNumber("Steer Scale",0.6);
		SmartDashboard::PutBoolean("Inverted",false);

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

		rbt_Drivetrain->ConfigureDrivetrainPID(2, 0, 0.0085);
		rbt_Drivetrain->ConfigPeakoutput(0.8, -0.8, 0.8, -0.8);

		//Elevator
		mtr_L_Elevator->SetInverted(true);
		mtr_R_Elevator->SetInverted(false);
		mtr_R_Elevator->Follow(*mtr_L_Elevator);
		mtr_L_Elevator->ConfigSelectedFeedbackSensor(FeedbackDevice::CTRE_MagEncoder_Absolute, 0, 0);

		//Intake
		mtr_L_Intake->SetInverted(true);
		mtr_R_Intake->SetInverted(false);


		mtr_L_Elevator->SetSensorPhase(!jmp_CompRobot->Get());

		SmartDashboard::PutNumber("Elevator Speed", 0.1);
	}

	void RobotInit() {

		rbt_Drivetrain->ConfigWheelDiameter(Configuration::Robot::Drivetrain::met_WheelDiameter);
		rbt_Drivetrain->ConfigWheelCircumfrance(Configuration::Robot::Drivetrain::met_WheelCircumfrance);
		rbt_Drivetrain->ConfigMaxVelocity(Configuration::Robot::Drivetrain::met_MaxVelocity);
		rbt_Drivetrain->ConfigureDrivetrainPID(0.0,0.0,0.0);

		rbt_Elevator->ConfigPeakOutput(Configuration::Robot::Elevator::PosElevatorPower,Configuration::Robot::Elevator::NegElevatorPower);
	}

	void Autonomous() {
		while(IsAutonomous() && IsEnabled()){
			prd_TeleOperated->Update();
			rbt_Drivetrain->Update();
			rbt_Elevator->Update();
			rbt_Intake->Update();
			frc::Wait(0.005);
		}
	}

	void OperatorControl() override {
		while (IsOperatorControl() && IsEnabled()) {
			//Update inputs
			prd_TeleOperated->Update();
			rbt_Drivetrain->Update();
			rbt_Elevator->Update();
			rbt_Intake->Update();
			frc::Wait(0.005);
			dsh_IsCompBot->Set(jmp_CompBot->Get());
		}
	}

	void Test() override {}

private:
	//Talons and Victors for Drivetrain																												_
	TalonSRX						*mtr_L_Drive_1		= new TalonSRX(0);
	VictorSPX						*mtr_L_Drive_2		= new VictorSPX(1);
	VictorSPX						*mtr_L_Drive_3		= new VictorSPX(2);
	TalonSRX						*mtr_R_Drive_1		= new TalonSRX(3);
	VictorSPX						*mtr_R_Drive_2		= new VictorSPX(4);
	VictorSPX						*mtr_R_Drive_3		= new VictorSPX(5);

	Segment							*leftTrajectory		= new Segment();
	Segment							*rightTrajectory	= new Segment();

	ADXRS450_Gyro					*gyr_DriveAngle		= new ADXRS450_Gyro(frc::SPI::Port::kOnboardCS0);
	//Create Drivetrain object																														_
	RobotMechanism::Drivetrain		*rbt_Drivetrain		= new RobotMechanism::Drivetrain(mtr_L_Drive_1,mtr_R_Drive_1,gyr_DriveAngle,leftTrajectory,rightTrajectory);

	//Victors for Intake																															_
	VictorSP						*mtr_L_Intake		= new VictorSP(0);
	VictorSP						*mtr_R_Intake		= new VictorSP(1);
	//Solenoids for Intake																															_
	Solenoid						*sol_Arm			= new Solenoid(0);
	Solenoid						*sol_Slider			= new Solenoid(1);
	Solenoid						*sol_Gripper		= new Solenoid(2);
	//Intake sensors																																_
	DigitalInput					*pho_Intake			= new DigitalInput(9);
	//Create Intake object																															_
	RobotMechanism::Intake			*rbt_Intake			= new RobotMechanism::Intake(mtr_L_Intake,mtr_R_Intake,sol_Arm,sol_Slider,sol_Gripper,pho_Intake);

	//Talon and Victor for Elevator																													_
	TalonSRX						*mtr_L_Elevator		= new TalonSRX(6);
	VictorSPX						*mtr_R_Elevator		= new VictorSPX(7);

	//Create Elevator object
	RobotMechanism::Elevator		*rbt_Elevator		= new RobotMechanism::Elevator(mtr_L_Elevator, mtr_R_Elevator, rbt_Intake);

	//Controllers for TeleOperated																													_
	MOLib::XboxController			*ctl_Driver			= new MOLib::XboxController(0);
	MOLib::XboxController			*ctl_Operator		= new MOLib::XboxController(1);

	//Create Control Periods																														_
	ControlPeriod::TeleOperated		*prd_TeleOperated	= new ControlPeriod::TeleOperated(rbt_Drivetrain,rbt_Elevator,rbt_Intake,ctl_Driver,ctl_Operator);
	ControlPeriod::Autonomous		*prd_Autonomous		= new ControlPeriod::Autonomous(rbt_Drivetrain,rbt_Elevator,rbt_Intake);

	DigitalInput					*jmp_CompRobot		= new DigitalInput(0);
	MOLib::Sensors::Jumper			*jmp_CompBot		= new MOLib::Sensors::Jumper(0);

	MOLib::Dashboard::Indicator		*dsh_IsCompBot		= new MOLib::Dashboard::Indicator("Is Comp Bot?");

};

START_ROBOT_CLASS(Lockdown);
