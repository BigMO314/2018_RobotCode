#pragma once
#include "MOLib.h"
#include "Configuration/Robot.h"
extern "C" {
#include "pathfinder.h"
}

namespace RobotMechanism{
	class Drivetrain{
	public:
		Drivetrain(TalonSRX *mtr_L_Drive_1, TalonSRX *mtr_R_Drive_1,ADXRS450_Gyro *gyr_DriveAngle,Segment *leftTrajectory,Segment *rightTrajectory){
			this->mtr_L_Drive_1		= mtr_L_Drive_1;
			this->mtr_R_Drive_1		= mtr_R_Drive_1;
			this->gyr_DriveAngle	= gyr_DriveAngle;
			this->leftTrajectory	= leftTrajectory;
			this->rightTrajectory	= rightTrajectory;
		};
		~Drivetrain(){};

		void ConfigWheelDiameter(double diameter) 			{ m_Diameter		= diameter;		}
		void ConfigWheelCircumfrance(double circumfrance)	{ m_Circumfrance	= circumfrance;	}
		void ConfigMaxVelocity(double max_velocity)			{ m_MaxVelocity		= max_velocity;	}
		void ConfigPIDConst(int pid_const)					{ m_PIDConst		= pid_const;	}
		void ConfigPIDTimeout(int pid_timeout)				{ m_PIDTimeOut		= pid_timeout;	}
		void ConfigMaxAcceleration(double acceleration)		{ m_MaxAcceleration	= acceleration;	}

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

		void ResetSensors(){
					mtr_L_Drive_1->SetSelectedSensorPosition(0, 0, 0);
					mtr_R_Drive_1->SetSelectedSensorPosition(0, 0, 0);
				}

				void StopPID(){
					mtr_L_Drive_1->Set(ControlMode::PercentOutput, 0.0);
					mtr_R_Drive_1->Set(ControlMode::PercentOutput, 0.0);
				}

				double GetGyroHeading(){
					return gyr_DriveAngle->GetAngle();
				}

				int GetLDrivePosition(){
					return mtr_L_Drive_1->GetSelectedSensorPosition(0);
				}

				int GetRDrivePosition(){
					return mtr_R_Drive_1->GetSelectedSensorPosition(0);
				}

				double GetLDriveVelocity(){
					return mtr_L_Drive_1->GetSelectedSensorVelocity(0);
				}

				double GetRDriveVelocity(){
					return mtr_R_Drive_1->GetSelectedSensorVelocity(0);
				}

				double GetLMP1Test(){
					return MP_1_l;
				}

				double GetRMP1Test(){
					return MP_1_r;
				}

				int isLMP1FollowerFinished(){
					return LTempVar;
				}

				int isRMP1FollowerFinished(){
					return RTempVar;
				}

				int GetlenthofMP(){
					return LengthTempVar;
				}

				void ConfigPeakoutput(double l_peakoutputfor, double l_peakoutputrev, double r_peakoutputfor, double r_peakoutputrev){
							mtr_L_Drive_1->ConfigPeakOutputForward(l_peakoutputfor, 0);
							mtr_L_Drive_1->ConfigPeakOutputReverse(l_peakoutputrev, 0);
							mtr_R_Drive_1->ConfigPeakOutputForward(r_peakoutputfor, 0);
							mtr_R_Drive_1->ConfigPeakOutputReverse(r_peakoutputrev, 0);
						}

						void ConfigureDrivetrainPID(double P, double I, double D) {
							mtr_L_Drive_1->Config_kP(m_PIDConst, P, m_PIDTimeOut);
							mtr_L_Drive_1->Config_kI(m_PIDConst, I, m_PIDTimeOut);
							mtr_L_Drive_1->Config_kD(m_PIDConst, D, m_PIDTimeOut);

							mtr_R_Drive_1->Config_kP(m_PIDConst, P, m_PIDTimeOut);
							mtr_R_Drive_1->Config_kI(m_PIDConst, I, m_PIDTimeOut);
							mtr_R_Drive_1->Config_kD(m_PIDConst, D, m_PIDTimeOut);
						}

						void GoToDistance(double dist) {
							 //dist *= (4096.0 / M_PI);
							 //dist *= (1024 / M_PI);

							 ref_distance = dist;


							mtr_L_Drive_1->Set(ControlMode::Position, dist);
							mtr_R_Drive_1->Set(ControlMode::Position, dist);

						}

						void RunMP1(){

							Waypoint points[POINT_LENGTH];

							Waypoint p1 = { 2, 0, 0 };
							Waypoint p2 = { 1, 0, 0 };

							points[0] = p1;
							points[1] = p2;


							TrajectoryCandidate candidate;
					// Prepare the Trajectory for Generation.
					//
					// Arguments:
					// Fit Function:        FIT_HERMITE_CUBIC (^3; a hermite polynomial is a sequence that exhibits  orthogonality [perpendicular nature]) or FIT_HERMITE_QUINTIC (^5)
					// Sample Count:        PATHFINDER_SAMPLES_HIGH (100 000)
					//                      PATHFINDER_SAMPLES_LOW  (10 000)
					//                      PATHFINDER_SAMPLES_FAST (1 000)
					// Time Step:           0.001 Seconds
					// Max Velocity:        15 m/s
					// Max Acceleration:    10 m/s/s
					// Max Jerk:            60 m/s/s/s


							//FIT_HERMITE_CUBIC, PATHFINDER_SAMPLES_HIGH
							pathfinder_prepare(points, POINT_LENGTH, FIT_HERMITE_CUBIC, PATHFINDER_SAMPLES_LOW, 0.001, m_MaxVelocity, m_MaxAcceleration, 60.0, &candidate);

							length = candidate.length;

					//Array of Segments (the trajectory points) to store the trajectory in
							Segment* trajectory = (Segment*)malloc(length * sizeof(Segment));

					// Generate the trajectory
							result = pathfinder_generate(&candidate, trajectory);
							if (result < 0) {
							    // An error occured
							    printf("ERROR! Trajectory could not be generated!\n");
							}
							leftTrajectory = (Segment*)malloc(length * sizeof(Segment));
							rightTrajectory = (Segment*)malloc(length * sizeof(Segment));

					// Generate the Left and Right trajectories of the wheelbase using the originally generated trajectory
							pathfinder_modify_tank(trajectory, length, leftTrajectory, rightTrajectory,m_Diameter);

							RunMotionProfile();
							free(trajectory);
							free(leftTrajectory);
							free(rightTrajectory);

						}

						int ReturnGenerateResult(){
							return result;
						}

						void RunMotionProfile(){
							EncoderFollower* MP_1_L_Follower = (EncoderFollower*)malloc(sizeof(EncoderFollower));
							MP_1_L_Follower->last_error = 0; MP_1_L_Follower->segment = 0; MP_1_L_Follower->finished = 0;     // Just in case!

							LTempVar = MP_1_L_Follower->finished;

							EncoderFollower* MP_1_R_Follower = (EncoderFollower*)malloc(sizeof(EncoderFollower));
							MP_1_R_Follower->last_error = 0; MP_1_R_Follower->segment = 0; MP_1_R_Follower->finished = 0;     // Just in case!

							RTempVar = MP_1_R_Follower->finished;


							EncoderConfig L_Config = {GetLDrivePosition(), 4090, m_Circumfrance,      // Position, Ticks per Rev, Wheel Circumference
							                         1.0, 0.0, 0.0, 1 / (m_MaxVelocity), 0.0};			// Kp, Ki, Kd and Kv, Ka (The Kv value is 1 over your robot's maximum velocity as provided in the initial trajectory)

							EncoderConfig R_Config = {GetRDrivePosition(), 4090, m_Circumfrance,      // Position, Ticks per Rev, Wheel Circumference
									                 1.0, 0.0, 0.0, 1 / (m_MaxVelocity), 0.0};			// Kp, Ki, Kd and Kv, Ka (The Kv value is 1 over your robot's maximum velocity as provided in the initial trajectory)

							// Arg 1: The EncoderConfig
							// Arg 2: The EncoderFollower for this side
							// Arg 3: The Trajectory generated from `pathfinder_modify_tank`
							// Arg 4: The Length of the Trajectory (length used in Segment seg[length];)
							// Arg 5: The current value of your encoder

							LengthTempVar = length;

							while(MP_1_L_Follower->finished != 1 && MP_1_R_Follower->finished != 1){
								MP_1_l = pathfinder_follow_encoder(L_Config, MP_1_L_Follower, leftTrajectory, length, GetLDrivePosition());
								MP_1_r = pathfinder_follow_encoder(R_Config, MP_1_R_Follower, rightTrajectory, length, GetRDrivePosition());
								SetDrive((MP_1_l), (MP_1_r));
							}

								free(MP_1_L_Follower);
								free(MP_1_R_Follower);


							// -- using l and r from the previous code block -- //
							/*double gyro_heading = gyr_DriveAngle->GetAngle();      // Assuming gyro angle is given in degrees
							double desired_heading = (MP_1_R_Follower.heading);

							double angle_difference = desired_heading - gyro_heading;*/    // Make sure to bound this from -180 to 180, otherwise you will get super large values

							//double turn = 0.8 * (-1.0/80.0) * angle_difference;

						}



		//update the motors
		void Update(){
			mtr_L_Drive_1->Set(ControlMode::PercentOutput, m_L_Power);
			mtr_R_Drive_1->Set(ControlMode::PercentOutput, m_R_Power);
		};

	private:
		TalonSRX		*mtr_L_Drive_1;
		TalonSRX		*mtr_R_Drive_1;

		Segment			*leftTrajectory;
		Segment			*rightTrajectory;

		ADXRS450_Gyro	*gyr_DriveAngle;

		float			m_L_Power			= 0.0;
		float			m_R_Power			= 0.0;

		double			m_Diameter 			= 0.0;
		double			m_Circumfrance		= 0.0;
		double			m_MaxVelocity		= 0.0;
		double			m_MaxAcceleration	= 0.0;

		int				length 				= 2;

		int 			POINT_LENGTH		= 2;

		int				m_PIDTimeOut		= 0;
		int				m_PIDConst			= 0;

		double			ref_distance		= 0;
		double			error_tolerance		= 0.5;
		double			MP_1_l				= 0.0;
		double			MP_1_r				= 0.0;

		int				result				= 0;

		int				LTempVar			= -1;
		int				RTempVar			= -1;
		int				LengthTempVar		= -1;
	};
}
