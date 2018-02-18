#pragma once
#include "MOLib.h"
#include "Configuration/Robot.h"

namespace RobotMechanism{
	class Intake{
	public:
		enum class ArmState		{ kLifted, kLowered };
		enum class SliderState	{ kExtended, kRetracted };
		enum class GripperState	{ kEngaged, kDisengaged };

		Intake(VictorSP *mtr_L_Intake, VictorSP *mtr_R_Intake, Solenoid *sol_Arm, Solenoid *sol_Slider, Solenoid *sol_Gripper,DigitalInput *pho_Intake){
			this->mtr_L_Intake	= mtr_L_Intake;
			this->mtr_R_Intake	= mtr_R_Intake;
			this->sol_Arm		= sol_Arm;
			this->sol_Slider	= sol_Slider;
			this->sol_Gripper	= sol_Gripper;
			this->pho_Intake	= pho_Intake;
		}
		Intake();

		//Direct functions																							_
		/**
		 * @brief Sets power variable for the intake motors.
		 * @param power Power to set variable to.
		 */
		void SetIntakePower(float power)			{ m_intakePower = power;									}

		/**
		 * @brief Sets the state of the tilt arm.
		 * @param state What to set the state as.
		 */

		void SetArmState(ArmState state)			{ m_State.Arm		= (state != m_DefaultState.Arm);		}

		/**
		 * @brief Sets the extension state of the intake arm.
		 * @param state What to set the slide state as.
		 */
		void SetSliderState(SliderState state)		{ m_State.Slider	= (state != m_DefaultState.Slider);		}

		/**
		 * @brief Sets the state of the solenoid for holding the power cube.
		 * @param state What to set the grip state as.
		 */
		void SetGripperState(GripperState state)	{ m_State.Gripper	= (state != m_DefaultState.Gripper);	}

		//intake motors																								_
		/**
		 * @brief Sets intake power to 1.0
		 */
		void EnableIntake()		{ SetIntakePower(-1.0);							} //start it

		/**
		 * @brief Sets intake power to -1.0
		 */
		void ReverseIntake()	{ SetIntakePower(1.0);							} //shoot things out

		/**
		 * @brief Sets intake power to 0.0
		 */
		void DisableIntake()	{ SetIntakePower(0.0);							} //stop intake

		//intake pnuematics																							_

		/**
		 * @brief Extends the intake arm
		 */
		void Extend()			{ SetSliderState(SliderState::kExtended);		} //extend the arm

		/**
		 * @brief Retracts the intake arm
		 */
		void Retract()			{ SetSliderState(SliderState::kRetracted);		} //pull back the arm


		/**
		 * @brief Lifts intake arm
		 */
		void Lift()				{ SetArmState(ArmState::kLifted);				} //lift arm

		/**
		 * @brief Lowers intake arm
		 */
		void Lower()			{ SetArmState(ArmState::kLowered);				} //lower ar


		/**
		 * @brief Extends (engages) gripper solenoid
		 */
		void Engage()			{ SetGripperState(GripperState::kEngaged);		} //hold the box in place

		/**
		 * @brief Retracts (disengages) gripper solenoid
		 */
		void Disengage()		{ SetGripperState(GripperState::kDisengaged);	} //stop holding the box

		//TODO: Automation
		void Automate(){
			/*if(pho == x_distance){
			*	Engage();
			*  } else {
			* 	Disengage();
			*  }
			*/
		}

		bool IsCubeDetected(){
					return pho_Intake->Get();
				}

		void Update(){
			IsCubeDetected() ? mtr_L_Intake->Set(0.0) : mtr_L_Intake->Set(m_intakePower);
			IsCubeDetected() ? mtr_R_Intake->Set(0.0) : mtr_R_Intake->Set(m_intakePower);


			sol_Arm->Set(m_State.Arm);
			sol_Slider->Set(m_State.Slider);
			sol_Gripper->Set(m_State.Gripper);
		}

	private:
		VictorSP		*mtr_L_Intake;
		VictorSP		*mtr_R_Intake;

		Solenoid		*sol_Arm;
		Solenoid		*sol_Slider;
		Solenoid		*sol_Gripper;

		DigitalInput	*pho_Intake;

		float			m_intakePower	= 0;

		bool			intake_loaded	= false;

		struct {
			bool	 Arm			= false;
			bool	 Slider			= false;
			bool	 Gripper		= false;
		}			 m_State;

		struct {
			const ArmState		Arm 	= ArmState::kLowered;
			const SliderState 	Slider	= SliderState::kRetracted;
			const GripperState	Gripper	= GripperState::kEngaged;
		} m_DefaultState;
	};
}
