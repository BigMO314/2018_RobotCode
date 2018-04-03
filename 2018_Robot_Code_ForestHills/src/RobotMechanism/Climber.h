#pragma once
#include "MOLib.h"
#include "Configuration/Robot.h"

namespace RobotMechanism {
class Climber {
public:
	Climber(VictorSP *mtr_Climb, Solenoid *sol_Climb) {
		this->mtr_Climb = mtr_Climb;
		this->sol_Climb = sol_Climb;
	}
	~Climber(){}

	void Climb() {
		SetClimb(1.0);
	}

	void SetClimb(float power) {
		m_Power = power;
	}

	void Lock() {
		m_Locked = true;
	}

	void Release() {
		m_Locked = false;
	}

	void Update() {
		mtr_Climb->Set(m_Power);
		sol_Climb->Set(m_Locked);
	}


private:
	float m_Power = 0.0;
	bool m_Locked = true;

	VictorSP	*mtr_Climb;
	Solenoid	*sol_Climb;

};
}
