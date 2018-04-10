#pragma once
#include "MOLib.h"
#include "Configuration/Robot.h"
#include "Dashboard.h"

namespace RobotMechanism {
class Climber {
public:
	Climber(VictorSP *mtr_Climb, Solenoid *sol_Climb, Solenoid *sol_Lock) {
		this->mtr_Climb = mtr_Climb;
		this->sol_Climb = sol_Climb;
		this->sol_Lock = sol_Lock;
		tmr_Climber.Reset();
		tmr_Climber.Stop();
	}
	~Climber(){}

	void Climb() {
		SetClimb(1.0);
	}

	void SetClimb(float power) {
		m_Power = power;
	}

	void Reset() {
		m_Locked = false;
		m_Released = false;
	}

	void Release() {
		if(m_Locked == false) {
			tmr_Climber.Reset();
			tmr_Climber.Start();
			m_Locked = true;
		}
	}

	void Update() {
		if(tmr_Climber.Get() > 0.5) {
			m_Released = true;
			tmr_Climber.Reset();
			tmr_Climber.Stop();
		}
		mtr_Climb->Set(m_Power);
		sol_Climb->Set(m_Released);
		sol_Lock->Set(m_Locked);
		Dashboard.Climber.LockState.Set(m_Locked);
	}


private:
	float m_Power = 0.0;
	bool m_Locked = false;
	bool m_Released = false;
	WPILib::Timer tmr_Climber;

	VictorSP	*mtr_Climb;
	Solenoid	*sol_Climb;
	Solenoid	*sol_Lock;

};
}
