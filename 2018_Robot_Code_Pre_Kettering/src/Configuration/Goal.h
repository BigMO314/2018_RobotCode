#pragma once
#include "WPILib.h"

using namespace std;

namespace Configuration{
	namespace Goal{

		enum class GoalOrientation{
			kLeft,
			kRight
		};

		string GetGameData(){
			return frc::DriverStation::GetInstance().GetGameSpecificMessage();
		}

		GoalOrientation GetSwitch(){
			return (GetGameData()[0]=='L' ? GoalOrientation::kLeft : GoalOrientation::kRight);
		}
		GoalOrientation GetScale(){
			return (GetGameData()[1]=='L' ? GoalOrientation::kLeft : GoalOrientation::kRight);
		}
	}
}
