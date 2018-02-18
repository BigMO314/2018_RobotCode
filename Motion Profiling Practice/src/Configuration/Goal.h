#pragma once
#include "WPILib.h"

using namespace std;

namespace Configuration{
	namespace Goal{

		string GetGameData(){
			return frc::DriverStation::GetInstance().GetGameSpecificMessage();
		}

		char GetSwitch(){
			return GetGameData()[0];
		}
		char GetScale(){
			return GetGameData()[1];
		}
	}
}
