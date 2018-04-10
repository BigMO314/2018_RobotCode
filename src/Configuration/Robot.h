#pragma once
#include "MOLib.h"

namespace Configuration{
	namespace Robot{

	namespace Drivetrain {
				const double WheelDiameter			= 6.0;
				const double WheelCircumference		= (M_PI * WheelDiameter);//18.84955592
				const double CountsPerInch			= (WheelCircumference / 4096);
				const double PeakOutput				= 0.75;
				const double met_WheelDiameter		= 0.1524;
				const double met_WheelCircumference	= (M_PI * met_WheelDiameter);

				const double MaxVelocity			= 3.6;
				const double met_MaxVelocity 		= 0.09144;

				const double MaxAcceleration		= 3.6;
				const double met_MaxAccelration		= 0.09144;
			}

			namespace Arm {
				const double DegreesPerCount		= (360.0 / 4096.0) * (1.0 / 4.0);

			}

			namespace Intake {
				const double MaxIntakePower 	= 0.5;
				const double MinIntakePower		= -0.5;

			}
	}
}
