#pragma once

namespace Configuration{
	namespace Robot{

	namespace Drivetrain {
				const double WheelDiameter			= 6.0;
				const double WheelCircumfrance		= (2 * M_PI * (WheelDiameter / 2));
				const double PeakOutput				= 0.75;
				const double met_WheelDiameter		= 0.1524;
				const double met_WheelCircumfrance	= (M_PI * met_WheelDiameter);

				const double MaxVelocity			= 3.6;
				const double met_MaxVelocity 		= 0.09144;

				const double MaxAcceleration		= 3.6;
				const double met_MaxAccelration		= 0.09144;
			}

			namespace Elevator {
				const double PosElevatorPower 	= 1.00;
				const double NegElevatorPower	= -0.85;

			}

			namespace Intake {
				const double MaxIntakePower 	= 0.5;
				const double MinIntakePower		= -0.5;

			}
	}
}
