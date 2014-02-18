#include "WPILib.h"
#include "ADXL345_I2C.h"
#include "DigitalModule.h"
#include "NetworkCommunication/UsageReporting.h"
#include "I2C.h"
#include "Encoder.h"

/**
 * This "BuiltinDefaultCode" provides the "default code" functionality as used in the "Benchtop Test."
 * 
 * The BuiltinDefaultCode extends the IterativeRobot base class to provide the "default code"
 * functionality to confirm the operation and usage of the core control system components, as 
 * used in the "Benchtop Test" described in Chapter 2 of the 2009 FRC Control System Manual.
 * 
 * This program provides features in the Disabled, Autonomous, and Teleop modes as described
 * in the benchtop test directions, including "once-a-second" debugging printouts when disabled, 
 * a "KITT light show" on the solenoid lights when in autonomous, and elementary driving
 * capabilities and "button mapping" of joysticks when teleoperated.  This demonstration
 * program also shows the use of the MotorSafety timer.
 * 
 * This demonstration is not intended to serve as a "starting template" for development of
 * robot code for a team, as there are better templates and examples created specifically
 * for that purpose.  However, teams may find the techniques used in this program to be
 * interesting possibilities for use in their own robot code.
 * 
 * The details of the behavior provided by this demonstration are summarized below:
 *  
 * Disabled Mode:
 * - Once per second, print (on the console) the number of seconds the robot has been disabled.
 * 
 * Autonomous Mode:
 * - Flash the solenoid lights like KITT in Knight Rider
 * - Example code (commented out by default) to drive forward at half-speed for 2 seconds
 * 
 * Teleop Mode:
 * - Select between two different drive options depending upon Z-location of Joystick1
 * - When "Z-Up" (on Joystick1) provide "arcade drive" on Joystick1
 * - When "Z-Down" (on Joystick1) provide "tank drive" on Joystick1 and Joystick2
 * - Use Joystick buttons (on Joystick1 or Joystick2) to display the button number in binary on
 *   the solenoid LEDs (Note that this feature can be used to easily "map out" the buttons on a
 *   Joystick.  Note also that if multiple buttons are pressed simultaneously, a "15" is displayed
 *   on the solenoid LEDs to indicate that multiple buttons are pressed.)
 *
 * This code assumes the following connections:
 * - Driver Station:
 *   - USB 1 - The "right" joystick.  Used for either "arcade drive" or "right" stick for tank drive
 *   - USB 2 - The "left" joystick.  Used as the "left" stick for tank drive
 * 
 * - Robot:
 *   - Digital Sidecar 1:
 *     - PWM 1 connected to front right
 *     - PWM 2 connected to front left
 *     - PWM 3 connected to rear right
 *     - PWM 4 connected to rear left
 *     - PWM 5 connected to load motors
 *     - PWM 6 connected to launch motor
 *   - Digital I/O:
 *     - Channel 1 connected to the launcher retracted switch
 *     - Channel 7 connected to launcher Encoder channel A
 *     - Channel 8 connected to launcher Encoder channel B
 *     - Channel 9 connected to launcher Encoder mag inc
 *     - Channel 10 connected to launcher Encoder mag dec
 */
class BuiltinDefaultCode : public IterativeRobot
{
	// Declare variable for the robot drive system and declare payload actuators and sensors
	RobotDrive *m_robot;				// Mecanum drive will use PWM's 2,4,1,3
	double m_driveGain;				// detune the drive system as needed
	Jaguar *m_loadMotor; // robot ball loader
	Talon *m_launchMotor; // robot ball launcher
	DigitalInput *m_launcherLimit; // limit switch for launcher
	ADXL345_I2C *m_Accelerometer; // accelerometer
	ADXL345_I2C::AllAxes *m_accelerations;
	Encoder *m_launchEncoder; // AS5145B Magnetic Encoder- Quadrature 4096 counts
	DigitalInput *m_launchEncoderIncFlag; // Flag indicating encoder mag too close 
	DigitalInput *m_launchEncoderDecFlag; // Flag indicating encoder mag too far
	PIDController *m_launchController; // PID Controller for the launcher
	LiveWindow *m_liveWindow; //
	
	//Declare driverstation display
	
	// Declare a variable to use to access the driver station object
	DriverStation *m_ds;						// driver station object
	UINT32 m_priorPacketNumber;					// keep track of the most recent packet number from the DS
	UINT8 m_dsPacketsReceivedInCurrentSecond;	// keep track of the ds packets received in the current second
	
	// Declare variables for the two joysticks being used
	Joystick *m_rightStick;			// joystick 1 (Logitech Extreme 3D Pro for driving)
	Joystick *m_leftStick;			// joystick 2 (Logitech Extreme 3D Pro for loading and launching)
	static const int NUM_JOYSTICK_BUTTONS = 16;
	bool m_rightStickButtonState[(NUM_JOYSTICK_BUTTONS+1)];
	bool m_leftStickButtonState[(NUM_JOYSTICK_BUTTONS+1)];	

	// Declare variables for launcher and loader
	int m_launchAngleCounts;		// launcher encoder counts
	double m_launchAngleDegrees;		// launch angle in degrees
	static const double m_degreesPerCount = 0.3515625;		// constant that converts count into degrees 
	bool m_loading;					// flag to know ball is being loaded into launcher
	double m_loadSpeed;				// Motor speed to load ball
	double m_launchSetpoint;		// Current active setpoint
	double home;					// Home angle for feeding
	double m_homeThrottle;			// Launch motor limit under manual control
	static const double low = 75; 	// Lowest launch angle
	static const double medium_low = 80; 	// Medium lowest launch angle
	static const double medium_high = 85; 	// Medium highest launch angle
	static const double high = 90; 	// Close ranged shooting (90)
	static const double launch_angle_slowdown_pos = 7;  // Angle at which launcher slows down
	static const double slow_range = 0.25;  // Speed of launcher in slowdown position 
	static const double neg_slow_range = -0.25;  // Speed of launcher in slowdown position
	static const double max_range = 1.0;  // Maximum speed of the launcher
	static const double neg_max_range = -1.0;  // Maximum speed of the launcher
	
	// Declare variables for launcher PID Controller
	static const float m_p = .1; // Launcher PID proportional gain (in-lbs/deg)
	static const float m_i = 0; // Launcher PID integral gain (in-lbs/(deg-sec))
	static const float m_d = .01; // Launcher PID derivative gain (in-lbs/(deg/sec))
	static const float m_f = 0; // Launcher PID feed forward gain (in-lbs/???)
	static const float m_launchPIDPeriod = 0.02; // Launcher PID refresh rate (in seconds)
	static const double drive_gain = 0.3;	// detune the drive system as needed
	
	// Declare variables for autonomous
	float m_driving;				// Variable for driving in autonomous
	
	// Declare variables for accelerometer
	double m_accelerationX;
	double m_accelerationY;
	double m_accelerationZ;

	// Declare variables for each of the eight solenoid outputs
	static const int NUM_SOLENOIDS = 8;
	Solenoid *m_solenoids[(NUM_SOLENOIDS+1)];
	
	// Local variables to count the number of periodic loops performed
	UINT32 m_autoPeriodicLoops;
	UINT32 m_disabledPeriodicLoops;
	UINT32 m_telePeriodicLoops;
		
public:
/**
 * Constructor for this "BuiltinDefaultCode" Class.
 * 
 * The constructor creates all of the objects used for the different inputs and outputs of
 * the robot.  Essentially, the constructor defines the input/output mapping for the robot,
 * providing named objects for each of the robot interfaces. 
 */
	BuiltinDefaultCode(void)

		{
		printf("BuiltinDefaultCode Constructor Started\n");

		// Create a robot using standard right/left robot drive on PWMS 1, 2, 3, and #4
		m_robot = new RobotDrive(2,4,1,3);
		
		//Defining member robot objects
		m_loadMotor = new Jaguar(5);
		m_launchMotor = new Talon(6);
		m_homeThrottle = 0.1;
		
		m_launcherLimit = new DigitalInput(1,1); //limit switch for launcher
		m_launchEncoder = new Encoder(1, 7, 1, 8, 0, Encoder::k4X);
		m_launchEncoderIncFlag = new DigitalInput(1,9); // Flag indicating encoder mag too close (false - good)
		m_launchEncoderDecFlag = new DigitalInput(1,10); // Flag indicating encoder mag too far (false = good)
		m_launchController = new PIDController(m_p, m_i, m_d, m_f, m_launchEncoder, m_launchMotor, m_launchPIDPeriod);
		
			
		// Acquire the Driver Station object
		m_ds = DriverStation::GetInstance();
		m_priorPacketNumber = 0;
		m_dsPacketsReceivedInCurrentSecond = 0;
		m_liveWindow = LiveWindow::GetInstance();
		m_liveWindow->AddActuator("Launch System", "PID Controller", m_launchController);
//		m_liveWindow->AddActuator("Launch System", "Talon", m_launchMotor);
	//	m_liveWindow->AddSensor("Launch System", "Encoder", m_launchEncoderIncFlag);
//		m_liveWindow->AddSensor("Launch System", "Encoder", m_launchEncoderDecFlag);
		

		// Define joysticks being used at USB port #1 and USB port #2 on the Drivers Station
		m_rightStick = new Joystick(1);		// First joystick plugged in
		m_leftStick = new Joystick(2);		// Second joystick plugged in

		// Iterate over all the buttons on each joystick, setting state to false for each
		UINT8 buttonNum = 1;						// start counting buttons at button 1
		for (buttonNum = 1; buttonNum <= NUM_JOYSTICK_BUTTONS; buttonNum++) {
			m_rightStickButtonState[buttonNum] = false;
			m_leftStickButtonState[buttonNum] = false;
		}

		// Iterate over all the solenoids on the robot, constructing each in turn
		UINT8 solenoidNum = 1;						// start counting solenoids at solenoid 1
		for (solenoidNum = 1; solenoidNum <= NUM_SOLENOIDS; solenoidNum++) {
			m_solenoids[solenoidNum] = new Solenoid(solenoidNum);
		}


		// Initialize counters to record the number of loops completed in autonomous and teleop modes
		m_autoPeriodicLoops = 0;
		m_disabledPeriodicLoops = 0;
		m_telePeriodicLoops = 0;
		
		printf("BuiltinDefaultCode Constructor Completed\n");
	}
	
	
	/********************************** Init Routines *************************************/

	void RobotInit(void) {
		// Actions which would be performed once (and only once) upon initialization of the
		// robot would be put here.
		
		
			
		printf("RobotInit() completed.\n");
	}
	
	void DisabledInit(void) {
		m_disabledPeriodicLoops = 0;			// Reset the loop counter for disabled mode
		// Move the cursor down a few, since we'll move it back up in periodic.
		m_launchEncoder->Stop();
		printf("\x1b[2B");
	}

	void AutonomousInit(void) {
		m_autoPeriodicLoops = 0;				// Reset the loop counter for autonomous mode
		m_launchMotor->Set(0.0);
		m_driving = 0;
	}

	void TeleopInit(void) {
		m_telePeriodicLoops = 0;				// Reset the loop counter for teleop mode
		m_dsPacketsReceivedInCurrentSecond = 0;	// Reset the number of dsPackets in current second
//	Set Drive Gain
		
		//		m_driveMode = UNINITIALIZED_DRIVE;		// Set drive mode to uninitialized
//		ClearSolenoidLEDsKITT();
		m_liveWindow->SetEnabled(1);
		
// Intialize encoder
		m_launchEncoder->SetPIDSourceParameter(PIDSource::kDistance); //
		m_launchEncoder->SetDistancePerPulse(m_degreesPerCount);
		m_launchEncoder->Reset();				// Reset encoder counter to 0 
		m_launchEncoder->Start();

		// Initialize PID controller
		m_launchController->SetInputRange(-10.0, 100.0);
		m_launchController->SetOutputRange(neg_max_range, max_range);
		home = 0;
		m_launchController->SetSetpoint(home); //
		m_launchController->Reset();	// Reset internal PID values
		m_launchController->Disable(); //Enables PID
		
		
	}

	/********************************** Periodic Routines *************************************/
	
	void DisabledPeriodic(void)  {
		static INT32 printSec = (INT32)GetClock() + 1;
		static const INT32 startSec = (INT32)GetClock();


		// increment the number of disabled periodic loops completed
		m_disabledPeriodicLoops++;
		
		// while disabled, printout the duration of current disabled mode in seconds
		if (GetClock() > printSec) {
			// Move the cursor back to the previous line and clear it.
			printf("\x1b[1A\x1b[2K");
			printf("Disabled seconds: %d\r\n", printSec - startSec);			
			printSec++;
		}
	}

	void AutonomousPeriodic(void) {
		
		GetWatchdog().Feed();
		m_autoPeriodicLoops++;

/*		for(m_driving = 0; m_driving <= 10
		0; m_driving++){
			m_robot->Drive(0.5, 0.0);
		}
		m_robot->Drive(0.0, 0.0);
*/		
		/* Driver station display output. */
				char msg[256];
		static	DriverStationLCD *dsLCD = DriverStationLCD::GetInstance();
				sprintf(msg, "Driving Count = %f ", m_driving);
				dsLCD->Printf(DriverStationLCD::kUser_Line1, 1, msg);
	
		dsLCD->UpdateLCD();
	}
		
	void TeleopPeriodic(void) {
		// increment the number of teleop periodic loops completed
		GetWatchdog().Feed();
		m_telePeriodicLoops++;

		// Get time critical measurements
		/* Payload/Launcher code
			Get the launcher position
		*/
		m_launchAngleCounts = m_launchEncoder->Get();
		m_launchAngleDegrees = double(m_launchAngleCounts) * m_degreesPerCount;
		m_launcherLimit->Get();

		
		/*
		 * No longer needed since periodic loops are now synchronized with incoming packets.
		if (m_ds->GetPacketNumber() != m_priorPacketNumber) {
		*/
			/* 
			 * Code placed in here will be called only when a new packet of information
			 * has been received by the Driver Station.  Any code which needs new information
			 * from the DS should go in here
			 */
			 
			m_dsPacketsReceivedInCurrentSecond++;					// increment DS packets received
						
			// put Driver Station-dependent code here

		// Control El Toro loader	
		m_loadMotor->Set(-m_leftStick->GetY());
			
		// Get desired launch angles
		if(!m_launchController->IsEnabled() && m_launchAngleDegrees <= 3.0){
			if(m_leftStick->GetRawButton(5)){
				m_launchSetpoint= low;
			}
			else if(m_leftStick->GetRawButton(3)){
				m_launchSetpoint= medium_low;			
			}
			else if(m_leftStick->GetRawButton(4)){
				m_launchSetpoint= medium_high;
		
			}
			else if(m_leftStick->GetRawButton(6)){
				m_launchSetpoint= high;			
			}
		}
		else if(!m_launchController->IsEnabled()){
			m_launchSetpoint= home;
		}
		
		m_launchController->SetSetpoint(m_launchSetpoint);

		if(m_leftStick->GetTrigger() && !m_leftStick->GetRawButton(11)){	// Trigger is depressed, but 11 is not
			m_launchController->Enable();
		}
		else{
			m_launchController->Disable();
		}
		
		if(m_launchAngleDegrees < launch_angle_slowdown_pos){
			m_launchController->SetOutputRange(neg_slow_range, slow_range);
		}
		else{
			m_launchController->SetOutputRange(neg_max_range, max_range);
		}
		// Set home position
		if (m_leftStick->GetRawButton(11)){
		//if (m_launcherLimit->Get() == 1 && m_leftStick->GetRawButton(11)){
			if(m_leftStick->GetRawButton(7)){
				m_launchMotor->Set(-m_homeThrottle);		// Adjusts launch motor slowly
			}
			else if(m_leftStick->GetRawButton(8)){
				m_launchMotor->Set(m_homeThrottle);		// Adjusts launch motor slowly
			}
			else{
				m_launchMotor->Set(0.0);		// Stops launch motor
			}
		}
		else if (!m_launcherLimit->Get() == 1 && m_leftStick->GetRawButton(11)) { // After we found the home switch
			m_launchMotor->Set(0.0);		// Stops launch motor
			m_launchEncoder->Reset();		// Resets the encoder counts to 0
			home = m_launchEncoder->GetDistance();	// Sets home to last reset value	
		}	
		
		
/* Driver station display output. */
		char msg[256];
static	DriverStationLCD *dsLCD = DriverStationLCD::GetInstance();
		sprintf(msg, "Mag Inc = %u ", m_launchEncoderIncFlag->Get());
		dsLCD->Printf(DriverStationLCD::kUser_Line1, 1, msg);
		sprintf(msg, "Mag Dec = %u ", m_launchEncoderDecFlag->Get());
		dsLCD->Printf(DriverStationLCD::kUser_Line2, 1, msg);
/*		sprintf(msg, "Deg/Count = %f ", m_degreesPerCount);
		dsLCD->Printf(DriverStationLCD::kUser_Line2, 1, msg);
		sprintf(msg, "Launch Setpoint = %f ", m_launchSetpoint);
		dsLCD->Printf(DriverStationLCD::kUser_Line1, 1, msg); */
		sprintf(msg, "Launcher Angle = %f ", m_launchAngleDegrees);				// Launcher angle
		dsLCD->Printf(DriverStationLCD::kUser_Line3, 1, msg);
/*		sprintf(msg, "PID In = %f ", m_launchEncoder->PIDGet());				// PID
		dsLCD->Printf(DriverStationLCD::kUser_Line2, 1, msg);*/
	/*	sprintf(msg, "Encoder Rate = %f ", m_launchEncoder->GetRate());			// 
		dsLCD->Printf(DriverStationLCD::kUser_Line3, 1, msg);*/							
		sprintf(msg, "PID Error = %f (?-?)", m_launchController->GetError());
		dsLCD->Printf(DriverStationLCD::kUser_Line4, 1, msg); 
		sprintf(msg, "Launch Motor = %f ", m_launchMotor->Get());
		dsLCD->Printf(DriverStationLCD::kUser_Line5, 1, msg);
		sprintf(msg, "PID Enabled = %u ", m_launchController->IsEnabled());
		dsLCD->Printf(DriverStationLCD::kUser_Line6, 1, msg); 
/*		sprintf(msg, "Limit Switch = %d ", m_launcherLimit->Get());
		dsLCD->Printf(DriverStationLCD::kUser_Line6, 1, msg);*/

		SmartDashboard::PutData("PIDSubsystem", m_launchController);
		//SmartDashboard::PutNumber("PIDError", m_launchController->GetError());
		
/*		SmartDashboard::PutData("Talon", m_launchMotor);
		SmartDashboard::PutBoolean("Encoder", m_launchEncoderIncFlag);
		SmartDashboard::PutBoolean("Encoder", m_launchEncoderDecFlag);*/

		// line number (enum), starting col, format string, args for format string ...
		dsLCD->UpdateLCD();
		
		// use arcade drive
		//m_driveGain = (1.0 - m_rightStick->GetZ())/(-2.0);
		//m_robot->ArcadeDrive(m_rightStick);			// drive with arcade style (use right stick)
		m_robot->MecanumDrive_Polar(m_rightStick->GetMagnitude()*drive_gain, m_rightStick->GetDirectionDegrees(), m_rightStick->GetZ()*drive_gain);			
			// drive with mecanum style (use right stick for steering and left stick for rotation)
				
	} 


/********************************** Continuous Routines *************************************/

	/* 
	 * These routines are not used in this demonstration robot
	 *
	 * 
	void DisabledContinuous(void) {
	}

	void AutonomousContinuous(void)	{
	}

	void TeleopContinuous(void) {
	}
	*/

	
/********************************** Miscellaneous Routines *************************************/
	
				
};

START_ROBOT_CLASS(BuiltinDefaultCode);
