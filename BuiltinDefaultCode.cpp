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
 *     - PWM 5 connected to feed motors
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
	Talon *m_loadMotor; // robot ball loader
	Talon *m_launchMotor; // robot ball launcher
	DigitalInput *m_launcherLimit; // limit switch for launcher
	ADXL345_I2C *m_Accelerometer; // accelerometer
	ADXL345_I2C::AllAxes *m_accelerations;
	Encoder *m_launchEncoder; // AS5145B Magnetic Encoder- Quadrature 4096 counts
	DigitalInput *m_launchEncoderIncFlag; // Flag indicating encoder mag too close 
	DigitalInput *m_launchEncoderDecFlag; // Flag indicating encoder mag too far
	PIDController *m_launchController; // PID Controller for the launcher
	DigitalOutput *m_digitalOutputTest; 
	
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
	float m_launchAngleDegrees;		// launch angle in degrees
	static const float m_degreesPerCount = 360/4096;		// constant that converts count into degrees 
	bool m_loading;					// flag to know ball is being loaded into launcher
	double m_loadSpeed;				// Motor speed to load ball
	
	// Declare variables for launcher PID Controller
	static const float m_p = 5; // Launcher PID proportional gain (in-lbs/deg)
	static const float m_i = 0; // Launcher PID integral gain (in-lbs/(deg-sec))
	static const float m_d = .05; // Launcher PID derivative gain (in-lbs/(deg/sec))
	static const float m_f = 0; // Launcher PID feed forward gain (in-lbs/???)
	float m_launchPIDOutput; // Launcher PID Output (in-lbs)
	static const float m_launchPIDPeriod = 0.02; // Launcher PID refresh rate (in seconds)

	
	
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
		m_loadMotor = new Talon(5);
		m_launchMotor = new Talon(6);
		
		//Defining local robot objects
		m_launcherLimit = new DigitalInput(1,1); //limit switch for launcher
		m_Accelerometer = new ADXL345_I2C(1,ADXL345_I2C::kRange_2G); // module 1
		m_launchEncoder = new Encoder(1, 7, 1, 8, 0, Encoder::k4X);
		m_launchEncoderIncFlag = new DigitalInput(1,9); // Flag indicating encoder mag too close (false - good)
		m_launchEncoderDecFlag = new DigitalInput(1,10); // Flag indicating encoder mag too far (false = good)
		m_launchController = new PIDController(m_p, m_i, m_d, m_f, m_launchEncoder, m_launchMotor, m_launchPIDPeriod);
		m_digitalOutputTest = new DigitalOutput(1,2); 
			
		// Acquire the Driver Station object
		m_ds = DriverStation::GetInstance();
		m_priorPacketNumber = 0;
		m_dsPacketsReceivedInCurrentSecond = 0;

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
		m_launchController->Disable();
		m_launchEncoder->SetPIDSourceParameter(PIDSource::kAngle); //
		
		/*while (m_launcherLimit->Get() == true){
			m_launchMotor->Set(-0.1);		// Retracts launch motor slowly
		}*/
		
		m_launchMotor->Set(0.0);		// Stops launch motor
		m_launchEncoder->Reset();	// Sets encoder to zero
		m_launchEncoder->Start();
		m_launchController->SetSetpoint(0); //
		m_launchController->Enable(); //Enables PID	 
			
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
		
/*		m_feeding = true;
		m_feedCount = 50;						// (loops)
		m_feedSpeed = 0.5;						// (unitless)
		m_feedCounter = 0;
		m_loading = true;
		m_loadCount = 60;						// (loops)
		m_loadPauseCount = 2;					// (loops)
		m_loadSpeed = 0.2;						// (unitless)
		m_loadCounter = 0;
		m_frisbeeCounter = 0;					// (loops) frisbee counter for autonomous
		m_frisbeeCountMax = 3;					// (loops) maximum number of frisbees we're going to feed in autonomous
		m_breachFrisbee = true;
*/		
//		m_robot->SetSafetyEnabled(false);
//		m_robotFrontDrive->SetSafetyEnabled(false);		
//		m_robotRearDrive->SetSafetyEnabled(false);
//		m_launchMotor->SetSafetyEnabled(false);
//		m_feedMotor->SetSafetyEnabled(false);
	}

	void TeleopInit(void) {
		m_telePeriodicLoops = 0;				// Reset the loop counter for teleop mode
		m_dsPacketsReceivedInCurrentSecond = 0;	// Reset the number of dsPackets in current second
//		m_driveMode = UNINITIALIZED_DRIVE;		// Set drive mode to uninitialized
//		ClearSolenoidLEDsKITT();
		m_launchEncoder->Reset();				// Reset encoder counter to 0 
		m_digitalOutputTest->Set(0); 
		
/*		m_feeding = false;
		m_feedCount = 50;						// (loops)
		m_feedSpeed = 0.5;						// (unitless)
		m_feedCounter = 0;
		m_loading = false;
		m_loadCount = 58;						// (loops)
		m_loadPauseCount = 2;					// (loops)
		m_loadSpeed = 0.2;						// (unitless)
		m_loadCounter = 0;
		*/
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
		m_launchAngleDegrees = m_launchAngleCounts * m_degreesPerCount;
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

			
		/* Grab z-wheel value and transform from [1, -1] to [0,4600] 
		 * 4600 rpm is a guess */ 
		float rawZ, transformedZ;
		rawZ = m_leftStick->GetZ();
//		transformedZ = (1.0 - rawZ)/(-2.0);
		transformedZ = -2300.0 * rawZ + 2300.0;
		
		m_digitalOutputTest->Set(0); 

		
		/* Driver station display output. */
		char msg[256];
static	DriverStationLCD *dsLCD = DriverStationLCD::GetInstance();
		sprintf(msg, "Mag Inc = %u ", m_launchEncoderIncFlag->Get());
		dsLCD->Printf(DriverStationLCD::kUser_Line1, 1, msg);
		sprintf(msg, "Mag Dec = %u ", m_launchEncoderDecFlag->Get());
		dsLCD->Printf(DriverStationLCD::kUser_Line2, 1, msg);
		sprintf(msg, "Encoder Bits = %d ", m_launchEncoder->Get()); //Bits = Counts
		dsLCD->Printf(DriverStationLCD::kUser_Line3, 1, msg);
		sprintf(msg, "Launch Ang Deg = %f ", m_launchAngleDegrees);
		dsLCD->Printf(DriverStationLCD::kUser_Line4, 1, msg);
		sprintf(msg, "PID Error = %f (?-?)", m_launchController->GetError());
		dsLCD->Printf(DriverStationLCD::kUser_Line5, 1, msg);
		sprintf(msg, "Launcher Limit = %u (On/Off)", m_launcherLimit->Get());
		dsLCD->Printf(DriverStationLCD::kUser_Line6, 1, msg);
		
		// line number (enum), starting col, format string, args for format string ...
		dsLCD->UpdateLCD();
		
		// use arcade drive
		//m_driveGain = (1.0 - m_rightStick->GetZ())/(-2.0);
		//m_robot->ArcadeDrive(m_rightStick);			// drive with arcade style (use right stick)
		m_robot->MecanumDrive_Polar(m_rightStick->GetMagnitude(), m_rightStick->GetDirectionDegrees(), m_rightStick->GetZ());			
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
