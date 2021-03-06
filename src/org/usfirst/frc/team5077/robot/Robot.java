
package org.usfirst.frc.team5077.robot;

import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.interfaces.Gyro;

// import com.ctre.CANTalon;

import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.Talon;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class Robot extends IterativeRobot {
	RobotDrive myRobot;
	Joystick stick;
	Timer timer;
	Gyro gyro;
	Talon winch;
	// CANTalon leftDoor;
	// Talon rightDoor;
	
	double Kp = 0.004; // Scaling constant. Tells the Gyro how quickly to correct for straightness.
	
	boolean buttonPressed = false;
	long doorOpenTime = (long) 0.0;
	
	
	public Robot(){
		System.out.println("***In constructor****");
		myRobot = new RobotDrive(0, 1, 2, 3);
		stick = new Joystick(0);
		timer = new Timer();
		gyro = new ADXRS450_Gyro();
		winch = new Talon(4);
		// leftDoor = new CANTalon(21);
		// rightDoor = new Talon(5);
		System.out.println("***Leaving constructor");
	}

	/**
	 * This function is run when the robot is first started up and should be
	 * used for any initialization code.
	 */
	@Override
	public void robotInit () {
		myRobot.setInvertedMotor(RobotDrive.MotorType.kFrontLeft, true);
		myRobot.setInvertedMotor(RobotDrive.MotorType.kFrontRight, true);
		myRobot.setInvertedMotor(RobotDrive.MotorType.kRearLeft, true);
		myRobot.setInvertedMotor(RobotDrive.MotorType.kRearRight, true);
		gyro.calibrate();
	}

	/**
	 * This function is run once each time the robot enters autonomous mode
	 */
	@Override
	public void autonomousInit () {
		System.out.println("**Starting to Initialize Autonomous Mode");
		gyro.reset();
		timer.reset();
		timer.start();
		System.out.println("**Done Initializing Autonomous Mode");
	}

	/**
	 * This function is called periodically during autonomous
	 */
	@Override
	public void autonomousPeriodic () {
		System.out.println("timer: " + timer.get());
		//centerGear();
		//goStraight();
		leftGear();
		//rightGear();
	}
	
	void centerGear () {
		double angle = gyro.getAngle();
		double matchTime = timer.get();
		if (matchTime < 0.6969) { 
			myRobot.drive(0.5, -angle * Kp);
		} else if ((matchTime >= 0.6969) && (matchTime < 1.9)) {
			myRobot.drive(0.3333, -angle* Kp);
		} else if ((matchTime >= 1.9) && (matchTime < 4.9)) {
			myRobot.drive(0.0, 0.0);
		} /*else if ((matchTime >=4.9) && (matchTime < 15.0)) {
			if ( angle > -180.0 ) {
				myRobot.drive(0.5, Math.exp(-18.0/24.5));
			} else {
				myRobot.drive(0.5, -(-180 - angle) * Kp);
			} 
		}*/ else {
			myRobot.drive(0.0, 0.0); // stop robot
		}
	}
	
	void leftGear () {
		double angle = gyro.getAngle();
		double matchTime = timer.get();
		if (matchTime < 0.6969) {									//begin drive forward 
			myRobot.drive(0.75, -angle * Kp);
		} else if ((matchTime >= 0.6969) && (matchTime < 1.9) ) {	//slow down at line
			myRobot.drive(0.3333, -angle* Kp);
		} else if ((matchTime > 1.9) && (matchTime < 2.9)){			//stop at line
			myRobot.drive(0.0, 0.0);
		} else if ((matchTime >= 2.9) && (matchTime < 4.9)) {		//rotate
			if (Math.abs(angle) < 60.0) {
				myRobot.drive(0.3, Math.exp(-12.25/24.5));
			} else {
				myRobot.drive(0.0, 0.0);
			}
		} else if ((matchTime >= 4.9) && (matchTime < 6.5)) {
			myRobot.drive(0.5, -(60 - angle) * Kp);			//move forward
			
		}else if ((matchTime >= 6.5) && (matchTime < 7.0)){		//stop at gear station
			myRobot.drive(0.0, 0.0);
		} else if (matchTime >=7.0 && matchTime < 7.3){
			myRobot.drive(0.0, 0.0);
			// rightDoor.set(0.5);
			// leftDoor.set((0.5));
		
		} else if (matchTime >= 7.3 && matchTime < 7.6){
			
			// rightDoor.set(-0.05);
			// leftDoor.set((-0.05)); 
		
		} else if (matchTime >= 7.6 && matchTime < 8.1) {
			myRobot.drive(.5, -(60 - angle) *Kp);
		/*else if ((matchTime >= 7.9) && (matchTime < 11.9)){		//back up from gear station
			if (angle < 0){
				myRobot.drive(0.3, Math.exp(-12.25/24.5));
			}else {
				myRobot.drive(0.3, (-angle) * Kp);
			}
		} else if ( (matchTime >=11.9) && (matchTime < 13.9)) {		//drive to white line	
			myRobot.drive(-0.5, (-angle)* Kp);
		}*/
		} else {
			myRobot.drive(0.0, 0.0); // stop robot
			
		}
	}
	
	void rightGear () {
		double angle = gyro.getAngle();
		double matchTime = timer.get();
		if (matchTime < 0.6969) {									//begin drive forward 
			myRobot.drive(0.75, -angle * Kp);
		} else if ((matchTime >= 0.6969) && (matchTime < 1.9) ) {	//slow down at line
			myRobot.drive(0.3333, -angle* Kp);
		} else if ((matchTime > 1.9) && (matchTime < 2.9)){			//stop at line
			myRobot.drive(0.0, 0.0);
		} else if ((matchTime >= 2.9) && (matchTime < 6.5)) {		//rotate
			if (Math.abs(angle) < 60.0) {
				myRobot.drive(0.5, Math.exp(-12.25/24.5));
			} else {
				myRobot.drive(0.5, (60 - angle) * Kp);			//move forward
			}
		}else if ((matchTime >= 6.5) && (matchTime < 7.0)){		//stop at gear station
			myRobot.drive(0.0, 0.0);
		} else if (matchTime >=7.0 && matchTime < 7.3){
			myRobot.drive(0.0, 0.0);
			// rightDoor.set(0.5);
			// leftDoor.set((0.5));
		
		} else if (matchTime >= 7.3 && matchTime < 7.6){
			
			// rightDoor.set(-0.05);
			// leftDoor.set((-0.05));
		/*else if ((matchTime >= 7.9) && (matchTime < 11.9)){		//back up from gear station

			if (angle < 0){
				myRobot.drive(0.3, Math.exp(-12.25/24.5));
			}else {
				myRobot.drive(0.3, (-angle) * Kp);
			}
		} else if ( (matchTime >=11.9) && (matchTime < 13.9)) {		//drive to white line	
			myRobot.drive(-0.5, (-angle)* Kp);
		}*/ 
		} else if (matchTime >= 7.6 && matchTime < 8.1) {
			myRobot.drive(.5, (60 - angle) *Kp);
		} else {
			myRobot.drive(0.0, 0.0); // stop robot
			
		}
	}
	
	void goStraight(){
		double angle = gyro.getAngle();
		double matchTime = timer.get();
		
		if (matchTime < 3.0){
			myRobot.drive(-0.5, -angle * Kp);
		} else {
			myRobot.drive(0.0, 0.0);
		}
	}


	/**
	 * This function is called once each time the robot enters tele-operated
	 * mode
	 */
	@Override
	public void teleopInit() {}

	/**
	 * This function is called periodically during operator control
	 */
	@Override
	public void teleopPeriodic () {
		boolean upButton = stick.getRawButton(5);
		boolean downButton = stick.getRawButton(6);
		boolean openGearDoorButton = stick.getRawButton(1);
		 
		if (upButton) {
			winch.set(0.75);
		} else {
			if (!downButton) 
				winch.set(0.0);
		}
		
		if (downButton) {
			winch.set(-0.75);
		} else {
			if (!upButton)
				winch.set(0.0);
		}
		
		if (openGearDoorButton) {
				OpenGearDoors ();
			
		} else {
			buttonPressed = false;
			CloseDoors();
		}
		
		myRobot.arcadeDrive(stick, true);  // true for squaredInputs (decrease sensitivity for small inputs)
	}
	
	void OpenGearDoors () {
		if (!buttonPressed) {
			buttonPressed = true;
			doorOpenTime = System.currentTimeMillis();
		}
		
		if ((System.currentTimeMillis() - doorOpenTime) >= 300){
			// rightDoor.set(0.0);
			// leftDoor.set((0.0));
		} else {
			// rightDoor.set(0.5);
			// leftDoor.set((0.5));
		}
	
	}
	
	void CloseDoors () {
		// rightDoor.set(-0.05);
		// leftDoor.set((-0.05));
	}

	/**
	 * This function is called periodically during test mode
	 */
	@Override
	public void testPeriodic () {
		LiveWindow.run();
	}
	
}