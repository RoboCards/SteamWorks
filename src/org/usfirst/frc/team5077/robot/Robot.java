package org.usfirst.frc.team5077.robot;

import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.interfaces.Gyro;
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
	RobotDrive myRobot = new RobotDrive(0, 1, 2, 3);
	Joystick stick = new Joystick(0);
	Timer timer = new Timer();
	Gyro gyro = new ADXRS450_Gyro();
	Talon winch = new Talon(4);
	Talon ballMotor = new Talon(5);
	
	double Kp = 0.004; // Scaling constant. Tells the Gyro how quickly to correct for straightness.

	/**
	 * This function is run when the robot is first started up and should be
	 * used for any initialization code.
	 */
	@Override
	public void robotInit() {
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
	public void autonomousInit() {
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
	public void autonomousPeriodic() {
		double angle = gyro.getAngle();
		double matchTime = timer.get();
		if (matchTime < 0.6969) { 
			myRobot.drive(-0.5, -angle * Kp);
		} else if ( (matchTime >= 0.6969) && (matchTime < 1.9) ) {
			myRobot.drive(-0.3333, -angle* Kp);
		} else if ( (matchTime >= 1.9) && (matchTime < 4.9)) {
			myRobot.drive(0.0, 0.0);
		} else if ( (matchTime >=4.9) && (matchTime < 15.0)) {
			if ( angle > -180.0 ) {
				myRobot.drive(0.5, Math.exp(-18.0/24.5));
			} else {
				myRobot.drive(0.5, -(-180 - angle) * Kp);
			} 
		} else {
			myRobot.drive(0.0, 0.0); // stop robot
		}
				
	}
	

	/**
	 * This function is called once each time the robot enters tele-operated
	 * mode
	 */
	@Override
	public void teleopInit() {
	}

	/**
	 * This function is called periodically during operator control
	 */
	@Override
	public void teleopPeriodic() {
		
		boolean upButton = stick.getRawButton(5);
		boolean downButton = stick.getRawButton(6);
		
		if (upButton) {
			// winch.set(1.0);
			ballMotor.set(1.0);
		} else {
			if (!downButton) 
				// winch.set(0.0);
				ballMotor.set(0.0);
		}
		
		
		if (downButton) {
			// winch.set(-1.0);
			ballMotor.set(-1.0);
		} else {
			if (!upButton)
				// winch.set(0.0);
				ballMotor.set(0.0);
		}
		myRobot.arcadeDrive(stick, true);  // true for squaredInputs (decrease sensitivity for small inputs)

	}

	/**
	 * This function is called periodically during test mode
	 */
	@Override
	public void testPeriodic() {
		LiveWindow.run();
	}
	
}