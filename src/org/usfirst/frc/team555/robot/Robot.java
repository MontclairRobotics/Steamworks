package org.usfirst.frc.team555.robot;

import org.montclairrobotics.sprocket.SprocketRobot;
import org.montclairrobotics.sprocket.drive.DriveTrain;
import org.montclairrobotics.sprocket.drive.DriveTrainBuilder;
import org.montclairrobotics.sprocket.drive.InvalidDriveTrainException;
import org.montclairrobotics.sprocket.geometry.Degrees;
import org.montclairrobotics.sprocket.geometry.Position;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Talon;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class Robot extends SprocketRobot {
	
	Joystick stick;
	DriveTrain driveTrain;
	
	@Override
	public void autonomousInit() {
		// TODO Auto-generated method stub
		super.autonomousInit();
	}

	@Override
	public void autonomousPeriodic() {
		// TODO Auto-generated method stub
		super.autonomousPeriodic();
	}

	@Override
	public void robotInit() {
		super.robotInit();
		
		stick = new Joystick(0);
		
		DriveTrainBuilder builder = new DriveTrainBuilder();
		builder	.addWheel(new Talon(0), new Degrees(0), Position.FL)
				.addWheel(new Talon(1), new Degrees(0), Position.FR)
				.addWheel(new Talon(2), new Degrees(0), Position.BL)
				.addWheel(new Talon(3), new Degrees(0), Position.BR);
		builder.setArcadeDriveInput(stick);
		try {
			driveTrain = builder.build();
		} catch (InvalidDriveTrainException e) {
			e.printStackTrace();
		}
	}

	@Override
	public void teleopInit() {
		// TODO Auto-generated method stub
		super.teleopInit();
	}

	@Override
	public void teleopPeriodic() {
		// TODO Auto-generated method stub
		super.teleopPeriodic();
	}
	
	
	
}

