package org.usfirst.frc.team555.robot;

import java.util.ArrayList;

import org.montclairrobotics.sprocket.SprocketRobot;
import org.montclairrobotics.sprocket.control.ArcadeDriveInput;
import org.montclairrobotics.sprocket.control.Button;
import org.montclairrobotics.sprocket.control.ButtonAction;
import org.montclairrobotics.sprocket.control.JoystickYAxis;
import org.montclairrobotics.sprocket.drive.ControlledMotor;
import org.montclairrobotics.sprocket.drive.DTPipeline;
import org.montclairrobotics.sprocket.drive.DriveModule;
import org.montclairrobotics.sprocket.drive.DriveTrain;
import org.montclairrobotics.sprocket.drive.DriveTrainBuilder;
import org.montclairrobotics.sprocket.drive.DriveTrainType;
import org.montclairrobotics.sprocket.drive.InvalidDriveTrainException;
import org.montclairrobotics.sprocket.drive.steps.GyroLock;
import org.montclairrobotics.sprocket.geometry.Angle;
import org.montclairrobotics.sprocket.geometry.Degrees;
import org.montclairrobotics.sprocket.geometry.Distance;
import org.montclairrobotics.sprocket.geometry.XY;
import org.montclairrobotics.sprocket.loop.Priority;
import org.montclairrobotics.sprocket.loop.Updatable;
import org.montclairrobotics.sprocket.loop.Updater;
import org.montclairrobotics.sprocket.motors.Motor;
import org.montclairrobotics.sprocket.utils.Input;
import org.montclairrobotics.sprocket.utils.PID;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Rect;
import org.opencv.imgproc.Imgproc;
import org.usfirst.frc.team555.robot.buttons.GearCloseAction;
import org.usfirst.frc.team555.robot.buttons.GearOpenAction;

import com.ctre.CANTalon;

import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.SPI.Port;
import edu.wpi.first.wpilibj.VictorSP;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.vision.VisionPipeline;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class Robot extends SprocketRobot {

	private static final int IMG_WIDTH = 640,IMG_HEIGHT = 480;
	private static final int 
		DriveStickID=0,
		AuxStickID=1,
		CloseSwitchID=0,
		OpenSwitchID=1,
		GearButtonID=1,
		FullSpeedButtonID=3,
		GyroLockButtonID=5,
		VisionButtonID=11;
	
	private static final Distance maxSpeed=new Distance(1);
	
	private Joystick driveStick;
	private Joystick auxStick;
	
	private DriveTrainBuilder builder;
	//private DriveTrain driveTrain;
	
	private Motor gearMotor;
	private DigitalInput openSwitch;
	private DigitalInput closeSwitch;
	
	private ControlledMotor ropeMotor1;
	private ControlledMotor ropeMotor2;
	
	@Override
	public void robotInit() {
		//Joysticks
		driveStick = new Joystick(DriveStickID);
		auxStick = new Joystick(AuxStickID);
		
		//Gear opened/closed limit switches
		openSwitch = new DigitalInput(OpenSwitchID);
		closeSwitch = new DigitalInput(CloseSwitchID);
		
		//Setting up gear trigger
		Button gearButton = new Button(driveStick, GearButtonID);
		gearMotor = new Motor(new CANTalon(5));
		gearButton.setHeldAction(new GearOpenAction(gearMotor, openSwitch));
		gearButton.setOffAction(new GearCloseAction(gearMotor, closeSwitch));
		
		//Rope climber motors
		ropeMotor1 = new ControlledMotor(new CANTalon(6), new JoystickYAxis(auxStick));
		ropeMotor1.getMotor().setInverted(true);
		ropeMotor2 = new ControlledMotor(new CANTalon(7), new JoystickYAxis(auxStick));
		
		//DriveTrain wheels
		builder = new DriveTrainBuilder();
		builder.setDriveTrainType(DriveTrainType.TANK);
		builder.addModule(new DriveModule(new XY(-13.75, 0), Angle.ZERO,maxSpeed, new Motor(new CANTalon(3)), new Motor(new CANTalon(4))));
		builder.addModule(new DriveModule(new XY(13.75, 0), new Degrees(180),maxSpeed, new Motor(new CANTalon(1)), new Motor(new CANTalon(2))));
		
		//DriveTrain joystick input
		ArcadeDriveInput input = new ArcadeDriveInput(driveStick);
		input.setSensitivity(0.5, 0.3);
		builder.setInput(input);
		
		//Full speed button
		Button fullSpeed = new Button(driveStick, FullSpeedButtonID);
		
		fullSpeed.setPressAction(new ButtonAction() {
			@Override
			public void onAction() {
				input.setSensitivity(1.0, 0.5);
			}
		});
		fullSpeed.setReleaseAction(new ButtonAction() {
			@Override
			public void onAction() {
				input.setSensitivity(0.5, 0.3);
			}
		});
		
		
		//Gyro lock
		NavXRollInput navX = new NavXRollInput(Port.kMXP);
		PID gyroPID = new PID(1,0,.000);
		gyroPID.setInput(navX);
		GyroLock gLock = new GyroLock(gyroPID);
		//Gyro lock button
		Button gLockButton = new Button(driveStick, GyroLockButtonID);
		gLockButton.setPressAction(new ButtonAction() {
			@Override
			public void onAction() {
				gLock.enable();
			}
		});
		gLockButton.setReleaseAction(new ButtonAction() {
			@Override
			public void onAction() {
				gLock.disable();
			}
		});
		
		//builder.addStep(gLock);
		
		Button visionButton=new Button(driveStick,VisionButtonID);

		UsbCamera camera = CameraServer.getInstance().startAutomaticCapture();
	    camera.setResolution(IMG_WIDTH, IMG_HEIGHT);
		Vision vision=new Vision(camera);
		VisionStep visionStep=new VisionStep(IMG_WIDTH/2, vision, -0.0001, 0.15, visionButton);
		
		builder.addStep(visionStep);
		
		try {
			builder.build();
		} catch (InvalidDriveTrainException e) {
			e.printStackTrace();
		}
	}
	
	public void update()
	{
		SmartDashboard.putNumber("MaxTurn",SprocketRobot.getDriveTrain().getMaxTurn().toDegrees());
		
	}
}

