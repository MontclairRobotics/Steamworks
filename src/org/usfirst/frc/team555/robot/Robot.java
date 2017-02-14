package org.usfirst.frc.team555.robot;

import java.util.ArrayList;

import org.montclairrobotics.sprocket.SprocketRobot;
import org.montclairrobotics.sprocket.control.ArcadeDriveInput;
import org.montclairrobotics.sprocket.control.Button;
import org.montclairrobotics.sprocket.control.ButtonAction;
import org.montclairrobotics.sprocket.control.JoystickYAxis;
import org.montclairrobotics.sprocket.drive.ControlledMotor;
import org.montclairrobotics.sprocket.drive.DTPipeline;
import org.montclairrobotics.sprocket.drive.DriveTrain;
import org.montclairrobotics.sprocket.drive.DriveTrainBuilder;
import org.montclairrobotics.sprocket.drive.DriveTrainType;
import org.montclairrobotics.sprocket.drive.InvalidDriveTrainException;
import org.montclairrobotics.sprocket.drive.steps.GyroLock;
import org.montclairrobotics.sprocket.geometry.Angle;
import org.montclairrobotics.sprocket.geometry.Degrees;
import org.montclairrobotics.sprocket.geometry.XY;
import org.montclairrobotics.sprocket.loop.Priority;
import org.montclairrobotics.sprocket.loop.Updatable;
import org.montclairrobotics.sprocket.loop.Updater;
import org.montclairrobotics.sprocket.motors.Motor;
import org.montclairrobotics.sprocket.utils.Input;
import org.montclairrobotics.sprocket.utils.PID;
import org.montclairrobotics.sprocket.vision.TurnDistanceInput;
import org.montclairrobotics.sprocket.vision.VisionDTInput;
import org.montclairrobotics.sprocket.vision.VisionTarget;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Rect;
import org.opencv.imgproc.Imgproc;
import org.usfirst.frc.team555.robot.buttons.GearCloseAction;
import org.usfirst.frc.team555.robot.buttons.GearOpenAction;

import com.ctre.CANTalon;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Joystick;
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
	
	private Joystick driveStick;
	private Joystick auxStick;
	
	private DriveTrainBuilder builder;
	private DriveTrain driveTrain;
	
	private Motor gearMotor;
	private DigitalInput openSwitch;
	private DigitalInput closeSwitch;
	
	@Override
	public void robotInit() {
		driveStick = new Joystick(0);
		
		auxStick = new Joystick(1);
		
		openSwitch = new DigitalInput(1);
		closeSwitch = new DigitalInput(0);
		
		Button gearButton = new Button(driveStick, 1);
		gearMotor = new Motor(new CANTalon(5));
		gearButton.setHeldAction(new GearOpenAction(gearMotor, openSwitch));
		gearButton.setOffAction(new GearCloseAction(gearMotor, closeSwitch));
		
		builder = new DriveTrainBuilder();
		builder.addWheels(new XY(-1, 0), Angle.ZERO, new Motor(new CANTalon(3)), new Motor(new CANTalon(4)));
		builder.addWheels(new XY(1, 0), new Degrees(180), new Motor(new CANTalon(1)), new Motor(new CANTalon(2)));
		
		ArcadeDriveInput input = new ArcadeDriveInput(driveStick);
		
		input.setSensitivity(0.5, 0.3);
		builder.setInput(input);
		
		builder.setDriveTrainType(DriveTrainType.TANK);
		
		try {
			builder.build();
		} catch (InvalidDriveTrainException e) {
			e.printStackTrace();
		}
		
		GripPipelineD pipeline=new GripPipelineD();
		
		VisionTarget visionTarget=new VisionTarget(pipeline,new Input<TurnDistanceInput>(){

			@Override
			public TurnDistanceInput get() {
				ArrayList<MatOfPoint> contours = pipeline.filterContoursOutput();
				SmartDashboard.putNumber("contours", contours.size());
				Rect r=Imgproc.boundingRect(contours.get(0));
				SmartDashboard.putNumber("Rect X",r.x);
				SmartDashboard.putNumber("Rect Y",r.y);
				SmartDashboard.putNumber("Rect Width",r.width);
				SmartDashboard.putNumber("Rect Height",r.height);
				
				return new TurnDistanceInput(160-r.x-r.width/2,120-r.y-r.height/2);
			}}, 320,240);
		VisionDTInput visionInput=new VisionDTInput(visionTarget,0.01,0.2,0.01,0.2);
		
		Button imageLockButton=new Button(0, 5);
		imageLockButton.setPressAction(new ButtonAction(){

			@Override
			public void onAction() {
				SprocketRobot.getDriveTrain().setTempInput(visionInput);
			}});
		imageLockButton.setOffAction(new ButtonAction(){

			@Override
			public void onAction() {
				SprocketRobot.getDriveTrain().useDefaultInput();
			}
			
		});
	}
	
}

