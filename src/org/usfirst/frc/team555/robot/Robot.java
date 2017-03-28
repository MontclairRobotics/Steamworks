package org.usfirst.frc.team555.robot;

import org.montclairrobotics.sprocket.SprocketRobot;
import org.montclairrobotics.sprocket.auto.AutoMode;
import org.montclairrobotics.sprocket.auto.states.*;
import org.montclairrobotics.sprocket.control.*;
import org.montclairrobotics.sprocket.drive.DriveModule;
import org.montclairrobotics.sprocket.drive.DriveTrainBuilder;
import org.montclairrobotics.sprocket.drive.DriveTrainType;
import org.montclairrobotics.sprocket.drive.InvalidDriveTrainException;
import org.montclairrobotics.sprocket.drive.steps.AccelLimit;
import org.montclairrobotics.sprocket.drive.steps.Deadzone;
import org.montclairrobotics.sprocket.drive.steps.GyroCorrection;
import org.montclairrobotics.sprocket.drive.steps.SpeedLimiter;
import org.montclairrobotics.sprocket.drive.steps.TurnLimiter;
import org.montclairrobotics.sprocket.drive.utils.GyroLock;
import org.montclairrobotics.sprocket.geometry.Angle;
import org.montclairrobotics.sprocket.geometry.Degrees;
import org.montclairrobotics.sprocket.geometry.Distance;
import org.montclairrobotics.sprocket.geometry.XY;
import org.montclairrobotics.sprocket.motors.Motor;
import org.montclairrobotics.sprocket.motors.SEncoder;
import org.montclairrobotics.sprocket.states.State;
import org.montclairrobotics.sprocket.states.StateMachine;
import org.montclairrobotics.sprocket.motors.Module.MotorInputType;
import org.montclairrobotics.sprocket.utils.Debug;
import org.montclairrobotics.sprocket.utils.PID;
import com.ctre.CANTalon;

import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.SPI.Port;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class Robot extends SprocketRobot {

	public static enum GEAR_MODE {MANUAL,AUTO};
	public static GEAR_MODE gearMode=GEAR_MODE.AUTO;
	
	private static final int IMG_WIDTH = 320,IMG_HEIGHT = 240;
	private static final int 
		DriveStickID=0,
		AuxStickID=1,
		CloseSwitchID=0,//limit switches
		OpenSwitchID=1,
		CloseSwitch2ID=6,
		OpenSwitch2ID=7,
		GearButtonID=1,
		FieldCentricButtonID=2,
		GyroLockButtonID=7,
		ResetGyroID=3,
		//VisionButtonID=4,
		LeftButtonID=4,
		RightButtonID=5,
		ManualOpen1=9,
		ManualClose1=11,
		ManualOpen2=12,
		ManualClose2=10;
	

	private static final Distance ENC_SPEED = new Distance(1);
	private static final double MAX_ENC_ACCEL = 13;
	private static final double MAX_ENC_TICKS = 25;
	
	private Joystick driveStick;
	private Joystick auxStick;
	
	private DriveTrainBuilder builder;
	//private DriveTrain driveTrain;
	
	private Motor gear1Motor, gear2Motor;
	private DigitalInput open1Switch, close1Switch, open2Switch, close2Switch;
	
	//private ControlledMotor ropeMotor1;
	//private ControlledMotor ropeMotor2;
	
	private SEncoder encRight;
	private SEncoder encLeft;

	private NavXRollInput navX;
	
	private Gear gear;
	private DashboardInput gearSpeedInput;

	
	@Override
	public void robotInit() {
		//Joysticks
		driveStick = new Joystick(DriveStickID);
		auxStick = new Joystick(AuxStickID);
		//Gear opened/closed limit switches
		open1Switch = new DigitalInput(OpenSwitchID);
		close1Switch = new DigitalInput(CloseSwitchID);
		open2Switch = new DigitalInput(OpenSwitch2ID);
		close2Switch = new DigitalInput(CloseSwitch2ID);
		
		//Setting up gear trigger
		gear1Motor = new Motor(new CANTalon(8));
		gear2Motor = new Motor(new CANTalon(9));
		gear = new Gear(gear1Motor,open1Switch,close1Switch, gear2Motor, open2Switch, close2Switch);
		gearSpeedInput = new DashboardInput("gear speed", 0.1);
		

		Button gearButton = new JoystickButton(driveStick, GearButtonID);
		gearButton.setHeldAction(new ButtonAction(){
			@Override
			public void onAction() {
				if(gearMode == GEAR_MODE.AUTO)
				{
					gear.openLimit();
				}
			}});
		gearButton.setOffAction(new ButtonAction(){
			@Override
			public void onAction() {
				if(gearMode == GEAR_MODE.AUTO)
				{
					gear.closeLimit();
				}
			}});
		
		Button manual1Open=new JoystickButton(auxStick,ManualOpen1);
		Button manual1Close=new JoystickButton(auxStick,ManualClose1);
		
		Button manual2Open = new JoystickButton(auxStick, ManualOpen2);
		Button manual2Close = new JoystickButton(auxStick, ManualClose2);
		
		manual1Open.setPressAction(new ButtonAction(){
			@Override
			public void onAction() {
				if(gearMode == GEAR_MODE.MANUAL)
				{
					gear.open1();
				}
			}});
		manual1Open.setReleaseAction(new ButtonAction(){

			@Override
			public void onAction() {
				if(gearMode == GEAR_MODE.MANUAL)
				{
					gear.stop1();
				}
			}});
		
		
		manual1Close.setPressAction(new ButtonAction(){
			@Override
			public void onAction() {
				if(gearMode == GEAR_MODE.MANUAL)
				{
					gear.close1();
				}
			}});
		manual1Close.setReleaseAction(new ButtonAction(){

			@Override
			public void onAction() {
				if(gearMode == GEAR_MODE.MANUAL)
				{
					gear.stop1();
				}
			}});
		
		manual2Open.setPressAction(new ButtonAction(){
			@Override
			public void onAction() {
				if(gearMode == GEAR_MODE.MANUAL)
				{
					gear.open2();
				}
			}});
		manual2Open.setReleaseAction(new ButtonAction(){

			@Override
			public void onAction() {
				if(gearMode == GEAR_MODE.MANUAL)
				{
					gear.stop2();
				}
			}});
		
		
		manual2Close.setPressAction(new ButtonAction(){
			@Override
			public void onAction() {
				if(gearMode == GEAR_MODE.MANUAL)
				{
					gear.close2();
				}
			}});
		manual2Close.setReleaseAction(new ButtonAction(){

			@Override
			public void onAction() {
				if(gearMode == GEAR_MODE.MANUAL)
				{
					gear.stop2();
				}
			}});
		
		Button manualGearToggle = new JoystickButton(auxStick, 8);
		manualGearToggle.setPressAction(new ButtonAction() {
			@Override
			public void onAction() {
				if(gearMode==GEAR_MODE.AUTO)
				{
					gearMode=GEAR_MODE.MANUAL;
				}
				else
				{
					gearMode=GEAR_MODE.AUTO;
				}
			}
		});
		
		
		
		/*
		//Rope climber motors
		ropeMotor1 = new ControlledMotor(new CANTalon(6), new JoystickYAxis(auxStick));
		ropeMotor1.constrain(0.0, 1.0);
		ropeMotor1.getMotor().setInverted(true);
		ropeMotor2 = new ControlledMotor(new CANTalon(7), new JoystickYAxis(auxStick));
		ropeMotor2.constrain(0.0, 1.0);
		ropeMotor2.getMotor().setInverted(true);*/
		
		
		

		//input.setSensitivity(0.5, 0.3);
		
		Deadzone deadzone=new Deadzone();
		
		AccelLimit accelLimit=new AccelLimit(1.2*4,0.4*4*Math.PI/180*4);
		
		/*
		//Full speed button
		Button fullSpeed = new JoystickButton(driveStick, FullSpeedButtonID);
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
		});*/
		
		//Gyro lock
		navX = new NavXRollInput(Port.kMXP);
		PID gyroPID = new PID(0.18*13.75,0,.0003*13.75);
		gyroPID.setInput(navX);
		GyroCorrection gCorrect=new GyroCorrection(navX,gyroPID);
		GyroLock gLock = new GyroLock(gCorrect);
		
		new JoystickButton(driveStick, LeftButtonID).setPressAction(new ButtonAction() {
			@Override
			public void onAction() {
				gLock.setTargetAngle(gLock.getTargetAngle().add(new Degrees(-5)));
			}
		});
		new JoystickButton(driveStick, RightButtonID).setPressAction(new ButtonAction() {
			@Override
			public void onAction() {
				gLock.setTargetAngle(gLock.getTargetAngle().add(new Degrees(5)));
			}
		});
		
		//DriveTrain joystick input
		SquaredDriveInput input = new SquaredDriveInput(driveStick);//new ArcadeDriveInput(driveStick);
		input.setXSensitivity(0.6);
		
		
		//Gyro lock button
		new ToggleButton(driveStick, GyroLockButtonID, gLock);

		
		//FIELD CENTRIC DRIVE!!!!
		FieldCentricDriveInput fieldCentric=new FieldCentricDriveInput(driveStick,gCorrect);
		new ToggleButton(driveStick,FieldCentricButtonID,fieldCentric);
		//END FIELD CENTRIC. :(
		
		
		Button resetGyroButton=new JoystickButton(driveStick,ResetGyroID);
		resetGyroButton.setPressAction(new ButtonAction(){

			@Override
			public void onAction() {
				gCorrect.reset();
			}});
		
		
		//Vision
		//UsbCamera camera = CameraServer.getInstance().startAutomaticCapture();
	    //camera.setResolution(IMG_WIDTH, IMG_HEIGHT);
		/*Vision vision=new Vision(camera);
		//VisionStep visionStep=new VisionStep(IMG_WIDTH/2, vision, -0.0001, 0, -0.00001, 10);
		
		//new ToggleButton(driveStick, VisionButtonID, visionStep);*/
		
		CameraServer server=CameraServer.getInstance();
		server.startAutomaticCapture();
		//server.startAutomaticCapture();
		
		/*CameraServers server=new CameraServers("cam0","cam1");
		server.start();
		
		
		Button frontCam=new JoystickButton(auxStick,11);
		Button rearCam=new JoystickButton(auxStick,10);
		
		frontCam.setPressAction(new ButtonAction(){

			@Override
			public void onAction() {
				server.switchTo(0);
			}});
		
		rearCam.setPressAction(new ButtonAction(){

			@Override
			public void onAction() {
				server.switchTo(1);
			}});
		*/
		//DriveTrain wheels
		builder = new DriveTrainBuilder();
		builder.setDriveTrainType(DriveTrainType.TANK);
		
		PID motorPID = new PID(0.5, 0.05, 0);
		//PID motorPID = new PID(0, 0, 0);
		encRight = new SEncoder(2, 3, /*5865/76.25/*952.0/(6.0*Math.PI)*/18208/239.4, true);
		encLeft = new SEncoder(4, 5, /*5865/76.25/*952.0/(6.0*Math.PI)*/18208/239.4, true);
		
		builder.addDriveModule(new DriveModule(new XY(-13.75, 0), Angle.ZERO, encLeft, motorPID.copy(), MotorInputType.SPEED, new Motor(new CANTalon(3)), new Motor(new CANTalon(4))));
		builder.addDriveModule(new DriveModule(new XY(13.75, 0), new Degrees(180), encRight, motorPID.copy(), MotorInputType.SPEED, new Motor(new CANTalon(1)), new Motor(new CANTalon(2))));
		//builder.addDriveModule(new DriveModule(new XY(-13.75, 0), Angle.ZERO, maxSpeed, new Motor(new CANTalon(3)), new Motor(new CANTalon(4))));
		//builder.addDriveModule(new DriveModule(new XY(13.75, 0), new Degrees(180), maxSpeed, new Motor(new CANTalon(1)), new Motor(new CANTalon(2))));

		

		
		
		
		builder.setInput(input);
		builder.addStep(deadzone);
		//builder.addStep(accelLimit);
		//builder.addStep(visionStep);
		builder.addStep(gCorrect);
		
		
		try {
			builder.build();
		} catch (InvalidDriveTrainException e) {
			e.printStackTrace();
		}
		
		Button resetButton= new JoystickButton(auxStick,7);
		resetButton.setPressAction(new ButtonAction(){

			@Override
			public void onAction() {
				gLock.disable();
				gearMode=GEAR_MODE.MANUAL;
			}});
		
		/*StateMachine dropGear=new StateMachine(
				new DriveEncoders(new Distance(22),0.2,ENC_SPEED),
				new DriveTime(0.8,0.1),
				new GearOpenState(gear),
				new Delay(0.2),
				new DriveEncoders(new Distance(-22),-0.3,ENC_SPEED)
				/*new State(){
					@Override
					public boolean isDone() {
						return true;
					}
					@Override
					public void start() {
						GEAR_MODE=2;
					}
					@Override
					public void stateUpdate() {
					}
					@Override
					public void stop() {
					}});*/
		
		
		
		AutoMode autoDrive=new AutoMode("AutoDriveEncoders", new DriveEncoders(new DashboardInput("drive-enc", 50), new DashboardInput("drive-enc-speed", 0.5), MAX_ENC_ACCEL, MAX_ENC_TICKS));
		super.addAutoMode(autoDrive);
		AutoMode autoTime=new AutoMode("AutoDriveTime",new DriveTime(6, .5));
		super.addAutoMode(autoTime);
		AutoMode autoTurn=new AutoMode("AutoTurn90",new TurnGyro(new Degrees(90),gCorrect, true));
		super.addAutoMode(autoTurn);
		
		AutoMode autoSmartDashboardTest=new AutoMode("AutoSmartDashboardTest (sdtest)",
				new DriveEncoders(new DashboardInput("sdtest-drive1-dist", 50), new DashboardInput("sdtest-drive1-power", 0.5), MAX_ENC_ACCEL, MAX_ENC_TICKS),
				new Delay(new DashboardInput("sdtest-delay")),
				new DriveEncoders(new DashboardInput("sdtest-drive2-dist", 50), new DashboardInput("sdtest-drive2-power", 0.5), MAX_ENC_ACCEL, MAX_ENC_TICKS));
		super.addAutoMode(autoSmartDashboardTest);
		
		/*AutoMode gearStraight = new AutoMode("Gear Straight Then Nothing Else", 
				new DriveEncoderGyro(new Distance(110-36-22), 0.35, maxEncAccel, maxEncTicksPerSec, gCorrect),
				dropGear);
		super.addAutoMode(gearStraight);*/
		
		/*AutoMode gearLeft = new AutoMode("Gear left peg",
				new DriveEncoderGyro(new DashboardInput("left-leg-1", 110-36-22), new DashboardInput("left-drive-speed", 0.35), maxEncAccel, maxEncTicksPerSec, gCorrect),
				new TurnGyro(new DashboardInput("left-turn-1", 60), gCorrect, true),
				dropGear,
				new DriveEncoderGyro(new DashboardInput("left-leg-1", 110-36-22), new DashboardInput("left-drive-speed", .35), maxEncAccel, maxEncTicksPerSec, gCorrect),
				new TurnGyro(new DashboardInput("left-turn-1", 60), gCorrect, true),
				new DriveEncoderGyro(new DashboardInput("left-leg-2", 60), new DashboardInput("left-drive-speed", .35), maxEncAccel, maxEncTicksPerSec, gCorrect)
						);
		super.addAutoMode(gearLeft);
		
		AutoMode gearRight = new AutoMode("Gear right peg",
				new DriveEncoderGyro(new DashboardInput("right-leg-1", 110-36-22), new DashboardInput("right-drive-speed", .35), maxEncAccel, maxEncTicksPerSec, gCorrect),
				new TurnGyro(new DashboardInput("right-turn-1", -60), gCorrect, true),
				dropGear,
				new DriveEncoderGyro(new DashboardInput("right-leg-1", 110-36-22), new DashboardInput("right-drive-speed", .35), maxEncAccel, maxEncTicksPerSec, gCorrect),
				new TurnGyro(new DashboardInput("right-turn-1", -60), gCorrect, true),
				new DriveEncoderGyro(new DashboardInput("right-leg-2", -60), new DashboardInput("right-drive-speed", .35), maxEncAccel, maxEncTicksPerSec, gCorrect )
						);
		super.addAutoMode(gearRight);*/
		
		/*AutoMode gearLeft = new AutoMode("Gear left peg",
				new DriveEncoderGyro(new DashboardInput("left-leg-1", 110-36-22), new DashboardInput("left-drive-speed", 0.35), maxEncAccel, maxEncTicksPerSec, gCorrect),
				new TurnGyro(new DashboardInput("left-turn-1", 60), gCorrect, true),
				dropGear,
				new DriveEncoderGyro(new DashboardInput("left-leg-1", 110-36-22), new DashboardInput("left-drive-speed", .35), maxEncAccel, maxEncTicksPerSec, gCorrect),
				new TurnGyro(new DashboardInput("left-turn-1", 60), gCorrect, true),
				new DriveEncoderGyro(new DashboardInput("left-leg-2", 60), new DashboardInput("left-drive-speed", .35), maxEncAccel, maxEncTicksPerSec, gCorrect)
						);
		super.addAutoMode(gearLeft);
		
		AutoMode gearRight = new AutoMode("Gear right peg",
				new DriveEncoderGyro(new DashboardInput("right-leg-1", 110-36-22), new DashboardInput("right-drive-speed", .35), maxEncAccel, maxEncTicksPerSec, gCorrect),
				new TurnGyro(new DashboardInput("right-turn-1", -60), gCorrect, true),
				dropGear,
				new DriveEncoderGyro(new DashboardInput("right-leg-1", 110-36-22), new DashboardInput("right-drive-speed", .35), maxEncAccel, maxEncTicksPerSec, gCorrect),
				new TurnGyro(new DashboardInput("right-turn-1", -60), gCorrect, true),
				new DriveEncoderGyro(new DashboardInput("right-leg-2", -60), new DashboardInput("right-drive-speed", .35), maxEncAccel, maxEncTicksPerSec, gCorrect )
						);
		super.addAutoMode(gearRight);*/
		//TODO: CONSTANT AUTO MODES
		
		State resetGyro=new State(){

			@Override
			public boolean isDone() {
				return true;
			}

			@Override
			public void start() {
				gCorrect.reset();
			}

			@Override
			public void stateUpdate() {
				// TODO Auto-generated method stub
				
			}

			@Override
			public void stop() {
				// TODO Auto-generated method stub
				
			}};
		
			AutoMode gearLeft = new AutoMode("Gear left peg",
					new DriveEncoderGyro(new DashboardInput("left-leg-1", 110-36-22), new DashboardInput("left-drive-speed", 0.35), MAX_ENC_ACCEL, MAX_ENC_TICKS, gCorrect),
					new TurnGyro(new DashboardInput("left-turn-1", 60), gCorrect, true),
					new DriveEncoderGyro(new DashboardInput("left-leg-1", 110-36-22), new DashboardInput("left-drive-speed", .35), MAX_ENC_ACCEL, MAX_ENC_TICKS, gCorrect),
					new TurnGyro(new DashboardInput("left-turn-1", 60), gCorrect, true),
					new DriveEncoderGyro(new DashboardInput("left-leg-2", 60), new DashboardInput("left-drive-speed", .35), MAX_ENC_ACCEL, MAX_ENC_TICKS, gCorrect)
							);
			super.addAutoMode(gearLeft);
		AutoMode autoDriveEncLock=new AutoMode("AutoDriveEncoders with gyrolock", new Enable(gLock),new DriveEncoders(new Distance(-96+6.3), 0.5,MAX_ENC_ACCEL, MAX_ENC_TICKS),new Disable(gLock));
		super.addAutoMode(autoDriveEncLock);
		AutoMode autoTimeLock=new AutoMode("AutoDriveTime with gyrolock",new Enable(gLock),new DriveTime(new DashboardInput("auto-time", 10.0), .5),new Disable(gLock));
		super.addAutoMode(autoTimeLock);
		AutoMode autoTurnTest90=new AutoMode("Auto Turn 90 With Bugfix",resetGyro,new TurnGyro(Angle.QUARTER,gCorrect,false));
		super.addAutoMode(autoTurnTest90);
		AutoMode autoTurnTest45=new AutoMode("Auto Turn 45",resetGyro,new TurnGyro(new Degrees(45),gCorrect,false));
		super.addAutoMode(autoTurnTest45);
		
		AutoMode testRoutine=new AutoMode("Auto Test Routine",
				new Enable(gLock),new DriveEncoders(new Distance(5*-12+6.3),0.5,MAX_ENC_ACCEL, MAX_ENC_TICKS),new Disable(gLock),
				new TurnGyro(new Degrees(45),gCorrect,true),
				new Enable(gLock),new DriveEncoders(new Distance(2*-12+6.3),0.5,MAX_ENC_ACCEL, MAX_ENC_TICKS),new Disable(gLock));
		super.addAutoMode(testRoutine);
		
		AutoMode testEncodersGyro=new AutoMode("TestEncodersGyro",resetGyro,
				new DriveEncoderGyro(new Distance(5*-12+6.3),Angle.ZERO,false,0.5,MAX_ENC_ACCEL, MAX_ENC_TICKS,gCorrect),
				new DriveEncoderGyro(new Distance(2*-12+6.3),Angle.QUARTER,false,0.5,MAX_ENC_ACCEL, MAX_ENC_TICKS,gCorrect),
				new DriveEncoderGyro(new Distance(2*-12+6.3),new Degrees(45),true,0.5,MAX_ENC_ACCEL, MAX_ENC_TICKS,gCorrect),
				new DriveEncoderGyro(new Distance(5*-12+6.3),Angle.ZERO,false,0.5,MAX_ENC_ACCEL, MAX_ENC_TICKS,gCorrect));
		super.addAutoMode(testEncodersGyro);
		
		super.sendAutoModes();
		
	}
	
	public void update()
	{
		//Debug.msg("Close switch", closeSwitch.get() ? "true" : "false");
		//Debug.msg("Open switch", openSwitch.get() ? "true" : "false");
		
		Debug.num("encRight speed", encRight.get());
		Debug.num("encLeft speed", encLeft.get());
		Debug.num("encRight raw dist", encRight.getTicks());
		Debug.num("encLeft raw dist", encLeft.getTicks());
		Debug.num("encRight inches", encRight.getInches().get());
		Debug.num("encLeft inches", encLeft.getInches().get());
		Debug.msg("gyroAngle", navX.get());
		//SmartDashboard.putNumber("MaxTurn",SprocketRobot.getDriveTrain().getMaxTurn().toDegrees());
		
		//if(GEAR_MODE == 1 && super.isOperatorControl() && auxStick != null && gear != null) {
			if(auxStick.getThrottle() < 0.5) {
				gearMode=GEAR_MODE.AUTO;
			} else {
				gearMode=GEAR_MODE.MANUAL;
			}
		//}
		Debug.msg("Gear control mode", gearMode);
		
		gear.gearSpeed = gearSpeedInput.get();
	}
}

