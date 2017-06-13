package org.usfirst.frc.team555.robot;

import org.montclairrobotics.sprocket.SprocketRobot;
import org.montclairrobotics.sprocket.auto.AutoMode;
import org.montclairrobotics.sprocket.auto.states.*;
import org.montclairrobotics.sprocket.control.*;
import org.montclairrobotics.sprocket.drive.ControlledMotor;
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
import org.montclairrobotics.sprocket.loop.Updater;
import org.montclairrobotics.sprocket.motors.Motor;
import org.montclairrobotics.sprocket.motors.SEncoder;
import org.montclairrobotics.sprocket.states.State;
import org.montclairrobotics.sprocket.states.StateMachine;
import org.montclairrobotics.sprocket.motors.Module.MotorInputType;
import org.montclairrobotics.sprocket.utils.Debug;
import org.montclairrobotics.sprocket.utils.DoubleInput;
import org.montclairrobotics.sprocket.utils.Input;
import org.montclairrobotics.sprocket.utils.PID;
import org.montclairrobotics.sprocket.utils.ZeroInput;

import com.ctre.CANTalon;

import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj.SPI.Port;
import edu.wpi.first.wpilibj.VictorSP;
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
	public static GEAR_MODE gearMode=GEAR_MODE.MANUAL;
	
	private static final int IMG_WIDTH = 320,IMG_HEIGHT = 240;
	private static final int 
		DriveStickID=0,
		AuxStickID=1,
		CloseSwitchLeftID=0,//limit switches
		OpenSwitchLeftID=1,
		CloseSwitchRightID=7,//%Better comments
		OpenSwitchRightID=6,
		GearButtonID=1,
		GearButtonAutoID=7,
		GearButtonManualID=8,
		GyroLockOff=6,
		FieldCentricButtonID=2,
		GyroLockButtonID=7,
		ResetGyroID=3,
		//VisionButtonID=4,
		LeftButtonID=4,
		RightButtonID=5,
		ManualOpen1=9,
		ManualClose1=11,
		ManualOpen2=10,
		ManualClose2=12,
		FullSpinButtonID=10,
		ClimbFastID=5,
		ClimbSlowID=3;
	

	private static final Distance ENC_SPEED = new Distance(1);
	private static final double MAX_ENC_ACCEL = 13;
	private static final double MAX_ENC_TICKS = 25;

	protected static final double FULL_SPIN_SPEED = 1;
	protected static final double SLOW_SPIN_SPEED = 0.45;

	protected static final double SPINNY_POWER = 1;
	
	private static boolean lastAutoGear=true;
	
	private Joystick driveStick;
	private Joystick auxStick;
	
	private DriveTrainBuilder builder;
	//private DriveTrain driveTrain;
	
	private Motor gearLeftMotor, gearRightMotor;
	private DigitalInput openLeftSwitch, closeLeftSwitch, openRightSwitch, closeRightSwitch;
	
	private Motor ropeMotor1;
	private Motor ropeMotor2;
	
	private Motor ballSpinny;
	private Motor ballShooty;
	
	private SEncoder encRight;
	private SEncoder encLeft;

	private NavXRollInput navX;
	
	private Gear gear;
	private DashboardInput gearSpeedInput;
	
	private PowerDistributionPanel pdp;
	
	private AccelLimit accelLimit;

	private double shootStartTime;

	
	@Override
	public void robotInit() {
		//Joysticks
		driveStick = new Joystick(DriveStickID);
		auxStick = new Joystick(AuxStickID);
		//Gear opened/closed limit switches
		openLeftSwitch = new DigitalInput(OpenSwitchLeftID);
		closeLeftSwitch = new DigitalInput(CloseSwitchLeftID);
		openRightSwitch = new DigitalInput(OpenSwitchRightID);
		closeRightSwitch = new DigitalInput(CloseSwitchRightID);
		
		//Setting up gear trigger
		gearLeftMotor = new Motor(new VictorSP(0));
		gearLeftMotor.setInverted(true);
		gearRightMotor = new Motor(new CANTalon(5));
		gearRightMotor.setInverted(true);
		gear = new Gear(gearLeftMotor,openLeftSwitch,closeLeftSwitch, gearRightMotor, openRightSwitch, closeRightSwitch);
		gearSpeedInput = new DashboardInput("gear speed", 0.5);
		
		//DRIVE STICK TO AUX STICK CHANGE
		Button gearButton = new JoystickButton(auxStick, GearButtonID);
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
		
		
		
		
		//Rope climber motors
		/*ropeMotor1 = new ControlledMotor(new CANTalon(6), new JoystickYAxis(auxStick));
		ropeMotor1.constrain(0.0, 1.0);
		ropeMotor1.getMotor().setInverted(true);
		ropeMotor2 = new ControlledMotor(new CANTalon(7), new JoystickYAxis(auxStick));
		ropeMotor2.constrain(0.0, 1.0);
		ropeMotor2.getMotor().setInverted(true);*/
		
		ropeMotor1=new Motor(new CANTalon(6));
		ropeMotor2=new Motor(new CANTalon(7));
		ropeMotor1.getMotor().setInverted(true);
		ropeMotor2.getMotor().setInverted(true);
		setClimbSpeed(0);
				
		Button climbFast=new JoystickButton(auxStick,ClimbFastID);
		Button climbSlow=new JoystickButton(auxStick,ClimbSlowID);
		
		climbFast.setHeldAction(new ButtonAction(){

			@Override
			public void onAction() {
				setClimbSpeed(1);
			}});
		climbFast.setReleaseAction(new ButtonAction(){

			@Override
			public void onAction() {
				setClimbSpeed(0);
			}});
		climbSlow.setHeldAction(new ButtonAction(){

				@Override
				public void onAction() {
					setClimbSpeed(0.5);
				}});
		climbSlow.setReleaseAction(new ButtonAction(){

				@Override
				public void onAction() {
					setClimbSpeed(0);
				}});
		
		//Shooter motors
		ballSpinny=new Motor(new CANTalon(9));
		ballShooty=new Motor(new CANTalon(10));
		
		//Shooter Triggers
		Button shootTrigger=new JoystickButton(auxStick,0);
		Button agitatorOverride=new JoystickButton(auxStick,4);
		
		shootTrigger.setHeldAction(new ButtonAction(){

			@Override
			public void onAction() {
				ballShooty.set(getShootSpeed());
				if(Updater.getTime()-shootStartTime>1)
				{
					ballSpinny.set(SPINNY_POWER);
				}
			}});
		shootTrigger.setPressAction(new ButtonAction(){

			@Override
			public void onAction() {
				shootStartTime=Updater.getTime();
			}});
		shootTrigger.setOffAction(new ButtonAction(){
			@Override
			public void onAction() {
				ballShooty.set(0);
			}});
		shootTrigger.setReleaseAction(new ButtonAction(){
			@Override
			public void onAction() {
				ballSpinny.set(0);
			}});
		
		agitatorOverride.setPressAction(new ButtonAction(){

			@Override
			public void onAction() {
				ballSpinny.set(SPINNY_POWER);
			}});
		agitatorOverride.setReleaseAction(new ButtonAction(){

			@Override
			public void onAction() {
				ballSpinny.set(0);
			}});

		//input.setSensitivity(0.5, 0.3);
		
		Deadzone deadzone=new Deadzone();
		
		accelLimit=new AccelLimit(4,16);
		
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
		GyroCorrection gCorrect=new GyroCorrection(navX,gyroPID,20,0.3*20);
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
		input.setXSensitivity(SLOW_SPIN_SPEED);
		
		
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
		
		//PID motorPID = new PID(0.5, 0.05, 0);
		PID motorPID = new PID(0, 0, 0);
		encRight = new SEncoder(2, 3, /*5865/76.25/*952.0/(6.0*Math.PI)*/18208/239.4, true);
		encLeft = new SEncoder(4, 5, /*5865/76.25/*952.0/(6.0*Math.PI)*/18208/239.4, true);
		
		builder.addDriveModule(new DriveModule(new XY(-13.75, 0), Angle.ZERO, encLeft, motorPID.copy(), MotorInputType.SPEED, new Motor(new CANTalon(3)), new Motor(new CANTalon(4))));
		builder.addDriveModule(new DriveModule(new XY(13.75, 0), new Degrees(180), encRight, motorPID.copy(), MotorInputType.SPEED, new Motor(new CANTalon(1)), new Motor(new CANTalon(2))));
		//builder.addDriveModule(new DriveModule(new XY(-13.75, 0), Angle.ZERO, maxSpeed, new Motor(new CANTalon(3)), new Motor(new CANTalon(4))));
		//builder.addDriveModule(new DriveModule(new XY(13.75, 0), new Degrees(180), maxSpeed, new Motor(new CANTalon(1)), new Motor(new CANTalon(2))));

		

		
		
		
		builder.setInput(input);
		builder.addStep(deadzone);
		builder.addStep(accelLimit);
		//builder.addStep(visionStep);
		builder.addStep(gCorrect);
		
		try {
			builder.build();
		} catch (InvalidDriveTrainException e) {
			e.printStackTrace();
		}
		
		pdp = new PowerDistributionPanel();
		
		Button gyroLockOff= new JoystickButton(auxStick,GyroLockOff);
		gyroLockOff.setPressAction(new ButtonAction(){

			@Override
			public void onAction() {
				gLock.disable();
			}});
		
		Button gearModeAuto=new JoystickButton(auxStick,GearButtonAutoID);
		gearModeAuto.setPressAction(new ButtonAction(){

			@Override
			public void onAction() {
				gearMode=GEAR_MODE.AUTO;
			}});
		Button gearModeManual=new JoystickButton(auxStick,GearButtonManualID);
		gearModeManual.setPressAction(new ButtonAction(){

			@Override
			public void onAction() {
				gearMode=GEAR_MODE.MANUAL;
				gear.stop();
			}});
		
		//FULL SPIN BUTTON
		Button fullSpinButton=new JoystickButton(driveStick,FullSpinButtonID);
		fullSpinButton.setPressAction(new ButtonAction(){

			@Override
			public void onAction() {
				input.setXSensitivity(FULL_SPIN_SPEED);
			}});
		fullSpinButton.setReleaseAction(new ButtonAction(){

			@Override
			public void onAction() {
				input.setXSensitivity(SLOW_SPIN_SPEED);
			}});
		
		
		//==================== AUTO SUBROUTINES ====================
		
		StateMachine dropGear=new StateMachine(
				new DriveEncoders(20, 0.3, MAX_ENC_ACCEL, MAX_ENC_TICKS),
				//new WiggleLeft(),
				//new WiggleRight(),
				new DriveTime(0.25,0.2),
				new GearOpenState(gear),
				new DriveEncoders(-22, -0.3, MAX_ENC_ACCEL, MAX_ENC_TICKS));

		State resetGyro= new State(){

			@Override
			public boolean isDone() {
				// TODO Auto-generated method stub
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
		
		
		//==================== TEST AUTO MODES ====================
		AutoMode autoDrive=new AutoMode("AutoDriveEncoders", new DriveEncoderGyro(new DashboardInput("drive-enc", 50), new DashboardInput("drive-enc-speed", 0.5), MAX_ENC_ACCEL, MAX_ENC_TICKS, gCorrect));
		super.addAutoMode(autoDrive);
		
		AutoMode autoSmartDashboardTest=new AutoMode("AutoSmartDashboardTest (sdtest)",
				new DriveEncoders(new DashboardInput("sdtest-drive1-dist", 50), new DashboardInput("sdtest-drive1-power", 0.5), MAX_ENC_ACCEL, MAX_ENC_TICKS),
				new Delay(new DashboardInput("sdtest-delay")),
				new DriveEncoders(new DashboardInput("sdtest-drive2-dist", 50), new DashboardInput("sdtest-drive2-power", 0.5), MAX_ENC_ACCEL, MAX_ENC_TICKS));
		super.addAutoMode(autoSmartDashboardTest);
		
		
		//==================== REAL AUTO MODES ====================
		double
			STRAIGHT_DRIVE_A=(110-36-22),//up to the peg
			SIDE_DRIVE_A=88-36,//first drive to the turn //52
			SIDE_DRIVE_B=(61.5-22+16),//from the turn to the peg  //55.5
			SIDE_DRIVE_C=100;//after backing up, across the baseline
		
		
		/*
		 * Left peg
		 * 88-25
		 * 52 degrees
		 * 61.5-22 forward
		 * 
		 * 
		 * Right peg
		 * 86 inch
		 * 53 degrees
		 * 4 inches
		 */
		
		double FULL_SPEED=0.2;//0.8
		
		
		super.addAutoMode(new AutoMode("Gear STRAIGHT Then Nothing Else", 
				resetGyro,
				new DriveEncoderGyro(new Distance(STRAIGHT_DRIVE_A), FULL_SPEED, MAX_ENC_ACCEL, MAX_ENC_TICKS, gCorrect),
				dropGear));
		
		super.addAutoMode(new AutoMode("Gear LEFT Peg (Turn RIGHT)",
				resetGyro,
				new DriveEncoderGyro(new Distance(SIDE_DRIVE_A), FULL_SPEED, MAX_ENC_ACCEL, MAX_ENC_TICKS, gCorrect),
				new DriveEncoderGyro(new Distance(SIDE_DRIVE_B),new Degrees(52),false, FULL_SPEED, MAX_ENC_ACCEL, MAX_ENC_TICKS, gCorrect),
				dropGear,
				new DriveEncoderGyro(new Distance(-SIDE_DRIVE_B),new Degrees(52),false, -FULL_SPEED, MAX_ENC_ACCEL, MAX_ENC_TICKS, gCorrect),
				new DriveEncoderGyro(new Distance(SIDE_DRIVE_C),Angle.ZERO,false, FULL_SPEED, MAX_ENC_ACCEL, MAX_ENC_TICKS, gCorrect)));
		
		super.addAutoMode(new AutoMode("Gear RIGHT Peg (Turn LEFT)",
				resetGyro,
				new DriveEncoderGyro(new Distance(SIDE_DRIVE_A), FULL_SPEED, MAX_ENC_ACCEL, MAX_ENC_TICKS, gCorrect),
				new DriveEncoderGyro(new Distance(SIDE_DRIVE_B),new Degrees(-52),false, FULL_SPEED, MAX_ENC_ACCEL, MAX_ENC_TICKS, gCorrect),
				dropGear,
				new DriveEncoderGyro(new Distance(-SIDE_DRIVE_B),new Degrees(-52),false, -FULL_SPEED, MAX_ENC_ACCEL, MAX_ENC_TICKS, gCorrect),
				new DriveEncoderGyro(new Distance(SIDE_DRIVE_C),Angle.ZERO,false, FULL_SPEED, MAX_ENC_ACCEL, MAX_ENC_TICKS, gCorrect)));
		
		super.addAutoMode(new AutoMode("SPENCER",
				resetGyro,
				new DriveEncoderGyro(new Distance(70), FULL_SPEED, MAX_ENC_ACCEL, MAX_ENC_TICKS, gCorrect),
				new DriveEncoderGyro(new Distance(55-22),new Degrees(52),false, FULL_SPEED, MAX_ENC_ACCEL, MAX_ENC_TICKS, gCorrect),
				dropGear,
				new DriveEncoderGyro(new Distance(-(55-22)),new Degrees(52),false, -FULL_SPEED, MAX_ENC_ACCEL, MAX_ENC_TICKS, gCorrect),
				new DriveEncoderGyro(new Distance(SIDE_DRIVE_C),Angle.ZERO,false, FULL_SPEED, MAX_ENC_ACCEL, MAX_ENC_TICKS, gCorrect)));
		
		super.addAutoMode(new AutoMode("Drive Forward 14 ft",
				resetGyro,
				new DriveEncoderGyro(new Distance(14*12), FULL_SPEED, MAX_ENC_ACCEL, MAX_ENC_TICKS, gCorrect)));
		
		super.addAutoMode(new AutoMode("6-90-2",
				resetGyro,
				new DriveEncoderGyro(new Distance(6*12), FULL_SPEED, MAX_ENC_ACCEL, MAX_ENC_TICKS, gCorrect),
				new DriveEncoderGyro(new Distance(2*12), new Degrees (90), false, FULL_SPEED, MAX_ENC_ACCEL,MAX_ENC_TICKS,gCorrect)));
		super.addAutoMode(new AutoMode("Random Test",
				resetGyro,
				new DriveEncoderGyro(new Distance(52), FULL_SPEED, MAX_ENC_ACCEL, MAX_ENC_TICKS, gCorrect),
				new DriveEncoderGyro(new Distance(72-17), new Degrees (60), false, FULL_SPEED, MAX_ENC_ACCEL,MAX_ENC_TICKS,gCorrect)));
		
		//==================== EDITABLE TEST AUTO MODES ====================
		DashboardInput
			STRAIGHT_DRIVE_A_INPUT=new DashboardInput("STRAIGHT_DRIVE_A",STRAIGHT_DRIVE_A),//up to the peg
			SIDE_DRIVE_A_INPUT=new DashboardInput("SIDE_DRIVE_A",SIDE_DRIVE_A),//first drive to the turn
			SIDE_DRIVE_B_INPUT=new DashboardInput("SIDE_DRIVE_B",SIDE_DRIVE_B),//from the turn to the peg
			SIDE_DRIVE_C_INPUT=new DashboardInput("SIDE_DRIVE_B",SIDE_DRIVE_B);//after backing up, across the baseline
		
		DashboardInput FULL_SPEED_INPUT=new DashboardInput("FULL_SPEED",FULL_SPEED);
		
		Input<Double> zeroInput=new ZeroInput();
		Input<Double> input60=new DoubleInput(60);
		Input<Double> inputNeg60=new DoubleInput(-60);
		
		super.addAutoMode(new AutoMode("EDITABLE Gear STRAIGHT Then Nothing Else", 
				resetGyro,
				new DriveEncoderGyro(STRAIGHT_DRIVE_A_INPUT, FULL_SPEED_INPUT, MAX_ENC_ACCEL, MAX_ENC_TICKS, gCorrect),
				dropGear));
		
		super.addAutoMode(new AutoMode("EDITABLE Gear LEFT Peg (Turn RIGHT)",
				resetGyro,
				new DriveEncoderGyro(SIDE_DRIVE_A_INPUT, FULL_SPEED_INPUT, MAX_ENC_ACCEL, MAX_ENC_TICKS, gCorrect),
				new DriveEncoderGyro(SIDE_DRIVE_B_INPUT,input60,false, FULL_SPEED_INPUT, MAX_ENC_ACCEL, MAX_ENC_TICKS, gCorrect),
				dropGear,
				new DriveEncoderGyro(Input.neg(SIDE_DRIVE_B_INPUT),input60,false, Input.neg(FULL_SPEED_INPUT), MAX_ENC_ACCEL, MAX_ENC_TICKS, gCorrect),
				new DriveEncoderGyro(SIDE_DRIVE_C_INPUT,zeroInput,false, FULL_SPEED_INPUT, MAX_ENC_ACCEL, MAX_ENC_TICKS, gCorrect)));
		
		super.addAutoMode(new AutoMode("EDITABLE Gear RIGHT Peg (Turn LEFT)",
				resetGyro,
				new DriveEncoderGyro(SIDE_DRIVE_A_INPUT, FULL_SPEED_INPUT, MAX_ENC_ACCEL, MAX_ENC_TICKS, gCorrect),
				new DriveEncoderGyro(SIDE_DRIVE_B_INPUT,inputNeg60,false, FULL_SPEED_INPUT, MAX_ENC_ACCEL, MAX_ENC_TICKS, gCorrect),
				dropGear,
				new DriveEncoderGyro(Input.neg(SIDE_DRIVE_B_INPUT),inputNeg60,false, Input.neg(FULL_SPEED_INPUT), MAX_ENC_ACCEL, MAX_ENC_TICKS, gCorrect),
				new DriveEncoderGyro(SIDE_DRIVE_C_INPUT,zeroInput,false, FULL_SPEED_INPUT, MAX_ENC_ACCEL, MAX_ENC_TICKS, gCorrect)));

		
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
				new TurnGyro(new DashboardInput("right-turn-1", -60), g-Correct, true),
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
		Debug.num("6-amps-pdp", pdp.getCurrent(13));
		Debug.num("7-amps-pdp", pdp.getCurrent(14));
		Debug.msg("gyroAngle", navX.get());
		Debug.msg("limit-openLeft", gear.getLeftOpen());
		Debug.msg("limit-openRight", gear.getRightOpen());
		Debug.msg("limit-closeLeft", gear.getLeftClose());
		Debug.msg("limit-closeRight", gear.getRightClose());
		//SmartDashboard.putNumber("MaxTurn",SprocketRobot.getDriveTrain().getMaxTurn().toDegrees());
		boolean autoGear=auxStick.getThrottle() < 0.5;
		//if(GEAR_MODE == 1 && super.isOperatorControl() && auxStick != null && gear != null) {
		
		//}
		Debug.msg("Gear control mode", gearMode);
		
		//gear.gearSpeed = gearSpeedInput.get();
	}
	
	public void setClimbSpeed(double spd)
	{
		ropeMotor1.set(spd);
		ropeMotor2.set(spd);
	}
	
	public double getShootSpeed() {
		// TODO Auto-generated method stub
		return 0.75+0.25*auxStick.getY();
	}
	
	@Override
	public void userAutonomousInit() {
		accelLimit.disable();
	}
	
	@Override
	public void userTeleopInit() {
		accelLimit.enable();
	}
	
}

