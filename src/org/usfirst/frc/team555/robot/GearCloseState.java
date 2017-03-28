package org.usfirst.frc.team555.robot;

import org.montclairrobotics.sprocket.auto.AutoState;
import org.montclairrobotics.sprocket.motors.Motor;
import org.usfirst.frc.team555.robot.Robot.GEAR_MODE;

public class GearCloseState extends AutoState {
	
private Gear g;
	
	public GearCloseState(Gear g) {
		this.g=g;
	}
	
	@Override
	public boolean isDone() {
		return this.timeInState() > 1.0;
	}
	
	@Override
	public void userStart()
	{
		Robot.gearMode=GEAR_MODE.MANUAL;
		g.close();
	}
	@Override
	public void userStop() {
		g.stop();
		Robot.gearMode=GEAR_MODE.MANUAL;
	}

	@Override
	public void stateUpdate() {
		// TODO Auto-generated method stub
		
	}
}
