package org.usfirst.frc.team555.robot;

import org.montclairrobotics.sprocket.auto.AutoState;
import org.montclairrobotics.sprocket.motors.Motor;
import org.usfirst.frc.team555.robot.Robot.GEAR_MODE;

public class GearOpenState extends AutoState {
	
	private Gear g;
	
	public GearOpenState(Gear g) {
		this.g=g;
	}
	
	@Override
	public boolean isDone() {
		return (g.getRightOpen() && g.getLeftOpen()) || (timeInState() > 2.0);
	}
	
	@Override
	public void userStart()
	{
		Robot.gearMode=GEAR_MODE.MANUAL;
		g.open();
	}
	@Override
	public void userStop() {
		g.stop();
	}

	@Override
	public void stateUpdate() {
		// TODO Auto-generated method stub
		
	}

}
