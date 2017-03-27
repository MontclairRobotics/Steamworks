package org.usfirst.frc.team555.robot;

import org.montclairrobotics.sprocket.auto.AutoState;
import org.montclairrobotics.sprocket.motors.Motor;

public class GearOpenState extends AutoState {
	
	private Gear g;
	
	public GearOpenState(Gear g) {
		this.g=g;
	}
	
	@Override
	public boolean isDone() {
		return this.timeInState() > 1.0;
	}
	
	@Override
	public void userStart()
	{
		Robot.GEAR_MODE=2;
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
