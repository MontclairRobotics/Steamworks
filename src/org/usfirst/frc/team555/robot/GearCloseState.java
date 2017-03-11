package org.usfirst.frc.team555.robot;

import org.montclairrobotics.sprocket.auto.AutoState;
import org.montclairrobotics.sprocket.motors.Motor;

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
		Robot.MANUAL_GEAR_CONTROL=true;
		g.close();
	}
	@Override
	public void userStop() {
		g.stop();
		Robot.MANUAL_GEAR_CONTROL=false;
	}

	@Override
	public void stateUpdate() {
		// TODO Auto-generated method stub
		
	}
}
