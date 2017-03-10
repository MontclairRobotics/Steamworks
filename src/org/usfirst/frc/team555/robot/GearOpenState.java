package org.usfirst.frc.team555.robot;

import org.montclairrobotics.sprocket.auto.AutoState;
import org.montclairrobotics.sprocket.motors.Motor;

public class GearOpenState extends AutoState {
	
	private Motor gearMotor;
	
	public GearOpenState(Motor m) {
		gearMotor = m;
	}
	
	@Override
	public boolean isDone() {
		return this.timeInState() > 1.0;
	}

	@Override
	public void stateUpdate() {
		gearMotor.set(1.0);
	}

}
