package org.usfirst.frc.team555.robot;

import org.montclairrobotics.sprocket.auto.AutoState;
import org.montclairrobotics.sprocket.motors.Motor;

public class GearCloseState extends AutoState {
	
	private Motor gearMotor;
	
	public GearCloseState(Motor m) {
		gearMotor = m;
	}
	
	@Override
	public boolean isDone() {
		return this.timeInState() > 1.0;
	}
	
	@Override
	public void userStop() {
		gearMotor.set(0);
	}
	
	@Override
	public void stateUpdate() {
		gearMotor.set(-1.0);
	}

}
