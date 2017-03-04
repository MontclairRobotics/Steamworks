package org.usfirst.frc.team555.robot;

import org.montclairrobotics.sprocket.auto.AutoState;
import org.montclairrobotics.sprocket.motors.Motor;
import org.montclairrobotics.sprocket.utils.Input;

public class GearOpenStep extends AutoState {
	
	private Input<Boolean> limitSwitch;
	private Motor gearMotor;
	
	@Override
	public boolean isDone() {
		return limitSwitch.get();
	}

	@Override
	public void stateUpdate() {
		gearMotor.set(1.0);
	}

}
