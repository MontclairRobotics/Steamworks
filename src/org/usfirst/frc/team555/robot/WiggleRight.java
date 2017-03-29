package org.usfirst.frc.team555.robot;

import org.montclairrobotics.sprocket.auto.AutoState;
import org.montclairrobotics.sprocket.geometry.Radians;
import org.montclairrobotics.sprocket.geometry.XY;

public class WiggleRight extends AutoState {

	@Override
	public boolean isDone() {
		return timeInState() > .25;
	}

	@Override
	public void stateUpdate() {
		tgtDir = new XY(0, .3);
		tgtTurn = new Radians(.3);
	}

}
