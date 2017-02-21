package org.usfirst.frc.team555.robot;

import org.montclairrobotics.sprocket.drive.DTTarget;
import org.montclairrobotics.sprocket.geometry.Angle;
import org.montclairrobotics.sprocket.geometry.Radians;
import org.montclairrobotics.sprocket.geometry.Vector;
import org.montclairrobotics.sprocket.pipeline.Step;
import org.montclairrobotics.sprocket.utils.Input;

public class VisionStep implements Step<DTTarget>{

	private double target;
	private Vision vision;
	private double turnP;
	private double minTurnError;
	private double turn;
	private Input<Boolean> enabled;
	
	public VisionStep(double target,Vision vision,double turnP,double minTurnError,Input<Boolean> enabled)
	{
		this.target=target;
		this.vision=vision;
		this.turnP=turnP;
		this.minTurnError=minTurnError;
		this.enabled=enabled;
	}
	
	@Override
	public DTTarget get(DTTarget arg0) {
		if(enabled.get())
		{
			turn=(target-vision.getX())*turnP;
			if(Math.abs(turn)<minTurnError)
				turn=0;
			return new DTTarget(arg0.getDirection(),turn);
		}
		return arg0;
	}

}
