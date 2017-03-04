package org.usfirst.frc.team555.robot;

import org.montclairrobotics.sprocket.drive.DTTarget;
import org.montclairrobotics.sprocket.geometry.Angle;
import org.montclairrobotics.sprocket.geometry.Radians;
import org.montclairrobotics.sprocket.geometry.Vector;
import org.montclairrobotics.sprocket.pipeline.Step;
import org.montclairrobotics.sprocket.utils.Input;
import org.montclairrobotics.sprocket.utils.Togglable;

public class VisionStep implements Step<DTTarget>,Togglable{

	private double goal;
	private Vision vision;
	private double turnP;
	private double minTurnError;
	private double turn;
	private boolean enabled=false;
	
	public VisionStep(double goal,Vision vision,double turnP,double minTurnError)
	{
		this.goal=goal;
		this.vision=vision;
		this.turnP=turnP;
		this.minTurnError=minTurnError;
	}
	
	
	@Override
	public DTTarget get(DTTarget arg0) {
		if(enabled)
		{
			double visionOutput=vision.getX();
			if(visionOutput<0||Math.abs(goal-visionOutput)<minTurnError)
				turn=0;
			else
				turn=(goal-visionOutput)*turnP;
			return new DTTarget(arg0.getDirection(),new Radians(turn));
		}
		return arg0;
	}


	@Override
	public void disable() {
		this.enabled=false;
	}


	@Override
	public void enable() {
		this.enabled=true;
	}

}
