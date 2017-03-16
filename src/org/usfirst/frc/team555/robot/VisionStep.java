package org.usfirst.frc.team555.robot;

import org.montclairrobotics.sprocket.drive.DTTarget;
import org.montclairrobotics.sprocket.geometry.Angle;
import org.montclairrobotics.sprocket.geometry.Radians;
import org.montclairrobotics.sprocket.geometry.Vector;
import org.montclairrobotics.sprocket.pipeline.Step;
import org.montclairrobotics.sprocket.utils.Input;
import org.montclairrobotics.sprocket.utils.PID;
import org.montclairrobotics.sprocket.utils.TargetablePID;
import org.montclairrobotics.sprocket.utils.Togglable;

public class VisionStep implements Step<DTTarget>,Togglable{

	private double goal;
	private Vision vision;
	private double turnP;
	private PID turnPID;
	private double minTurnError;
	private double turn;
	private boolean enabled=false;
	
	public VisionStep(double goal,Vision vision,double turnP, double turnI, double turnD,double minTurnError)
	{
		this.goal=goal;
		this.vision=vision;
		this.turnPID = new TargetablePID(turnP, turnI, turnD).setInput(vision).setTarget(goal);
		this.minTurnError=minTurnError;
	}
	
	
	@Override
	public DTTarget get(DTTarget arg0) {
		if(enabled)
		{
			turn=turnPID.get();
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
