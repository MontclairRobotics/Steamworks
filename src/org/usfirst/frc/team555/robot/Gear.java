package org.usfirst.frc.team555.robot;

import org.montclairrobotics.sprocket.motors.Motor;

import edu.wpi.first.wpilibj.DigitalInput;

public class Gear {
	private Motor m;
	private DigitalInput openSwitch;
	private DigitalInput closeSwitch;

	public Gear(Motor m,DigitalInput openSwitch,DigitalInput closeSwitch)
	{
		this.m=m;
		this.openSwitch=openSwitch;
		this.closeSwitch=closeSwitch;
	}
	
	public void openLimit()
	{
		if(openSwitch.get())
			stop();
		else
			open();
	}
	public void closeLimit()
	{
		if(closeSwitch.get())
			stop();
		else
			close();
	}
	
	public void open()
	{
		m.set(1.0);
	}
	public void close()
	{
		m.set(-1.0);
	}
	public void stop()
	{
		m.set(0.0);
	}
}
