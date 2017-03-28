package org.usfirst.frc.team555.robot;

import org.montclairrobotics.sprocket.motors.Motor;

import edu.wpi.first.wpilibj.DigitalInput;

public class Gear {
	
	private Motor m1;
	private Motor m2;
	private DigitalInput open1Switch;
	private DigitalInput open2Switch;
	private DigitalInput close1Switch;
	private DigitalInput close2Switch;
	
	public static double gearSpeed = 0.2;
	

	public Gear(Motor m1,DigitalInput openSwitch,DigitalInput closeSwitch,Motor m2,DigitalInput open2Switch,DigitalInput close2Switch)
	{
		this.m1=m1;
		this.m2 = m2;
		this.open1Switch=openSwitch;
		this.close1Switch=closeSwitch;
		this.open2Switch = open2Switch;
		this.close2Switch = close2Switch;
	}
	
	public void openLimit()
	{
		if(!open1Switch.get()) {
			m1.set(gearSpeed);;
		} else {
			m1.set(0.0);
		}
		
		if(!open2Switch.get()) {
			m2.set(gearSpeed);
		} else {
			m2.set(0.0);
		}
	}
	public void closeLimit()
	{
		if(!close1Switch.get()) {
			m1.set(-gearSpeed);
		} else {
			m1.set(0.0);
		}
		
		if(!close2Switch.get()) {
			m2.set(-gearSpeed);
		} else {
			m2.set(0.0);
		}
	}
	
	public void open()
	{
		m1.set(gearSpeed);
		m2.set(gearSpeed);
	}
	public void close()
	{
		m1.set(-gearSpeed);
		m2.set(-gearSpeed);
	}
	public void stop()
	{
		m1.set(0.0);
		m2.set(0.0);
	}
	//6,7   10,11
	public void open1() {
		m1.set(gearSpeed);
	}
	
	public void close1() {
		m1.set(-gearSpeed);
	}
	
	public void stop1() {
		m1.set(0.0);
	}
	
	public void open2() {
		m2.set(gearSpeed);
	}
	
	public void close2() {
		m2.set(-gearSpeed);
	}
	
	public void stop2() {
		m2.set(0.0);
	}
}
