package org.usfirst.frc.team555.robot;

import org.montclairrobotics.sprocket.utils.Input;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.SPI.Port;

public class NavXRollInput extends AHRS implements Input<Double>{

	public NavXRollInput(Port id) {
		super(id);
	}

	@Override
	public Double get() {
		return new Double(super.getRoll());
	}

}
