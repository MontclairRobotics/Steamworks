package org.usfirst.frc.team555.robot;

import org.montclairrobotics.sprocket.loop.Priority;
import org.montclairrobotics.sprocket.loop.Updatable;
import org.montclairrobotics.sprocket.loop.Updater;
import org.opencv.core.Rect;
import org.opencv.imgproc.Imgproc;

import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.vision.VisionThread;

import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.SPI.Port;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.vision.VisionRunner;
import edu.wpi.first.wpilibj.vision.VisionThread;

public class Vision implements Updatable{

	private static final int IMG_WIDTH = 320,IMG_HEIGHT = 240;
	private VisionThread visionThread;
	private Object imgLock;
	private int centerX;
	private int centerY;
	private int savedX;
	private int savedY;

	public Vision()
	{
		UsbCamera camera = CameraServer.getInstance().startAutomaticCapture();
	    camera.setResolution(IMG_WIDTH, IMG_HEIGHT);
	    
	    visionThread = new VisionThread(camera, new GripPipelineC(), pipeline -> {
	        if (!pipeline.filterContoursOutput().isEmpty()) {
	            Rect r = Imgproc.boundingRect(pipeline.filterContoursOutput().get(0));
	            synchronized (imgLock) {
	                centerX = r.x + (r.width / 2);
	                centerY = r.y + (r.height/2);
	            }
	        }
	    });
	    visionThread.start();
	    
	    Updater.add(this, Priority.INPUT);
	}
	
	public void update()
	{
		synchronized(imgLock)
		{
			savedX=centerX;
			savedY=centerY;
		}
	}
	public int getX()
	{
		return savedX;
	}
	public int getY()
	{
		return savedY;
	}
}
