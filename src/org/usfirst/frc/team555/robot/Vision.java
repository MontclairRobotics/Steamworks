package org.usfirst.frc.team555.robot;

import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;

import org.montclairrobotics.sprocket.loop.Priority;
import org.montclairrobotics.sprocket.loop.Updatable;
import org.montclairrobotics.sprocket.loop.Updater;
import org.opencv.core.MatOfPoint;
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

	private UsbCamera camera;
	private VisionThread visionThread;
	private Object imgLock=new Object();
	private int centerX;
	private int centerY;
	private int savedX;
	private int savedY;

	public Vision(UsbCamera camera)
	{
	    
	    visionThread = new VisionThread(camera, new GripPipelineC(), pipeline -> {
	    	int x,y;
	    	ArrayList<MatOfPoint> contours = pipeline.filterContoursOutput();
	        if (contours.size()>0) {
	        	Collections.sort(contours, new Comparator<MatOfPoint>() {
					@Override
					public int compare(MatOfPoint o1, MatOfPoint o2) {
						if(Imgproc.boundingRect(o1).area() >= Imgproc.boundingRect(o2).area()) {
							return 1;
						} else {
							return -1;
						}
					}
		    	});
	            Rect a = Imgproc.boundingRect(contours.get(0));
	            //Rect b = Imgproc.boundingRect(contours.get(1));
	            Rect b=a;
                x = (a.x + b.x)/2 + (a.width + b.width) / 4;
                y = (a.y + b.y)/2 + (a.height + b.height) / 4;
	            
	        }
	        else
	        {
	        	x=320/2;
	        	y=240/2;
	        }
	        synchronized (imgLock) {
                centerX = x;
                centerY = y;
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
		SmartDashboard.putNumber("x", savedX);
	}
	public int getX()
	{
		return savedX;
	}
	public int getY()
	{
		return savedY;
	}
	
	public void stop()
	{
		visionThread.interrupt();
	}
}
