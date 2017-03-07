package org.usfirst.frc.team555.robot;

import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;

import org.montclairrobotics.sprocket.loop.Priority;
import org.montclairrobotics.sprocket.loop.Updatable;
import org.montclairrobotics.sprocket.loop.Updater;
import org.montclairrobotics.sprocket.utils.Debug;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Rect;
import org.opencv.imgproc.Imgproc;

import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.wpilibj.vision.VisionThread;

public class Vision implements Updatable{

	private UsbCamera camera;
	private VisionThread visionThread;
	private Object imgLock=new Object();
	private double dist, x, y;
	private double savedDist, savedX, savedY;

	public Vision(UsbCamera camera)
	{
	    
	    visionThread = new VisionThread(camera, new RedGripD(), pipeline -> {
	    	double d = -1;
	    	double x = -1;
	    	double y = -1;
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
	            Rect b = Imgproc.boundingRect(contours.get(1));
	            double distx = Math.pow((a.x + a.width/2) - (b.x + b.width/2), 2);
	            double disty = Math.pow((a.y + a.height/2) - (b.y + b.height/2), 2);
	            d = Math.sqrt(distx + disty);
	            
	            x = (a.x + b.x)/2 + (a.width + b.width) / 4;
                y = (a.y + b.y)/2 + (a.height + b.height) / 4;
                x=a.x+a.width/2;
                Debug.msg("Vision X", x);
                Debug.msg("Vision Y", a.y+a.height/2);
                Debug.msg("Vision Width", a.width);
                Debug.msg("Vision Height", a.height);
	            Debug.msg("Vision B X", b.x);
	            Debug.msg("Vision B Y", b.y + b.height/2);
	            Debug.msg("Vision B Width", b.width);
	            Debug.msg("Vision B Height", b.height);
	            Debug.num("Dist btwn", d);
	            
	        }
	        synchronized (imgLock) {
                dist = d;
                this.x = x;
                this.y = y;
            }
	    });
	    visionThread.start();
	    
	    Updater.add(this, Priority.INPUT);
	}
	
	public void update()
	{
		synchronized(imgLock)
		{
			savedDist=dist;
			savedX = x;
			savedY = y;
		}
		//SmartDashboard.putNumber("x", savedX);
	}
	public double getDist() {
		return savedDist;
	}
	
	public double getX() {
		return savedX;
		}

	public double getY() {
		return savedY;
	}


	public void stop()
	{
		visionThread.interrupt();
	}
}
