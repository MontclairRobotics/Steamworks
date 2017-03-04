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
	private double dist;
	private double savedDist;

	public Vision(UsbCamera camera)
	{
	    
	    visionThread = new VisionThread(camera, new RedGripD(), pipeline -> {
	    	double d = 0.0;
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
	            double x = Math.pow((a.x + a.width/2) - (b.x + b.width/2), 2);
	            double y = Math.pow((a.y + a.height/2) - (b.y + b.height/2), 2);
	            d = Math.sqrt(x + y);
	            
	            Debug.num("Dist btwn", d);
	            
	        }
	        else
	        {
	        	d=-1;
	        }
	        synchronized (imgLock) {
                dist = d;
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
		}
		//SmartDashboard.putNumber("x", savedX);
	}
	public double getDist() {
		return savedDist;
	}
	
	public void stop()
	{
		visionThread.interrupt();
	}
}
