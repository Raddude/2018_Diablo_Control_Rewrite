package org.usfirst.frc.team1787.robot.vision;

import edu.wpi.cscore.CvSink;
import edu.wpi.cscore.CvSource;
import edu.wpi.cscore.UsbCamera;
import edu.wpi.cscore.VideoSink;
import edu.wpi.cscore.VideoCamera.WhiteBalance;
import edu.wpi.first.wpilibj.CameraServer;

public class JakeCamera {

	private CameraServer camServer = CameraServer.getInstance();
	private CvSink topCamFrame;
	private CvSink botCamFrame;
	private CvSource outputStream;
	
	private UsbCamera topCam = new UsbCamera("topCam", 0);
	private UsbCamera botCam = new UsbCamera("botCam", 1);
	
	private final int IMAGE_WIDTH_PIXELS = 160;
	private final int IMAGE_HEIGHT_PIXELS = 120;
	
	private static final JakeCamera instance = new JakeCamera();
	
	private void CameraController() {
	  
		
		
	    
	    /* Configure settings like resolution, exposure, white balance, etc. */
	    //configCam(topCam); // <- "true" indicates cam will be used for image processing
	    //configCam(botCam);
	    
	    // used to push processed frames to the dashboard for viewing.
	    //outputStream = camServer.putVideo("Custom Camera Stream", IMAGE_WIDTH_PIXELS, IMAGE_HEIGHT_PIXELS);
	}
	
	public void updateCams() {
		//topCamFrame = camServer.
		//botCamFrame = camServer.getVideo("botCam");
	}
	
	
	
	public void switchToTopCam() {
		//outputStream = camServer.putVideo(name, width, height)
	}
	
	public void switchToBottomCam() {
		
	}
	
	
	
	public void configCam(UsbCamera cam) {
	    cam.setResolution(IMAGE_WIDTH_PIXELS, IMAGE_HEIGHT_PIXELS);
	    cam.setFPS(30);
	    cam.setExposureAuto();
	    cam.setBrightness(50);
	    cam.setWhiteBalanceAuto();
	    
	  }
	
	public static JakeCamera getInstance() {
		return instance;
	}
}
