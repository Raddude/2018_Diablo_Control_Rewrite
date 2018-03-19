package org.usfirst.frc.team1787.robot;

import java.util.ArrayList;

import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Scalar;
import org.usfirst.frc.team1787.robot.auto.AutoMethods;
import org.usfirst.frc.team1787.robot.subsystems.DriveTrain;
import org.usfirst.frc.team1787.robot.subsystems.Flywheel;
import org.usfirst.frc.team1787.robot.subsystems.Intake;
import org.usfirst.frc.team1787.robot.subsystems.Shooter;
import org.usfirst.frc.team1787.robot.subsystems.Turret;
import org.usfirst.frc.team1787.robot.subsystems.Winch;
import org.usfirst.frc.team1787.robot.vision.CameraController;
import org.usfirst.frc.team1787.robot.vision.ImageProcessor;
import org.usfirst.frc.team1787.robot.vision.JakeCamera;

import edu.wpi.cscore.CvSink;
import edu.wpi.cscore.CvSource;
import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class Robot extends TimedRobot {
  // Don't Ask
  protected int farfar37;
  
  // Controls
  private final int RIGHT_JOYSTICK_ID = 0;
  private final int LEFT_JOYSTICK_ID = 1;
  private Joystick rightStick = new Joystick(RIGHT_JOYSTICK_ID);
  private Joystick leftStick = new Joystick(LEFT_JOYSTICK_ID);
  private final int JOYSTICK_TWIST_AXIS = 2;
  private final int JOYSTICK_SLIDER_AXIS = 3;
  
  // Button Map
  private final int DEPLOY_ARM_BUTTON = 3;
  private final int RETRACT_ARM_BUTTON = 4;
  private final int INTAKE_BUTTON = 1;
  private final int EXPELL_BUTTON = 2;
  private final int INTAKE_CAMERA_BUTTON = 3;
  private final int TOP_CAMERA_BUTTON = 4;
  
  private final int HIGH_POWER_SHOOTING_BUTTON = 10;
  private final int BALANCED_POWER_SHOOTING_BUTTON = 9;
  private final int SWITCH_POWER_SHOOTING_BUTTON = 8;
  private final int REVERSE_SHOOTER_BUTTON = 2;
  
  private final int WINCH_CLIMB_FORWARD_BUTTON = 13;
  private final int WINCH_CLIMB_REVERSE_BUTTON = 14;

  private final int TOGGLE_SHOOTER_CONTROL_BUTTON = 2;
  
  private final int TOGGLE_ACTIVE_CAMERA_BUTTON = 10;
  
  private final int TOGGLE_TUNING_MODE_ENABLED_BUTTON = 14;
  private final int CHANGE_CURRENT_TUNING_MODE_BUTTON = 15;
  
  // Control State Variables
  
  /** Determines which mode the shooter is in.
   * 0 = manual control,
   * 1 = automatic target tracking & shooting */
  private int shooterControlMode = 0;
  
  /** If tuning mode is active, this variable 
   * determines what exactly is being tuned */
  private int tuningMode = 0;
  private boolean tuningModeActive = false;
  
  // the frame published to the SmartDash stream
  private Mat outputFrame = new Mat();
  // where to get the image to publish to the SmartDash
  private int selectedStreamingSource = 0;
  
  // Instances of Subsystems
  private DriveTrain driveTrain = DriveTrain.getInstance();
  private Intake pickupArm = Intake.getInstance();
  private Shooter shooter = Shooter.getInstance();
  private Winch winch = Winch.getInstance();
  private CameraController camController = CameraController.getInstance();
  private ImageProcessor imgProcessor = ImageProcessor.getInstance();
  private AutoMethods auto = AutoMethods.getInstance();
  
  /* These subsystems are normally controlled collectively through the Shooter class,
   * but they are included here individually to tune PID loops for each component
   * separately. */
  private Flywheel flywheel = Flywheel.getInstance();
  private Turret turret = Turret.getInstance();
  
  // Preferences (interface used to get values from the SmartDash)
  Preferences prefs = Preferences.getInstance();
  
  
  //Auto Code
  SendableChooser<Integer> autoChooser;
  String gameData;
  private int autonomousTimer;
  
  
  
  
  
  
  
  
  //Camera code
  CameraServer server = CameraServer.getInstance();
  private JakeCamera jakeCamera = JakeCamera.getInstance();
  
  private UsbCamera topCam = new UsbCamera("topCam", 0);
  private UsbCamera botCam = new UsbCamera("botCam", 1);
  
  private final int IMAGE_WIDTH_PIXELS = 160;
  private final int IMAGE_HEIGHT_PIXELS = 120;
  CvSink cvSink;
  CvSource cvSource;
  Mat currentFrame;
  
  
  
  
  
  
  
  
  
  /* ----------------------------------------------------------------
   * Member variables end here; only functions below this point!
   * ---------------------------------------------------------------- */
  
  /** This function is run once when the robot is first started up
   *  and should be used  for any initialization code. */
  @Override
  public void robotInit() {
	  // Default Period is 0.02 seconds per loop.
	  this.setPeriod(0.02);
	  
	  autoChooser = new SendableChooser<Integer>();
	  autoChooser.addDefault("Move Straight", 1);
	  autoChooser.addObject("Do Nothing", 0);
	  autoChooser.addObject("Short/Left Side", 2);
	  autoChooser.addObject("Long/Right Side", 3);
	  SmartDashboard.putData("Auto Chooser", autoChooser);
	  
	  
	  topCam.setResolution(IMAGE_WIDTH_PIXELS, IMAGE_HEIGHT_PIXELS);
	  topCam.setFPS(15);
	  topCam.setExposureAuto();
	  topCam.setBrightness(50);
	  topCam.setWhiteBalanceAuto();
	  
	  botCam.setResolution(IMAGE_WIDTH_PIXELS, IMAGE_HEIGHT_PIXELS);
	  botCam.setFPS(15);
	  botCam.setExposureAuto();
	  botCam.setBrightness(50);
	  botCam.setWhiteBalanceAuto();
	  
	  
	  
	  /*
	   * TODO:
	   * 4) finish going through all talon config features
	   * 5) practice working with a sensor that's attatched to the talon
	   * 6) figure out if it's possible to use a gyro with a talon / investigate Pidgeon IMU
	   * 7) Test out using the follow feature with talons
	   * 8) See what code structure would be like if you have the talons control pretty much everything
	   * 9) actually use TalonConfigurer everywhere.
	   * 
	   * 9) Review multi-threading for img processing
	   * 10) review img processing code and see if cleanup is necessary
	   * 
	   * 11) Review what's up with the new network tables protocols
	   * 12) Finish setting up the dashboard so that all values show
	   * 
	   * 13) make arcadeDrive consistent with what I learned in robotics + fix documentation.
	   * 
	   * 14) double check it's ok to remove all instances of CustomPIDController.
	   * 15) Replace all instances of CustomPIDController if this is the case.
	   */
  }
  
  /* While the robot is on, it can be in one of the following modes at a time:
   * 1) teleop (driver control from driver station)
   * 2) autonomous (no driver control, fully autonomous)
   * 3) test mode (intended to be helpful for testing, but I find it annoying)
   * 4) disabled (robot does nothing)
   * 
   * Each of these modes have an associated init() function
   * and an associated periodic() function. The init() function is
   * run once upon entering a given mode, then the periodic() function is looped
   * until a different mode is entered.
   * 
   * For example: Changing the current mode from 
   * disabled to teleop will cause teleopInit() to be run once, and then teleopPeriodic()
   * to be looped until a different mode is entered.
   */

  
  
  
  
  public void autonomousInit() {
	    autonomousTimer = 0;
  }

  public void autonomousPeriodic() {
	  
    gameData = DriverStation.getInstance().getGameSpecificMessage();
    
    if (gameData.length() > 0) {
    	//Short/Left is 2, Long/Right is 3
    	if (gameData.charAt(0) == 'L' && autoChooser.getSelected() == 2) {
    		auto.auto2(autonomousTimer);
    	}
    	else if (gameData.charAt(1) == 'L' && autoChooser.getSelected() == 2) {
    		auto.auto3(autonomousTimer);
    	}
    	else if (gameData.charAt(0) == 'R' && autoChooser.getSelected() == 3) {
    		auto.auto4(autonomousTimer);
    	}
    	else if (gameData.charAt(1) == 'R' && autoChooser.getSelected() == 3) {
    		auto.auto5(autonomousTimer);
    	}
    	else {
    		auto.auto0(autonomousTimer);
    	}
    	
    	autonomousTimer++;
    	
    	
    }
    
    cvSink.setSource(topCam);
	cvSink.grabFrame(currentFrame);
    cvSource.putFrame(currentFrame);
  }
  
  
  
  
  public void teleopInit() {
	  cvSink.setSource(botCam);
  }

  public void teleopPeriodic() {
	  
	  
	  if (rightStick.getRawButtonPressed(3)) {
		  cvSink.setSource(topCam);
			
	  }
	  else if (rightStick.getRawButtonPressed(4)) {
		  cvSink.setSource(botCam);
	  }
	  
	  cvSink.grabFrame(currentFrame);
	  cvSource.putFrame(currentFrame);
	  
	  
	  
    // Driving
    driveTrain.arcadeDrive(-rightStick.getY(), rightStick.getX());
	  //driveTrain.tryCurveDrive(rightStick.getY(), -rightStick.getX(), rightStick.getRawButton(EXPELL_BUTTON));

    // Gear Shifter
    if (leftStick.getRawAxis(JOYSTICK_SLIDER_AXIS) < 0) {
      driveTrain.setGear(driveTrain.HIGH_GEAR);
    }
    
    else {
      driveTrain.setGear(driveTrain.LOW_GEAR);
    }
    
    
    //Pickup arm lift
    if (leftStick.getRawButton(DEPLOY_ARM_BUTTON)) {
        pickupArm.moveArm(pickupArm.DEPLOY);
    } else if (rightStick.getRawButton(RETRACT_ARM_BUTTON)) {
      pickupArm.moveArm(pickupArm.RETRACT);
    }
    
    
    // Pickup Wheels
    if (rightStick.getRawButton(INTAKE_BUTTON) || leftStick.getRawButton(INTAKE_BUTTON)) {
      pickupArm.spinIntake(pickupArm.DEFAULT_INTAKE_SPEED);
    } else if (rightStick.getRawButton(EXPELL_BUTTON)) {
      pickupArm.spinIntake(-1 * pickupArm.DEFAULT_INTAKE_SPEED);
    } else {
      pickupArm.spinIntake(0);
    }
    
    
    //Climb
    if (rightStick.getRawButton(WINCH_CLIMB_FORWARD_BUTTON)) {
      winch.spin(winch.DEFAULT_CLIMB_SPEED);
    } else if (rightStick.getRawButton(WINCH_CLIMB_REVERSE_BUTTON)) {
    	winch.spin(winch.DEFAULT_DECEND_SPEED);
    } else {
      winch.spin(0);
    }
    
    
    //Turret
    if (rightStick.getRawAxis(JOYSTICK_TWIST_AXIS) > 0.1) {
    	turret.manualControl(0.75);
    }
    else if (rightStick.getRawAxis(JOYSTICK_TWIST_AXIS) < -0.1) {
    	turret.manualControl(-0.75);
    }
    else {
    	turret.manualControl(0);
    }
    
    //Shooter
    if (rightStick.getRawButton(HIGH_POWER_SHOOTING_BUTTON)) {
    	shooter.manualControl(0, 1, 0.5);
    }
    else if (rightStick.getRawButton(BALANCED_POWER_SHOOTING_BUTTON)) {
    	shooter.manualControl(0, 0.66,0.5);
    }
    else if (rightStick.getRawButton(SWITCH_POWER_SHOOTING_BUTTON)) {
    	shooter.manualControl(0, 0.33, 0.5);
    }
    else if (leftStick.getRawButton(REVERSE_SHOOTER_BUTTON)) {
    	shooter.manualControl(0, -0.25, -0.5);
    }
    else {
    	shooter.manualControl(0, 0, 0);
    }
    
    //Switch cams
    /*
    if (rightStick.getRawButtonPressed(INTAKE_CAMERA_BUTTON)) {
    	server.startAutomaticCapture(0);
    }
    else if (rightStick.getRawButtonPressed(TOP_CAMERA_BUTTON)) {
    	server.startAutomaticCapture(1);
    }
    */
  }
  
  
  
  
  
  public void disabledInit() {
    shooter.stop();
    autonomousTimer = 0;
  }
	  
  public void disabledPeriodic() {
		  
  } 
  
  public void testInit() {
    
  }

  public void testPeriodic() {
    
  }
}