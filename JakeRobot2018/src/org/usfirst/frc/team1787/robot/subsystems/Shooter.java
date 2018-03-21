package org.usfirst.frc.team1787.robot.subsystems;



import edu.wpi.first.wpilibj.Joystick;

/**
 * The shooter class is composed of the turret, the flywheel, and the feeder.
 * This class serves to coordinate those mechanisms.
 */
public class Shooter {

  // Sub-Mechanisms
  private Turret turret = Turret.getInstance();
  private Flywheel flywheel = Flywheel.getInstance();
  private Feeder feeder = Feeder.getInstance();
  
  // Vision

  // Singleton Instance
  private static Shooter instance;

  private Shooter() {
    // Intentionally left blank; no initialization needed.
  }
  
  public void enablePIDControllers() {
    turret.getPIDController().enable();
    flywheel.getPIDController().enable();
  }
  
  public boolean pidIsEnabled() {
    return turret.getPIDController().isEnabled() ||
           flywheel.getPIDController().isEnabled();
  }
  

 
  
  public void zeroSensors() {
    turret.zeroSensors();
    flywheel.zeroSensors();
  }
  
  public void manualControl(double turretMoveValue, double flywheelMoveValue, double feederMoveValue) {
    //turret.manualControl(turretMoveValue);
    flywheel.manualControl(flywheelMoveValue);
    feeder.spin(feederMoveValue);
  }
  
  public void manualControl(Joystick stick) {
    turret.manualControl(stick.getX());
    flywheel.manualControl(stick.getY());
    if (stick.getTrigger()) {
      feeder.spin(feeder.DEFAULT_FEEDER_SPEED);
    } else {
      feeder.stop();
    }
  }
  
  public void stop() {
    turret.stop();
    flywheel.stop();
    feeder.stop();
  }
  
  public void publishDataToSmartDash() {
    turret.publishDataToSmartDash();
    flywheel.publishDataToSmartDash();
    feeder.publishDataToSmartDash();
  }
  
  public static Shooter getInstance() {
	if (instance == null) {
      instance = new Shooter();
	}
    return instance;
  }
}
