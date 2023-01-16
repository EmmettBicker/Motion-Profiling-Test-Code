// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.TimedRobot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

// Talons
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import edu.wpi.first.wpilibj.XboxController;
/**
 * 
 */
public class RobotFalcons extends TimedRobot {

  private TalonSRX mTalon;
 

  private int desiredPosition;

  /**
   * 
   */
  @Override
  public void robotInit() {
    int port = 0;
    mTalon = new TalonSRX(port);

    // What is a PID index?
    mTalon.configSelectedFeedbackSensor(TalonSRXFeedbackDevice.IntegratedSensor, 0,0);
    

    // Constants defined in example 

    mPID.setP(5e-5);
    mPID.setI( 1e-6);
    mPID.setD(0);
    mPID.setIZone(0);
    mPID.setFF(0.000156);
    mPID.setOutputRange(-1,1);

    // Smart motion is the PID over a longer period of time, which allows
    // a maxiumum velocity and maximum acceleration to make a gradual movement

    int smartMotionSlot = 0; //What??
    mPID.setSmartMotionMaxVelocity(2000, smartMotionSlot); // max rpm
    mPID.setSmartMotionMinOutputVelocity(0, 0); // min rpm
    mPID.setSmartMotionMaxAccel(1500,smartMotionSlot); // 2 rpm ^ 2 
    mPID.setSmartMotionAllowedClosedLoopError(0.05,0); // Allowed distance from desired goal in rotations
     
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {

    desiredPosition = 5000; // rpm 

    if (mController.getBButton())
    {
      desiredPosition = (-1) * desiredPosition;
    }

    mPID.setReference(desiredPosition, CANSparkMax.ControlType.kSmartMotion);


  }

  /**
   * This autonomous (along with the chooser code above) shows how to select between different
   * autonomous modes using the dashboard. The sendable chooser code works with the Java
   * SmartDashboard. If you prefer the LabVIEW Dashboard, remove all of the chooser code and
   * uncomment the getString line to get the auto name from the text box below the Gyro
   *
   * <p>You can add additional auto modes by adding additional comparisons to the switch structure
   * below with additional strings. If using the SendableChooser make sure to add them to the
   * chooser code above as well.
   */
  @Override
  public void autonomousInit() {

  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {

  
  }

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {}

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {}

  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {}

  /** This function is called periodically when disabled. */
  @Override
  public void disabledPeriodic() {}

  /** This function is called once when test mode is enabled. */
  @Override
  public void testInit() {}

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}
}
