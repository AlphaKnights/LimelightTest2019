/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;



/**
 * This is a demo program showing how to use Mecanum control with the RobotDrive
 * class.
 */
public class Robot extends TimedRobot {
  private static final int kFrontLeftChannel = 1;
  private static final int kRearLeftChannel = 2;
  private static final int kFrontRightChannel = 4;
  private static final int kRearRightChannel = 3;
  private double fixedThrottle;
  private double xValue;
  private double yValue;
  private double zValue;

  NetworkTableEntry tx;
  NetworkTableEntry ty;
  NetworkTableEntry ta;


  private static final int kJoystickChannel = 0;
  //private static final int xboxControllerChannel = 1;

  private MecanumDrive m_robotDrive;
  private Joystick m_stick;

  @Override
  public void robotInit() {
    
  }

  @Override
  public void teleopInit() {
    WPI_TalonSRX frontLeft = new WPI_TalonSRX(kFrontLeftChannel);
    WPI_TalonSRX rearLeft = new WPI_TalonSRX(kRearLeftChannel);
    WPI_TalonSRX frontRight = new WPI_TalonSRX(kFrontRightChannel);
    WPI_TalonSRX rearRight = new WPI_TalonSRX(kRearRightChannel);

    // Invert the left side motors.
    // You may need to change or remove this to match your robot.
    // frontLeft.setInverted(true);
    // rearLeft.setInverted(true);

    m_robotDrive = new MecanumDrive(frontLeft, rearLeft, frontRight, rearRight);
    m_stick = new Joystick(kJoystickChannel);
    m_stick.setThrottleChannel(3);

    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    tx = table.getEntry("tx");
    ty = table.getEntry("ty");
    ta = table.getEntry("ta");
  }
//chezburgers...oof...slurp, noice
  @Override
  public void teleopPeriodic() {
    // Use the joystick X axis for lateral movement, Y axis for forward
    // movement, and Z axis for rotation.
    fixedThrottle = 1 - ((m_stick.getThrottle() + 1) / 2);
    
    if(m_stick.getX() < 0){
     xValue = -1 * (Math.pow(m_stick.getX(),2)  * fixedThrottle);
    } else {
      xValue = (Math.pow(m_stick.getX(),2)  * fixedThrottle);
    }
    if(m_stick.getY() < 0){
      yValue = (Math.pow(m_stick.getY(),2)  * fixedThrottle);
     } else {
       yValue = -1 * (Math.pow(m_stick.getY(),2)  * fixedThrottle);
     }
     if(m_stick.getZ() < 0){
      zValue = -1 * (Math.pow(m_stick.getZ(),2)  * fixedThrottle);
     } else {
       zValue = (Math.pow(m_stick.getZ(),2)  * fixedThrottle);
     }
    m_robotDrive.driveCartesian(xValue, yValue, zValue);

    double x = tx.getDouble(0.0);
    double y = ty.getDouble(0.0);
    double area = ta.getDouble(0.0);
    
    SmartDashboard.putNumber("LimelightX", x);
    SmartDashboard.putNumber("LimelightY", y);
    SmartDashboard.putNumber("LimelightArea", area);
  }

  @Override
  public void autonomousInit() {
    WPI_TalonSRX frontLeft = new WPI_TalonSRX(kFrontLeftChannel);
    WPI_TalonSRX rearLeft = new WPI_TalonSRX(kRearLeftChannel);
    WPI_TalonSRX frontRight = new WPI_TalonSRX(kFrontRightChannel);
    WPI_TalonSRX rearRight = new WPI_TalonSRX(kRearRightChannel);

    // Invert the left side motors.
    // You may need to change or remove this to match your robot.
    // frontLeft.setInverted(true);
    // rearLeft.setInverted(true);

    m_robotDrive = new MecanumDrive(frontLeft, rearLeft, frontRight, rearRight);
    m_stick = new Joystick(kJoystickChannel);
    m_stick.setThrottleChannel(3);

    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    tx = table.getEntry("tx");
    ty = table.getEntry("ty");
    ta = table.getEntry("ta");
  }
//chezburgers...oof
  @Override
  public void autonomousPeriodic() {
    double x = tx.getDouble(0.0);
    double y = ty.getDouble(0.0);
    double area = ta.getDouble(0.0);
    

    if (x < -2){
      zValue = -0.15;
    }
    else if (x > 2){
      zValue = 0.15;
    }
    else {
      zValue = 0;
    }

    SmartDashboard.putNumber("LimelightArea", area);
    if (area > 2){
      // Finding if somthing is close
      System.out.print("Object is close");

      yValue = 0.15;
    }
    else if (area < 1.5 && area > 0.1) {
      // Finding if somthing is far
      System.out.print("Object is far");
      yValue = -((-0.25 * area) + 0.5);
    } else {
      yValue = 0;
    }

    xValue = 0;

    m_robotDrive.driveCartesian(xValue, yValue, zValue);
    
    SmartDashboard.putNumber("LimelightX", x);
    SmartDashboard.putNumber("LimelightY", y);
    SmartDashboard.putNumber("Y Moving Value", yValue);
    SmartDashboard.putNumber("X Moving Value", xValue);
  }

  @Override
  public void testInit() {

  }
  
  @Override
  public void testPeriodic() {
    

    /*if(x != 0){
      xValue -= x;
    }
    */
    /*if(x != 0){
    m_robotDrive.driveCartesian(ySpeed, xSpeed, zRotation);
    }
    */
    }
  }
