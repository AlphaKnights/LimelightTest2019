/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/
//dont dead open inside 
 package frc.robot;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import com.mach.LightDrive.*;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;


/**
 * This is the program to drive the 6695 2018 robot, codenamed TallBoi.
 */
public class Robot extends TimedRobot {
  private static final int kFrontLeftChannel = 1;
  private static final int kRearLeftChannel = 3;
  private static final int kFrontRightChannel = 2;
  private static final int kRearRightChannel = 4;
  private double fixedThrottle;
  private double xValue;
  private double yValue;
  private double zValue;

  NetworkTableEntry tx;
  NetworkTableEntry ty;
  NetworkTableEntry ta;

  private static final int kJoystickChannel = 0;
  private static final int xboxControllerChannel = 1;

  private MecanumDrive m_robotDrive;
  private Joystick m_stick;
  private XboxController m_xbox;
  
  private DoubleSolenoid MiddleSolonoid;
  private DoubleSolenoid RearSolonoid;
  private DoubleSolenoid FrontSolonoid;

  private boolean isSolonidDown;
  boolean xboxAButtonDown;
  boolean xboxBButtonDown;
  boolean xboxXButtonDown;

  @Override
  public void robotInit() {
  }

  @Override
  public void teleopInit() {
    WPI_TalonSRX frontLeft = new WPI_TalonSRX(kFrontLeftChannel);
    frontLeft.setInverted(true);
    WPI_TalonSRX rearLeft = new WPI_TalonSRX(kRearLeftChannel);
    WPI_TalonSRX frontRight = new WPI_TalonSRX(kFrontRightChannel);
    WPI_TalonSRX rearRight = new WPI_TalonSRX(kRearRightChannel);

    m_robotDrive = new MecanumDrive(frontLeft, rearLeft, frontRight, rearRight);
    m_stick = new Joystick(kJoystickChannel);
    m_xbox = new XboxController(xboxControllerChannel);
    m_stick.setThrottleChannel(3);

    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    tx = table.getEntry("tx");
    ty = table.getEntry("ty");
    ta = table.getEntry("ta");

    MiddleSolonoid = new DoubleSolenoid(0,1);
    RearSolonoid = new DoubleSolenoid(2,3);
    FrontSolonoid = new DoubleSolenoid(4, 5);
    xboxXButtonDown = false;
    xboxAButtonDown = false;
    xboxBButtonDown = false;

  }

  @Override
  public void teleopPeriodic() {
    double[] driveValues;
    double x = tx.getDouble(0.0);
    double y = ty.getDouble(0.0);
    double area = ta.getDouble(0.0);

    // Use the joystick X axis for lateral movement, Y axis for forward
    // movement, and Z axis for rotation.
    fixedThrottle = 1 - ((m_stick.getThrottle() + 1) / 2);

    if (m_stick.getRawButton(2)) {
      // Get the data from the limelight if the button is pressed
      driveValues = LimelightMethods.AutoAlign(x, y);

    } else if (isSolonidDown) {
      fixedThrottle = 0.15;
      // Drives with lower speed since a button is being pressed
      if (m_stick.getX() < 0) {
        xValue = -1 * (Math.pow(m_stick.getX(), 2) * fixedThrottle);
      } else {
        xValue = (Math.pow(m_stick.getX(), 2) * fixedThrottle);
      }
      if (m_stick.getY() < 0) {
        yValue = (Math.pow(m_stick.getY(), 2) * fixedThrottle);
      } else {
        yValue = -1 * (Math.pow(m_stick.getY(), 2) * fixedThrottle);
      }
      if (fixedThrottle != 0){
        if (m_stick.getZ() < 0) {
          zValue = -1 * Math.pow(m_stick.getZ(), 2) * 0.3;
        } else {
          zValue = Math.pow(m_stick.getZ(), 2) * 0.3;
        }
      }
      driveValues = new double[]{xValue, yValue, zValue};

    } else {
      // Drives manually
      if (m_stick.getX() < 0) {
        xValue = -1 * (Math.pow(m_stick.getX(), 2) * fixedThrottle);
      } else {
        xValue = (Math.pow(m_stick.getX(), 2) * fixedThrottle);
      }
      if (m_stick.getY() < 0) {
        yValue = (Math.pow(m_stick.getY(), 2) * fixedThrottle);
      } else {
        yValue = -1 * (Math.pow(m_stick.getY(), 2) * fixedThrottle);
      }
      if (fixedThrottle != 0){
        if (m_stick.getZ() < 0) {
          zValue = -1 * Math.pow(m_stick.getZ(), 2) * 0.3;
        } else {
          zValue = Math.pow(m_stick.getZ(), 2) * 0.3;
        }
      }
      // Make it so both methods of moving the robot use the same output :)
      driveValues = new double[]{xValue, yValue, zValue};
    }


    /* 
    * Controls the Robot's pistons. Sets isSolonoidDown to True if any solonoid in in the kForward position.
    */
    // Controlls the middle pistons
    if(m_xbox.getAButtonPressed()) {
      if(!xboxAButtonDown){
        MiddleSolonoid.set(DoubleSolenoid.Value.kForward);
        isSolonidDown = true;
        xboxAButtonDown = true;
      } else {
        MiddleSolonoid.set(DoubleSolenoid.Value.kReverse);
        isSolonidDown = false;
        xboxAButtonDown = false;
      }
    }

    // Controlls the rear pistons
    if(m_xbox.getBButtonPressed()) {
      if(!xboxBButtonDown){
        RearSolonoid.set(DoubleSolenoid.Value.kForward);
        isSolonidDown = true;
        xboxBButtonDown = true;
      } else {
        RearSolonoid.set(DoubleSolenoid.Value.kReverse);
        isSolonidDown = false;
        xboxBButtonDown = false;
      }
    }

    // Controlls the front pistons
    if(m_xbox.getXButtonPressed()) {
      if(!xboxXButtonDown){
        FrontSolonoid.set(DoubleSolenoid.Value.kForward);
        isSolonidDown = true;
        xboxXButtonDown = true;
      } else {
        FrontSolonoid.set(DoubleSolenoid.Value.kReverse);
        isSolonidDown = false;
        xboxXButtonDown = false;
      }
    }



    // Actually make the robot move; for reals?
    m_robotDrive.driveCartesian(driveValues[0], driveValues[1], driveValues[2]);

    // Put limelight values onto smart dashboard for debuging. oh ok
    SmartDashboard.putNumber("LimelightX", x);
    SmartDashboard.putNumber("LimelightY", y);
    SmartDashboard.putNumber("LimelightArea", area);

    m_xbox.setRumble(RumbleType.kLeftRumble, m_xbox.getTriggerAxis(Hand.kLeft));
    m_xbox.setRumble(RumbleType.kRightRumble, m_xbox.getTriggerAxis(Hand.kRight));
  }

  /*
   * Autonomus is target folowing right now
   *
   * Will follow the vision target, rotating to face the target
   */
  @Override
  public void autonomousInit() {
    WPI_TalonSRX frontLeft = new WPI_TalonSRX(kFrontLeftChannel);
    WPI_TalonSRX rearLeft = new WPI_TalonSRX(kRearLeftChannel);
    WPI_TalonSRX frontRight = new WPI_TalonSRX(kFrontRightChannel);
    WPI_TalonSRX rearRight = new WPI_TalonSRX(kRearRightChannel);

    m_robotDrive = new MecanumDrive(frontLeft, rearLeft, frontRight, rearRight);
    m_stick = new Joystick(kJoystickChannel);
    m_stick.setThrottleChannel(3);

    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    tx = table.getEntry("tx");
    ty = table.getEntry("ty");
    ta = table.getEntry("ta");
  }

  @Override
  public void autonomousPeriodic() {

    double x = tx.getDouble(0.0);
    double y = ty.getDouble(0.0);
    double area = ta.getDouble(0.0);

    if (x < -2) {
      zValue = -0.15;
    } else if (x > 2) {
      zValue = 0.15;
    } else {
      zValue = 0;
    }

    SmartDashboard.putNumber("LimelightArea", area);
    if (area > 2) {
      // Finding if somthing is close
      System.out.print("Object is close");

      yValue = 0.15;
    } else if (area < 1.5 && area > 0.1) {
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

  LightDrivePWM lightDrive;

  @Override
  public void testInit() {
    //redLine = new WPI_TalonSRX(5);
    
    Servo servo1 = new Servo(0);
		Servo servo2 = new Servo(1);
    lightDrive = new LightDrivePWM(servo1, servo2);
    m_stick = new Joystick(kJoystickChannel);
    m_stick.setThrottleChannel(3);

    m_xbox = new XboxController(xboxControllerChannel);

  }

  @Override
  public void testPeriodic() {
    m_xbox.setRumble(RumbleType.kLeftRumble, m_xbox.getTriggerAxis(Hand.kLeft));
    m_xbox.setRumble(RumbleType.kRightRumble, m_xbox.getTriggerAxis(Hand.kRight));

    if (m_stick.getRawButton(2)){
      lightDrive.SetColor(1, Color.RED);
    } else {
      lightDrive.SetColor(1, Color.BLUE);
    }
    }
}



