/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/
//dont dead open inside 
 package frc.robot;

import java.io.FileInputStream;
import java.io.InputStream;
import java.util.Properties;
import java.io.IOException;
import java.io.File;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import com.mach.LightDrive.*;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.cameraserver.CameraServer;
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
  private static final int kLiftMotor1 = 5;
  private static final int kLiftMotor2 = 6;
  private static final int kGrabberMotor = 7;


  private WPI_TalonSRX FrontLeft;
  private WPI_TalonSRX RearLeft;
  private WPI_TalonSRX FrontRight;
  private WPI_TalonSRX RearRight;
  private WPI_TalonSRX GrabberMotor;
  private WPI_TalonSRX LiftMotor1; 
  private WPI_TalonSRX LiftMotor2;

  private PowerDistributionPanel pdp;
  private ADIS16448_IMU gyro;

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

  private boolean isSolonoidDown;
  private boolean xboxAButtonDown;
  private boolean xboxBButtonDown;
  private boolean xboxXButtonDown;

  @Override
  public void robotInit() {
    try{
    File file = new File("version.properties");
    verson_properties = new FileInputStream(file);
    props.load(verson_properties);
    } catch (IOException io) {
      io.printStackTrace();
    }

    System.out.println("Build Number: " + props.getProperty("VERSION_BUILD"));
  }

  @Override
  public void teleopInit() {
    FrontLeft = new WPI_TalonSRX(kFrontLeftChannel);
    FrontLeft.setInverted(true);
    RearLeft = new WPI_TalonSRX(kRearLeftChannel);
    FrontRight = new WPI_TalonSRX(kFrontRightChannel);
    RearRight = new WPI_TalonSRX(kRearRightChannel);
    
    LiftMotor1 = new WPI_TalonSRX(kLiftMotor1);
    LiftMotor2 = new WPI_TalonSRX(kLiftMotor2);
    //LiftMotor2.follow(LiftMotor1);
    GrabberMotor = new WPI_TalonSRX(kGrabberMotor);

    GrabberMotor.configContinuousCurrentLimit(10);
    GrabberMotor.configPeakCurrentLimit(10);
    //GrabberMotor.configPeakCurrentDuration(500);
    GrabberMotor.enableCurrentLimit(true);
  
    pdp = new PowerDistributionPanel();

    m_robotDrive = new MecanumDrive(FrontLeft, RearLeft, FrontRight, RearRight);
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

    gyro = new ADIS16448_IMU();
    gyro.calibrate();
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

    } else if (isSolonoidDown) {
      fixedThrottle = fixedThrottle * 0.4;
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


    if(m_xbox.getBumper(Hand.kLeft)){
      LiftMotor1.set(-(m_xbox.getY(Hand.kLeft)));
      LiftMotor2.set(-(m_xbox.getY(Hand.kLeft)));
    } else {
      LiftMotor1.set(0);
      LiftMotor2.set(0);
    }

    if(m_xbox.getTriggerAxis(Hand.kLeft) > 0.1){
      GrabberMotor.set(m_xbox.getTriggerAxis(Hand.kLeft) * 0.5);
    } else if (m_xbox.getTriggerAxis(Hand.kRight) >= 0.1){
      GrabberMotor.set(-(m_xbox.getTriggerAxis(Hand.kRight)) * 0.5);
    } else {
      GrabberMotor.set(-0.1);
    }
    //GrabberMotor.set(m_xbox.getTriggerAxis(Hand.kLeft));
    //GrabberMotor.set(-(m_xbox.getTriggerAxis(Hand.kRight)));



//hmmmmm oof


    
    /* 
    * Controls the Robot's pistons. Sets isSolonoidDown to True if any solonoid in in the kForward position.
    */
    // Controlls the middle pistons
    if(m_xbox.getAButtonPressed()) {
      if(!xboxAButtonDown){
        MiddleSolonoid.set(DoubleSolenoid.Value.kForward);
        isSolonoidDown = true;
        xboxAButtonDown = true;
      } else {
        MiddleSolonoid.set(DoubleSolenoid.Value.kReverse);
        isSolonoidDown = false;
        xboxAButtonDown = false;
      }
    }

    // Controlls the rear pistons
    if(m_xbox.getBButtonPressed()) {
      if(!xboxBButtonDown){
        RearSolonoid.set(DoubleSolenoid.Value.kForward);
        isSolonoidDown = true;
        xboxBButtonDown = true;
      } else {
        RearSolonoid.set(DoubleSolenoid.Value.kReverse);
        isSolonoidDown = false;
        xboxBButtonDown = false;
      }
    }

    // Controlls the front pistons
    if(m_xbox.getXButtonPressed()) {
      if(!xboxXButtonDown){
        FrontSolonoid.set(DoubleSolenoid.Value.kForward);
        isSolonoidDown = true;
        xboxXButtonDown = true;
      } else {
        FrontSolonoid.set(DoubleSolenoid.Value.kReverse);
        isSolonoidDown = false;
        xboxXButtonDown = false;
      }
    }



    // Actually make the robot move; for reals?
    m_robotDrive.driveCartesian(driveValues[0], driveValues[1], driveValues[2]);

    // Put limelight values onto smart dashboard for debuging. oh ok
    SmartDashboard.putNumber("LimelightX", x);
    SmartDashboard.putNumber("LimelightY", y);
    SmartDashboard.putNumber("LimelightArea", area);
    SmartDashboard.putNumber("GrabberCurrentDraw", GrabberMotor.getOutputCurrent());
    SmartDashboard.putData(m_robotDrive);
    SmartDashboard.putData(gyro);

    SmartDashboard.putNumber("Gyro-X", gyro.getAngleX());

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

  @Override
  public void disabledInit() {
    pdp = new PowerDistributionPanel();
  }

  @Override
  public void disabledPeriodic() {
    super.disabledPeriodic();
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
    CameraServer.getInstance().startAutomaticCapture();

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



