/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import java.io.FileInputStream;
import java.io.InputStream;
import java.util.Properties;
import java.io.IOException;
import java.io.File;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.NeutralMode;

import com.mach.LightDrive.*;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
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


  private double fixedTranslationThrottle;
  private double fixedRotationThrottle;
  private double xValue;
  private double yValue;
  private double zValue;

  NetworkTableEntry tx;
  NetworkTableEntry ty;
  NetworkTableEntry ta;
  NetworkTableEntry camtran;

  private static final int kTranslateStickChannel = 0;
  private static final int kRotateStickChannel = 1;
  private static final int xboxControllerChannel = 2;

  private MecanumDrive m_robotDrive;
  private Joystick m_translateStick;
  private Joystick m_rotateStick;
  private XboxController m_xbox;
  
  private DoubleSolenoid MiddleSolonoid;
  private DoubleSolenoid RearSolonoid;
  private DoubleSolenoid FrontSolonoid;
  private DoubleSolenoid GrabberSolonoid;

  private DigitalInput UpperGliftSwitch;
  private DigitalInput LowerGliftSwitch;
  
  private PowerDistributionPanel pdp;

  private boolean xboxAButtonDown;
  private boolean xboxBButtonDown;
  private boolean xboxXButtonDown;
  private boolean xboxYButtonDown;

  private Properties props;
  private InputStream verson_properties;

  private Timer grabberTimer;

  private UsbCamera upperCamera;
  private UsbCamera lowerCamera;
  private CameraServer camServer;

  @Override
  public void robotInit() {
    try{
    File file = new File("version.properties");
    verson_properties = new FileInputStream(file);
    props.load(verson_properties);
    System.out.println("Build Number: " + props.getProperty("VERSION_BUILD"));

    } catch (IOException io) {
      io.printStackTrace();
    }  

    camServer = CameraServer.getInstance();
    upperCamera = camServer.startAutomaticCapture("UpperCam", 0);
    lowerCamera = camServer.startAutomaticCapture("LowerCam", 1);
    upperCamera.setExposureAuto();
    lowerCamera.setExposureAuto();
    upperCamera.setFPS(15);
    lowerCamera.setFPS(15);
    //upperCamera.setResolution(568, 320);
  }

  @Override
  public void teleopInit() {
    FrontLeft = new WPI_TalonSRX(3);
    FrontLeft.setInverted(true);
    RearLeft = new WPI_TalonSRX(1);
    FrontRight = new WPI_TalonSRX(4);
    RearRight = new WPI_TalonSRX(2);
    
    LiftMotor1 = new WPI_TalonSRX(kLiftMotor1);
    LiftMotor2 = new WPI_TalonSRX(kLiftMotor2);
    GrabberMotor = new WPI_TalonSRX(kGrabberMotor);

    GrabberMotor.configContinuousCurrentLimit(10);
    GrabberMotor.configPeakCurrentLimit(10);
    //GrabberMotor.configPeakCurrentDuration(500);
    GrabberMotor.enableCurrentLimit(true);
  
    if (UpperGliftSwitch == null){
      UpperGliftSwitch = new DigitalInput(1);
    }
    if (LowerGliftSwitch == null){
      LowerGliftSwitch = new DigitalInput(0);
    }

    // Initializes the drivetrain
    m_robotDrive = new MecanumDrive(FrontLeft, RearLeft, FrontRight, RearRight);

    // Sets up the control inputs
    m_translateStick = new Joystick(kTranslateStickChannel);
    m_rotateStick = new Joystick(kRotateStickChannel);
    m_xbox = new XboxController(xboxControllerChannel);
    m_translateStick.setThrottleChannel(3);
    m_rotateStick.setThrottleChannel(3);

    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    camtran = table.getEntry("camtran");
    tx = table.getEntry("tx");
    ty = table.getEntry("ty");
    ta = table.getEntry("ta");

    // Initializes the 4 solonoids. Null checks fix a stupid issue
    // TODO: Change this to one line ifs
    if (MiddleSolonoid == null){
      MiddleSolonoid = new DoubleSolenoid(0,1);
    }
    if (RearSolonoid == null){
      RearSolonoid = new DoubleSolenoid(2,3);
    }
    if (FrontSolonoid == null){
      FrontSolonoid = new DoubleSolenoid(4, 5);
    }
    if (GrabberSolonoid == null){
      GrabberSolonoid = new DoubleSolenoid(6, 7);
    }

    if (pdp == null){
      pdp = new  PowerDistributionPanel();
    }
    xboxXButtonDown = false;
    xboxAButtonDown = false;
    xboxBButtonDown = false;
    xboxYButtonDown = false;

    m_robotDrive.setSafetyEnabled(false);

    grabberTimer = new Timer();
  }

  @Override
  public void teleopPeriodic() {
    // Tracks how long the periodic loop takes. Used for debuging.
    Timer profilePeriodicTimer = new Timer();
    profilePeriodicTimer.start();
    // One place for all the driving values
    double[] driveValues;
    // Limelight Data
    double[] backupDouble = new double[]{0.0,0.0,0.0,0.0,0.0};
    double[] camtranData = camtran.getDoubleArray(backupDouble);
    
    // Tracks Solonoid state to apply the speed limit
    Boolean isSolonoidExtended;
    // Calcs the rotation power including the deadband
    double fixedRotationPower;

    // Checks if any solonoid is extended. If so, set isSolonoidExtended to true. Otherwise, set it to false. 
    if (FrontSolonoid.get() == DoubleSolenoid.Value.kForward || MiddleSolonoid.get() == DoubleSolenoid.Value.kForward || RearSolonoid.get() == DoubleSolenoid.Value.kForward){
      isSolonoidExtended = true;
    } else {
      isSolonoidExtended = false;
    }
    

    // Use the translate stick X axis for lateral movement, Y axis for forward
    // movement, and Z axis on the rotate stick for rotation.
    fixedTranslationThrottle = 1 - ((m_translateStick.getThrottle() + 1) / 2);
    fixedRotationThrottle = (-m_rotateStick.getThrottle() + 1) / 2;

    if (m_translateStick.getRawButton(2)) {
      // Get the data from the limelight if the button is pressed
      driveValues = LimelightMethods.autoAlign(camtranData);
      
    } else if (isSolonoidExtended) {
      fixedTranslationThrottle = fixedTranslationThrottle * 0.4;
      // Drives with lower speed since a button is being pressed
      if (m_translateStick.getX() < 0) {
        xValue = -1 * (Math.pow(m_translateStick.getX(), 2) * fixedTranslationThrottle);
      } else {
        xValue = (Math.pow(m_translateStick.getX(), 2) * fixedTranslationThrottle);
      }
      if (m_translateStick.getY() < 0) {
        yValue = (Math.pow(m_translateStick.getY(), 2) * fixedTranslationThrottle);
      } else {
        yValue = -1 * (Math.pow(m_translateStick.getY(), 2) * fixedTranslationThrottle);
      }
      if (m_rotateStick.getZ() > 0.1 || m_rotateStick.getZ() < 0.1) {
        fixedRotationPower = m_rotateStick.getZ() * 0.7 * fixedRotationThrottle;
      } else {
        fixedRotationPower = 0.0;
      }

      driveValues = new double[]{xValue, yValue, fixedRotationPower};

    } else if (m_translateStick.getPOV() != -1) {
      // Drives using Single Axis Movement (SAM) Momement controlls for absolute movement. Implementation may be changed later.
      yValue = 0;
      xValue = 0;
      zValue = 0;
      switch (m_translateStick.getPOV()) {
        case 0:
          yValue = fixedTranslationThrottle * 0.6;
          break;
        case 90:
          xValue = fixedTranslationThrottle * 0.75;
          break;
        case 180:
          yValue = -fixedTranslationThrottle * 0.6;
          break;
        case 270: 
          xValue = -fixedTranslationThrottle * 0.75;
          break;
        default:
          break;
      }
      driveValues = new double[]{xValue, yValue, zValue};

    } else 
    {
      // Drives manually
      if (m_translateStick.getX() < 0) {
        xValue = -1 * (Math.pow(m_translateStick.getX(), 2) * fixedTranslationThrottle);
      } else {
        xValue = (Math.pow(m_translateStick.getX(), 2) * fixedTranslationThrottle);
      }
      if (m_translateStick.getY() < 0) {
        yValue = (Math.pow(m_translateStick.getY(), 2) * fixedTranslationThrottle);
      } else {
        yValue = -1 * (Math.pow(m_translateStick.getY(), 2) * fixedTranslationThrottle);
      }
      if (m_rotateStick.getZ() > 0.1 || m_rotateStick.getZ() < 0.1) {
        fixedRotationPower = m_rotateStick.getZ() * 0.7  * fixedRotationThrottle;
      } else {
        fixedRotationPower = 0.0;
      }
      // Make it so all methods of moving the robot use the same output :)
      driveValues = new double[]{xValue, yValue, fixedRotationPower};
    }

    // Use the lefy hand stick to controll the robot's slide movement
    double output = -m_xbox.getY(Hand.kLeft);
    if(m_xbox.getBumper(Hand.kLeft)){
      // Used to aproximately fight gravity
      output = 0.08;
    } else {
      // Checks the limit switch, will keep slide from hitting anything wrong
      if (UpperGliftSwitch.get() == false) {
        output = Math.min(output, 0);
      } 
      if (LowerGliftSwitch.get() == false){
        output = Math.max(output, 0);
      }
    } 
    // Do the movement at the end
    LiftMotor1.set(output * 0.75);
    LiftMotor2.set(output * 0.75);


    // Controlls the speed of the grabber motor when pulling the trigger
    if(m_xbox.getTriggerAxis(Hand.kLeft) > 0.1){
      // Push out
      GrabberMotor.set(m_xbox.getTriggerAxis(Hand.kLeft) * 1.0);
    } else if (m_xbox.getTriggerAxis(Hand.kRight) >= 0.1){
      // Pull in
      GrabberMotor.set(-(m_xbox.getTriggerAxis(Hand.kRight)) * 0.5);
    } else {
      GrabberMotor.set(-0.15);
    }
    
    //GrabberMotor.set(m_xbox.getTriggerAxis(Hand.kLeft));
    //GrabberMotor.set(-(m_xbox.getTriggerAxis(Hand.kRight)));

    
    /* 
    * Controls the Robot's pistons. 
    */
    if(m_xbox.getBumper(Hand.kRight)){
      // Runs only when the right bumper **IS** pressed

      // Controlls the middle pistons
      if(m_xbox.getAButtonPressed()) {
        if(!xboxAButtonDown){
          MiddleSolonoid.set(DoubleSolenoid.Value.kForward);
          xboxAButtonDown = true;
        } else {
          MiddleSolonoid.set(DoubleSolenoid.Value.kReverse);
          xboxAButtonDown = false;
        }
      }

      // Controlls the rear pistons
      if(m_xbox.getBButtonPressed()) {
        if(!xboxBButtonDown){
          RearSolonoid.set(DoubleSolenoid.Value.kForward);
          xboxBButtonDown = true;
        } else {
          RearSolonoid.set(DoubleSolenoid.Value.kReverse);
          xboxBButtonDown = false;
        }
      }

      // Controlls the front pistons
      if(m_xbox.getXButtonPressed()) {
        if(!xboxXButtonDown){
          FrontSolonoid.set(DoubleSolenoid.Value.kForward);
          xboxXButtonDown = true;
        } else {
          FrontSolonoid.set(DoubleSolenoid.Value.kReverse);
          xboxXButtonDown = false;
        }
      }

      // Fixes weird issue with the button being pressed, then the bumper, and then the button action coming
      // Basically, the getStartButton functions returned true if the button was pressed since it was last checked, sort of queuing up presses for the next check.
      m_xbox.getYButtonPressed();
      m_xbox.getStartButtonPressed();
    } else {
      // Runs when the right bumber **ISN'T** pressed
      // Controlls the grabber piston
      if(m_xbox.getYButtonPressed()) {
        if(!xboxYButtonDown){
          GrabberSolonoid.set(DoubleSolenoid.Value.kForward);
          xboxYButtonDown = true;
        } else {
          GrabberSolonoid.set(DoubleSolenoid.Value.kReverse);
          xboxYButtonDown = false;
        }
      }

      /* 
      * Controlls the fine tuned ball drop sequence. May need to extend window
      */
      if(m_xbox.getStartButtonPressed()) {
        grabberTimer.reset();
        grabberTimer.start();
        System.out.println("State 4");
      }
      
      if(grabberTimer.get() <= 0.02 && grabberTimer.get() > 0.01) {
        GrabberSolonoid.set(DoubleSolenoid.Value.kForward);
        System.out.println("State 1");
      } else if (grabberTimer.get() <= 0.2 && grabberTimer.get() >= 0.17) {
        GrabberSolonoid.set(DoubleSolenoid.Value.kReverse);
        System.out.println("State 2");
      } else if (grabberTimer.get() >= 1) {
        grabberTimer.reset();
        grabberTimer.stop();
        System.out.println("State 3");
      }
      

      // Fixes weird issue with the button being pressed, then the bumper, and then the button action coming
      // Basically, the action was acting if it was quened.
      m_xbox.getXButtonPressed();
      m_xbox.getAButtonPressed();
      m_xbox.getBButtonPressed();
    }


    // Allow the trigger to put on the brakes and override the power of all drive motors to zero
    // TODO: Fix the potential slowdowns that this check may cause
    if (m_translateStick.getTrigger()){
      xValue = 0.0;
      yValue = 0.0;
      zValue = 0.0;
      FrontLeft.setNeutralMode(NeutralMode.Brake);
      FrontRight.setNeutralMode(NeutralMode.Brake);
      RearRight.setNeutralMode(NeutralMode.Brake);
      RearLeft.setNeutralMode(NeutralMode.Brake);

      driveValues = new double[]{xValue, yValue, zValue};
    } else {
      FrontLeft.setNeutralMode(NeutralMode.Coast);
      FrontRight.setNeutralMode(NeutralMode.Coast);
      RearRight.setNeutralMode(NeutralMode.Coast);
      RearLeft.setNeutralMode(NeutralMode.Coast);
    }

    // Actually make the robot move, uses the values stored at any earlier point.
    m_robotDrive.driveCartesian(driveValues[0], driveValues[1], driveValues[2]);

    // Put limelight values onto smart dashboard for debuging.
    // SmartDashboard.putNumber("LimelightX", x);
    // SmartDashboard.putNumber("LimelightY", y);
    // SmartDashboard.putNumber("LimelightArea", area);
    SmartDashboard.putNumber("GrabberCurrentDraw", GrabberMotor.getOutputCurrent());
    SmartDashboard.putData(m_robotDrive);
    SmartDashboard.putData(pdp);
    SmartDashboard.putNumberArray("Camtran Data", camtranData);

    SmartDashboard.putBoolean("X", xboxXButtonDown);
    SmartDashboard.putBoolean("Y", xboxYButtonDown);
    SmartDashboard.putBoolean("B", xboxBButtonDown);
    SmartDashboard.putBoolean("A", xboxAButtonDown);
    SmartDashboard.putBoolean("Pistons Down", isSolonoidExtended);
    SmartDashboard.putBoolean("Upper Limit", UpperGliftSwitch.get());
    SmartDashboard.putBoolean("Lower Limit", !LowerGliftSwitch.get());
    SmartDashboard.putNumber("Fixed Throttle", fixedTranslationThrottle);

    m_xbox.setRumble(RumbleType.kLeftRumble, m_xbox.getTriggerAxis(Hand.kLeft));
    m_xbox.setRumble(RumbleType.kRightRumble, m_xbox.getTriggerAxis(Hand.kRight));

    System.out.println(profilePeriodicTimer.get());
  }

  /*
   * Autonomus is target folowing right now
   *
   * Will follow the vision target, rotating to face the target
   */
  @Override
  public void autonomousInit() {
    teleopInit();
  }

  @Override
  public void autonomousPeriodic() {
    teleopPeriodic();
  }

  @Override
  public void disabledInit() {
    //pdp = new PowerDistributionPanel();

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
    m_translateStick = new Joystick(kTranslateStickChannel);
    m_translateStick.setThrottleChannel(3);

    m_xbox = new XboxController(xboxControllerChannel);

  }

  @Override
  public void testPeriodic() {
    m_xbox.setRumble(RumbleType.kLeftRumble, m_xbox.getTriggerAxis(Hand.kLeft));
    m_xbox.setRumble(RumbleType.kRightRumble, m_xbox.getTriggerAxis(Hand.kRight));

    if (m_translateStick.getRawButton(2)){
      lightDrive.SetColor(1, Color.RED);
    } else {
      lightDrive.SetColor(1, Color.BLUE);
    }
    }
}



