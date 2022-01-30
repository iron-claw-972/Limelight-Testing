// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private static final String kDefaultAuto = "Default";
  private static final String kCustomAuto = "My Auto";
  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();
  
  private final WPI_TalonFX m_leftMotor1 = new WPI_TalonFX(1);
  //private final WPI_TalonFX m_leftMotor2 = new WPI_TalonFX(2);

  private final WPI_TalonFX m_rightMotor1 = new WPI_TalonFX(2);
  //private final WPI_TalonFX m_rightMotor2 = new WPI_TalonFX(4);

  private final DifferentialDrive m_drive = new DifferentialDrive(m_leftMotor1, m_rightMotor1);

  private final Joystick m_joystick = new Joystick(0);

  private double speedFactor = 0.9;

  // private final double kPAim = 0.03;
  // private final double kPDistance = 0.03;
  // private final double min_command = 0.03;

  private boolean m_LimelightHasValidTarget = false;
  private double m_LimelightDriveCommand = 0.0;
  private double m_LimelightSteerCommand = 0.0;

  private PIDController drivePID = new PIDController(0.05, 0, 0);
  private PIDController steerPID = new PIDController(0.03, 0, 0);

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
    m_chooser.addOption("My Auto", kCustomAuto);
    SmartDashboard.putData("Auto choices", m_chooser);
    //m_leftMotor2.follow(m_leftMotor1);
    //m_rightMotor2.follow(m_rightMotor1);
    m_rightMotor1.setInverted(true);

    m_leftMotor1.setNeutralMode(NeutralMode.Coast);
    m_rightMotor1.setNeutralMode(NeutralMode.Coast);
  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for items like
   * diagnostics that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {}

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
    m_autoSelected = m_chooser.getSelected();
    // m_autoSelected = SmartDashboard.getString("Auto Selector", kDefaultAuto);
    System.out.println("Auto selected: " + m_autoSelected);   
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    switch (m_autoSelected) {
      case kCustomAuto:
        // Put custom auto code here
        break;
      case kDefaultAuto:
      default:
        // Put default auto code here
        break;
    }
  }

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {}

  private double deadbandPower(double power) {
    if (Math.abs(power) < 0.02) {
      return 0.0;
    }
    return power;
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    double drive = m_joystick.getRawAxis(1);
    double steer = -m_joystick.getRawAxis(4);
    boolean redAuto = m_joystick.getRawButton(2);
    boolean blueAuto = m_joystick.getRawButton(3);

    steer *= speedFactor;
    drive *= speedFactor;

    System.out.println(NetworkTableInstance.getDefault().getTable("limelight").getEntry("getpipe").getNumber(0));

    if (redAuto || blueAuto) {
      updateLimelightTracking();
      if (redAuto) {
        NetworkTableInstance.getDefault().getTable("limelight").getEntry("pipeline").setNumber(0);
      } else if (blueAuto) {
        NetworkTableInstance.getDefault().getTable("limelight").getEntry("pipeline").setNumber(1);
      }
      if (m_LimelightHasValidTarget) {
        m_drive.arcadeDrive(deadbandPower(-m_LimelightDriveCommand), deadbandPower(m_LimelightSteerCommand));
      } else {
        m_drive.arcadeDrive(0.0, 0.0);
      }
    } else {
      m_drive.arcadeDrive(deadbandPower(drive*0.7), deadbandPower(steer*0.7));
    }

    // drive.arcadeDrive(-joystick.getRawAxis(1)*0.4, joystick.getRawAxis(4)*0.4);
    // double tx = getTxLimelight();
    // double ty = getTyLimelight();
    // System.out.println("Tx: " + tx);
    // System.out.println("Ty: " + ty);
    // if (joystick.getRawButton(1)) {
    //   double headingError = -tx;
    //   double distanceError = -ty;
    //   double steeringAdjust = 0.0;
    //   if (tx > 0.5) {
    //     steeringAdjust = kPAim*headingError - min_command;
    //   } else if (tx < 0.5) {
    //     steeringAdjust = kPAim*headingError + min_command;
    //   }
    //   double distanceAdjust = kPDistance * distanceError;

    //   System.out.println("Steering adjust: " + steeringAdjust);
    //   System.out.println("Distance adjust: " + distanceAdjust);

    //   drive.tankDrive(-steeringAdjust, steeringAdjust);
    //   //drive.tankDrive(distanceAdjust, distanceAdjust);
    // }
    
  }

  public void updateLimelightTracking() {
    // These numbers must be tuned for your Robot! Be careful!
    final double STEER_K = 0.03; // how hard to turn toward the target
    final double DRIVE_K = 0.26; // how hard to drive fwd toward the target
    final double DESIRED_TARGET_AREA = 15.0; // Area of the target when the robot reaches the wall
    final double MAX_DRIVE = 0.5; // Simple speed limit so we don't drive too fast

    double tv = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(0);
    double tx = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0);
    double ty = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(0);
    double ta = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ta").getDouble(0);

    if (tv < 1.0) {
      m_LimelightHasValidTarget = false;
      m_LimelightDriveCommand = 0.0;
      m_LimelightSteerCommand = 0.0;
      return;
    }

    m_LimelightHasValidTarget = true;

    // Start with proportional steering
    //double steer_cmd = tx * STEER_K;
    double steer_cmd = steerPID.calculate(tx, 0);
    m_LimelightSteerCommand = steer_cmd;

    // try to drive forward until the target area reaches our desired area
    //double drive_cmd = (DESIRED_TARGET_AREA - ta) * DRIVE_K;
    double drive_cmd = drivePID.calculate(ta, DESIRED_TARGET_AREA);

    // don't let the robot drive too fast into the goal
    if (drive_cmd > MAX_DRIVE) {
      drive_cmd = MAX_DRIVE;
    }
    m_LimelightDriveCommand = drive_cmd;
  }

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
}
