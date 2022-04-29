// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import java.util.List;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxRelativeEncoder.Type;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.Trajectory.State;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.drive.DifferentialDrive.WheelSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import frc.robot.RobotMap;
import edu.wpi.first.wpilibj.SPI;
/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 * 
 * <p>
 * 
 * Test Kitbot Code By Jun Malit
 * In Progress...
 * Current Successful Tests:
 * None!
 * 
 * </p>
 */
public class Robot extends TimedRobot {
  private static final String kDefaultAuto = "Default";
  private static final String kCustomAuto = "My Auto";
  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();

  private XboxController driverController = new XboxController(RobotMap.DRIVER_PORT);
  private XboxController operatorController = new XboxController(RobotMap.OPERATOR_PORT);

  private CANSparkMax left1;
  private CANSparkMax left2;
  private CANSparkMax right1;
  private CANSparkMax right2;

  private RelativeEncoder[] driveEncoders;

  private boolean isArcade = true;
  private Timer m_timer = new Timer();

  private DifferentialDrive drivebase;

  private AHRS gyro = new AHRS(SPI.Port.kMXP, (byte) 200);
  private DifferentialDriveOdometry m_odometry = new DifferentialDriveOdometry(gyro.getRotation2d(), new Pose2d());
  private RamseteController autoController = new RamseteController();
  private Trajectory m_trajectory;
  private int autoStep;

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    left1 = new CANSparkMax (RobotMap.LEFT1, MotorType.kBrushless);
    left2 = new CANSparkMax (RobotMap.LEFT2, MotorType.kBrushless);
    right1 = new CANSparkMax (RobotMap.RIGHT1, MotorType.kBrushless);
    right2 = new CANSparkMax (RobotMap.RIGHT2, MotorType.kBrushless);
    
    left1.restoreFactoryDefaults();
    left2.restoreFactoryDefaults();
    right1.restoreFactoryDefaults();
    right2.restoreFactoryDefaults();

    left1.setInverted(RobotMap.IS_LEFT1_INVERTED);
    left2.setInverted(RobotMap.IS_LEFT2_INVERTED);
    right1.setInverted(RobotMap.IS_RIGHT1_INVERTED);
    right2.setInverted(RobotMap.IS_RIGHT2_INVERTED);

    

    driveEncoders = new RelativeEncoder[]{
      left1.getEncoder(Type.kHallSensor, 42), left2.getEncoder(Type.kHallSensor, 42),
      right1.getEncoder(Type.kHallSensor, 42), right2.getEncoder(Type.kHallSensor, 42)
    };

    zeroEncoders();

    drivebase = new DifferentialDrive(
      new MotorControllerGroup(left1, left2),
      new MotorControllerGroup(right1, right2)
    );

    m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
    m_chooser.addOption("My Auto", kCustomAuto);
    SmartDashboard.putData("Auto choices", m_chooser);
    SmartDashboard.putBoolean("isArcade", isArcade);
    
    
    updateDashboard();
  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for items like
   * diagnostics that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    m_odometry.update(gyro.getRotation2d(), driveEncoders[0].getPosition(), driveEncoders[3].getPosition());
    updateDashboard();
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
    m_autoSelected = m_chooser.getSelected();
    m_autoSelected = SmartDashboard.getString("Auto Selector", kDefaultAuto);
    System.out.println("Auto selected: " + m_autoSelected);

    TrajectoryConfig config = new TrajectoryConfig(8, 1.5);

    switch (m_autoSelected) {
      case kCustomAuto:
        m_trajectory = TrajectoryGenerator.generateTrajectory(
            m_odometry.getPoseMeters(), 
            List.of(new Translation2d(1, 1), new Translation2d(2, -1)), 
            new Pose2d(3, 0, Rotation2d.fromDegrees(0)), 
            config
          );
        break;
      case kDefaultAuto:
      default:
        m_trajectory = TrajectoryGenerator.generateTrajectory(
          m_odometry.getPoseMeters(), 
          List.of(), 
          new Pose2d(1, 0, Rotation2d.fromDegrees(0)), 
          config
        );
        break;
    }
    
    autoStep = 1;

    m_timer.stop();
    m_timer.reset();
    m_timer.start();
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    switch (m_autoSelected) {
      case kCustomAuto:
        // Auto running in steps
        switch (autoStep){
          // Step 1
          case 1:
            followTrajectory();
            if(autoController.atReference())
              autoStep = 2;
          break;
          // Step 2 starts once the robot completes the trajectory
          case 2:
            doNothing();
            if(m_timer.hasElapsed(10))
              autoStep = 3;
          break;
          // Step 3 (last step) starts once the timer elapses 10 seconds
          case 3:
          default:
            drivebase.stopMotor();
          break;
        }
        break;
      case kDefaultAuto:
      default:
        // Put default auto code here
        doNothing();
        break;
    }
  }
  public void followTrajectory(){
    State desiredState = m_trajectory.sample(m_timer.get());
    ChassisSpeeds m_ChassisSpeeds = autoController.calculate(m_odometry.getPoseMeters(), desiredState);
    WheelSpeeds m_wheelspeeds = DifferentialDrive.arcadeDriveIK(m_ChassisSpeeds.vxMetersPerSecond, m_ChassisSpeeds.omegaRadiansPerSecond, true);
    drivebase.tankDrive(m_wheelspeeds.left, m_wheelspeeds.right, true);
  }
  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    // drivebase.arcadeDrive(driverController.getLeftY(), driverController.getRightX());
    changeDrive();
    drive();
  }

  /** Toggles the drivebase between tank drive and arcade drive. */
  public void changeDrive(){
    SmartDashboard.putBoolean("isArcade", isArcade);
    if(driverController.getAButtonPressed()){
      if(isArcade)
        isArcade = false;
      else
        isArcade = true;
    }
  }

  /** Using joystick inputs from driver 1, drives the robot based on whether arcade drive is enabled. */
  public void drive(){
    if(isArcade)
      drivebase.arcadeDrive(driverController.getLeftY(), driverController.getRightX(), true);
    else
      drivebase.tankDrive(driverController.getLeftY(), driverController.getRightY(), true);
  }
  /** Does nothing. */
  public void doNothing(){
    System.out.println("haha the robot did nothing");
  }
  /** Sets drive encoders values to 0. */
  public void zeroEncoders(){
    for(RelativeEncoder encoder : driveEncoders){
      encoder.setPosition(0);
    }
  }
  /** Resets the angle of the robot. */
  public void resetGyro(){
    gyro.reset();
  }
  /** Sets the current position of the robot as the origin. */
  public void zeroOdometry(){
    m_odometry = new DifferentialDriveOdometry(gyro.getRotation2d());
  }
  /** Puts important values onto the dashboard. */
  public void updateDashboard(){
    SmartDashboard.putNumber("Left1", driveEncoders[0].getVelocity());
    SmartDashboard.putNumber("Left2", driveEncoders[1].getVelocity());
    SmartDashboard.putNumber("Right1", driveEncoders[2].getVelocity());
    SmartDashboard.putNumber("Right2", driveEncoders[3].getVelocity());
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
