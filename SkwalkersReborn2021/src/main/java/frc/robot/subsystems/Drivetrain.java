// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.simulation.ADXRS450_GyroSim;
import edu.wpi.first.wpilibj.simulation.AnalogGyroSim;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.system.plant.DCMotor;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpiutil.math.VecBuilder;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;

public class Drivetrain extends SubsystemBase {
  /** Creates a new Drivetrain. */
  private final WPI_TalonFX leftMaster = new WPI_TalonFX(Constants.DriveConstants.kLeftMasterPort);
  private final WPI_TalonFX leftFollower = new WPI_TalonFX(Constants.DriveConstants.kLeftFollowerPort);
  private final WPI_TalonFX rightMaster = new WPI_TalonFX(Constants.DriveConstants.kRightMasterPort);
  private final WPI_TalonFX rightFollower = new WPI_TalonFX(Constants.DriveConstants.kRightFollowerPort);

  private final SpeedControllerGroup m_leftGroup = new SpeedControllerGroup(leftMaster, leftFollower);
  private final SpeedControllerGroup m_rightGroup = new SpeedControllerGroup(rightMaster, rightFollower);

  private final DifferentialDrive drive = new DifferentialDrive(m_leftGroup, m_rightGroup);

  private Pose2d startingPose = new Pose2d(5.0, 13.5, new Rotation2d());

  private Pose2d pose;

  private final DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(Units.inchesToMeters(DriveConstants.kTrackWidth));
  private DifferentialDriveOdometry m_odometry = new DifferentialDriveOdometry(getHeading(), startingPose);

  SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(DriveConstants.ks, DriveConstants.kv, DriveConstants.ka);

  PIDController leftPidController = new PIDController(DriveConstants.kPDriveVel, 0, 0);
  PIDController rightPidController = new PIDController(DriveConstants.kPDriveVel, 0, 0);

  private final ADXRS450_Gyro m_gyro = new ADXRS450_Gyro();

  private boolean isQuickTurn = true;

  private final DifferentialDrivetrainSim driveSim = new DifferentialDrivetrainSim(
    DCMotor.getFalcon500(2),       // 2 Falcon 500 motors on each side of the drivetrain.
    10.71,                    // 10.71:1 gearing reduction.
    7.5,                     // MOI of 7.5 kg m^2 (from CAD model).
    60.0,                    // The mass of the robot is 60 kg.
    Units.inchesToMeters(6), // The robot uses 3" radius wheels.
    0.5558,                  // The track width is 0.5558 meters.

    // The standard deviations for measurement noise:
    // x and y:          0.001 m
    // heading:          0.001 rad
    // l and r velocity: 0.1   m/s
    // l and r position: 0.005 m
    VecBuilder.fill(0.001, 0.001, 0.001, 0.1, 0.1, 0.005, 0.005));

  private ADXRS450_GyroSim m_gyroSim = new ADXRS450_GyroSim(m_gyro);

  private Field2d field = new Field2d();


  public Drivetrain() {

    TalonFXConfiguration leftConfig = new TalonFXConfiguration();
    TalonFXConfiguration rightConfig = new TalonFXConfiguration();

    leftConfig.primaryPID.selectedFeedbackSensor = FeedbackDevice.IntegratedSensor;
    rightConfig.primaryPID.selectedFeedbackSensor = FeedbackDevice.IntegratedSensor;

    leftMaster.configFactoryDefault();
    leftFollower.configFactoryDefault();
    rightMaster.configFactoryDefault();
    rightFollower.configFactoryDefault();

    leftFollower.follow(leftMaster);
    rightFollower.follow(rightMaster);

    leftMaster.configAllSettings(leftConfig);
    leftFollower.configAllSettings(leftConfig);
    rightMaster.configAllSettings(rightConfig);
    rightFollower.configAllSettings(rightConfig);

    leftMaster.setNeutralMode(NeutralMode.Brake);
    leftFollower.setNeutralMode(NeutralMode.Brake);
    rightMaster.setNeutralMode(NeutralMode.Brake);
    rightFollower.setNeutralMode(NeutralMode.Brake);

    leftMaster.setInverted(Constants.DriveConstants.kLeftInvertType);
    leftFollower.setInverted(InvertType.FollowMaster);
    rightMaster.setInverted(Constants.DriveConstants.kRightInvertType);
    rightFollower.setInverted(InvertType.FollowMaster);

    drive.setRightSideInverted(false);

    SmartDashboard.putData("Field", field);

    m_odometry = new DifferentialDriveOdometry(m_gyro.getRotation2d());
  
  }

  // Drive Functions
  public void driveWithJoysticks(XboxController controller, double speed) {
    /* Experiment with Drive methods:
         - Arcade Drive: control forward and control turning
         - Tank Drive: control left and right
         - Curvature Drive: normal arcade drive when quick turn, 
           more subtle turning control with not quickturn (for quick traversing across field) 
    */

    arcadeDrive(controller, speed);
    // tankDrive(controller, speed);
    // curvatureDrive(controller, speed);
  }

  public void stop() {
    drive.stopMotor();
  }

  public void arcadeDrive(XboxController controller, double speed) {
    //drive.arcadeDrive(controller.getY(Hand.kLeft) * speed, controller.getX(Hand.kRight) * speed);
    drive.arcadeDrive(controller.getRawAxis(1) * speed, controller.getRawAxis(0) * speed);
  }

  public void tankDrive(XboxController controller, double speed) {
    drive.tankDrive(controller.getY(Hand.kLeft) * speed, controller.getY(Hand.kRight) * speed);
  }

  public void curvatureDrive(XboxController controller, double speed) {
    drive.curvatureDrive(controller.getY(Hand.kLeft) * speed, controller.getX(Hand.kRight) * speed, isQuickTurn);
  }

  public void toggleQuickTurn() {
    isQuickTurn = !isQuickTurn;
  }

  // Getters

  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(getLeftEncoderRate(), getRightEncoderRate());
  }

  public SimpleMotorFeedforward getSimpleMotorFeedforward() {
    return feedforward;
  }

  public PIDController getLeftPidController() {
    return leftPidController;
  }

  public PIDController getRightPidController() {
    return rightPidController;
  }

  public DifferentialDriveKinematics getKinematics() {
    return kinematics;
  }

  public Pose2d getPose() {
    return pose;
  }

  //Encoder getters

  public double getAverageEncoderDistance() {
    return (getLeftEncoderDistance() + getRightEncoderDistance()) / 2.0;
  }

  public double getLeftEncoderDistance(){
    return leftMaster.getSelectedSensorPosition() * Constants.DriveConstants.distancePerPulseFactor;
  }

  public double getRightEncoderDistance(){
    return rightMaster.getSelectedSensorPosition() * Constants.DriveConstants.distancePerPulseFactor;
  }

  public double getRightEncoderRate() {
    return rightMaster.getSelectedSensorVelocity() * Constants.DriveConstants.distancePerPulseFactor;
  }

  public double getLeftEncoderRate() {
    return leftMaster.getSelectedSensorVelocity() * Constants.DriveConstants.distancePerPulseFactor;
  }


  //Setters
  
  public void tankDriveVolts(double leftVolts, double rightVolts) {
    m_leftGroup.setVoltage(leftVolts);
    m_rightGroup.setVoltage(-rightVolts);
    drive.feed();
  }

  public void setMaxOutput(double maxOutput) {
    drive.setMaxOutput(maxOutput);
  }

  //Resetters

  public void resetDrivetrainEncoders(){
    leftMaster.setSelectedSensorPosition(0);
    rightMaster.setSelectedSensorPosition(0);
  }

  public void resetOdometry(Pose2d pose) {
    resetDrivetrainEncoders();
    m_odometry.resetPosition(pose, m_gyro.getRotation2d());
  }
  

  public void zeroHeading() {
    m_gyro.reset();
  }

 
  public Rotation2d getHeading() {
    return Rotation2d.fromDegrees(-1 * m_gyro.getAngle());
  }

  
  public double getTurnRate() {
    return -m_gyro.getRate();
  }

  @Override
  public void periodic() {
    pose = m_odometry.update(getHeading(), getLeftEncoderDistance(), getRightEncoderDistance());
    field.setRobotPose(m_odometry.getPoseMeters());
  }

  @Override
  public void simulationPeriodic() {
    driveSim.setInputs(leftMaster.get() * RobotController.getInputVoltage(), rightMaster.get() * RobotController.getInputVoltage());
    driveSim.update(0.02);

    m_gyroSim.setAngle(-driveSim.getHeading().getDegrees());
  }
}
