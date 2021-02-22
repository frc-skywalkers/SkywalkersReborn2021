// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
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

  private final ADXRS450_Gyro m_gyro = new ADXRS450_Gyro();

  private DifferentialDriveOdometry m_odometry;

  private boolean isQuickTurn = true;


  private Field2d field = new Field2d();

  NetworkTableEntry m_xEntry = NetworkTableInstance.getDefault().getTable("troubleshooting").getEntry("X");
  NetworkTableEntry m_yEntry = NetworkTableInstance.getDefault().getTable("troubleshooting").getEntry("Y");

  public Drivetrain() {

    leftMaster.configFactoryDefault();
    leftFollower.configFactoryDefault();
    rightMaster.configFactoryDefault();
    rightFollower.configFactoryDefault();

    TalonFXConfiguration configs = new TalonFXConfiguration();
    configs.primaryPID.selectedFeedbackSensor = FeedbackDevice.IntegratedSensor;

    leftMaster.configAllSettings(configs);
    leftFollower.configAllSettings(configs);
    rightMaster.configAllSettings(configs);
    rightFollower.configAllSettings(configs);

    leftFollower.follow(leftMaster);
    rightFollower.follow(rightMaster);

    leftMaster.setNeutralMode(NeutralMode.Brake);
    leftFollower.setNeutralMode(NeutralMode.Brake);
    rightMaster.setNeutralMode(NeutralMode.Brake);
    rightFollower.setNeutralMode(NeutralMode.Brake);

    leftMaster.setInverted(Constants.DriveConstants.kLeftInvertType);
    leftFollower.setInverted(Constants.DriveConstants.kLeftInvertType);
    rightMaster.setInverted(Constants.DriveConstants.kRightInvertType);
    rightFollower.setInverted(Constants.DriveConstants.kRightInvertType);

    

    drive.setRightSideInverted(false);

    SmartDashboard.putData("Field", field);

    m_odometry = new DifferentialDriveOdometry(m_gyro.getRotation2d());
    
    resetDrivetrainEncoders();
  
  }

  public void stop() {
    drive.stopMotor();
  }

  public void arcadeDrive(double fwd, double rot, double speed) {
    drive.arcadeDrive(fwd * speed, rot * speed);
  }

  public void tankDrive(double leftPwr, double rightPwr, double speed) {
    drive.tankDrive(leftPwr * speed, rightPwr * speed);
  }

  public void curvatureDrive(double fwd, double rot, double speed) {
    SmartDashboard.putNumber("fwd", fwd);
    SmartDashboard.putNumber("rot", rot);
    drive.curvatureDrive(fwd * speed, rot * speed, isQuickTurn);
  }

  public void toggleQuickTurn() {
    isQuickTurn = !isQuickTurn;
  }

  // Getters

  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(getLeftEncoderRate(), getRightEncoderRate());
  }


  //Encoder getters

  public double getAverageEncoderDistance() {
    return (getLeftEncoderDistance() + getRightEncoderDistance()) / 2.0;
  }

  public double getLeftEncoderDistance(){
    return leftMaster.getSelectedSensorPosition() * Constants.DriveConstants.kDistancePerPulseFactor;
    
  }

  public double getRightEncoderDistance(){
    return rightMaster.getSelectedSensorPosition() * Constants.DriveConstants.kDistancePerPulseFactor;
    
  }

  public double getRightEncoderRate() {
      return rightMaster.getSelectedSensorVelocity() * Constants.DriveConstants.kDistancePerPulseFactor / 60;
    
  }

  public double getLeftEncoderRate() {
      return leftMaster.getSelectedSensorVelocity() * Constants.DriveConstants.kDistancePerPulseFactor / 60;
    
  }


  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }


  //Setters
  
  public void tankDriveVolts(double leftVolts, double rightVolts) {
    var batteryVoltage = RobotController.getBatteryVoltage();
    if (Math.max(Math.abs(leftVolts), Math.abs(rightVolts)) > batteryVoltage) {
      leftVolts *= batteryVoltage / 12.0;
      rightVolts *= batteryVoltage / 12.0;
    }
    m_leftGroup.setVoltage(leftVolts);
    m_rightGroup.setVoltage(rightVolts);
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
    m_odometry.resetPosition(pose, Rotation2d.fromDegrees(getHeading()));
  }
  

  public void zeroHeading() {
    m_gyro.reset();
  }

 
  public double getHeading() {
    return Math.IEEEremainder(m_gyro.getAngle(), 360) * (DriveConstants.kGyroReversed ? -1.0: 1.0);
  }

  
  public double getTurnRate() {
    return -m_gyro.getRate();
  }

  @Override
  public void periodic() {
    m_odometry.update(
      Rotation2d.fromDegrees(getHeading()), 
      getLeftEncoderDistance(), 
      getRightEncoderDistance());
    field.setRobotPose(getPose());

    SmartDashboard.putNumber("gryo-angle", getHeading());
    SmartDashboard.putNumber("left-distance", getLeftEncoderDistance());
    SmartDashboard.putNumber("right-distance", getRightEncoderDistance());
    SmartDashboard.putNumber("right-power", rightMaster.getMotorOutputPercent());
    SmartDashboard.putNumber("left-power", leftMaster.getMotorOutputPercent());

    var translation = m_odometry.getPoseMeters().getTranslation();
    m_xEntry.setNumber(translation.getX());
    m_yEntry.setNumber(translation.getY());
    SmartDashboard.putNumber("x", translation.getX());
    SmartDashboard.putNumber("y", translation.getX());
  }

  
}
