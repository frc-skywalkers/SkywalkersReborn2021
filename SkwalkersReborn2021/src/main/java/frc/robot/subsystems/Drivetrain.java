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
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.simulation.ADXRS450_GyroSim;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.system.plant.DCMotor;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpiutil.math.VecBuilder;
import frc.robot.Constants;
import frc.robot.Robot;
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

  private final DifferentialDrivetrainSim driveSim = new DifferentialDrivetrainSim(
    DCMotor.getFalcon500(2),       // 2 Falcon 500 motors on each side of the drivetrain.
    10.71,                    // 10.71:1 gearing reduction.
    7.5,                     // MOI of 7.5 kg m^2 (from CAD model).
    60.0,                    // The mass of the robot is 60 kg.
    Units.inchesToMeters(3), // The robot uses 3" radius wheels.
    0.5558,                  // The track width is 0.5558 meters.

    // The standard deviations for measurement noise:
    // x and y:          0.001 m
    // heading:          0.001 rad
    // l and r velocity: 0.1   m/s
    // l and r position: 0.005 m
    VecBuilder.fill(0.001, 0.001, 0.001, 0.1, 0.1, 0.005, 0.005));

  private Encoder m_leftEncoder;
  private Encoder m_rightEncoder;
    
  private EncoderSim m_leftEncoderSim;
  private EncoderSim m_rightEncoderSim;

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
    leftFollower.setInverted(Constants.DriveConstants.kLeftInvertType);
    rightMaster.setInverted(Constants.DriveConstants.kRightInvertType);
    rightFollower.setInverted(Constants.DriveConstants.kRightInvertType);

    drive.setRightSideInverted(false);

    SmartDashboard.putData("Field", field);

    m_odometry = new DifferentialDriveOdometry(m_gyro.getRotation2d());

    if(Robot.isSimulation()) {
      m_leftEncoder = new Encoder(0, 1);
      m_rightEncoder = new Encoder(2, 3, true);
      m_leftEncoderSim = new EncoderSim(m_leftEncoder);
      m_rightEncoderSim = new EncoderSim(m_rightEncoder);

      m_leftEncoder.setDistancePerPulse(DriveConstants.kDistancePerPulseFactor);
      m_rightEncoder.setDistancePerPulse(DriveConstants.kDistancePerPulseFactor);
    }
  
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
    if(Robot.isReal()) {
      return leftMaster.getSelectedSensorPosition() * Constants.DriveConstants.kDistancePerPulseFactor;
    } else {
      return m_leftEncoder.getDistance();
    }
    
  }

  public double getRightEncoderDistance(){
    if(Robot.isReal()) {
      return rightMaster.getSelectedSensorPosition() * Constants.DriveConstants.kDistancePerPulseFactor;
    } else {
      return m_rightEncoder.getDistance();
    }
    
  }

  public double getRightEncoderRate() {
    if(Robot.isReal()) {
      return rightMaster.getSelectedSensorVelocity() * Constants.DriveConstants.kDistancePerPulseFactor / 60;
    } else {
      return m_rightEncoder.getRate();
    }
    
  }

  public double getLeftEncoderRate() {
    if(Robot.isReal()) {
      return leftMaster.getSelectedSensorVelocity() * Constants.DriveConstants.kDistancePerPulseFactor / 60;
    } else {
      return m_leftEncoder.getRate();
    }
    
  }

  public double getDrawnCurrentAmps() {
    return driveSim.getCurrentDrawAmps();
  }

  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
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
    return Rotation2d.fromDegrees(-m_gyro.getAngle());
  }

  
  public double getTurnRate() {
    return -m_gyro.getRate();
  }

  @Override
  public void periodic() {
    m_odometry.update(
      getHeading(), 
      getLeftEncoderDistance(), 
      getRightEncoderDistance());
    field.setRobotPose(m_odometry.getPoseMeters());
  }

  @Override
  public void simulationPeriodic() {
    driveSim.setInputs(
      leftMaster.get() * RobotController.getInputVoltage(), 
      rightMaster.get() * RobotController.getInputVoltage());
    driveSim.update(0.02);

    m_leftEncoderSim.setDistance(driveSim.getLeftPositionMeters());
    m_leftEncoderSim.setRate(driveSim.getLeftVelocityMetersPerSecond());
    m_rightEncoderSim.setDistance(driveSim.getRightPositionMeters());
    m_rightEncoderSim.setRate(driveSim.getRightVelocityMetersPerSecond());

    m_gyroSim.setAngle(-driveSim.getHeading().getDegrees());
  }
}
