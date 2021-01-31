// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Drivetrain extends SubsystemBase {
  /** Creates a new Drivetrain. */
  private final WPI_TalonFX leftMaster = new WPI_TalonFX(Constants.DriveConstants.kLeftMasterPort);
  private final WPI_TalonFX leftFollower = new WPI_TalonFX(Constants.DriveConstants.kLeftFollowerPort);
  private final WPI_TalonFX rightMaster = new WPI_TalonFX(Constants.DriveConstants.kRightMasterPort);
  private final WPI_TalonFX rightFollower = new WPI_TalonFX(Constants.DriveConstants.kRightFollowerPort);

  private final SpeedControllerGroup m_leftGroup = new SpeedControllerGroup(leftMaster, leftFollower);
  private final SpeedControllerGroup m_rightGroup = new SpeedControllerGroup(rightMaster, rightFollower);

  private final DifferentialDrive drive = new DifferentialDrive(m_leftGroup, m_rightGroup);

  private boolean isQuickTurn = true;

  public Drivetrain() {

    leftMaster.configFactoryDefault();
    leftFollower.configFactoryDefault();
    rightMaster.configFactoryDefault();
    rightFollower.configFactoryDefault();

    leftFollower.follow(leftMaster);
    rightFollower.follow(rightMaster);

    leftMaster.setNeutralMode(NeutralMode.Brake);
    leftFollower.setNeutralMode(NeutralMode.Brake);
    rightMaster.setNeutralMode(NeutralMode.Brake);
    rightFollower.setNeutralMode(NeutralMode.Brake);

    leftMaster.setInverted(Constants.DriveConstants.kLeftInvertType);
    leftFollower.setInverted(InvertType.FollowMaster);
    rightMaster.setInverted(Constants.DriveConstants.kRightInvertType);
    rightFollower.setInverted(InvertType.FollowMaster);

    drive.setRightSideInverted(false);
  
  }

  public void driveWithJoysticks(XboxController controller, double speed) {
    /* Experiment with Drive methods:
         - Arcade Drive: control forward and control turning
         - Tank Drive: control left and right
         - Curvature Drive: normal arcade drive when quick turn, 
           more subtle turning control with not quickturn (for quick traversing across field) 
    */

    // arcadeDrive(controller, speed);
    // tankDrive(controller, speed);
    // curvatureDrive(controller, speed);
  }

  public void stop() {
    drive.stopMotor();
  }

  public void arcadeDrive(XboxController controller, double speed) {
    drive.arcadeDrive(controller.getY(Hand.kLeft) * speed, controller.getX(Hand.kRight) * speed);
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

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
