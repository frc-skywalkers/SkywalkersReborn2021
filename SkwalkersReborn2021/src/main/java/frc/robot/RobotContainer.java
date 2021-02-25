// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.IOException;
import java.nio.file.Path;
import java.util.List;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.SlewRateLimiter;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.trajectory.TrajectoryParameterizer.TrajectoryGenerationException;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final Drivetrain drive = new Drivetrain();
  private final Intake intake = new Intake();
  private final Arm arm = new Arm();
  private final String [] paths = {
    "paths/GalacticSearchABlue.wpilib.json",
    "paths/GalacticSearchARed.wpilib.json",
    "paths/GalacticSearchBBlue.wpilib.json",
    "paths/GalacticSearchBRed.wpilib.json",
    "paths/Slalom.wpilib.json",
    "paths/slalomV1.wpilib.json"
  };
  private final int pathToRun = 4;

  private final SlewRateLimiter speedLimiter = new SlewRateLimiter(3);
  private final SlewRateLimiter rotLimiter = new SlewRateLimiter(3);

  private XboxController driveController = new XboxController(Constants.OIConstants.kDriverControllerPort);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    var xSpeed = -speedLimiter.calculate(driveController.getRawAxis(OIConstants.kLeftY));
    
    var rot = -rotLimiter.calculate(driveController.getRawAxis(OIConstants.kRightX));
    
    drive.setDefaultCommand(
      new RunCommand(
        () -> drive.curvatureDrive(xSpeed,rot, DriveConstants.kDriveSpeed), drive)
      );
      
    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {

    // Toggle Quick Turn when driver presses the Toggle Quick Turn button
    new JoystickButton(driveController, Constants.OIConstants.kToggleQuickTurnButton.value).whenPressed(drive::toggleQuickTurn);

    new JoystickButton(driveController, Constants.OIConstants.kIntakeButton.value).whenPressed(intake::intake);

    new JoystickButton(driveController, Constants.OIConstants.kStopRollerButton.value).whenPressed(intake::stopRoller);

    new JoystickButton(driveController, Constants.OIConstants.kOuttakeButton.value).whenPressed(intake::outtake);

    new JoystickButton(driveController, Constants.OIConstants.kLiftArmButton.value).whileHeld(() -> arm.moveArm(Constants.ArmConstants.kArmSpeed));

    new JoystickButton(driveController, Constants.OIConstants.kLowerArmButton.value).whileHeld(() -> arm.moveArm(-Constants.ArmConstants.kArmSpeed));

  
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {

    var autoVoltageConstraint = 
      new DifferentialDriveVoltageConstraint(
        new SimpleMotorFeedforward(
          DriveConstants.ksVolts, 
          DriveConstants.kvVoltSecondsPerMeter,
          DriveConstants.kaVoltSecondsSquaredPerMeter), 
        DriveConstants.kDriveKinematics, 
        10);
    
    TrajectoryConfig config = 
      new TrajectoryConfig(
        AutoConstants.kMaxSpeedMetersPerSecond, 
        AutoConstants.kMaxAccelerationMetersPerSecondSquared)
      .setKinematics(DriveConstants.kDriveKinematics)
      .addConstraint(autoVoltageConstraint);

    Trajectory slalom = 
      TrajectoryGenerator.generateTrajectory(
        new Pose2d(1.23, 0.55, new Rotation2d(0)), 
        List.of(
          new Translation2d(3.03, 2.21), 
          new Translation2d(4.64, 2.75),
          new Translation2d(6.68, 1.43),
          new Translation2d(7.34, 0.64),
          new Translation2d(8.36, 0.75),
          new Translation2d(8.75, 1.47),
          new Translation2d(7.83, 2.44),
          new Translation2d(6.91, 2.01),
          new Translation2d(6.68, 1.43),
          new Translation2d(6.38, 0.89),
          new Translation2d(4.6, 0.48),
          new Translation2d(2.82, 0.95)), 
        new Pose2d(1.25, 2.31, new Rotation2d(180)), 
        config);


    // String trajectoryJSON = "paths/Slalomv2.wpilib.json";
    // Trajectory trajectory = new Trajectory();
    // try {
    //   Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON);
    //   trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
    // } catch (IOException ex) {
    //   DriverStation.reportError("Unable to open trajectory: " + trajectoryJSON, ex.getStackTrace());
    // }

    RamseteCommand ramseteCommand = new RamseteCommand(
        slalom,
        drive::getPose,
        new RamseteController(
          AutoConstants.kRamseteB, AutoConstants.kRamseteZeta),
        new SimpleMotorFeedforward(
          DriveConstants.ksVolts, 
          DriveConstants.kvVoltSecondsPerMeter, 
          DriveConstants.kaVoltSecondsSquaredPerMeter),
        DriveConstants.kDriveKinematics,
        drive::getWheelSpeeds,
        new PIDController(DriveConstants.kPDriveVel, 0, 0),
        new PIDController(DriveConstants.kPDriveVel, 0, 0),
        drive::tankDriveVolts,
        drive
    );

    // Reset odometry to the starting pose of the trajectory.
    drive.resetOdometry(slalom.getInitialPose());

    // Run path following command, then stop at the end.
    return ramseteCommand.andThen(() -> drive.tankDriveVolts(0, 0));
  }
}
