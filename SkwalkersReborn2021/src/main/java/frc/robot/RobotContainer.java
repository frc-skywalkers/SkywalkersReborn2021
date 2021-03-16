// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.DetectPath;
import frc.robot.commands.MoveArmForTime;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
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

  private XboxController driveController = new XboxController(OIConstants.kDriverControllerPort);

  private Paths paths = new Paths();

  private Command gsDetection, gsRedA, gsBlueA, gsRedB, gsBlueB, slalom, barrel, bounce;

  private SendableChooser<Command> chooser = new SendableChooser<>();
  

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    drive.setDefaultCommand(
        new RunCommand(
            () ->
                drive.curvatureDrive(
                    -driveController.getRawAxis(OIConstants.kLeftY),
                    driveController.getRawAxis(OIConstants.kRightX),
                    DriveConstants.kDriveSpeed),
            drive));
    
    gsDetection =  new SequentialCommandGroup(
        new InstantCommand(() -> System.out.println("BEGIN COMMANDS")),
        new MoveArmForTime(arm, ArmConstants.kLowerArmSpeed, 3),
        new InstantCommand(() -> System.out.println("STARTED WAITING!!!")),
        new WaitCommand(3),
        new DetectPath(drive, intake)
      );

    gsRedA = ramseteInit(paths.getGSAR())
      .alongWith(new RunCommand(intake::intake, intake))
      .andThen(() -> drive.tankDriveVolts(0, 0))
      .andThen(intake::stopRoller, intake);

    gsBlueA = ramseteInit(paths.getGSAB())
      .alongWith(new RunCommand(intake::intake, intake))
      .andThen(() -> drive.tankDriveVolts(0, 0))
      .andThen(intake::stopRoller, intake);

    gsRedB = ramseteInit(paths.getGSBR())
      .alongWith(new RunCommand(intake::intake, intake))
      .andThen(() -> drive.tankDriveVolts(0, 0))
      .andThen(intake::stopRoller, intake);

    gsBlueB = ramseteInit(paths.getGSBB())
      .alongWith(new RunCommand(intake::intake, intake))
      .andThen(() -> drive.tankDriveVolts(0, 0))
      .andThen(intake::stopRoller, intake);

    slalom = ramseteInit(paths.getSlalom()).andThen(() -> drive.tankDriveVolts(0, 0));
    barrel = ramseteInit(paths.getBarrel()).andThen(() -> drive.tankDriveVolts(0, 0));
    bounce = ramseteInit(paths.getBounce()).andThen(() -> drive.tankDriveVolts(0, 0));
    
    chooser.addOption("GS-Detection", gsDetection);
    chooser.addOption("GS-RedA", gsRedA);
    chooser.addOption("GS-BlueA", gsBlueA);
    chooser.addOption("GS-RedB", gsRedB);
    chooser.addOption("GS-BlueB", gsBlueB);
    chooser.addOption("Slalom", slalom);
    chooser.addOption("Barrel", barrel);
    chooser.addOption("Bounce", bounce);

    SmartDashboard.putData("Autonmous", chooser);
    
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
    new JoystickButton(driveController, OIConstants.kToggleQuickTurnButton.value).whenPressed(drive::toggleQuickTurn);

    new JoystickButton(driveController, OIConstants.kIntakeButton.value).whenPressed(intake::intake);

    new JoystickButton(driveController, OIConstants.kStopRollerButton.value).whenPressed(intake::stopRoller);

    new JoystickButton(driveController, OIConstants.kOuttakeButton.value).whenPressed(intake::outtake);

    new JoystickButton(driveController, OIConstants.kLiftArmButton.value).whileHeld(() -> arm.moveArm(ArmConstants.kLiftArmSpeed));

    new JoystickButton(driveController, OIConstants.kLowerArmButton.value).whileHeld(() -> arm.moveArm(ArmConstants.kLowerArmSpeed));

  
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
   
    return chooser.getSelected();
  }

  public Command ramseteInit(Trajectory traj) {
    System.out.println("RAMSETEINIT CALLED");

    RamseteCommand ramseteCommand = new RamseteCommand(
        traj,
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

    drive.resetOdometry(traj.getInitialPose());

    return ramseteCommand;
  }

  
}
