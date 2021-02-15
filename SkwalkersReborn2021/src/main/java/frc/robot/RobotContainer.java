// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
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

  private XboxController driveController = new XboxController(Constants.OIConstants.kDriverControllerPort);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    drive.setDefaultCommand(
      new RunCommand(
        // () -> drive.arcadeDrive(driveController.getRawAxis(OIConstants.kLeftY) * DriveConstants.kDriveSpeed,
        //   driveController.getRawAxis(OIConstants.kRightX) * DriveConstants.kDriveSpeed), drive)
        //() -> drive.tankDrive(driveController.getRawAxis(OIConstants.kLeftY),
        //    driveController.getRawAxis(OIConstants.kRightY), DriveConstants.kDriveSpeed), drive)
        () -> drive.curvatureDrive(driveController.getRawAxis(OIConstants.kLeftY),
         driveController.getRawAxis(OIConstants.kRightX), DriveConstants.kDriveSpeed), drive)
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
    // An ExampleCommand will run in autonomous
    return new InstantCommand();
  }
}
