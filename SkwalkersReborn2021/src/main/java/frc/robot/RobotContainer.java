// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.ietf.jgss.Oid;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.Paths.PathMapper;
import frc.robot.commands.DetectPath;
import frc.robot.commands.MoveArmForTime;
import frc.robot.commands.FollowPath;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpiutil.math.Pair;

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
  private final Climber climber = new Climber();

  private XboxController driveController = new XboxController(OIConstants.kDriverControllerPort);

  private Paths paths = new Paths();

  private SendableChooser<Integer> chooser = new SendableChooser<>();
  

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
    

    chooser.addOption("GS-Detection", PathMapper.DETECTION);
    chooser.addOption("GS-RedA", PathMapper.GSAR);
    chooser.addOption("GS-BlueA", PathMapper.GSAB);
    chooser.addOption("GS-RedB", PathMapper.GSBR);
    chooser.addOption("GS-BlueB", PathMapper.GSBB);
    chooser.addOption("Slalom", PathMapper.SLALOM);
    chooser.addOption("Barrel", PathMapper.BARREL);
    chooser.addOption("Bounce", PathMapper.BOUNCE);


    SmartDashboard.putData("Autonmous", chooser);
    
    // Configure the button bindings
    configureButtonBindings();
  }

  public Intake getIntake()
  {
    return intake;
  }

  public Drivetrain getDriveTrain()
  {
    return drive;
  }

  

  public Paths getPaths()
  {
    return paths;
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

    new JoystickButton(driveController, OIConstants.kClimbButton.value).whileHeld(() -> climber.climb());

    new JoystickButton(driveController, OIConstants.kDescendButton.value).whileHeld(() -> climber.lower());

  
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    Integer selected = chooser.getSelected();
    //MyRamSete mrs = new MyRamSete(drive);
    //selected = PathMapper.BARREL; //CAN BE COMMENTED OUT (HARDCODED)
    if (selected == PathMapper.DETECTION) {
      return new SequentialCommandGroup(
                new PrintCommand("START OF SEQUENTIAL COMMANDS"),
                new MoveArmForTime(arm, ArmConstants.kLowerArmSpeed, 3),
                new PrintCommand("START OF WAIT"),
                new WaitCommand(3),
                new PrintCommand("DETECT PATH"),
                new DetectPath(this),
                new PrintCommand("DETECTION DONE"),
                new FollowPath(this)
              );
    } else {
      // return mrs.getRamSeteCommand(paths.getPathByIndex(selected))
      //   .alongWith(new RunCommand(intake::intake, intake))
      //   .andThen(() -> drive.tankDriveVolts(0, 0))
      //   .andThen(intake::stopRoller, intake);
      paths.pathIndex = selected;
      return new FollowPath(this);
    }
  } 
}
