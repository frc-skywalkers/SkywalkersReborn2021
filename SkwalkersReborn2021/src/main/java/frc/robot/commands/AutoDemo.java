// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Paths;
import frc.robot.RobotContainer;
import frc.robot.Constants.ArmConstants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoDemo extends SequentialCommandGroup {
  /** Creates a new AutoDemo. */

  private Drivetrain drive;
  private Intake intake;
  private Arm arm;
  private Paths paths;

  public AutoDemo(RobotContainer rc) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());

    drive = rc.getDriveTrain();
    intake = rc.getIntake();
    paths = rc.getPaths();
    arm = rc.getArm();

    addCommands(
      new MoveArmForTime(arm, ArmConstants.kLowerArmSpeed, 2),
      drive.createCommandForTrajectory(paths.GSAR_demo, true).deadlineWith(new RunCommand(intake::intake, intake)),
      new MoveArmForTime(arm, ArmConstants.kLiftArmSpeed, 2),
      drive.createCommandForTrajectory(paths.demo_straight, false),
      new RunCommand(intake::outtake, intake).withTimeout(3)
    );
  }
}
