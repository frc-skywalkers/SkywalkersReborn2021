// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

import frc.robot.Paths;
import frc.robot.RobotContainer;
//import frc.robot.MyRamSete;

import edu.wpi.first.wpilibj.trajectory.Trajectory;
import frc.robot.commands.DetectPath;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.Drivetrain;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpiutil.math.Pair;

public class FollowPath extends CommandBase {
  
  private Drivetrain drive;
  private Intake intake;
  private Arm arm;
  //private MyRamSete myRamSete;
  private double pathIndex = -1;
  Paths paths;

  
  public FollowPath(RobotContainer rc) {
    drive = rc.getDriveTrain();
    intake = rc.getIntake();
    paths = rc.getPaths();
    //myRamSete = new MyRamSete(drive);
    addRequirements(drive);
    addRequirements(intake);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() { }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if (interrupted){
      return;
    }
    //Trajectory traj = paths.getPathByIndex(paths.pathIndex);
    Pair<Trajectory[], Boolean> pair = paths.getPathByIndex(paths.pathIndex);
    Trajectory[] traj = pair.getFirst();
    Boolean startRoller = pair.getSecond();

    if (traj == null) {
      return;
    }

    if (startRoller){ //GALACTIC SEARCH
      // new SequentialCommandGroup(
      //   new ParallelCommandGroup(getRamSeteCommand(traj), new RunCommand(intake::intake, intake)),
      //   new InstantCommand(() -> drive.tankDriveVolts(0, 0)),
      //   new InstantCommand(intake::stopRoller)
      // ).schedule();
      SequentialCommandGroup galacticSearch =  new SequentialCommandGroup();
      for (int i = 0; i < traj.length; i++) {
        galacticSearch.addCommands(new ParallelCommandGroup(getRamSeteCommand(traj[i], i), new RunCommand(intake::intake, intake)));
      }
      galacticSearch.addCommands(new InstantCommand(() -> drive.tankDriveVolts(0, 0)), new InstantCommand(intake::stopRoller));
      galacticSearch.schedule();
    }
    else{ //AUTONAV
      // new SequentialCommandGroup(
      //   getRamSeteCommand(traj),
      //   new InstantCommand(() -> drive.tankDriveVolts(0, 0)),
      //   new InstantCommand(intake::stopRoller)
      // ).schedule();
      SequentialCommandGroup autoNav = new SequentialCommandGroup();
      for (int i = 0; i < traj.length; i++) {
        System.out.println("Arrry Length: " + traj.length);
        System.out.println("INDEX: " + i);
        System.out.println("Traj: " + traj[i]);
        autoNav.addCommands(getRamSeteCommand(traj[i], i));
      }
      autoNav.addCommands(new InstantCommand(() -> drive.tankDriveVolts(0, 0)));
      autoNav.schedule();

    }
  }

  private Command getRamSeteCommand(Trajectory traj, int pathNumber) {

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
    if (pathNumber == 0) {
      drive.resetOdometry(traj.getInitialPose());
    } 
    return ramseteCommand;
}
  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }

}
