// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

import frc.robot.Paths;
import frc.robot.RobotContainer;

import frc.robot.commands.DetectPath;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;


public class DetectPath extends CommandBase {
  /** Creates a new Detectpath. */
  private NetworkTableInstance inst = NetworkTableInstance.getDefault();
  private NetworkTable table = inst.getTable("datatable");
  private Drivetrain drive;
  private Intake intake;
  private double pathIndex = -1;
  Paths paths;

  
  public DetectPath(RobotContainer rc) {
    drive = rc.getDriveTrain();
    intake = rc.getIntake();
    paths = rc.getPaths();
    addRequirements(drive);
    addRequirements(intake);

    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    pathIndex = table.getEntry("path").getDouble(-1);
    System.out.println("PATH RETURNED:::" + pathIndex);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (pathIndex >= 1.0) {
      System.out.println("PATH DETECTED == " + pathIndex);
      // Update the pathIndex in the paths
      paths.pathIndex = pathIndex;
      return true;
    } else {
      System.out.println("NOT DETECTED YET");
      return false;
    }
  }
}
