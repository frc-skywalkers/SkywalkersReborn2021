// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Arm;

public class MoveArmForTime extends CommandBase {
  /** Creates a new MoveArmForTime. */

  private Arm arm;
  private Timer timer;
  private double seconds;
  private double speed;
  private boolean finished = false;

  public MoveArmForTime(Arm a, double sp, double secs) {
    // Use addRequirements() here to declare subsystem dependencies.
    arm = a;
    addRequirements(arm);
    seconds = secs;
    speed = sp;
    timer = new Timer();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.reset();
    timer.start();
    while (timer.get() < seconds) {
      arm.moveArm(speed);
    }
    finished = true;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    arm.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return finished;
  }
}
