// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;

public class Climber extends SubsystemBase {
  /** Creates a new Climber. */

  private final CANSparkMax climber = new CANSparkMax(ClimberConstants.kClimberPort, MotorType.kBrushless);

  public Climber() {
    climber.restoreFactoryDefaults();
    climber.setInverted(ClimberConstants.kClimberInvert);
    climber.set(0);

  }

  public void climb() {
    climber.set(ClimberConstants.kClimbSpeed);
  }

  public void lower() {
    climber.set(-ClimberConstants.kClimbSpeed);
  }

  public void stop() {
    climber.set(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
