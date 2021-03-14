// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Arm extends SubsystemBase {
  /** Creates a new Arm. */
  private final CANSparkMax arm = new CANSparkMax(Constants.ArmConstants.kArmPort, MotorType.kBrushless);
  
  public Arm() {
    arm.restoreFactoryDefaults();
    arm.setInverted(Constants.ArmConstants.kArmInvert);

  }

  public void moveArm(double speed) {
    arm.set(speed);

  }

  public void stop() {
    arm.set(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
