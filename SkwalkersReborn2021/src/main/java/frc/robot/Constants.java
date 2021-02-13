/// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.TalonFXInvertType;

import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

    public static final class OIConstants {
        public static final int kDriverControllerPort = 0;
        public static final int kOperatorControllerPort = 1;

        //public static final Button kToggleQuickTurnButton = Button.kA;
        public static final Button kIntakeButton = Button.kA;
        public static final Button kStopRollerButton = Button.kB;
        public static final Button kOuttakeButton = Button.kX;
        public static final Button kLiftArmButton = Button.kBumperRight;
        public static final Button kLowerArmButton = Button.kBumperRight;
        
    }

    public static final class AutoConstants {

		public static double kMaxSpeedMetersPerSecond = 3;
		public static double kMaxAccelerationMetersPerSecondSquared = 3;
		public static double kRamseteB = 2;
		public static double kRamseteZeta = 0.7;

    }

    public static final class DriveConstants {
        public static final int kLeftMasterPort = 1;
        public static final int kLeftFollowerPort = 2;
        public static final int kRightMasterPort = 3;
        public static final int kRightFollowerPort = 4;

        public static final TalonFXInvertType kLeftInvertType = TalonFXInvertType.Clockwise;
        public static final TalonFXInvertType kRightInvertType = TalonFXInvertType.CounterClockwise;

        public static final double kDriveSpeed = 0.8;

        public static final double kTrackWidth = 0.5558; // meters->needs to be inputted
        public static final DifferentialDriveKinematics kDriveKinematics = new DifferentialDriveKinematics(kTrackWidth);
        
        public static final double kWheelRadius = Units.inchesToMeters(3); // meters->needs to be inputted
        public static final int kEncoderResolution = 2048; // <-precision for integrated talon FX encoder
        public static final double kDistancePerPulseFactor = (2 * Math.PI * kWheelRadius)/ kEncoderResolution;
        
        public static double ksVolts = 022;
		public static double kvVoltSecondsPerMeter = 1.98;
		public static double kaVoltSecondsSquaredPerMeter = 0.2;
		public static double kPDriveVel = 8.5;
    }

    public static final class IntakeConstants {
        
        public static final int kRollerPort = 6;

        
        public static final boolean kRollerInvert = false;

        public static final double kIntakeSpeed = 0.8;
        public static final double kOuttakeSpeed = 0.8;
    }

    public static final class ArmConstants {
        public static final int kArmPort = 5;
        public static final boolean kArmInvert = false;
        public static final double kArmSpeed = 0.2;

    }
}
