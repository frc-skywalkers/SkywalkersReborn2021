package frc.robot;

import java.io.IOException;
import java.nio.file.Path;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.trajectory.constraint.CentripetalAccelerationConstraint;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpiutil.math.Pair;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;

public class Paths {

    private Trajectory[] bounce;
    private Trajectory[] GSAB;
    private Trajectory[] GSAR;
    private Trajectory[] GSBR;
    private Trajectory[] GSBB;
    private Trajectory[] slalom;
    private Trajectory[] barrel;
    

    public double pathIndex = -1;
    public class PathMapper{
        static final int DETECTION = 8;
        static final int GSAR = 1;
        static final int GSAB = 2;
        static final int GSBR = 3;
        static final int GSBB = 4;
        static final int SLALOM = 5;
        static final int BARREL = 6;
        static final int BOUNCE = 7;
    }

    public Paths() {
        var autoVoltageConstraint =
        new DifferentialDriveVoltageConstraint(
            new SimpleMotorFeedforward(DriveConstants.ksVolts,
                                       DriveConstants.kvVoltSecondsPerMeter,
                                       DriveConstants.kaVoltSecondsSquaredPerMeter),
            DriveConstants.kDriveKinematics,
            10);
        
        var autoCentripetalAccelerationConstraint = 
        new CentripetalAccelerationConstraint(0.1);

        // Create config for trajectory
        TrajectoryConfig config =
            new TrajectoryConfig(AutoConstants.kMaxSpeedMetersPerSecond,
                                AutoConstants.kMaxAccelerationMetersPerSecondSquared)
                // Add kinematics to ensure max speed is actually obeyed
                .setKinematics(DriveConstants.kDriveKinematics)
                // Apply the voltage constraint
                .addConstraint(autoVoltageConstraint)
                .addConstraint(autoCentripetalAccelerationConstraint);

        GSAB = new Trajectory[] {pwFileToPath("PathWeaver/Paths/GSABv2.path", config)};
        GSAR = new Trajectory[] {pwFileToPath("PathWeaver/Paths/GSARv4.path", config)};
        GSBR = new Trajectory[] {pwFileToPath("PathWeaver/Paths/GSBRv1.path", config)};
        GSBB = new Trajectory[] {pwFileToPath("PathWeaver/Paths/GSBBv1.path", config)};
        slalom = new Trajectory[] {pwFileToPath("PathWeaver/Paths/Slalomv4.path", config)};
        barrel = new Trajectory[] {pwFileToPath("PathWeaver/Paths/BarrelRacingv1.path", config)};
        bounce = new Trajectory[] {
            pwFileToPath("PathWeaver/Paths/BounceP1.path", config),
            pwFileToPath("PathWeaver/Paths/BounceP2.path", config),
            pwFileToPath("PathWeaver/Paths/BounceP3.path", config),
            pwFileToPath("PathWeaver/Paths/BounceP4.path", config)
        };

                

    }

    public Trajectory jsonToPath(String json) {
        try {
            Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(json);
            return TrajectoryUtil.fromPathweaverJson(trajectoryPath);
        } catch (IOException ex) {
            DriverStation.reportError("Unable to open trajectory: " + json, ex.getStackTrace());
            return new Trajectory();
        }
    }

    public Trajectory pwFileToPath(String json, TrajectoryConfig config) {
        try {
            return Trajectory6391.importPathToQuinticTrajectory("PathWeaver/Paths/GSABv2.path", config);
        } catch (IOException e) {
            DriverStation.reportError("Unable to open trajectory: " + json, e.getStackTrace());
            return new Trajectory();
        }
    }
    public Pair<Trajectory[], Boolean> getPathByIndex(double pIndex) {
        int index = (int)pIndex;
        
        switch (index) {
            case PathMapper.GSAR: return new Pair<Trajectory[], Boolean>(GSAR, true);
            case PathMapper.GSAB: return new Pair<>(GSAB, true);
            case PathMapper.GSBR: return new Pair<>(GSBR, true);
            case PathMapper.GSBB: return new Pair<>(GSBB, true);
            case PathMapper.SLALOM: return new Pair<>(slalom, false);
            case PathMapper.BARREL: return new Pair<>(barrel, false);
            case PathMapper.BOUNCE: return new Pair<>(bounce, false);
            default:  return null; 
        }
    }
    
}
