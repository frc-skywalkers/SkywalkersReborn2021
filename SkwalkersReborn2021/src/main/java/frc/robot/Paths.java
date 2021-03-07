package frc.robot;

import java.io.IOException;
import java.nio.file.Path;
import java.util.List;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.util.Units;

public class Paths {
    
    private Trajectory GSAB;
    private Trajectory GSAR;
    private Trajectory slalom;

    public Paths() {
        // config = cf;

        // slalom = TrajectoryGenerator.generateTrajectory(
        //     new Pose2d(1.23, 0.55, new Rotation2d(0)), 
        //     List.of(
        //         new Translation2d(3.03, 2.21), 
        //         new Translation2d(4.64, 2.75),
        //         new Translation2d(6.68, 1.43),
        //         new Translation2d(7.34, 0.64),
        //         new Translation2d(8.36, 0.75),
        //         new Translation2d(8.75, 1.47),
        //         new Translation2d(7.83, 2.44),
        //         new Translation2d(6.91, 2.01),
        //         new Translation2d(6.68, 1.43),
        //         new Translation2d(6.38, 0.89),
        //         new Translation2d(4.6, 0.48),
        //         new Translation2d(2.82, 0.95)), 
        //     new Pose2d(1.25, 2.31, new Rotation2d(180)), 
        //     config);
        
        // GSAR = TrajectoryGenerator.generateTrajectory(
        //     new Pose2d(Units.feetToMeters(2.5), Units.feetToMeters(7.5), new Rotation2d(0)), 
        //     List.of(
        //         new Translation2d(Units.feetToMeters(7.5), Units.feetToMeters(7.5)),
        //         new Translation2d(Units.feetToMeters(12.5), Units.feetToMeters(5)),
        //         new Translation2d(Units.feetToMeters(15), Units.feetToMeters(12.5))), 
        //     new Pose2d(Units.feetToMeters(27.5), Units.feetToMeters(12.5), new Rotation2d(0)), 
        //     config);
        
        // GSAB = TrajectoryGenerator.generateTrajectory(
        //     new Pose2d(0.8, 2.25, new Rotation2d(0)), 
        //     List.of(
        //         new Translation2d(4.6, 0.7),
        //         new Translation2d(5.32, 2.97),
        //         new Translation2d(6.95, 2.25)), 
        //     new Pose2d(8.75, 2.25, new Rotation2d(0)), config);

        String arJSON = "paths/GSARv1.wpilib.json";
        GSAR = new Trajectory();
        try {
            Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(arJSON);
            GSAR = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
        } catch (IOException ex) {
            DriverStation.reportError("Unable to open trajectory: " + arJSON, ex.getStackTrace());
        }

        String abJSON = "paths/GSAB.wpilib.json";
        GSAB = new Trajectory();
        try {
            Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(abJSON);
            GSAB = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
        } catch (IOException ex) {
            DriverStation.reportError("Unable to open trajectory: " + abJSON, ex.getStackTrace());
        }

        String slalomJSON = "paths/slalom37.wpilib.json";
        slalom = new Trajectory();
        try {
            Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(slalomJSON);
            slalom = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
        } catch (IOException ex) {
            DriverStation.reportError("Unable to open trajectory: " + slalomJSON, ex.getStackTrace());
        }


        
    }

    public Trajectory getDetectedPath(double index) {
        if (index == 1.0) {
            System.out.println("RED A---------------");
            return GSAR;
        } else if (index == 2.0) {
            System.out.println("Blue A-------------");
            return GSAB;
        } else {
            System.out.println("TRAJECTORY NOT DETECTED!!!");
            return GSAR;
            
        }
    }

    public Trajectory getSlalom() {
        return slalom;
    }

    
}
