package frc.robot;

import java.io.IOException;
import java.nio.file.Path;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryUtil;

public class Paths {
    
    private Trajectory GSAB;
    private Trajectory GSAR;
    private Trajectory GSBR;
    private Trajectory GSBB;

    private Trajectory slalom;
    private Trajectory barrel;
    private Trajectory bounce;

    public static double pathIndex = -1;

    public Paths() {

        GSAR = jsonToPath("paths/GSARv4.wpilib.json");
        GSAB = jsonToPath("paths/GSABv2.wpilib.json");
        GSBB = jsonToPath("paths/GSBBv1.wpilib.json");
        GSBR = jsonToPath("/paths/GSBRv1.wpilib.json");

        slalom = jsonToPath("paths/Slalomv4.wpilib.json");
        barrel = jsonToPath("paths/BarrelRacingv1.wpilib.json");
        bounce = jsonToPath("paths/Bouncev1.wpilib.json");

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

    public Trajectory getDetectedPath() {
        if (pathIndex == 1.0) {
            System.out.println("RED A---------------");
            return GSAR;
        } else if (pathIndex == 2.0) {
            System.out.println("Blue A-------------");
            return GSAB;
        } else {
            System.out.println("TRAJECTORY NOT DETECTED!!!");
            return new Trajectory();
            
        }
    }

    public Trajectory getGSAR() {
        return GSAR;
    }
    
    public Trajectory getGSAB() {
        return GSAB;
    }

    public Trajectory getGSBR() {
        return GSBR;
    }

    public Trajectory getGSBB() {
        return GSBB;
    }

    public Trajectory getSlalom() {
        return slalom;
    }

    public Trajectory getBarrel() {
        return barrel;
    }

    public Trajectory getBounce() {
        return bounce;
    }

    
}
