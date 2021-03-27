package frc.robot;

import java.io.IOException;
import java.nio.file.Path;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryUtil;
import edu.wpi.first.wpiutil.math.Pair;

public class Paths {
    
    private Trajectory GSAB;
    

    private Trajectory GSAR;
   

    private Trajectory GSBR;
    

    private Trajectory GSBB;
    

    private Trajectory slalom;
    
    private Trajectory barrel;
    

    // private Trajectory bounce;

    private Trajectory bounceP1;
    private Trajectory bounceP2;
    private Trajectory bounceP3;
    private Trajectory bounceP4;

    private Trajectory[] bounce_list = new Trajectory[4];
    private Trajectory[] GSAB_list = {GSAB};
    private Trajectory[] GSAR_list = {GSAR};
    private Trajectory[] GSBR_list = {GSBR};
    private Trajectory[] GSBB_list = {GSBB};
    private Trajectory[] slalom_list = {slalom};
    private Trajectory[] barrel_list = {barrel};
    

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

        GSAR = jsonToPath("paths/GSARv4.wpilib.json");
        GSAB = jsonToPath("paths/GSABv2.wpilib.json");
        GSBB = jsonToPath("paths/GSBBv1.wpilib.json");
        GSBR = jsonToPath("paths/GSBRv1.wpilib.json");

        slalom = jsonToPath("paths/Slalomv4.wpilib.json");
        barrel = jsonToPath("paths/BarrelRacingv1.wpilib.json");
        // bounce = jsonToPath("paths/Bouncev1.wpilib.json");
        bounceP1 = jsonToPath("paths/BounceP1.wpilib.json");
        bounceP2 = jsonToPath("paths/BounceP2.wpilib.json");
        bounceP3 = jsonToPath("paths/BounceP3.wpilib.json");
        bounceP4 = jsonToPath("paths/BounceP4.wpilib.json");
        bounce_list[0] = bounceP1;
        bounce_list[1] = bounceP2;
        bounce_list[2] = bounceP3;
        bounce_list[3] = bounceP4;
                

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

    // public Trajectory getDetectedPath() {
    //     if (pathIndex == 1.0) {
    //         System.out.println("RED A---------------");
    //         return GSAR;
    //     } else if (pathIndex == 2.0) {
    //         System.out.println("Blue A-------------");
    //         return GSAB;
    //     } else {
    //         System.out.println("TRAJECTORY NOT DETECTED!!!");
    //         return new Trajectory();
            
    //     }
    // }
    public Pair<Trajectory[], Boolean> getPathByIndex(double pIndex) {
        int index = (int)pIndex;
        
        switch (index) {
            case PathMapper.GSAR: return new Pair<Trajectory[], Boolean>(GSAR_list, true);
            case PathMapper.GSAB: return new Pair<>(GSAB_list, true);
            case PathMapper.GSBR: return new Pair<>(GSBR_list, true);
            case PathMapper.GSBB: return new Pair<>(GSBB_list, true);
            case PathMapper.SLALOM: return new Pair<>(slalom_list, false);
            case PathMapper.BARREL: return new Pair<>(barrel_list, false);
            case PathMapper.BOUNCE: return new Pair<>(bounce_list, false);
            default:  return null; 
        }
    }
    
//  PREVIOUS CODE:
    // public Trajectory getPathByIndex(double pIndex) {
    //     int index = (int)pIndex;
        
    //     switch (index) {
    //         case 1: return GSAR;
    //         case 2: return GSAB;
    //         case 3: return GSBR;
    //         case 4: return GSBB;
    //         case 5: return slalom;
    //         case 6: return barrel;
    //         case 7: return bounce;
    //         default:  return null; 
    //     }
    // }
    
}
