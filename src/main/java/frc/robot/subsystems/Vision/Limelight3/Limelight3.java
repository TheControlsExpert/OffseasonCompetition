package frc.robot.subsystems.Vision.Limelight3;


import java.util.ArrayList;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.DoubleArrayTopic;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.LimelightTarget_Detector;
import frc.robot.LimelightHelpers.LimelightTarget_Fiducial;


public class Limelight3 extends SubsystemBase{

  //  public final Matrix

 


public Limelight3() {
    
  }
    

   // public  Pose2d pose_final;

   private String LL3name = "limelight-three";
private double initTimeLED;

   


    

 






 


   

   
 
      

   


   

    
   


    public double getBestArea(LimelightTarget_Fiducial[] apriltagList) {
        double max = 0;

        for (int i =0; i < apriltagList.length; i++) {
            if (apriltagList[i].ta > max) {
                max = apriltagList[i].ta;
            }
        }
        return max;
    }



     public double getBestDistance(LimelightTarget_Fiducial[] apriltagList) {
        double min = 0;

        for (int i =0; i < apriltagList.length; i++) {
            if (apriltagList[i].getCameraPose_TargetSpace().getTranslation().getNorm() < min) {
                min = apriltagList[i].getCameraPose_TargetSpace().getTranslation().getNorm();
            }
        }
        return min;
    }

    
 




    public void setLED() {
        initTimeLED = Timer.getFPGATimestamp();
        
    }

    @Override
    public void periodic() {
        //led control
        if (Timer.getFPGATimestamp() - initTimeLED > 0.5) {
            LimelightHelpers.setLEDMode_ForceOff(LL3name);

        }
        else {
            LimelightHelpers.setLEDMode_ForceOn(LL3name);
        }

        //state control 
      

        //is the pose new
      
    }

    public void initialize() {
        LimelightTarget_Detector[] detector =   LimelightHelpers.getLatestResults(LL3name).targetingResults.targets_Detector;
    }


    public double[] getBestDetection() {
        LimelightHelpers.LimelightResults results = LimelightHelpers.getLatestResults(LL3name);
        LimelightHelpers.LimelightTarget_Detector[] detectoresults  = results.targetingResults.targets_Detector;
        SmartDashboard.putNumber("number detections", detectoresults.length);

        
        double tx = 0;
        double  maxty = 90;
        double ta = 0;

        for (LimelightHelpers.LimelightTarget_Detector detector: detectoresults) {
            if (detector.ty < maxty) {
                 ta = detector.ta;
                 tx = detector.tx;
                 maxty = detector.ty;
                

            }
            
        }

        double[] xNy = {tx, maxty, ta, detectoresults.length};

        return xNy;

    }

}