package frc.robot.subsystems.Vision.Limelight3G;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.RobotState;
import frc.robot.Constants.VisionConstants;

public class Vision_3GF_IO_class implements Vision_3GF_IO {
    private Pose2d lastpose_3gf;
    private Pose2d lastpose_3gs;

    private String name_ll3gf;
    private String name_ll3gs;



    public void updateInputs(Vision_3GF_IOInputs inputs) {

        

        //ll3gf (facing shooter)
      LimelightHelpers.SetRobotOrientation(name_ll3gf, RobotState.getEstimatedPosition().getRotation().getDegrees(), RobotState.getAngularVelocityFieldRel(), 0, 0, 0, 0);
      LimelightHelpers.PoseEstimate mt2f = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(name_ll3gf);
      if(Math.abs(RobotState.getAngularVelocityFieldRel()) > Constants.VisionConstants.MAX_ANGULAR_VEL | mt2f.pose.equals(lastpose_3gf) | mt2f.tagCount == 0) // if our angular velocity is greater than 720 degrees per second, ignore vision updates
      {
        inputs.ignoreUpdatef = true;
      }
      else
      {
        inputs.ignoreUpdatef = false;
        inputs.ll3gf_pose = mt2f.pose;
        inputs.timestamp_ll3gf = Timer.getFPGATimestamp() - LimelightHelpers.getLatency_Capture(name_ll3gf) - LimelightHelpers.getLatency_Pipeline(name_ll3gf);
        inputs.tx = LimelightHelpers.getLatestResults(name_ll3gf).targets_Detector[RobotState.getMiddleTag()].tx;
        inputs.avgDistance_3gf = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(name_ll3gf).avgTagDist;
        inputs.avgTagAreaf = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(name_ll3gf).avgTagArea;

     
      }

      //ll3gs (facing intake)

      LimelightHelpers.SetRobotOrientation(name_ll3gs, RobotState.getEstimatedPosition().getRotation().getDegrees(), RobotState.getAngularVelocityFieldRel(), 0, 0, 0, 0);
      LimelightHelpers.PoseEstimate mt2s = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(name_ll3gf);
      if(Math.abs(RobotState.getAngularVelocityFieldRel()) > Constants.VisionConstants.MAX_ANGULAR_VEL | mt2s.pose.equals(lastpose_3gf) | mt2s.tagCount == 0) // if our angular velocity is greater than 720 degrees per second, ignore vision updates
      {
        inputs.ignoreUpdatef = true;
      }
      else
      {
        inputs.ignoreUpdatef = false;
        inputs.ll3gs_pose = mt2s.pose;
        inputs.timestamp_ll3gs = Timer.getFPGATimestamp() - LimelightHelpers.getLatency_Capture(name_ll3gs) - LimelightHelpers.getLatency_Pipeline(name_ll3gs);
        inputs.avgDistance_3gs = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(name_ll3gs).avgTagDist;
        inputs.avgTagAreas = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(name_ll3gs).avgTagArea;
 
      }

        
    }

    public void setLEDsll3gf(double n) {
        NetworkTableInstance.getDefault().getTable(name_ll3gf).getEntry("ledMode").setNumber(n);

    }

    public void setLEDsll3gs(double n) {
        NetworkTableInstance.getDefault().getTable(name_ll3gs).getEntry("ledMode").setNumber(n);

    }

   
}
