package frc.robot.subsystems.Vision.Limelight3;

import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.LimelightResults;


public class Vision_3_IO_class implements Vision_3_IO {
    
    private String name = "limelight-3";
    
    private LimelightHelpers.LimelightTarget_Detector[] results;


    

    public void updateInputs(Vision_3_IOInputs inputs) {
        results = LimelightHelpers.getLatestResults(name).targets_Detector;
        for (int i = 0; i < results.length; i++) {
            inputs.tx3[i] = results[i].tx;
            inputs.ty3[i] = results[i].ty;
        }
        inputs.hasTargets = results.length > 0;
    }


    public void setLEDs(double n) {
        NetworkTableInstance.getDefault().getTable(name).getEntry("ledMode").setNumber(n);

    }
}