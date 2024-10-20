package frc.robot.subsystems.Vision.Limelight3;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import java.util.ArrayList;

public class Vision_3_Subsystem extends SubsystemBase {

    


    public Vision_3_IOInputsAutoLogged inputs = new Vision_3_IOInputsAutoLogged();
    public Vision_3_IO_class vision3IO;
    public Vision_3_Subsystem(Vision_3_IO_class vision3IO) {
        this.vision3IO = vision3IO;

    }

    @Override
    public void periodic() {
        vision3IO.updateInputs(inputs);
    }

    public record Detection(double tx3, double ty3, double distance, boolean hasTargets) {}


    public Detection getBestDetection(double angleofvel) {
       if (inputs.hasTargets) {
        //based on distance
        ArrayList<Detection> bestDetections = new ArrayList<Detection>();
        double bestD = 100;

        for (int i = 0; i< inputs.tx3.length; i++) {
            //angle must be in rad
            double distance = Constants.VisionConstants.Height_LL3 / Math.tan(inputs.ty3[i] + Constants.VisionConstants.Angle_LL3);
            //distances must be within 75 cm of each other
            if (distance < bestD) {
                
                ArrayList<Detection> newArray = new ArrayList<>();
                for (int j = 0; j < bestDetections.size(); j++) {
                    //adds old good detections to new array
                    if (bestDetections.get(j).distance - distance < 0.75) {
                        newArray.add(bestDetections.get(j));
                    }
                }

                bestD = distance;
                bestDetections = newArray;
            }

            else if (distance - bestD < 0.75) {
                bestDetections.add(new Detection(inputs.tx3[i], inputs.ty3[i], distance, true));
            }

        }

        Detection detection_best = bestDetections.get(0);
        for (Detection detection: bestDetections) {
            if (Math.abs(angleofvel-detection.tx3) < Math.abs(angleofvel - detection_best.tx3)) {
                detection_best = detection;
            }
        }

        return detection_best;

       }

       return new Detection(0, 0, 0,  false);
    }

    
    
}
