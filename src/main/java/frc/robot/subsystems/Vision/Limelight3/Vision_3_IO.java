package frc.robot.subsystems.Vision.Limelight3;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Pose2d;

public interface Vision_3_IO {
    
    @AutoLog
    public static class Vision_3_IOInputs {


        //ll3 (intake detection)

        public double[] tx3;
        public double[] ty3;
        public boolean hasTargets;

    }

    default void updateInputs(Vision_3_IOInputs inputs) {}

    default void setLEDs(double n) {}





    
}










































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































