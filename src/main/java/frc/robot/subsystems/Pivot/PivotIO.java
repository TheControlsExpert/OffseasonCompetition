package frc.robot.subsystems.Pivot;

import org.littletonrobotics.junction.AutoLog;

public interface PivotIO {
    @AutoLog
    public static class PivotIOInputs {
        public double encoderpos;
        public double encodervel;
        public double current;
        public double temperature;
    }

    default void updateInputs(PivotIOInputs inputs) {}

    default void setSetpointInRotations(double setpointInRotations) {
        
    }  

    

    
}  
