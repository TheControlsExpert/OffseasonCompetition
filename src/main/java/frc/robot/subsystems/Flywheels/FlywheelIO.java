package frc.robot.subsystems.Flywheels;

import org.littletonrobotics.junction.AutoLog;

public interface FlywheelIO {

    @AutoLog
    public static class FlywheelIOInputs {
        public double Tvelocity;
        public double Tcurrent;
        public double Ttemperature;
        public double Bvelocity;
        public double Bcurrent;
        public double Btemperature;
    }


    default void set_T_vel(double velocity) {

    }

    default void set_B_vel(double velocity) {
         
    }

    default void updateInputs(FlywheelIOInputs inputs) {

    }

    
    
}
