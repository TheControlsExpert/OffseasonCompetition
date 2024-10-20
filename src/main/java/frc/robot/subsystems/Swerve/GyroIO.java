package frc.robot.subsystems.Swerve;

import org.littletonrobotics.junction.AutoLog;

public interface GyroIO {
    


    @AutoLog
    public static class GyroIOInputs {

    public boolean connected = false;
    public double yawPositionRads = 0.0;
    public double Timestamp = 0;
    public double yawvelrads = 0;

  }

  default void updateInputs(GyroIOInputs inputs) {}
}
    

