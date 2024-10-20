package frc.robot.subsystems.Swerve;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

public interface SwerveModuleIO {
  @AutoLog
  class ModuleIOInputs {
    public boolean driveMotorConnected = true;
    public boolean turnMotorConnected = true;

    public double drivePositionRotations = 0.0;
    public double driveVelocityRotationsPerSec = 0.0;
    public double driveAppliedVolts = 0.0; 
    public double Drivelatency = 0;
    public double driveSupplyCurrentAmps = 0.0;

    
  
    public double turnPosition = 0.0;
    public double turnVelocityRotationsPerSec = 0.0;
    public double turnAppliedVolts = 0.0;
    public double turnSupplyCurrentAmps = 0.0;
    public double Steerlatency = 0;


    public double currentTimestamp = 0;

  }

  /** Updates the set of loggable inputs. */
  default void updateInputs(ModuleIOInputs inputs) {}

  /** Run drive motor at volts */
  default void runDriveVolts(double volts) {}

  /** Run turn motor at volts */
  default void runTurnVolts(double volts) {}

  /** Run characterization input (amps or volts) into drive motor */
  default void runCharacterization(double input) {}

  /** Run to drive velocity setpoint with feedforward */
  default void runDriveVelocitySetpoint(double velocityRadsPerSec) {}

  /** Run to turn position setpoint */
  default void runTurnPositionSetpoint(double angleRads) {}

}
