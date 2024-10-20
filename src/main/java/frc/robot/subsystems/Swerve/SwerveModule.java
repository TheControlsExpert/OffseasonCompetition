package frc.robot.subsystems.Swerve;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveWheelPositions;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Constants;

public class SwerveModule {

    private ModuleIOInputsAutoLogged inputs = new ModuleIOInputsAutoLogged();
    private SwerveModuleIOFalcon moduleIO;
    private SwerveModuleState previousState = new SwerveModuleState();
    private int index;


    public SwerveModule(SwerveModuleIOFalcon moduleIO, int index) {
        this.moduleIO = moduleIO;
        this.index = index;
    }



    public void updateInputs() {
        moduleIO.updateInputs(inputs);
    }

    public ModuleIOInputsAutoLogged getModInputs() {
        return inputs;

    }

    public SwerveModulePosition getWheelPosition() {

        double Rots = inputs.drivePositionRotations - previousState.angle.getRotations();
        double Meters = Rots * Constants.SwerveConstants.Ratio_MotortoWheel * Constants.SwerveConstants.WheelDiameterMeters * Math.PI;
        Logger.recordOutput("Meters travelled" + index, Meters);
        return new SwerveModulePosition(Meters, new Rotation2d(inputs.turnPosition));

    }

    public void setSetpoint(SwerveModuleState state) {
        moduleIO.runDriveVelocitySetpoint(state.speedMetersPerSecond);
        moduleIO.runTurnPositionSetpoint(state.angle.getRotations());
        previousState = state;
    }
    
}
