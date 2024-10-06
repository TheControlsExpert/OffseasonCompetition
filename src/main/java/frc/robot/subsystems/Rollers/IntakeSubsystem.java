package frc.robot.subsystems.Rollers;

import org.littletonrobotics.junction.AutoLogOutput;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Rollers.RollersIO.RollersIOInputs;


public class IntakeSubsystem extends SubsystemBase {
    public enum Checkpoint {
        //intaking status
        DETECTED,
        REACHED_HANDOFF,
        REACHED_SHOOTER,
        STATIONED,

        //outtaking status
        OUTTAKING,
        TRANSITIONING,   

        //No note or just initialized (we don't want intake turning on)
        IDLE,

        //Shooting

        EJECTED
    }


    @AutoLogOutput(key = "IntakeSubsystem-LastCompletedCheckpoint")
    private Checkpoint CompletedCheckpoint = Checkpoint.IDLE;

    private final RollersIO io;
    private final RollersIOInputsAutoLogged inputs;
}