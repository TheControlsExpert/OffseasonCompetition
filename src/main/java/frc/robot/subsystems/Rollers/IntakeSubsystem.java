package frc.robot.subsystems.Rollers;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Rollers.RollersIO.RollersIOInputs;
import frc.robot.subsystems.Rollers.SensorsIO.SensorsIOInputs;


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

    private  RollersIO Rollersio; 
    private final RollersIOInputsAutoLogged inputs_rollers = new RollersIOInputsAutoLogged();

    //private final SensorsIO Sensorsio = new SensorsIO();
    
    
    private SensorsIO Sensorsio;
    
    //private final SensorsIOInputsAutoLogged 
    //private final SensorsIOInputsAutoLogged inputs_sensors = new SensorsIOInputsAutoLogged();

    public IntakeSubsystem(RollersIO ioRollers, SensorsIO Sensorsio) {
        this.Rollersio = Rollersio;
        this.Sensorsio = Sensorsio;
    }



    @Override
    public void periodic() {
        //io.updateInputs(inputs);
        //Logger.processInputs("Intake Motors", inputs);
    }


}