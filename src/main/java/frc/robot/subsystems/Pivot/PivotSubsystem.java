package frc.robot.subsystems.Pivot;

import org.littletonrobotics.junction.AutoLogOutput;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Pivot.PivotIO.PivotIOInputs;

public class PivotSubsystem extends SubsystemBase {

    enum States {
        AMP,
        SUBWOOFER,
        PODIUM,
        PASSING,
        ALIGNING,
        TRAVELLING,
        HOME
    }

    @AutoLogOutput 
    private States PivotCurrentState = States.HOME;
    private PivotIONEO pivotIO = new PivotIONEO();
    private PivotIOInputsAutoLogged inputs = new PivotIOInputsAutoLogged();
   // private PivotIOInputsAutoLogged inputs = new PivotIOInputsAutoLogged();

    
}
