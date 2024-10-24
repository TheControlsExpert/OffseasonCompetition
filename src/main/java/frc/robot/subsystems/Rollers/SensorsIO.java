package frc.robot.subsystems.Rollers;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Constants;

public class SensorsIO {
    DigitalInput firstIR = new DigitalInput(0);
    DigitalInput lastIR = new DigitalInput(1);

    

    @AutoLog
    public static class SensorsIOInputs {
        public boolean firstReading = false;
        public boolean lastReading = false;
    }


    public void updateInputs(SensorsIOInputs inputs) {
        inputs.firstReading = firstIR.get();
        inputs.lastReading = lastIR.get();
    }

    

}