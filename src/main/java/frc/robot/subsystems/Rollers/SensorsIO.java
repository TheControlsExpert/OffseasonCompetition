package frc.robot.subsystems.Rollers;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Constants;

public class SensorsIO {
    DigitalInput firstIR = new DigitalInput(Constants.IntakeConstants.firstIR_ID);
    DigitalInput middleIR = new DigitalInput(Constants.IntakeConstants.middleIR_ID);
    DigitalInput lastIR = new DigitalInput(Constants.IntakeConstants.lastIR_ID);


    @AutoLog
    public class SensorIOInputs {
        public boolean firstReading = firstIR.get();
        public boolean middleReading = middleIR.get();
        public boolean lastReading = lastIR.get();
    }


    public void updateInputs(SensorIOInputs inputs) {
        inputs.firstReading = firstIR.get();
        inputs.middleReading = middleIR.get();
        inputs.lastReading = lastIR.get();
    }

    

}