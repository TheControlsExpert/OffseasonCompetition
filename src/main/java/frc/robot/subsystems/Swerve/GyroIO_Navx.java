package frc.robot.subsystems.Swerve;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.SerialPort.Port;

public class GyroIO_Navx implements GyroIO {
    private AHRS gyro = new AHRS(Port.kUSB);

    public GyroIO_Navx() {
        

    }


    @Override
    public void updateInputs(GyroIOInputs inputs) {
        inputs.connected = gyro.isConnected();
        inputs.yawPositionRads = gyro.getRotation2d().getRadians();
        inputs.Timestamp = Timer.getFPGATimestamp();    
        inputs.yawvelrads = gyro.getRate() * Math.PI / 180;
    }


    public void resetGyro() {
        gyro.reset();
    }

    
}
