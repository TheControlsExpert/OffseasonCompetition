package frc.robot.commands.IntakeCommands;



import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Rollers.IntakeSubsystem;
import frc.robot.subsystems.Rollers.IntakeSubsystem.Checkpoint;
import frc.robot.subsystems.SwerveActual.Swerve;

public class DriveTime extends Command {
    Swerve swervy;
    double time;
    double inittime;
    private IntakeSubsystem intake;

    public DriveTime(IntakeSubsystem intake, Swerve swervo, double time) {
         
        this.swervy = swervo;
        this.intake = intake;
       this.time = time;

        addRequirements(swervo, intake);


    }
    @Override
    public void initialize() {
        inittime =  Timer.getFPGATimestamp();
    }

    @Override
    public void execute() {
       swervy.drive(new Translation2d(3, 0 ), 0, false, false);
    }

    @Override
    public boolean isFinished() {
        return intake.CompletedCheckpoint.equals(Checkpoint.DETECTED) |(( Timer.getFPGATimestamp() - inittime ) > time);

        
    }

    @Override
    public void end(boolean interrupted) {
        if (( Timer.getFPGATimestamp() - inittime ) > time) {
            intake.CompletedCheckpoint = Checkpoint.IDLE;   
        }
        
    }

    
   
}
