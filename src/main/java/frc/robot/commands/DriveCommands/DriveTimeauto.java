package frc.robot.commands.DriveCommands;



import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Rollers.IntakeSubsystem;
import frc.robot.subsystems.Rollers.IntakeSubsystem.Checkpoint;
import frc.robot.subsystems.SwerveActual.Swerve;

public class DriveTimeauto extends Command {
    Swerve swervy;
    double time = 2;
    double inittime;
    private IntakeSubsystem intake;

    public DriveTimeauto(Swerve swervo) {
         
        this.swervy = swervo;

        addRequirements(swervo);


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


        return (( Timer.getFPGATimestamp() - inittime ) > time);

        
    }

    @Override
    public void end(boolean interrupted) {
        swervy.drive(new Translation2d(0,0), 0, false, false);
        
    }

    
   
}
