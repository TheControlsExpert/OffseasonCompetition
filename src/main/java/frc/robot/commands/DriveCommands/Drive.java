package frc.robot.commands.DriveCommands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.Swerve.SwerveSubsystem;

public class Drive extends Command {    
    private SwerveSubsystem s_Swerve;    
    private DoubleSupplier translationSup;
    private DoubleSupplier strafeSup;
    private DoubleSupplier rotationSup;
    private CommandXboxController controller;
    private int speed;
    private int rotationspeed;
    public Drive(SwerveSubsystem s_Swerve, DoubleSupplier translationSup, DoubleSupplier strafeSup, DoubleSupplier rotationSup, CommandXboxController controller) {
        this.s_Swerve = s_Swerve;
        rotationspeed = 7;
        this.controller = controller;
        addRequirements(s_Swerve);
    
        this.translationSup = translationSup;
        this.strafeSup = strafeSup;
        this.rotationSup = rotationSup;
    }
    @Override
    public void initialize(){
        // s_Swerve.resetGyro();
    }
    @Override
    public void execute() {
        /* Get Values, Deadband*/
        
        if(controller.povDown().getAsBoolean()) {
            speed = 2;
        }   
        
        else{
            speed = 7;
        }
        double translationVal = Math.signum(translationSup.getAsDouble()) * Math.pow(translationSup.getAsDouble(), 2);
        double strafeVal = Math.signum(strafeSup.getAsDouble()) * Math.pow(strafeSup.getAsDouble(), 2);
        double rotationval = Math.signum(rotationSup.getAsDouble()) * Math.pow(rotationSup.getAsDouble(), 2);

        translationVal = MathUtil.applyDeadband(translationVal, 0.05);
        strafeVal = MathUtil.applyDeadband(strafeVal, 0.05);
        rotationval = MathUtil.applyDeadband(rotationval, 0.1);

        /* Drive */
       
            s_Swerve.setChassisSpeeds(new ChassisSpeeds(translationVal, strafeVal, rotationval));
            SmartDashboard.putNumber("translation val", translationVal);
        
}
}