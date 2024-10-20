package frc.robot.commands.DriveCommands;

import frc.robot.Constants;
import frc.robot.subsystems.SwerveActual.Swerve;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.XboxController;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;


public class TeleopSwerve extends Command {    
    private Swerve s_Swerve;    
    private DoubleSupplier translationSup;
    private DoubleSupplier strafeSup;
    private DoubleSupplier rotationSup;
    private XboxController controller;
    private BooleanSupplier robotCentricSup;
    private int speed;
    private int rotationspeed;
    public TeleopSwerve(Swerve s_Swerve, DoubleSupplier translationSup, DoubleSupplier strafeSup, DoubleSupplier rotationSup, XboxController controller) {
        this.s_Swerve = s_Swerve;
        rotationspeed = 7;
        this.controller = controller;
        addRequirements(s_Swerve);
    
        this.translationSup = translationSup;
        this.strafeSup = strafeSup;
        this.rotationSup = rotationSup;
        SmartDashboard.putBoolean("Swerve ON", true);
    }
    @Override
    public void initialize(){
        // s_Swerve.resetGyro();
    }
    @Override
    public void execute() {
        /* Get Values, Deadband*/
        
        if(controller.getBButton()) {
            speed = 2;
            rotationspeed = 3;
        }   
        
        else{
            speed = 10;
            rotationspeed = 5;
        }
        double translationVal = Math.signum(translationSup.getAsDouble()) * Math.pow(translationSup.getAsDouble(), 2);
        double strafeVal = Math.signum(strafeSup.getAsDouble()) * Math.pow(strafeSup.getAsDouble(), 2);
        double rotationval = Math.signum(rotationSup.getAsDouble()) * Math.pow(rotationSup.getAsDouble(), 2);

        translationVal = MathUtil.applyDeadband(translationVal, 0.05);
        strafeVal = MathUtil.applyDeadband(strafeVal, 0.05);
        rotationval = MathUtil.applyDeadband(rotationval, 0.1);

        /* Drive */
       
            s_Swerve.drive(
                new Translation2d(-translationVal, -strafeVal).times(speed), 
                -rotationval * rotationspeed, 
                true, 
                true
                );
        }
        
}