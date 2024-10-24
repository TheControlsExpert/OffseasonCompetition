// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Autos;
import frc.robot.commands.ExampleCommand;
import frc.robot.commands.DriveCommands.DriveTimeauto;
import frc.robot.commands.DriveCommands.TeleopSwerve;
import frc.robot.commands.IntakeCommands.AlignIntakeDrive;
import frc.robot.commands.IntakeCommands.AligningCombination;
import frc.robot.commands.IntakeCommands.DriveTime;
import frc.robot.commands.IntakeCommands.IntakeCommand;
import frc.robot.commands.IntakeCommands.IntakeManualCommand;
import frc.robot.commands.IntakeCommands.OuttakeCommand;
import frc.robot.commands.IntakeCommands.Shuffler;
import frc.robot.commands.ShooterCommands.Shoot;
import frc.robot.commands.ShooterCommands.Aligning.Amp;
import frc.robot.commands.ShooterCommands.Aligning.Passing;
import frc.robot.commands.ShooterCommands.Aligning.Podiumshot;
import frc.robot.commands.ShooterCommands.Aligning.Subwoofer;
import frc.robot.commands.ShooterCommands.Aligning.ZeroPivot;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.Flywheels.FlywheelIONEO;
import frc.robot.subsystems.Flywheels.ShooterSubsystem;
import frc.robot.subsystems.Pivot.PivotIONEO;
import frc.robot.subsystems.Pivot.PivotSubsystem;
import frc.robot.subsystems.Rollers.IntakeSubsystem;
import frc.robot.subsystems.Rollers.RollersIONEO;
import frc.robot.subsystems.Rollers.SensorsIO;

import frc.robot.subsystems.SwerveActual.Swerve;
import frc.robot.subsystems.Vision.Limelight3.Limelight3;

import javax.swing.plaf.basic.BasicBorders.RolloverButtonBorder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  //private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
  private final RollersIONEO rollers = new RollersIONEO();
  private final SensorsIO sensors = new SensorsIO();
  private final PivotIONEO pivotIO = new PivotIONEO();
  private final FlywheelIONEO shooterIO = new FlywheelIONEO();
  
  public final CommandXboxController m_copilotController = 
  new CommandXboxController(1);    

  public  Limelight3 ll3 = new Limelight3();
  private final IntakeSubsystem intake = new IntakeSubsystem(rollers, sensors, ll3);
  private final PivotSubsystem pivot = new PivotSubsystem(pivotIO, intake, m_copilotController);
  private final ShooterSubsystem shooter = new ShooterSubsystem(shooterIO);

  //private final SwerveSubsystem swervy = new SwerveSubsystem();
  private final Swerve swervo = new Swerve();
  private final XboxController driver = new XboxController(0);


   private final int translationAxis = XboxController.Axis.kLeftY.value;
    private final int strafeAxis = XboxController.Axis.kLeftX.value;
    private final int rotationAxis = XboxController.Axis.kRightX.value;

  // Replace with CommandPS4Controller or CommandJoystick if needed
  public final CommandXboxController m_driverController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);



  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
    // cancelling on release.
   // m_driverController.leftBumper().whileTrue(new IntakeCommand(intake, pivot));
    m_driverController.rightBumper().whileTrue(new OuttakeCommand(intake, pivot));
    m_driverController.x().whileTrue(new IntakeCommand(intake, pivot));
    

    m_driverController.leftTrigger().whileTrue(new Subwoofer(pivot, shooter, m_driverController, intake));
    m_driverController.y().whileTrue(new Podiumshot(swervo,  () -> driver.getRawAxis(translationAxis), () -> driver.getRawAxis(strafeAxis), new PIDController(0.05, 0, 0), shooter, pivot, intake, m_driverController));

    m_copilotController.x().whileTrue(new Shuffler(intake));
    //m_copilotController.a().whileTrue(new Amp(pivot, shooter, m_driverController, intake));

    m_driverController.a().whileTrue(new Amp(pivot, shooter, m_driverController, intake));
    m_driverController.povDown().whileTrue(new Passing(pivot, shooter, m_driverController, intake));

    swervo.setDefaultCommand(
            new TeleopSwerve(
                swervo, 
                () -> driver.getRawAxis(translationAxis), 
                () -> driver.getRawAxis(strafeAxis), 
                () -> driver.getRawAxis(rotationAxis), 
                driver
            )
        );

    m_driverController.povUp().onTrue(swervo.runOnce(() -> swervo.resetGyro()));  

    m_driverController.leftBumper().whileTrue(new AligningCombination(new AlignIntakeDrive(swervo, ll3, new PIDController(8, 0, 0), new PIDController(0.2,0,0), intake, pivot), new DriveTime(intake, swervo, 1)));
   // m_copilotController.x().whileTrue(new ZeroPivot(pivot, m_copilotController));
   


    
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return new Autos(swervo, pivot, shooter, intake).andThen(new DriveTimeauto(swervo));
  }
}
