// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RPM;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.BallElevator;
import frc.robot.subsystems.IntakeMain;
import frc.robot.subsystems.IntakePivot;
import frc.robot.subsystems.Rollers;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import java.io.File;
import swervelib.SwerveInputStream;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a "declarative" paradigm, very
 * little robot logic should actually be handled in the {@link Robot} periodic methods (other than the scheduler calls).
 * Instead, the structure of the robot (including subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer
{

 public static final double PIVOT_DOWN_SPEED = 0.35;
 public static final double PIVOT_UP_SPEED = -0.35;

  // Replace with CommandPS4Controller or CommandJoystick if needed
  final         CommandXboxController driverXbox = new CommandXboxController(0);
  final         CommandXboxController operatorXbox = new CommandXboxController(1);
  // The robot's subsystems and commands are defined here...
  private final SwerveSubsystem       drivebase  = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),
                                                                                "swerve"));

  private final IntakeMain intakeMain = new IntakeMain();
  private final IntakePivot intakePivot = new IntakePivot();
  private final Shooter shooter = new Shooter();
  private final BallElevator ballElevator = new BallElevator();
  private final Rollers rollers = new Rollers();
  // Establish a Sendable Chooser that will be able to be sent to the SmartDashboard, allowing selection of desired auto
  private final SendableChooser<Command> autoChooser;

  /**
   * Converts driver input into a field-relative ChassisSpeeds that is controlled by angular velocity.
   */
  SwerveInputStream driveAngularVelocity = SwerveInputStream.of(drivebase.getSwerveDrive(),
                                                                () -> driverXbox.getLeftY(),
                                                                () -> driverXbox.getLeftX())
                                                            .withControllerRotationAxis(() -> driverXbox.getRightX() * -1)
                                                            .deadband(OperatorConstants.DEADBAND)
                                                            .scaleTranslation(0.8)
                                                            .allianceRelativeControl(true);

  /**
   * Clone's the angular velocity input stream and converts it to a fieldRelative input stream.
   */
  SwerveInputStream driveDirectAngle = driveAngularVelocity.copy().withControllerHeadingAxis(driverXbox::getRightX,
                                                                                             driverXbox::getRightY)
                                                           .headingWhile(true);

  /**
   * Clone's the angular velocity input stream and converts it to a robotRelative input stream.
   */
  SwerveInputStream driveRobotOriented = driveAngularVelocity.copy().robotRelative(true)
                                                             .allianceRelativeControl(false);

  SwerveInputStream driveAngularVelocityKeyboard = SwerveInputStream.of(drivebase.getSwerveDrive(),
                                                                        () -> -driverXbox.getLeftY(),
                                                                        () -> -driverXbox.getLeftX())
                                                                    .withControllerRotationAxis(() -> driverXbox.getRawAxis(
                                                                        2))
                                                                    .deadband(OperatorConstants.DEADBAND)
                                                                    .scaleTranslation(0.8)
                                                                    .allianceRelativeControl(true);
  // Derive the heading axis with math!
  SwerveInputStream driveDirectAngleKeyboard     = driveAngularVelocityKeyboard.copy()
                                                                               .withControllerHeadingAxis(() ->
                                                                                                              Math.sin(
                                                                                                                  driverXbox.getRawAxis(
                                                                                                                      2) *
                                                                                                                  Math.PI) *
                                                                                                              (Math.PI *
                                                                                                               2),
                                                                                                          () ->
                                                                                                              Math.cos(
                                                                                                                  driverXbox.getRawAxis(
                                                                                                                      2) *
                                                                                                                  Math.PI) *
                                                                                                              (Math.PI *
                                                                                                               2))
                                                                               .headingWhile(true)
                                                                               .translationHeadingOffset(true)
                                                                               .translationHeadingOffset(Rotation2d.fromDegrees(
                                                                                   0));

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer()
  {
     // Set the default command to force the arm to go to 0.
   // intakePivot.setDefaultCommand(intakePivot.setAngle(Degrees.of(0)));
    // Configure the trigger bindings
    configureBindings();
    DriverStation.silenceJoystickConnectionWarning(true);
    
    //Create the NamedCommands that will be used in PathPlanner
    NamedCommands.registerCommand("startShoot", ballElevator.set(1).alongWith(rollers.set(0.25)));
    NamedCommands.registerCommand("stopShoot", ballElevator.set(1).alongWith(rollers.set(0.25)));

    NamedCommands.registerCommand("startIntake" , intakeMain.set(1));
    NamedCommands.registerCommand("stopIntake", intakeMain.set(0));


    //Have the autoChooser pull in all PathPlanner autos as options
    autoChooser = AutoBuilder.buildAutoChooser();

    //Set the default auto (do nothing) 
    autoChooser.setDefaultOption("Do Nothing", Commands.none());

    //Add a simple auto option to have the robot drive forward for 1 second then stop
    autoChooser.addOption("Drive Forward", drivebase.driveForward().withTimeout(1));

    //auto option to have the robot move back and shoot0\11
    autoChooser.addOption("Standard Auto", drivebase.driveForward().withTimeout(1.6)
                                                .andThen(intakePivot.set(PIVOT_DOWN_SPEED).alongWith(ballElevator.set(1)).alongWith(shooter.setVelocity(RPM.of(3100))).withTimeout(1))
                                                .andThen(intakePivot.set(PIVOT_UP_SPEED).alongWith(ballElevator.set(1)).alongWith(shooter.setVelocity(RPM.of(3100))).withTimeout(1))
                                                .andThen(intakePivot.set(PIVOT_DOWN_SPEED).alongWith(ballElevator.set(1)).alongWith(shooter.setVelocity(RPM.of(3100))).withTimeout(1))
                                                .andThen(intakePivot.set(PIVOT_UP_SPEED).alongWith(ballElevator.set(1)).alongWith(shooter.setVelocity(RPM.of(3100))).withTimeout(1))
                                                .andThen(intakePivot.set(PIVOT_DOWN_SPEED).alongWith(ballElevator.set(1)).alongWith(shooter.setVelocity(RPM.of(3100))).withTimeout(1))
                                                .andThen(intakePivot.set(PIVOT_UP_SPEED).alongWith(ballElevator.set(1)).alongWith(shooter.setVelocity(RPM.of(3100))).withTimeout(1))
                                                .andThen(intakePivot.set(PIVOT_DOWN_SPEED).alongWith(ballElevator.set(1)).alongWith(shooter.setVelocity(RPM.of(3100))).withTimeout(1))
                                                .andThen(intakePivot.set(PIVOT_UP_SPEED).alongWith(ballElevator.set(1)).alongWith(shooter.setVelocity(RPM.of(3100))).withTimeout(1))
                                                 .andThen(intakePivot.set(PIVOT_DOWN_SPEED).alongWith(ballElevator.set(1)).alongWith(shooter.setVelocity(RPM.of(3100))).withTimeout(1))
                                                .andThen(intakePivot.set(PIVOT_UP_SPEED).alongWith(ballElevator.set(1)).alongWith(shooter.setVelocity(RPM.of(3100))).withTimeout(1))
                                                .andThen(intakePivot.set(PIVOT_DOWN_SPEED).alongWith(ballElevator.set(1)).alongWith(shooter.setVelocity(RPM.of(3100))).withTimeout(1))
                                                .andThen(intakePivot.set(PIVOT_UP_SPEED).alongWith(ballElevator.set(1)).alongWith(shooter.setVelocity(RPM.of(3100))).withTimeout(1))
                                                .andThen(drivebase.drivebackward().withTimeout(1.6)));
    
    //Put the autoChooser on the SmartDashboard
    SmartDashboard.putData("Auto Chooser", autoChooser);

    
    // Set the default command to force the intake main rest.
    intakeMain.setDefaultCommand(intakeMain.set(0));

    // set the default comman to foroce the shooter rest. 3500
    shooter.setDefaultCommand(shooter.setVelocity(RPM.of(3200)));

    // set the default command to force the ballElevator rest.
    ballElevator.setDefaultCommand(ballElevator.set(0));

    // set the default command to force rollers to rest.
    rollers.setDefaultCommand(rollers.set(0));

    //set the default comman to force the intakePivot down
   // intakePivot.setDefaultCommand(intakePivot.set(0.25));

   

  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary predicate, or via the
   * named factories in {@link edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller PS4}
   * controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight joysticks}.
   */
  private void configureBindings()
  {
    Command driveFieldOrientedDirectAngle      = drivebase.driveFieldOriented(driveDirectAngle);
    Command driveFieldOrientedAnglularVelocity = drivebase.driveFieldOriented(driveAngularVelocity);
    Command driveRobotOrientedAngularVelocity  = drivebase.driveFieldOriented(driveRobotOriented);
    Command driveSetpointGen = drivebase.driveWithSetpointGeneratorFieldRelative(
        driveDirectAngle);
    Command driveFieldOrientedDirectAngleKeyboard      = drivebase.driveFieldOriented(driveDirectAngleKeyboard);
    Command driveFieldOrientedAnglularVelocityKeyboard = drivebase.driveFieldOriented(driveAngularVelocityKeyboard);
    Command driveSetpointGenKeyboard = drivebase.driveWithSetpointGeneratorFieldRelative(
        driveDirectAngleKeyboard);

    if (RobotBase.isSimulation())
    {
      drivebase.setDefaultCommand(driveFieldOrientedDirectAngleKeyboard);
    } else
    {
      drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocity);
    }

    if (Robot.isSimulation())
    {
      Pose2d target = new Pose2d(new Translation2d(1, 4),
                                 Rotation2d.fromDegrees(90));
      //drivebase.getSwerveDrive().field.getObject("targetPose").setPose(target);
      driveDirectAngleKeyboard.driveToPose(() -> target,
                                           new ProfiledPIDController(5,
                                                                     0,
                                                                     0,
                                                                     new Constraints(5, 2)),
                                           new ProfiledPIDController(5,
                                                                     0,
                                                                     0,
                                                                     new Constraints(Units.degreesToRadians(360),
                                                                                     Units.degreesToRadians(180))
                                           ));
      driverXbox.start().onTrue(Commands.runOnce(() -> drivebase.resetOdometry(new Pose2d(3, 3, new Rotation2d()))));
      driverXbox.button(1).whileTrue(drivebase.sysIdDriveMotorCommand());
      driverXbox.button(2).whileTrue(Commands.runEnd(() -> driveDirectAngleKeyboard.driveToPoseEnabled(true),
                                                     () -> driveDirectAngleKeyboard.driveToPoseEnabled(false)));

//      driverXbox.b().whileTrue(
//          drivebase.driveToPose(
//              new Pose2d(new Translation2d(4, 4), Rotation2d.fromDegrees(0)))
//                              );

    }
    if (DriverStation.isTest())
    {
      drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocity); // Overrides drive command above!

      driverXbox.x().whileTrue(Commands.runOnce(drivebase::lock, drivebase).repeatedly());
      driverXbox.start().onTrue((Commands.runOnce(drivebase::zeroGyro)));
      driverXbox.back().whileTrue(drivebase.centerModulesCommand());
      driverXbox.leftBumper().onTrue(Commands.none());
      driverXbox.rightBumper().onTrue(Commands.none());
    } else
    {
      driverXbox.a().onTrue((Commands.runOnce(drivebase::zeroGyroWithAlliance)));
      driverXbox.x().onTrue(Commands.runOnce(drivebase::addFakeVisionReading));
      driverXbox.start().whileTrue(Commands.none());
      driverXbox.back().whileTrue(Commands.none());
      driverXbox.leftBumper().whileTrue(Commands.runOnce(drivebase::lock, drivebase).repeatedly());
      driverXbox.rightBumper().onTrue(Commands.none());
    }
// intake 
operatorXbox.leftBumper().whileTrue(intakeMain.set(1));
operatorXbox.rightBumper().whileTrue(intakeMain.set(-1));


// shoot 
operatorXbox.a().whileTrue(ballElevator.set(1).alongWith(rollers.set(.25)));

// intakePivot requested to comment out because of chain untill it is fixed.
operatorXbox.povUp().whileTrue(intakePivot.set(PIVOT_UP_SPEED).alongWith(intakeMain.set(0.95)));
operatorXbox.povDown().whileTrue(intakePivot.set(PIVOT_DOWN_SPEED));

//intake piviot move down
//operatorXbox.y().whileTrue(intakePivot.set(0.25));

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand()
  {
    // Pass in the selected auto from the SmartDashboard as our desired autnomous commmand 
    return autoChooser.getSelected();
  }

  public void setMotorBrake(boolean brake)
  {
    drivebase.setMotorBrake(brake);
  }

  public void resetHeading()
  {
    drivebase.zeroGyroWithAlliance();
  }
}
