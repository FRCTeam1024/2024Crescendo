package frc.robot;

import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.Autos;
import frc.robot.subsystems.*;
import monologue.Logged;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer implements Logged {
  /* Controllers */
  private final XboxController driver = new XboxController(0);
  private final CommandXboxController operator = new CommandXboxController(1);

  /* Drive Controls */
  private final int translationAxis = XboxController.Axis.kLeftY.value;
  private final int strafeAxis = XboxController.Axis.kLeftX.value;
  private final int rotationXAxis = XboxController.Axis.kRightX.value;
  private final int rotationYAxis = XboxController.Axis.kRightY.value;

  /* Driver Buttons */
  private final JoystickButton zeroGyro =
      new JoystickButton(driver, XboxController.Button.kY.value);
  private final JoystickButton robotCentric =
      new JoystickButton(driver, XboxController.Button.kLeftBumper.value);
  private final JoystickButton headingControl =
      new JoystickButton(driver, XboxController.Button.kRightBumper.value);
  private final JoystickButton targetTrack =
      new JoystickButton(driver, XboxController.Button.kA.value);

  /* Subsystems */
  private final Swerve swerve = new Swerve();

  private final Climber climber = new Climber();

  private final Intake intake = new Intake();
  private final Feed feed = new Feed();
  private final Shooter shooter = new Shooter();
  private final EndEffector endEffector = new EndEffector(intake, feed, shooter);

  private final Wrist wrist = new Wrist();
  private final Arm arm = new Arm();
  private final Superstructure superstructure = new Superstructure(arm, wrist);

  private final Autos autos = new Autos(swerve, superstructure, endEffector);

  private final SendableChooser<Command> autoChooser = new SendableChooser<>();

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    swerve.setDefaultCommand(
        swerve.teleopDriveCommand(
            () -> -driver.getRawAxis(translationAxis),
            () -> -driver.getRawAxis(strafeAxis),
            () -> -driver.getRawAxis(rotationXAxis)));

    // Configure the button bindings
    initializeNamedCommands();
    configureButtonBindings();
    setupAutoChooser();
    setupDashboard();
  }

  public void setupAutoChooser() {
    autoChooser.setDefaultOption("Drive Straight", autos.driveStraight());
    autoChooser.addOption("Circuit", autos.circuitAuto());
    autoChooser.addOption("NL Speaker", autos.runPathPlannerAuto("NL Speaker"));
    autoChooser.addOption("LNE1", autos.runPathAsAuto("LNEnd Speaker"));
    autoChooser.addOption("NL AMP", autos.runPathPlannerAuto("NL AMP"));
    autoChooser.addOption("RN Start", autos.runPathAsAuto("RN Start"));
    autoChooser.addOption("3 Near", autos.runPathPlannerAuto("3 Near"));
    autoChooser.addOption("NM Speaker", autos.runPathPlannerAuto("NM Speaker"));
    autoChooser.addOption("NR Speaker", autos.runPathPlannerAuto("NR Speaker"));
    autoChooser.addOption("LN and F", autos.runPathPlannerAuto("LN and F"));
    autoChooser.addOption("FR", autos.runPathPlannerAuto("FR"));
    autoChooser.addOption("MN and F", autos.runPathPlannerAuto("MN and F"));
    SmartDashboard.putData(autoChooser);
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    /* Driver Buttons */
    zeroGyro.onTrue(new InstantCommand(() -> swerve.zeroHeading()));

    headingControl.whileTrue(
        swerve.teleopHeadingDriveCommand(
            () -> -driver.getRawAxis(translationAxis),
            () -> -driver.getRawAxis(strafeAxis),
            () -> -driver.getRawAxis(rotationYAxis),
            () -> -driver.getRawAxis(rotationXAxis),
            targetTrack));

    /*Operator Buttons */
    SmartDashboard.putNumber("Shooter Speed (RPS)", 70);
    // Spin up shooter
    operator
        .rightTrigger()
        .whileTrue(
            shooter.velocityCommand(() -> SmartDashboard.getNumber("Shooter Speed (RPS)", 0)));
    // Fire
    operator.y().onTrue(endEffector.fireNote());
    // Reverse shooter
    operator.rightBumper().whileTrue(shooter.velocityCommand(-10));
    // Intake
    operator.leftTrigger().whileTrue(endEffector.intakeNote());
    // Reverse intake
    operator
        .leftBumper()
        .whileTrue(intake.runIntakeCommand(-0.7).alongWith(feed.runFeedCommand(-0.7)));

    // Intake + Shoot Position
    operator.b().onTrue(superstructure.setGoalState(Superstructure.State.intake));

    // Stow
    operator.a().onTrue(superstructure.setGoalState(Superstructure.State.stow));

    // Score amp
    operator.x().onTrue(superstructure.setGoalState(Superstructure.State.scoreAmp));

    operator.pov(0).onTrue(arm.incrementGoalCommand(Units.degreesToRadians(5)));
    operator.pov(180).onTrue(arm.incrementGoalCommand(Units.degreesToRadians(-5)));
    operator.pov(270).onTrue(wrist.incrementGoalCommand(Units.degreesToRadians(1)));
    operator.pov(90).onTrue(wrist.incrementGoalCommand(Units.degreesToRadians(-1)));

    climber.setDefaultCommand(
        climber.climbCommand(() -> MathUtil.applyDeadband(-operator.getLeftY(), 0.04)));

    // operator.start().onTrue(wristSysID.fullTest(operator.back(), operator.start()));
    // spotless:off
    // arm.setDefaultCommand(arm.run(() -> arm.setVoltage((-operator.getLeftY()*3) + (0.6 * Math.cos(arm.getPosition())))));
    // spotless:on
    /*
    wrist.setDefaultCommand(wrist.run(() -> {
      var output = -operator.getLeftY() * 3;
      var s = Math.signum(output) * 0.22;
      var g = 0.36 * Math.cos(wrist.getCOGPosition());
      wrist.setVoltage(g + output);
    }));
    */
  }

  public void initializeNamedCommands() {
    NamedCommands.registerCommand("runIntakeUntilNote", endEffector.intakeNote());
  }

  public void setupDashboard() {
    var tab = Shuffleboard.getTab("Driver");
    tab.add(autoChooser);
    tab.addBoolean("Ready to Shoot", shooter::readyToLaunch);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}
