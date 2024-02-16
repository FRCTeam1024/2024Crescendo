package frc.robot.subsystems;

import static frc.robot.Constants.WristConstants.*;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.DoubleSupplier;
import monologue.Annotations.Log;
import monologue.Logged;

public class Wrist extends SubsystemBase implements Logged {
  private final TalonFX wristMotor = new TalonFX(kWristId);
  private final DutyCycleEncoder absoluteEncoder = new DutyCycleEncoder(kAbsEncoderPin);
  private final Encoder quadEncoder = new Encoder(kQuadEncoderAPin, kQuadEncoderBPin);

  @Log.NT
  private final ProfiledPIDController controller =
      new ProfiledPIDController(
          kP,
          kI,
          kD,
          new TrapezoidProfile.Constraints(
              kMaxVelocityRadiansPerSecond, kMaxAccelerationRadiansPerSecondSquared));

  private final ArmFeedforward armFeedforward = new ArmFeedforward(kS, kG, kV, kA);

  private final VoltageOut voltageRequest = new VoltageOut(0);

  private final double kInitializationOffset;

  public Wrist() {
    var wristConfig = new TalonFXConfiguration();
    wristConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    wristConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    wristConfig.Feedback.SensorToMechanismRatio = kMotorToWristRatio;
    wristMotor.getConfigurator().apply(wristConfig);

    quadEncoder.setDistancePerPulse(2.0 * Math.PI / kQuadTicks);
    quadEncoder.setSamplesToAverage(80);
    quadEncoder.reset();
    // Wait for encoder to produce valid values
    Timer.delay(2);
    kInitializationOffset = getAbsolutePosition();

    setGoal(getPosition());
    setDefaultCommand(holdPositionCommand());
  }

  /**
   * Sets the output of the motor as a proportion of 12v
   *
   * @param output The proportion of 12v to output (-1, 1)
   */
  public void setOutput(double output) {
    setVoltage(output * 12);
  }

  /** Disables all motor output. */
  public void stop() {
    setOutput(0);
  }

  /**
   * Sets the voltage output to the motor
   *
   * @param voltage Voltage to output
   */
  public void setVoltage(double voltage) {
    wristMotor.setControl(voltageRequest.withOutput(voltage));
  }

  /**
   * Returns the voltage applied to the motor
   *
   * @return Voltage applied to the motor
   */
  @Log.NT
  public double getAppliedVoltage() {
    return wristMotor.getMotorVoltage().getValue();
  }

  /**
   * Sets the goal for the closed loop controller
   *
   * @param goal Goal, in radians
   */
  public void setGoal(double goal) {
    controller.setGoal(MathUtil.clamp(goal, kMinPosition, kMaxPosition));
  }

  /**
   * Returns the current goal position in radians
   *
   * @return goal position in radians
   */
  @Log.NT
  public double getGoal() {
    return controller.getGoal().position;
  }

  /**
   * Returns the current position of the wrist in radians. The value will be bounded by -pi, pi
   *
   * @return current position in radians
   */
  @Log.NT
  public double getPosition() {
    return getQuadPosition() + kInitializationOffset;
  }

  @Log.NT
  public double getVelocity() {
    return quadEncoder.getRate();
  }

  /**
   * Gets the angle of the quadrature encoder since the start
   *
   * @return angle of the quad encoder
   */
  @Log.NT
  public double getQuadPosition() {
    return quadEncoder.getDistance();
  }

  public State getSetpoint() {
    return controller.getSetpoint();
  }

  /**
   * Returns the absolute position of the wrist in radians, bounded by (-pi, pi)
   *
   * @return absolute position
   */
  @Log.NT
  public double getAbsolutePosition() {
    // Encoder is out of phase with wrist
    var rawPosition = -absoluteEncoder.getAbsolutePosition();
    var positionRadians = Units.rotationsToRadians(rawPosition);
    positionRadians -= kPositionOffset;
    // Bring us back into (-pi, pi)
    return MathUtil.angleModulus(positionRadians);
  }

  /**
   * Returns true when the arm is at the current goal
   *
   * @return true when the arm is at the goal, otherwise false.
   */
  @Log.NT
  public boolean atGoal() {
    return controller.atGoal();
  }

  /** Updates the wrist's closed loop controller. */
  public void updatePositionController() {
    var position = getPosition();
    var feedback = controller.calculate(position);
    var feedforward = armFeedforward.calculate(position, controller.getSetpoint().velocity);
    setVoltage(feedforward + feedback);
  }

  /**
   * Returns a command that will constantly update the goal to the value returned by goalSupplier
   *
   * @param goalSupplier Function that returns the desired goal in radians
   * @return the command
   */
  public Command continuousGoalCommand(DoubleSupplier goalSupplier) {
    return run(
        () -> {
          setGoal(goalSupplier.getAsDouble());
          updatePositionController();
        });
  }

  /**
   * Returns a command that will: 1) set the goal to the value returned by goalSupplier 2) update
   * the closed loop controller until the goal is reached
   *
   * @param goalSupplier function that returns desired goal in radians. This will be polled once
   *     each time the command is initialized.
   * @return the command
   */
  public Command setGoalCommand(DoubleSupplier goalSupplier) {
    return runOnce(() -> setGoal(goalSupplier.getAsDouble()))
        .andThen(holdPositionCommand().until(this::atGoal));
  }

  public Command incrementGoalCommand(double incrementBy) {
    return setGoalCommand(() -> getGoal() + incrementBy);
  }

  /**
   * Returns a command that continuously updates the closed loop controller
   *
   * @return the command
   */
  public Command holdPositionCommand() {
    return run(this::updatePositionController);
  }

  @Override
  public void periodic() {
    // Reset the controller when disabled so it profiles to the setpoint when re-enabled
    if (DriverStation.isDisabled()) {
      controller.reset(getPosition());
    }
    var setpoint = getSetpoint();
    log("Setpoint Position", setpoint.position);
    log("Setpoint Velocity", setpoint.velocity);
    log("Stator Current", wristMotor.getStatorCurrent().getValue());
    log("Supply Voltage", wristMotor.getSupplyVoltage().getValue());
  }
}