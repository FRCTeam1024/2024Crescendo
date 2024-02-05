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
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.DoubleSupplier;
import monologue.Logged;

public class Wrist extends SubsystemBase implements Logged {

  private final TalonFX wristMotor = new TalonFX(kWristId);
  private final DutyCycleEncoder absoluteEncoder = new DutyCycleEncoder(kAbsEncoderPin);
  private final Encoder quadEncoder = new Encoder(kQuadEncoderAPin, kQuadEncoderBPin);

  private final ProfiledPIDController controller =
      new ProfiledPIDController(
          kP,
          kI,
          kD,
          new TrapezoidProfile.Constraints(
              kMaxVelocityRadiansPerSecond, kMaxAccelerationRadiansPerSecondSquared));

  private final ArmFeedforward armFeedforward = new ArmFeedforward(kS, kG, kV, kA);

  private final VoltageOut voltageRequest = new VoltageOut(0);

  public Wrist() {
    var wristConfig = new TalonFXConfiguration();
    wristConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    wristConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    wristConfig.Feedback.SensorToMechanismRatio = kMotorToWristRatio;
    wristMotor.getConfigurator().apply(wristConfig);

    quadEncoder.setDistancePerPulse(2.0 * Math.PI / kQuadTicks);
    absoluteEncoder.setDistancePerRotation(-1 * 2 * Math.PI);
    absoluteEncoder.setPositionOffset(kPositionOffset);
    // Wait for encoder to produce valid values
    Timer.delay(2);
    setGoal(getPosition());

    setDefaultCommand(holdPositionCommand());
    SmartDashboard.putData("Wrist PID Controller", controller);
  }

  public void setOutput(double output) {
    setVoltage(output * 12);
  }

  public void setVoltage(double voltage) {
    wristMotor.setControl(voltageRequest.withOutput(voltage));
  }

  public void setGoal(double goal) {
    controller.setGoal(MathUtil.clamp(goal, kMinPosition, kMaxPosition));
  }

  public void stop() {
    setOutput(0);
  }

  public double getGoal() {
    return controller.getGoal().position;
  }

  /**
   * Returns the current position of the wrist in radians. The value will be bounded by -pi, pi
   *
   * @return current position in radians
   */
  public double getPosition() {
    return absoluteEncoder.getDistance();
  }

  public double getQuadPosition() {
    return quadEncoder.getDistance();
  }

  public double getSetpoint() {
    return controller.getSetpoint().position;
  }

  public boolean atGoal() {
    return controller.atGoal();
  }

  private void updatePositionController() {
    var position = getPosition();
    var feedback = controller.calculate(position);
    var feedforward = armFeedforward.calculate(position, controller.getSetpoint().velocity);
    setVoltage(feedforward + feedback);
  }

  public Command continuousGoalCommand(DoubleSupplier goalSupplier) {
    return run(
        () -> {
          setGoal(goalSupplier.getAsDouble());
          updatePositionController();
        });
  }

  public Command setGoalCommand(DoubleSupplier goalSupplier) {
    return runOnce(() -> setGoal(goalSupplier.getAsDouble()))
        .andThen(holdPositionCommand().until(this::atGoal));
  }

  public Command holdPositionCommand() {
    return run(this::updatePositionController);
  }

  @Override
  public void periodic() {
    if (DriverStation.isDisabled()) {
      controller.reset(getPosition());
    }
    log("Position", getPosition());
    log("Setpoint", getSetpoint());
    log("Goal", getGoal());
    log("AtGoal", atGoal());
    log("AbsEncoderAbsolute", absoluteEncoder.getAbsolutePosition());
    log("AbsEncoderGet", absoluteEncoder.get());
  }
}
