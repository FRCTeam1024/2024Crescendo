package frc.robot.subsystems;

import static frc.robot.Constants.ShooterConstants.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.DoubleSupplier;
import monologue.Annotations.Log;
import monologue.Logged;

public class Shooter extends SubsystemBase implements Logged {
  private final TalonFX shooterA = new TalonFX(kShooterAId);
  private final TalonFX shooterB = new TalonFX(kShooterBId);

  private final StatusSignal<Double> shooterAVelocity = shooterA.getVelocity();
  private final StatusSignal<Double> shooterBVelocity = shooterB.getVelocity();
  private final StatusSignal<Double> shooterAStatorCurrent = shooterA.getStatorCurrent();
  private final StatusSignal<Double> shooterBStatorCurrent = shooterB.getStatorCurrent();
  private final StatusSignal<Double> shooterAVoltage = shooterA.getMotorVoltage();
  private final StatusSignal<Double> shooterBVoltage = shooterB.getMotorVoltage();
  private final StatusSignal<Double> shooterASupplyVoltage = shooterA.getSupplyVoltage();
  private final StatusSignal<Double> shooterBSupplyVoltage = shooterB.getSupplyVoltage();

  private final VelocityVoltage velocityRequest = new VelocityVoltage(0).withSlot(0);
  private final NeutralOut stopRequest = new NeutralOut();

  public Shooter() {
    var shooterConfig = new TalonFXConfiguration();
    shooterConfig.MotorOutput.Inverted = kMotorAInversionSetting;
    shooterConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    shooterConfig.Feedback.SensorToMechanismRatio = kShooterGearRatio;

    shooterConfig.Slot0.kS = kS;
    shooterConfig.Slot0.kV = kV;
    shooterConfig.Slot0.kA = kA;
    shooterConfig.Slot0.kP = kP;
    shooterConfig.Slot0.kI = kI;
    shooterConfig.Slot0.kD = kD;

    shooterA.getConfigurator().apply(shooterConfig);

    shooterConfig.MotorOutput.Inverted = kMotorBInversionSetting;

    shooterB.getConfigurator().apply(shooterConfig);
  }

  public void setVelocity(double speed) {
    if (speed < 0) {
      velocityRequest.LimitForwardMotion = true;
      velocityRequest.LimitReverseMotion = false;
    } else {
      velocityRequest.LimitForwardMotion = false;
      velocityRequest.LimitReverseMotion = true;
    }
    shooterA.setControl(velocityRequest.withVelocity(speed));
    shooterB.setControl(velocityRequest.withVelocity(speed));
  }

  @Log.NT(key = "Velocity Setpoint (RPS)")
  public double getVelocitySetpointRPS() {
    return velocityRequest.Velocity;
  }

  public void stop() {
    shooterA.setControl(stopRequest);
    shooterB.setControl(stopRequest);
  }

  public Command velocityCommand(double velocityRPS) {
    return velocityCommand(() -> velocityRPS);
  }

  public Command velocityCommand(DoubleSupplier velocitySupplier) {
    return runEnd(() -> setVelocity(velocitySupplier.getAsDouble()), this::stop);
  }

  @Override
  public void periodic() {
    BaseStatusSignal.refreshAll(
        shooterAVelocity,
        shooterBVelocity,
        shooterAStatorCurrent,
        shooterBStatorCurrent,
        shooterAVoltage,
        shooterBVoltage,
        shooterASupplyVoltage,
        shooterBSupplyVoltage);
    log("Upper Velocity", shooterAVelocity.getValue());
    log("Lower Velocity", shooterBVelocity.getValue());
    log("Upper Stator Current", shooterAStatorCurrent.getValue());
    log("Lower Stator Current", shooterBStatorCurrent.getValue());
    log("Upper Voltage", shooterAVoltage.getValue());
    log("Lower Voltage", shooterBVoltage.getValue());
    log("Upper Supply Voltage", shooterASupplyVoltage.getValue());
    log("Lower Supply Voltage", shooterBSupplyVoltage.getValue());
  }
}
