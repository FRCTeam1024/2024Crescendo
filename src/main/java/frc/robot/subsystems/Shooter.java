package frc.robot.subsystems;

import static frc.robot.Constants.ShooterConstants.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
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
  private final StatusSignal<Double> shooterAError = shooterA.getClosedLoopError();
  private final StatusSignal<Double> shooterBError = shooterB.getClosedLoopError();

  private final VelocityVoltage velocityRequest = new VelocityVoltage(0).withSlot(0);
  private final NeutralOut stopRequest = new NeutralOut();

  private Timer stableTime = new Timer();

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

    stableTime.restart();

    BaseStatusSignal.setUpdateFrequencyForAll(
        50,
        shooterAVelocity,
        shooterBVelocity,
        shooterAVoltage,
        shooterBVoltage,
        shooterAError,
        shooterBError);
    BaseStatusSignal.setUpdateFrequencyForAll(
        10,
        shooterASupplyVoltage,
        shooterBSupplyVoltage,
        shooterAStatorCurrent,
        shooterBStatorCurrent);

    if (Constants.disableUnusedSignals) {
      shooterA.optimizeBusUtilization();
      shooterB.optimizeBusUtilization();
    }
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
    if (shooterA.getAppliedControl().equals(stopRequest)) {
      return 0;
    } else {
      return velocityRequest.Velocity;
    }
  }

  /**
   * Checks if the shooter is running at stable speed near the setpoint
   *
   * @return TRUE if shooter is spinning at setpoint
   */
  @Log.NT(key = "Ready To Launch")
  public Boolean readyToLaunch() {
    return getVelocitySetpointRPS() > 0 && stableTime.hasElapsed(1);
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
        shooterBSupplyVoltage,
        shooterAError,
        shooterBError);
    log("Upper Velocity", shooterAVelocity.getValue());
    log("Lower Velocity", shooterBVelocity.getValue());
    log("Upper Stator Current", shooterAStatorCurrent.getValue());
    log("Lower Stator Current", shooterBStatorCurrent.getValue());
    log("Upper Voltage", shooterAVoltage.getValue());
    log("Lower Voltage", shooterBVoltage.getValue());
    log("Upper Supply Voltage", shooterASupplyVoltage.getValue());
    log("Lower Supply Voltage", shooterBSupplyVoltage.getValue());
    log("Upper Velocity Error", shooterAError.getValue());
    log("Lower Velocity Error", shooterBError.getValue());

    /* Check if running at stable speed and restart stableTime if not*/
    double errorA = shooterAError.getValue();
    double errorB = shooterBError.getValue();
    double velocityA = shooterAVelocity.getValue();
    double velocityB = shooterBVelocity.getValue();

    if (errorA > 5 || errorB > 5 || Math.abs(velocityA - velocityB) > 5) {
      stableTime.restart();
    }
  }
}
