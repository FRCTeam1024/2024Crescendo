package frc.robot.subsystems;

import static frc.robot.Constants.FeedConstants.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import monologue.Logged;

public class Feed extends SubsystemBase implements Logged {
  private final TalonFX feedMotor = new TalonFX(FeedMotorID);

  private final StatusSignal<Double> feedMotorVelocity = feedMotor.getVelocity();
  private final StatusSignal<Double> feedMotorVoltage = feedMotor.getMotorVoltage();
  private final StatusSignal<Double> feedMotorStatorCurrent = feedMotor.getStatorCurrent();

  private final DutyCycleOut dutyCycleRequest = new DutyCycleOut(0);

  public Feed() {
    var feedConfig = new TalonFXConfiguration();
    feedConfig.MotorOutput.Inverted = kFeedMotorInversionSetting;
    feedConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    feedConfig.Feedback.SensorToMechanismRatio = kFeedGearRatio;

    feedMotor.getConfigurator().apply(feedConfig);
    feedMotorVelocity.setUpdateFrequency(25);
    feedMotorVoltage.setUpdateFrequency(25);
    feedMotorStatorCurrent.setUpdateFrequency(4);

    if (Constants.disableUnusedSignals) {
      feedMotor.optimizeBusUtilization();
    }
  }

  public void setOutput(double output) {
    feedMotor.setControl(dutyCycleRequest.withOutput(output));
  }

  public void stop() {
    setOutput(0);
  }

  public Command runFeedCommand(double output) {
    return runEnd(
        () -> {
          setOutput(output);
        },
        () -> stop());
  }

  @Override
  public void periodic() {
    BaseStatusSignal.refreshAll(feedMotorVelocity, feedMotorStatorCurrent, feedMotorVoltage);
  }
}
