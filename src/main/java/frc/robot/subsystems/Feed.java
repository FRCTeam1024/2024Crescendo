package frc.robot.subsystems;

import static frc.robot.Constants.FeedConstants.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import monologue.Annotations.Log;
import monologue.Logged;

public class Feed extends SubsystemBase implements Logged {
  private final TalonFX feedMotor = new TalonFX(FeedMotorID);
  

  private final StatusSignal<Double> feedMotorVelocity = feedMotor.getVelocity();
  private final StatusSignal<Double> feedMotorStatorCurrent = feedMotor.getStatorCurrent();
  private final StatusSignal<Double> feedMotorVoltage = feedMotor.getMotorVoltage();
  private final StatusSignal<Double> feedMotorSupplyVoltage = feedMotor.getSupplyVoltage();
  
  private final DutyCycleOut dutyCycleRequest = new DutyCycleOut(0);

  public Feed() {
    var feedConfig = new TalonFXConfiguration();
    feedConfig.MotorOutput.Inverted = kFeedMotorInversionSetting;
    feedConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    feedConfig.Feedback.SensorToMechanismRatio = kFeedGearRatio;

    feedMotor.getConfigurator().apply(feedConfig);
  }

  public void setOutput(double output) {

    feedMotor.setControl(dutyCycleRequest.withOutput(output));

  }

  public void stop() {
    setOutput(0);
  }
}
