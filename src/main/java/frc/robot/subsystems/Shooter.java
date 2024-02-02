package frc.robot.subsystems;

import static frc.robot.Constants.ShooterConstants.*;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {
  private final TalonFX shooterA = new TalonFX(kShooterAId);
  private final TalonFX shooterB = new TalonFX(kShooterBId);

  public Shooter() {
    var shooterConfig = new TalonFXConfiguration();
    shooterConfig.MotorOutput.Inverted = kMotorAInversionSetting;
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
}
