package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {
  private final CANSparkMax intakeMotor =
      new CANSparkMax(Constants.IntakeConstants.intakeMotorId, MotorType.kBrushless);

  public Intake() {
    intakeMotor.restoreFactoryDefaults();
    intakeMotor.setInverted(Constants.IntakeConstants.isInverted);
    intakeMotor.setIdleMode(IdleMode.kCoast);
    intakeMotor.burnFlash();
  }

  public void setOutput(double output) {
    intakeMotor.set(output);
  }

  public void stop() {
    setOutput(0);
  }
}
