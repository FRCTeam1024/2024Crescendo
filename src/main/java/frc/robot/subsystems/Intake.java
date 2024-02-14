package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import monologue.Logged;

public class Intake extends SubsystemBase implements Logged {
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

  public Command runIntakeCommand(double output) {
    return runEnd(
        () -> {
          setOutput(output);
        },
        () -> stop());
  }
}
