package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import java.util.function.DoubleSupplier;
import monologue.Logged;

public class Climber extends SubsystemBase implements Logged {
  private final CANSparkMax climberMotorA =
      new CANSparkMax(Constants.ClimberConstants.ClimberMotorAId, MotorType.kBrushless);
  private final CANSparkMax climberMotorB =
      new CANSparkMax(Constants.ClimberConstants.ClimberMotorBId, MotorType.kBrushless);

  public Climber() {
    // Motor A
    climberMotorA.restoreFactoryDefaults();
    climberMotorA.setIdleMode(IdleMode.kCoast);
    climberMotorA.setInverted(false);
    climberMotorA.burnFlash();
    // Motor B
    climberMotorB.restoreFactoryDefaults();
    climberMotorB.setIdleMode(IdleMode.kCoast);
    climberMotorB.setInverted(true);
    climberMotorB.burnFlash();
  }

  public void setOutput(double output) {
    climberMotorA.set(output);
    climberMotorB.set(output);
  }

  public void stop() {
    setOutput(0);
  }

  public Command getClimbCommand(DoubleSupplier outputSupplier) {
    return runEnd(() -> setOutput(outputSupplier.getAsDouble()), () -> stop());
  }
}
