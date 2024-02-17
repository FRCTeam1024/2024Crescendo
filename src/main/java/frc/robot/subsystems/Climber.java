package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkLowLevel.PeriodicFrame;
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
    configurePeriodicFrames(climberMotorA);
    climberMotorA.burnFlash();
    // Motor B
    climberMotorB.restoreFactoryDefaults();
    climberMotorB.setIdleMode(IdleMode.kCoast);
    climberMotorB.setInverted(true);
    configurePeriodicFrames(climberMotorB);
    climberMotorB.burnFlash();
  }

  public void setOutput(double output) {
    climberMotorA.set(output);
    climberMotorB.set(output);
  }

  public void stop() {
    setOutput(0);
  }

  public Command climbCommand(DoubleSupplier outputSupplier) {
    return runEnd(() -> setOutput(outputSupplier.getAsDouble()), () -> stop());
  }

  private static void configurePeriodicFrames(CANSparkMax sparkMax) {
    sparkMax.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 20);
    sparkMax.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 20);
    sparkMax.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 20);
    sparkMax.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 65535);
    sparkMax.setPeriodicFramePeriod(PeriodicFrame.kStatus4, 65535);
    sparkMax.setPeriodicFramePeriod(PeriodicFrame.kStatus5, 65535);
    sparkMax.setPeriodicFramePeriod(PeriodicFrame.kStatus6, 65535);
    sparkMax.setPeriodicFramePeriod(PeriodicFrame.kStatus7, 65535);
  }
}
