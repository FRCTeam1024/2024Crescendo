package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkLowLevel.PeriodicFrame;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Constants;
import monologue.Logged;

public class Intake extends SubsystemBase implements Logged {
  private final CANSparkMax intakeMotor =
      new CANSparkMax(Constants.IntakeConstants.intakeMotorId, MotorType.kBrushless);
  private final DigitalInput noteSensor = 
    new DigitalInput(Constants.IntakeConstants.noteSensorId);

  public Intake() {
    intakeMotor.restoreFactoryDefaults();
    intakeMotor.setInverted(Constants.IntakeConstants.isInverted);
    intakeMotor.setIdleMode(IdleMode.kCoast);
    configurePeriodicFrames(intakeMotor);
    intakeMotor.burnFlash();
  }

  public void setOutput(double output) {
    intakeMotor.set(output);
  }

  public void stop() {
    setOutput(0);
  }
  
  public Boolean hasNote() {
    return !noteSensor.get();
  }


  public Command runIntakeCommand(double output) {
    return runEnd(
        () -> {
          setOutput(output);
        },
        () -> stop());
  }

  public static void configurePeriodicFrames(CANSparkMax sparkMax) {
    sparkMax.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 20);
    sparkMax.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 20);
    sparkMax.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 65535);
    sparkMax.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 65535);
    sparkMax.setPeriodicFramePeriod(PeriodicFrame.kStatus4, 65535);
    sparkMax.setPeriodicFramePeriod(PeriodicFrame.kStatus5, 65535);
    sparkMax.setPeriodicFramePeriod(PeriodicFrame.kStatus6, 65535);
    sparkMax.setPeriodicFramePeriod(PeriodicFrame.kStatus7, 65535);
  }
}
