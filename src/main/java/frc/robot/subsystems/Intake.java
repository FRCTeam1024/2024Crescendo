package frc.robot.subsystems;

import static frc.robot.Constants.IntakeConstants.*;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkLowLevel.PeriodicFrame;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import monologue.Annotations.Log;
import monologue.Logged;

public class Intake extends SubsystemBase implements Logged {
  private final CANSparkMax intakeMotor = new CANSparkMax(intakeMotorId, MotorType.kBrushless);
  private final RelativeEncoder encoder = intakeMotor.getEncoder();

  private final DigitalInput noteSensor = new DigitalInput(noteSensorId);
  private final Debouncer filter = new Debouncer(.02);

  public Intake() {

    intakeMotor.restoreFactoryDefaults();
    intakeMotor.setInverted(isInverted);
    intakeMotor.setIdleMode(IdleMode.kCoast);
    intakeMotor.setSmartCurrentLimit(50);
    configurePeriodicFrames(intakeMotor);
    intakeMotor.burnFlash();
  }

  public void setOutput(double output) {
    intakeMotor.setVoltage(output * 12);
  }

  public void stop() {
    setOutput(0);
  }

  @Log.NT
  public boolean hasNote() {
    return filter.calculate(unfilteredHasNote());
  }

  @Log.NT
  public boolean unfilteredHasNote() {
    return noteSensor.get();
  }

  @Log.NT(key = "Velocity")
  public double getVelocity() {
    return encoder.getVelocity();
  }

  @Log.NT(key = "Current")
  public double getCurrent() {
    return intakeMotor.getOutputCurrent();
  }

  @Log.NT(key = "Voltage")
  public double getAppliedVoltage() {
    return intakeMotor.getAppliedOutput() * intakeMotor.getBusVoltage();
  }

  public Command runIntakeCommand(double output) {
    return runEnd(
        () -> {
          setOutput(output);
        },
        () -> stop());
  }

  public Command intakeNoteCommand() {
    return runIntakeCommand(intakingSetpoint).until(this::hasNote);
  }

  public static void configurePeriodicFrames(CANSparkMax sparkMax) {
    sparkMax.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 20);
    sparkMax.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 20);
    sparkMax.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 32767);
    sparkMax.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 32767);
    sparkMax.setPeriodicFramePeriod(PeriodicFrame.kStatus4, 32767);
    sparkMax.setPeriodicFramePeriod(PeriodicFrame.kStatus5, 32767);
    sparkMax.setPeriodicFramePeriod(PeriodicFrame.kStatus6, 32767);
    sparkMax.setPeriodicFramePeriod(PeriodicFrame.kStatus7, 32767);
  }
}
