package frc.robot.commands;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.SignalLogger;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.subsystems.Arm;
import java.util.function.BooleanSupplier;

public class ArmSysID {
  Arm arm;
  SysIdRoutine routine;

  MutableMeasure<Angle> angularPosition = Radians.of(0).mutableCopy();
  MutableMeasure<Velocity<Angle>> angularVelocity = RadiansPerSecond.of(0).mutableCopy();
  MutableMeasure<Voltage> voltage = Volts.of(0).mutableCopy();

  public ArmSysID(Arm arm) {
    this.arm = arm;
    routine =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                Volts.of(0.25).per(Second.one()),
                Volts.of(3),
                null,
                state -> SignalLogger.writeString("arm-sysid-state", state.toString())),
            new SysIdRoutine.Mechanism(this::drive, this::log, arm));
  }

  public Command fullTest(BooleanSupplier stopCondition, BooleanSupplier nextCondition) {
    return Commands.sequence(
        Commands.run(() -> SignalLogger.start()),
        routine.quasistatic(Direction.kForward).until(stopCondition),
        Commands.waitUntil(nextCondition),
        routine.quasistatic(Direction.kReverse).until(stopCondition),
        Commands.waitUntil(nextCondition),
        routine.dynamic(Direction.kForward).until(stopCondition),
        Commands.waitUntil(nextCondition),
        routine.dynamic(Direction.kReverse).until(stopCondition),
        Commands.run(() -> SignalLogger.stop()));
  }

  private void drive(Measure<Voltage> volts) {
    arm.setVoltage(volts.in(Volts));
  }

  private void log(SysIdRoutineLog log) {
    log.motor("arm")
        .angularPosition(angularPosition.mut_replace(arm.getPosition(), Radians))
        .angularVelocity(angularVelocity.mut_replace(arm.getVelocity(), RadiansPerSecond))
        .voltage(voltage.mut_replace(arm.getAppliedVoltage(), Volts));
  }
}
