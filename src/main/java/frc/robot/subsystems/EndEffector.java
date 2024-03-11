package frc.robot.subsystems;

import static edu.wpi.first.wpilibj2.command.Commands.*;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.FeedConstants;
import frc.robot.Constants.IntakeConstants;
import monologue.Annotations.IgnoreLogged;
import monologue.Annotations.Log;
import monologue.Logged;

public class EndEffector implements Logged {
  @IgnoreLogged private final Intake intake;
  @IgnoreLogged private final Feed feed;
  @IgnoreLogged private final Shooter shooter;

  private final DigitalInput noteSensor = new DigitalInput(IntakeConstants.noteSensorId);
  private final Debouncer filter = new Debouncer(.02);

  private boolean hasNoteIndexed;

  public EndEffector(Intake intake, Feed feed, Shooter shooter) {
    this.intake = intake;
    this.feed = feed;
    this.shooter = shooter;
  }

  @Log.NT
  public boolean hasNote() {
    return filter.calculate(unfilteredHasNote());
  }

  @Log.NT
  public boolean unfilteredHasNote() {
    return !noteSensor.get();
  }

  @Log.NT
  public boolean hasNoteIndexed() {
    return hasNoteIndexed;
  }

  public Command setNoteIndexed(boolean indexed) {
    return runOnce(() -> hasNoteIndexed = true);
  }

  public Command intakeNote() {
    return intake
        .runIntakeCommand(IntakeConstants.intakingSetpoint)
        .alongWith(feed.runFeedCommand(FeedConstants.intakingSetpoint))
        .until(this::hasNote);
  }

  public Command backOffNote() {
    return feed.runFeedCommand(-0.1).alongWith(shooter.velocityCommand(-10)).withTimeout(1);
  }

  public Command intakeNoteAndIndex() {
    return intakeNote().andThen(backOffNote()).andThen(setNoteIndexed(true));
  }

  public Command fireNote() {
    return setNoteIndexed(false)
        .andThen(
            feed.runFeedCommand(FeedConstants.fireSetpoint)
                .alongWith(intake.runIntakeCommand(IntakeConstants.fireSetpoint)))
        .withTimeout(1.0);
  }

  public Command fireWhenReady() {
    return waitUntil(shooter::readyToLaunch).andThen(fireNote());
  }

  public Command spinUpAndShoot(double shotSetpoint) {
    return race(
        shooter.velocityCommand(() -> shotSetpoint), waitSeconds(1).andThen(fireWhenReady()));
  }
}
