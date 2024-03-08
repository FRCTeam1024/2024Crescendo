package frc.robot.subsystems;

import static edu.wpi.first.wpilibj2.command.Commands.*;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.FeedConstants;
import frc.robot.Constants.IntakeConstants;
import monologue.Annotations.IgnoreLogged;
import monologue.Logged;

public class EndEffector implements Logged {
  @IgnoreLogged private final Intake intake;
  @IgnoreLogged private final Feed feed;
  @IgnoreLogged private final Shooter shooter;

  public EndEffector(Intake intake, Feed feed, Shooter shooter) {
    this.intake = intake;
    this.feed = feed;
    this.shooter = shooter;
  }

  public Command intakeNote() {
    return intake
        .runIntakeCommand(IntakeConstants.intakingSetpoint)
        .alongWith(feed.runFeedCommand(FeedConstants.intakingSetpoint))
        .until(intake::hasNote);
  }

  public Command backOffNote() {
    return feed.runFeedCommand(-0.1).alongWith(shooter.velocityCommand(-10)).withTimeout(1);
  }

  public Command intakeNoteAndIndex() {
    return intakeNote().andThen(backOffNote());
  }

  public Command fireNote() {
    return feed.runFeedCommand(FeedConstants.fireSetpoint)
        .alongWith(intake.runIntakeCommand(IntakeConstants.fireSetpoint))
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
