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

  public boolean hasNote() {
    return intake.hasNote();
  }

  public Command intakeNote() {
    return intake
        .runIntakeCommand(IntakeConstants.intakingSetpoint)
        .alongWith(feed.runFeedCommand(FeedConstants.intakingSetpoint))
        .until(this::hasNote);
  }

  public Command backOffNote() {
    return feed.runFeedCommand(-0.1).alongWith(shooter.velocityCommand(-10)).withTimeout(0.25);
  }

  public Command intakeNoteAndIndex() {
    return intakeNote().andThen(backOffNote());
  }

  public Command fireNote() {
    return race(
        feed.runFeedCommand(FeedConstants.fireSetpoint)
            .alongWith(intake.runIntakeCommand(IntakeConstants.fireSetpoint))
            .withTimeout(1.0),
        waitUntil(() -> !hasNote()).andThen(waitSeconds(0.5)));
  }

  public Command fireWhenReady() {
    return waitUntil(shooter::readyToLaunch).andThen(fireNote());
  }

  public Command preSpinUp(double shotSetpoint) {
    return shooter.velocityCommand(() -> shotSetpoint);
  }

  public Command spinUpAndShoot(double shotSetpoint) {
    return race(
            shooter.velocityCommand(() -> shotSetpoint), waitSeconds(.2).andThen(fireWhenReady()))
        // brake during auto to prevent next note from getting shot during intaking
        .andThen(shooter.runOnce(() -> shooter.brake()));
  }
}
