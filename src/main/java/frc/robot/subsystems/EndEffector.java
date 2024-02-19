package frc.robot.subsystems;

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

  public Command fireNote() {
    return feed.runFeedCommand(FeedConstants.fireSetpoint)
        .alongWith(intake.runIntakeCommand(IntakeConstants.fireSetpoint))
        .withTimeout(1.0);
  }
}
