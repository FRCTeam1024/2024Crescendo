package frc.robot.subsystems;

import static edu.wpi.first.math.util.Units.*;
import static edu.wpi.first.wpilibj2.command.Commands.*;

import edu.wpi.first.wpilibj2.command.Command;
import monologue.Annotations.IgnoreLogged;
import monologue.Logged;

public class Superstructure implements Logged {
  @IgnoreLogged Arm arm;
  @IgnoreLogged Wrist wrist;

  public Superstructure(Arm arm, Wrist wrist) {
    this.arm = arm;
    this.wrist = wrist;
  }

  /**
   * Returns a command that sets the goal state of the superstructure
   *
   * @param goalState the goal state
   * @return the command
   */
  public Command setGoalState(State goalState) {
    return directToGoal(goalState);
  }

  /**
   * Naiive "direct to goal" command - this ignores any global kinematic/range constraints.
   *
   * @param goalState The goal state
   * @return command
   */
  private Command directToGoal(State goalState) {
    return parallel(
        arm.setGoalCommand(goalState::armPosition), wrist.setGoalCommand(goalState::wristPosition));
  }

  /**
   * @param armPosition Arm goal position in radians
   * @param wristPosition Arm wrist position in radians
   */
  public record State(double armPosition, double wristPosition) {

    public static final State stow = new State(-0.5, 2.279);
    public static final State intake = new State(stow.armPosition(), 0.087266);
    public static final State scoreFromSubwoofer = new State(stow.armPosition(), 0.087266);
    public static final State scoreFromSpikeMark =
        new State(stow.armPosition(), degreesToRadians(15));
    public static final State scoreTrap = new State(degreesToRadians(60), degreesToRadians(10));
    public static final State scoreAmp = new State(0.808997, 0.506145);
    public static final State scoreFromAmp = new State(stow.armPosition(), 0.314159);
    public static final State scoreOverDefense =
        new State(degreesToRadians(30), degreesToRadians(15));
  }
}
