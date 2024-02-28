package frc.robot.subsystems;

import static edu.wpi.first.math.util.Units.*;
import static edu.wpi.first.wpilibj2.command.Commands.*;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.math.TwoJointArmKinematics;
import monologue.Annotations.IgnoreLogged;
import monologue.Annotations.Log;
import monologue.Logged;

public class Superstructure implements Logged {
  @IgnoreLogged Arm arm;
  @IgnoreLogged Wrist wrist;

  private final TwoJointArmKinematics kinematics =
      new TwoJointArmKinematics(Units.inchesToMeters(22.5), Units.inchesToMeters(13.65));

  public Superstructure(Arm arm, Wrist wrist) {
    this.arm = arm;
    this.wrist = wrist;
  }

  @Log.NT
  public Translation2d getEndEffectorPosition() {
    return kinematics.getEndEffectorTranslation(
        Rotation2d.fromRadians(arm.getPosition()), Rotation2d.fromRadians(wrist.getTipPosition()));
  }

  @Log.NT
  public boolean outOfBounds() {
    Translation2d robotToArmOrigin =
        new Translation2d(Units.inchesToMeters(-9.5), Units.inchesToMeters(18.625));
    double topBoundY = Units.feetToMeters(4);
    double robotLength = Units.inchesToMeters(27.25);
    double frameBoundXPos = (robotLength / 2) + Units.feetToMeters(1);
    double frameBoundXNeg = -frameBoundXPos;

    var endEffectorPosition = getEndEffectorPosition();
    var endEffectorInRobotFrame = robotToArmOrigin.plus(endEffectorPosition);

    boolean topbound = endEffectorInRobotFrame.getY() > topBoundY;
    boolean frontBound = endEffectorInRobotFrame.getX() > frameBoundXPos;
    boolean backBound = endEffectorInRobotFrame.getX() < frameBoundXNeg;
    return topbound || frontBound || backBound;
  }

  @Log.NT(key = "Wrist Angle To Arm")
  private double wristAngleToArm() {
    return wrist.getPosition() - arm.getPosition();
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

  private double nearestAlwaysSafeWristState(double curState) {
    double upperSafeState = Superstructure.State.stow.wristPosition;
    double lowerSafeState = Superstructure.State.intake.wristPosition;

    if (Math.abs(curState - upperSafeState) < Math.abs(curState - lowerSafeState)) {
      return upperSafeState;
    } else {
      return lowerSafeState;
    }
  }

  private boolean goesThroughPotentialUnsafeState(
      double curState, double goalState, double upperUnsafeBound, double lowerUnsafeBound) {
    // To be safe, both the starting state and the end state must be on the same side of the bounds
    // and outside of the bounds
    return !((curState > upperUnsafeBound && goalState > upperUnsafeBound)
        || (curState < lowerUnsafeBound && goalState < lowerUnsafeBound));
  }

  /**
   * Moves the wrist to the nearest always known-safe wrist state before moving the arm to the goal
   * state, then moving the wrist.
   *
   * @param goalState The goal state
   * @return command
   */
  private Command safeWristFirst(State goalState) {
    return wrist
        .setGoalCommand(() -> nearestAlwaysSafeWristState(wrist.getPosition()))
        .andThen(arm.setGoalCommand(() -> goalState.armPosition))
        .andThen(wrist.setGoalCommand(() -> goalState.armPosition));
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
    public static final State intake =
        new State(stow.armPosition(), 0.087266 - Units.degreesToRadians(1));
    public static final State scoreFromSubwoofer =
        new State(stow.armPosition(), 0.087266 + Units.degreesToRadians(1));
    public static final State scoreFromSpikeMark =
        new State(stow.armPosition(), degreesToRadians(15));
    public static final State scoreTrap = new State(0.6344640748005669, 2.007128156715125);
    public static final State scoreAmp = new State(0.808997, 0.506145);
    public static final State scoreFromAmp = new State(stow.armPosition(), 0.314159);
    public static final State scoreOverDefense =
        new State(degreesToRadians(30), degreesToRadians(15));
    public static final State climb = new State(1.158062789394613, 1.0122905035569956);
  }

  private static boolean isBetween(double value, double upper, double lower) {
    return value < upper && value > lower;
  }
}
