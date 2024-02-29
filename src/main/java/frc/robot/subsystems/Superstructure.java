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
  public static final double scoringPositionThreshold = Units.degreesToRadians(25);

  public static final double wristLowerUnsafeBound = 0.26 - Units.degreesToRadians(5);
  public static final double wristUpperUnsafeBound = 1.02 + Units.degreesToRadians(5);

  public static final double wristLowerSafeState = wristLowerUnsafeBound;
  public static final double wristUpperSafeState = wristUpperUnsafeBound;

  public static final double armLowerUnsafeBound = -0.48;
  public static final double armUpperUnsafeBound = 0.19;

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
    return safeWristFirst(goalState);
  }

  public State getState() {
    return new State(arm.getPosition(), wrist.getPosition());
  }

  public boolean isInFiringPosition() {
    return wrist.getPosition() < scoringPositionThreshold;
  }

  private double nearestAlwaysSafeWristState(double curState) {
    if (Math.abs(curState - wristUpperSafeState) < Math.abs(curState - wristLowerSafeState)) {
      return wristUpperSafeState;
    } else {
      return wristLowerSafeState;
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
   * Returns a safe intermediate wrist state given a goal
   *
   * @param goal the goal to move to from the current state
   * @return safe intermediate wrist state
   */
  private double getSafeWristState(State goal) {
    var curState = getState();
    // If the wrist movement never goes through an unsafe state, go direct to goal
    if (!goesThroughPotentialUnsafeState(
        curState.wristPosition, goal.wristPosition, wristLowerUnsafeBound, wristUpperUnsafeBound)) {
      return goal.wristPosition;
    }
    // Wrist must pass through potential unsafe state
    // If we're outside of the arm no-go zone, we can move the wrist wherever
    if (!isBetween(curState.armPosition, armLowerUnsafeBound, armUpperUnsafeBound)) {
      // If the goal is safe, wrist can go directly to goal
      if (!isBetween(goal.wristPosition, wristLowerUnsafeBound, wristUpperUnsafeBound)) {
        return goal.wristPosition;
      } else {
        // We can move the wrist anywhere currently, but the goal is unsafe- move to the most
        // optimal safe position
        var upperSafePositionDistance =
            Math.abs(curState.wristPosition - wristUpperSafeState)
                + Math.abs(wristUpperSafeState - goal.wristPosition);
        var lowerSafePositionDistance =
            Math.abs(curState.wristPosition - wristLowerSafeState)
                + Math.abs(wristLowerSafeState - goal.wristPosition);
        if (upperSafePositionDistance < lowerSafePositionDistance) {
          return wristLowerSafeState;
        } else {
          return wristUpperSafeState;
        }
      }
    } else {
      // We are inside the arm no-go zone, and the wrist movement is potentially unsafe
      // If the wrist is currently safe, keep it there
      if (!isBetween(curState.wristPosition, wristLowerUnsafeBound, wristUpperUnsafeBound)) {
        return curState.wristPosition;
      } else {
        // This is bad. We don't have a guarantee that the wrist is safe currently.
        return nearestAlwaysSafeWristState(wristLowerSafeState);
      }
    }
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
        .andThen(wrist.setGoalCommand(() -> goalState.wristPosition));
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

  private static boolean isBetween(double value, double lower, double upper) {
    return value < upper && value > lower;
  }
}
