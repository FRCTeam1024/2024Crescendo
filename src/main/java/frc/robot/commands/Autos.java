package frc.robot.commands;

import static edu.wpi.first.wpilibj2.command.Commands.parallel;
import static edu.wpi.first.wpilibj2.command.Commands.runOnce;
import static edu.wpi.first.wpilibj2.command.Commands.sequence;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.GeometryUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.EndEffector;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.Swerve;

public class Autos {

  public static final double autoShooterSpeed = 80;
  Swerve drivetrain;
  Superstructure superstructure;
  EndEffector endEffector;

  public Autos(Swerve drivetrain, Superstructure superstructure, EndEffector endEffector) {
    this.drivetrain = drivetrain;
    this.superstructure = superstructure;
    this.endEffector = endEffector;
  }

  public Command driveStraight() {
    return runPathWithReset("3 Feet");
  }

  public Command circuitAuto() {
    return runPathWithReset("Circuit");
  }

  public Command shootStay() {
    return fireNoteFromSubwoofer();
  }

  /** shoot and leave away from other near notes */
  public Command shootOutsideLeave() {
    return sequence(
        fireNoteFromSubwoofer(),
        superstructure.setGoalState(Superstructure.State.stow),
        runPath("SourceShoot_to_OutsideLeave"));
  }

  /** shoots, goes to far note1 to pickup and goes to subwoofer to shoot */
  public Command FarNote1() {
    return sequence(
        fireNoteFromSubwoofer(),
        superstructure.setGoalState(Superstructure.State.intake),
        parallel(runPathWithReset("SourceShoot_to_FarNote1"), endEffector.intakeNoteAndIndex()),
        parallel(superstructure.setGoalState(Superstructure.State.stow), runPath("FarSourceShoot1")),
        fireNoteFromSubwoofer());
  }

  /** shoots, goes to far note2 to pickup and goes to subwoofer to shoot */
  public Command FarNote2() {
    return sequence(
        fireNoteFromSubwoofer(),
        superstructure.setGoalState(Superstructure.State.intake),
        parallel(runPathWithReset("Source_to_FarNote2"), endEffector.intakeNoteAndIndex()),
        parallel(
        superstructure.setGoalState(Superstructure.State.stow),
        runPath("FarSourceShoot2")),
        fireNoteFromSubwoofer());
  }

  /** shoots, goes to far note3(center) to pickup and goes to subwoofer to shoot */
  public Command FarNote3() {
    return sequence(
        fireNoteFromSubwoofer(),
        superstructure.setGoalState(Superstructure.State.intake),
        parallel(runPathWithReset("C_to_CN"), endEffector.intakeNoteAndIndex()),
        runPath("CN_to_C"),
        endEffector.spinUpAndShoot(autoShooterSpeed),
        parallel(runPathWithReset("Center_to_FarNote3"), endEffector.intakeNoteAndIndex()),
        parallel(superstructure.setGoalState(Superstructure.State.stow),runPath("FarNote3_to_Center")),
        fireNoteFromSubwoofer());
  }
   /** shoots, goes to far note4 to pickup and goes to subwoofer to shoot */
  public Command FarNote4() {
    return sequence(
        fireNoteFromSubwoofer(),
        superstructure.setGoalState(Superstructure.State.intake),
        parallel(runPathWithReset("AMP_to_AMPNote4"), endEffector.intakeNoteAndIndex()),
        parallel(superstructure.setGoalState(Superstructure.State.stow),runPath("AMPNote4_to_AMP")),
        fireNoteFromSubwoofer());
  }
  /** shoots, goes to far note5 to pickup and goes to subwoofer to shoot */
  public Command FarNote5() {
    return sequence(
        fireNoteFromSubwoofer(),
        superstructure.setGoalState(Superstructure.State.intake),
        parallel(runPathWithReset("Center_to_AMPNote5"), endEffector.intakeNoteAndIndex()),
        parallel(superstructure.setGoalState(Superstructure.State.stow),runPath("AMPNote5_to_AMP")),
        fireNoteFromSubwoofer());
  }

  /** shoots preload, intake center note and shoot, and leave/stow */
  public Command centerTwoNote() {
    return sequence(
        fireNoteFromSubwoofer(),
        superstructure.setGoalState(Superstructure.State.intake),
        runPathWithReset("C_to_CN"),
        parallel(superstructure.setGoalState(Superstructure.State.stow), runPath("CN_to_C")),
        superstructure.setGoalState(Superstructure.State.intake),
        endEffector.spinUpAndShoot(autoShooterSpeed),
        parallel(superstructure.setGoalState(Superstructure.State.stow), runPath("C_to_Leave")));
  }

  /** fire preload, intake and shoot center note, and leave/stow */
  public Command SourceTwoNote() {
    return sequence(
        fireNoteFromSubwoofer(),
        superstructure.setGoalState(Superstructure.State.intake),
        runPathWithReset("Source_to_SourceN"),
        superstructure.setGoalState(Superstructure.State.scoreFromSubwoofer),
        runPath("SourceN_to_SourceShoot"),
        endEffector.spinUpAndShoot(autoShooterSpeed),
        parallel(
            superstructure.setGoalState(Superstructure.State.stow),
            runPath("SourceShoot_to_Leave")));
  }

  /** shoot preload, intake SourceNote and leave/stow. */
  public Command SourceNoteLeave() {
    return sequence(
        fireNoteFromSubwoofer(),
        superstructure.setGoalState(Superstructure.State.intake),
        runPathWithReset("Source_to_SourceN"),
        parallel(
            superstructure.setGoalState(Superstructure.State.stow),
            runPath("SourceShoot_to_Leave")));
  }

  /** shoots, goes to AMPNote, set shoot position, shoot and Leave/stow. */
  public Command AMPTwoNote() {
    return sequence(
        fireNoteFromSubwoofer(),
        superstructure.setGoalState(Superstructure.State.intake),
        runPathWithReset("AMP_to_AMPN"),
        superstructure.setGoalState(Superstructure.State.scoreFromSubwoofer),
        runPath("AMPN_to_AMPShoot"),
        new WaitCommand(.5),
        endEffector.spinUpAndShoot(autoShooterSpeed),
        parallel(
            superstructure.setGoalState(Superstructure.State.stow), runPath("AMPShoot_to_Leave")));
  }

  /** Shoot preload, set intake and pickup while leaving/stow. */
  public Command AMPNoteLeave() {
    return sequence(
        fireNoteFromSubwoofer(),
        superstructure.setGoalState(Superstructure.State.intake),
        runPath("AMP_to_Leave"),
        superstructure.setGoalState(Superstructure.State.stow));
  }

  /** Shoot preload, set intake and pickup while leaving then stow. */
  public Command centerNoteLeave() {
    return sequence(
        fireNoteFromSubwoofer(),
        superstructure.setGoalState(Superstructure.State.intake),
        runPathWithReset("C_to_CN1"),
        superstructure.setGoalState(Superstructure.State.stow));
  }

  /** shoot preload, pickup and shoot all notes near, and leave/stow. */
  public Command allNear() {
    return sequence(
        fireNoteFromSubwoofer(),
        superstructure.setGoalState(Superstructure.State.intake),
        runPathWithReset("C_to_LN"),
        superstructure.setGoalState(Superstructure.State.scoreFromSubwoofer),
        runPath("LN_to_C"),
        endEffector.spinUpAndShoot(autoShooterSpeed),
        superstructure.setGoalState(Superstructure.State.intake),
        runPath("C_to_CN"),
        superstructure.setGoalState(Superstructure.State.scoreFromSubwoofer),
        runPath("CN_to_C"),
        endEffector.spinUpAndShoot(autoShooterSpeed),
        superstructure.setGoalState(Superstructure.State.intake),
        runPath("C_to_RN"),
        superstructure.setGoalState(Superstructure.State.scoreFromSubwoofer),
        runPath("RN_to_C"),
        endEffector.spinUpAndShoot(autoShooterSpeed),
        parallel(runPath("C_to_Leave"), superstructure.setGoalState(Superstructure.State.stow)));
  }

  public Command fireNoteFromSubwoofer() {
    return sequence(
        superstructure.setGoalState(Superstructure.State.scoreFromSubwoofer),
        endEffector.spinUpAndShoot(autoShooterSpeed));
  }

  public Command runPath(String pathName) {
    var path = PathPlannerPath.fromPathFile(pathName);
    return AutoBuilder.followPath(path);
  }

  public Command runPathWithReset(String pathName) {
    var path = PathPlannerPath.fromPathFile(pathName);
    return runOnce(
            () -> {
              var startingPose = path.getPreviewStartingHolonomicPose();
              if (drivetrain.shouldFlipPath()) {
                startingPose = GeometryUtil.flipFieldPose(startingPose);
              }
              drivetrain.setPose(startingPose);
            })
        .andThen(AutoBuilder.followPath(path));
  }

  public Command runPathPlannerAuto(String autoName) {
    return new PathPlannerAuto(autoName);
  }
}
