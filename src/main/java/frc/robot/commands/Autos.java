package frc.robot.commands;

import static edu.wpi.first.wpilibj2.command.Commands.sequence;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.EndEffector;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.Swerve;

public class Autos {
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

  public Command centerTwoNote() {
    return sequence(
        superstructure.setGoalState(Superstructure.State.intake),
        endEffector.spinUpAndShoot(70),
        
        runPathWithReset("C_to_CN"),
        runPath("CN_to_C"),
        endEffector.spinUpAndShoot(70),
        runPath("C_to_Leave"));
  }

  public Command runPath(String pathName) {
    var path = PathPlannerPath.fromPathFile(pathName);
    return AutoBuilder.followPath(path);
  }

  public Command runPathWithReset(String pathName) {
    var path = PathPlannerPath.fromPathFile(pathName);
    return AutoBuilder.followPath(path)
        .beforeStarting(
            Commands.runOnce(
                () -> {
                  drivetrain.setPose(path.getPreviewStartingHolonomicPose());
                }));
  }

  public Command runPathPlannerAuto(String autoName) {
    return new PathPlannerAuto(autoName);
  }
}
