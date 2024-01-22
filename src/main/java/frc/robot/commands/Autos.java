package frc.robot.commands;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.Swerve;

public class Autos {
  Swerve drivetrain;

  public Autos(Swerve drivetrain) {
    this.drivetrain = drivetrain;
  }

  public Command driveStraight() {
    var path = PathPlannerPath.fromPathFile("Circuit");
    System.out.println(path.getPreviewStartingHolonomicPose());
    return AutoBuilder.followPath(path)
        .beforeStarting(
            Commands.runOnce(
                () -> {
                  drivetrain.setPose(path.getPreviewStartingHolonomicPose());
                }));
  }
}
