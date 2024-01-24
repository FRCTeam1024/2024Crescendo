package frc.robot.commands;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
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
    return runPathAuto("3 Feet");
  }

  public Command circuitAuto() {
    return runPathAuto("Circuit");
  }

  public Command runPathAuto(String pathName) {
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
