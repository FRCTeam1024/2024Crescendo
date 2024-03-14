package frc.lib.util;

import static edu.wpi.first.wpilibj2.command.Commands.startEnd;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class CommandUtils {

  public static Command rumbleController(
      CommandXboxController controller, double value, Subsystem... requirements) {
    return startEnd(
        () -> controller.getHID().setRumble(RumbleType.kBothRumble, value),
        () -> controller.getHID().setRumble(RumbleType.kBothRumble, 0),
        requirements);
  }

  public static Command rumbleController(
      CommandXboxController controller, Subsystem... requirements) {
    return rumbleController(controller, 1, requirements);
  }
}
