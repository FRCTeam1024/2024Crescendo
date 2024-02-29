package frc.robot;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import java.util.function.BooleanSupplier;

public class StatusDashboard {
  private static ShuffleboardTab tab = Shuffleboard.getTab("Status");

  private static int maxColumns = 5;

  private static int count = 0;

  public static void addStatusIndicator(String identifier, BooleanSupplier status) {
    tab.addBoolean(identifier, status)
        .withSize(1, 1)
        .withPosition(count % maxColumns, count / maxColumns);
    count++;
  }

  public static void addStatusIndicator(String identifier, boolean status) {
    tab.add(identifier, status).withSize(1, 1).withPosition(count % maxColumns, count / maxColumns);
    count++;
  }
}
