package frc.robot.util;

import frc.robot.Constants;
import java.util.HashMap;
import java.util.Map;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

/**
 * A number that can be changed live from the dashboard while the robot is running.
 *
 * <p>When {@link Constants#tuningMode} is true, the value appears in NetworkTables under
 * "/Tuning/[key]". Open AdvantageScope (or Elastic), find the key, type a new value, and the robot
 * code picks it up on the next loop. No redeploy needed, which matters a lot when you only get a
 * couple of hours with the robot.
 *
 * <p>IMPORTANT: values typed into the dashboard are NOT saved into the code. When you find a good
 * value, copy it back into the default in the Java file.
 *
 * <p>Because it goes through AdvantageKit's LoggedNetworkNumber, every change is recorded in the
 * log file, so you can see exactly which value was active at any moment when reviewing a log.
 */
public class TunableNumber implements DoubleSupplier {
  private static final String TABLE_KEY = "/Tuning";

  private final String key;
  private final double defaultValue;
  private final LoggedNetworkNumber dashboardNumber;
  private final Map<Integer, Double> lastValues = new HashMap<>();

  /**
   * @param key Name shown on the dashboard, e.g. "Aim/kP"
   * @param defaultValue Value used when tuning mode is off, and the starting value when it's on
   */
  public TunableNumber(String key, double defaultValue) {
    this.key = TABLE_KEY + "/" + key;
    this.defaultValue = defaultValue;
    this.dashboardNumber =
        Constants.tuningMode ? new LoggedNetworkNumber(this.key, defaultValue) : null;
  }

  /** Current value (from the dashboard in tuning mode, otherwise the default). */
  public double get() {
    return dashboardNumber != null ? dashboardNumber.get() : defaultValue;
  }

  @Override
  public double getAsDouble() {
    return get();
  }

  /**
   * Returns true if the value changed since the last time this was called with the same id. Use it
   * to push new gains into a controller only when they actually change, e.g.
   *
   * <pre>
   * if (kP.hasChanged(hashCode())) controller.setP(kP.get());
   * </pre>
   *
   * @param id A unique number for the caller (hashCode() of the calling object works well)
   */
  public boolean hasChanged(int id) {
    double currentValue = get();
    Double lastValue = lastValues.get(id);
    if (lastValue == null || currentValue != lastValue) {
      lastValues.put(id, currentValue);
      return true;
    }
    return false;
  }

  public String getKey() {
    return key;
  }
}
