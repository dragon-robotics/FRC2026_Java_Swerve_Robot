package frc.robot.io;

import com.ctre.phoenix6.BaseStatusSignal;
import java.util.ArrayList;
import java.util.List;

/**
 * Registry for CTRE status signals that should be refreshed in one batch call.
 *
 * <p>Robot code should register each {@link MotorIO} once, then call {@link #refreshAll()} once per
 * robot loop before subsystem periodic methods read cached signal values.
 */
public class SignalRegistry {

  private static final SignalRegistry INSTANCE = new SignalRegistry();

  private final List<BaseStatusSignal> allSignals = new ArrayList<>();
  private BaseStatusSignal[] signalArray = new BaseStatusSignal[0];

  private SignalRegistry() {}

  /** Returns the process-wide signal registry. */
  public static SignalRegistry getInstance() {
    return INSTANCE;
  }

  /** Adds CTRE signals exposed by a motor IO implementation. */
  public void registerMotorIO(MotorIO motorIO) {
    BaseStatusSignal[] signals = motorIO.getStatusSignals();

    if (signals.length == 0) {
      return;
    }

    for (BaseStatusSignal signal : signals) {
      allSignals.add(signal);
    }

    signalArray = allSignals.toArray(new BaseStatusSignal[0]);
  }

  /**
   * Batch-refresh all registered CTRE signals in one CAN call. Call once at the top of
   * robotPeriodic() BEFORE CommandScheduler.run().
   */
  public void refreshAll() {
    if (hasRegisteredSignals()) {
      BaseStatusSignal.refreshAll(signalArray);
    }
  }

  /** Returns the number of registered CTRE status signals. */
  public int getSignalCount() {
    return signalArray.length;
  }

  private boolean hasRegisteredSignals() {
    return signalArray.length > 0;
  }
}
