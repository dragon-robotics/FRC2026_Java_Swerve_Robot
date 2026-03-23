package frc.robot.io;

import com.ctre.phoenix6.BaseStatusSignal;
import dev.doglog.DogLog;
import java.util.ArrayList;
import java.util.List;

public class SignalRegistry {

  // Singleton instance — one registry for the entire robot
  private static final SignalRegistry INSTANCE = new SignalRegistry();

  private final List<BaseStatusSignal> allSignals = new ArrayList<>();
  private BaseStatusSignal[] signalArray = new BaseStatusSignal[0];

  // Private constructor — use getInstance()
  private SignalRegistry() {}

  public static SignalRegistry getInstance() {
    return INSTANCE;
  }

  public void registerMotorIO(MotorIO motorIO) {
    BaseStatusSignal[] signals = motorIO.getStatusSignals();

    if (signals.length == 0) {
      return;
    }

    for (BaseStatusSignal signal : signals) {
      allSignals.add(signal);
    }

    signalArray = allSignals.toArray(new BaseStatusSignal[0]);

    DogLog.log("SignalRegistry/" + motorIO.getMotorName(),
        "Registered " + signals.length + " signals");
  }

  /**
   * Batch-refresh all registered CTRE signals in one CAN call.
   * Call once at the top of robotPeriodic() BEFORE CommandScheduler.run().
   */
  public void refreshAll() {
    if (signalArray.length > 0) {
      BaseStatusSignal.refreshAll(signalArray);
    }
  }

  public int getSignalCount() {
    return signalArray.length;
  }
}