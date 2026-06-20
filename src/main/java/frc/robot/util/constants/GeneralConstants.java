package frc.robot.util.constants;

import edu.wpi.first.wpilibj.RobotBase;

/** Global robot-mode flags shared by subsystems, tests, and simulation helpers. */
public final class GeneralConstants {

  /** Runtime mode selected from whether the code is running on real robot hardware. */
  public static final RobotMode CURRENT_MODE = RobotBase.isReal() ? RobotMode.COMP : RobotMode.SIM;

  public static enum RobotMode {
    /** Running in unit-test or nonstandard harness mode. */
    TEST,
    /** Running on competition robot hardware. */
    COMP,
    /** Running in WPILib simulation. */
    SIM
  }

  /** Mutable test hook used to bypass HAL-dependent paths when needed. */
  public static boolean disableHAL = false;
}
