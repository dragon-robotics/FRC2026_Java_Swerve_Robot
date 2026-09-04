// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

class VisionDisabledAutoReseedLifecycleTest {

  @Test
  void startupDisabledAllowsPreMatchReseed() {
    VisionSubsystem vision = createVisionSubsystem();

    assertTrue(
        vision.shouldAutoReseedForRobotState(false),
        "A fresh program should allow vision localization before its first enabled period");
  }

  @Test
  void disabledAfterEnableStaysBlockedThroughCommunicationLoss() {
    VisionSubsystem vision = createVisionSubsystem();

    assertFalse(
        vision.shouldAutoReseedForRobotState(true),
        "Vision must not auto-reseed while the robot is enabled");
    assertFalse(
        vision.shouldAutoReseedForRobotState(false),
        "Losing Driver Station communication after enable must not reopen auto-reseeding");
    assertFalse(
        vision.shouldAutoReseedForRobotState(true),
        "Reconnecting must preserve the reseed lockout for this program run");
    assertFalse(
        vision.shouldAutoReseedForRobotState(false),
        "A later disabled period must remain locked out after reconnecting");
  }

  @Test
  void newProgramRunAllowsStartupRecoveryAgain() {
    VisionSubsystem previousProgram = createVisionSubsystem();
    previousProgram.shouldAutoReseedForRobotState(true);
    assertFalse(previousProgram.shouldAutoReseedForRobotState(false));

    VisionSubsystem restartedProgram = createVisionSubsystem();

    assertTrue(
        restartedProgram.shouldAutoReseedForRobotState(false),
        "A program restart should permit vision to initialize the new pose estimator");
  }

  private static VisionSubsystem createVisionSubsystem() {
    return new VisionSubsystem(null, (pose, timestamp, standardDeviations) -> {});
  }
}
