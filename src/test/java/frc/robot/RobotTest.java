package frc.robot;

import static org.junit.jupiter.api.Assertions.assertDoesNotThrow;

import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;
import org.wpilib.hardware.hal.HAL;
import org.wpilib.simulation.SimHooks;

/**
 * Smoke test: builds the whole robot (YAGSL JSON parsing, the YAMS swerve drive, and every opmode) in simulation and
 * runs the robot loop for a second, so a broken JSON config, vendordep, or API change fails the build instead of the
 * robot.
 */
class RobotTest
{

  @BeforeAll
  static void initializeHal()
  {
    HAL.initialize();
  }

  @Test
  void robotBuildsAndRuns()
  {
    try (Robot robot = new Robot())
    {
      assertDoesNotThrow(() -> {
        for (int i = 0; i < 50; i++)
        {
          robot.robotPeriodic();
          robot.simulationPeriodic();
          SimHooks.stepTiming(0.02);
        }
      });
    }
  }
}
