package frc.robot.commands.shooter;

import static org.junit.Assert.assertEquals;

import org.junit.Test;

public class TurretSetPostitionCommandTest {

  @Test
  public void test() {
    for (int i = -360; i <= 360; i++) {
      for (int j = -225; j <= 225; j++) {
        double o1 = TurretSetPositionCommand.getTargetPosition(i, j);
        double o2 = checkValidAngle(i, j);
        assertEquals("i = " + i + ", j = " + j, o1, o2, 0.00001);
      }
    }
  }

  public double checkValidAngle(double reference, double current) {
    double currentAngle = current;
    double newReference = reference % 360;

    // Checks if reference is out of bounds
    if (newReference > 225.0) {
      newReference -= 360;
      return newReference;
    } else if (newReference < -225.0) {
      newReference += 360;
      return newReference;
    }

    // Checks if there's a faster way to reference
    if (newReference - currentAngle <= -180) {
      if (newReference + 360 < 225.0) {
        newReference = newReference + 360;
      }
    } else if (newReference - currentAngle >= 180) {
      if (newReference - 360 > -225.0) {
        newReference = newReference - 360;
      }
    }

    return newReference;
  }
}
