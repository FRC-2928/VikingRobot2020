package org.ballardrobotics.utilities;

import org.hamcrest.core.IsEqual;
import org.junit.Rule;
import org.junit.Test;
import org.junit.rules.ErrorCollector;

public class RotationUtilityTest {
    private static final double kMinAngle = -225.0;
    private static final double kMaxAngle = 225.0;

    @Rule
    public ErrorCollector collector = new ErrorCollector();

    @Test
    public void testGetTargetAngle() {
        class Test {
            double inputAngle;
            double currentAngle;
            double minAngle;
            double maxAngle;
            double expectedOutput;

            Test(double inputAngle, double currentAngle, double expectedOutput) {
                this(inputAngle, currentAngle, kMinAngle, kMaxAngle, expectedOutput);
            }

            Test(double inputAngle, double currentAngle, double minAngle, double maxAngle, double expectedOutput) {
                this.inputAngle = inputAngle;
                this.currentAngle = currentAngle;
                this.minAngle = minAngle;
                this.maxAngle = maxAngle;
                this.expectedOutput = expectedOutput;
            }
        }

        var tests = new Test[]{
            new Test(25.0, 30.0, 25.0),
            new Test(180.0, -190, -180.0),
        };

        for (var test : tests) {
            String message = "getTargetAngle(" + test.inputAngle + ", " + test.currentAngle + ", " + test.minAngle + ", " + test.maxAngle + ")";
            double got = TurretUtility.getTargetAngle(test.inputAngle, test.currentAngle, test.minAngle, test.maxAngle);
            double want = test.expectedOutput;
            collector.checkThat(message, got, IsEqual.equalTo(want));
        }
    }

}
