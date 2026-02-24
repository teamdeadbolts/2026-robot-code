/* The Deadbolts (C) 2026 */
package org.teamdeadbolts.subsystems.shooter;

import static org.junit.jupiter.api.Assertions.assertNotEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;
import static org.mockito.ArgumentMatchers.anyDouble;
import static org.mockito.ArgumentMatchers.eq;
import static org.mockito.Mockito.mock;
import static org.mockito.Mockito.mockStatic;
import static org.mockito.Mockito.when;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation3d;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.littletonrobotics.junction.Logger;
import org.mockito.MockedStatic;
import org.teamdeadbolts.RobotState;
import org.teamdeadbolts.utils.tuning.SavedLoggedNetworkNumber;

public class ShotCalculatorTest {
    private ShotCalculator calculator;

    private MockedStatic<SavedLoggedNetworkNumber> mockedNetworkNumbers;
    private MockedStatic<RobotState> mockedRobotStateClass;
    private MockedStatic<Logger> mockedLogger;

    private SavedLoggedNetworkNumber mockCalcIterations;
    private SavedLoggedNetworkNumber mockImpactAngle;
    private SavedLoggedNetworkNumber mockLatency;

    private RobotState mockRobotState;

    @BeforeEach
    public void setup() {
        calculator = new ShotCalculator();
        mockedLogger = mockStatic(Logger.class);

        mockedNetworkNumbers = mockStatic(SavedLoggedNetworkNumber.class);
        mockCalcIterations = mock(SavedLoggedNetworkNumber.class);
        mockImpactAngle = mock(SavedLoggedNetworkNumber.class);
        mockLatency = mock(SavedLoggedNetworkNumber.class);

        mockedNetworkNumbers
                .when(() -> SavedLoggedNetworkNumber.get(eq("Tuning/Shooter/CalcIterations"), anyDouble()))
                .thenReturn(mockCalcIterations);
        mockedNetworkNumbers
                .when(() -> SavedLoggedNetworkNumber.get(eq("Tuning/Shooter/ImpactAngleDegrees"), anyDouble()))
                .thenReturn(mockImpactAngle);
        mockedNetworkNumbers
                .when(() -> SavedLoggedNetworkNumber.get(eq("Tuning/Shooter/ShooterLatencyMs"), anyDouble()))
                .thenReturn(mockLatency);

        mockedRobotStateClass = mockStatic(RobotState.class);
        mockRobotState = mock(RobotState.class);
        mockedRobotStateClass.when(RobotState::getInstance).thenReturn(mockRobotState);
        when(mockRobotState.getRobotPose()).thenReturn(new Pose3d());
    }

    @AfterEach
    public void teardown() {
        mockedLogger.close();
        mockedNetworkNumbers.close();
        mockedRobotStateClass.close();
    }

    @Test
    public void testStaticShot() {
        Pose3d robotPOse = new Pose3d();
        Translation3d target = new Translation3d(5, 0, 0);
        calculator.updateVelocityState(1000.0, 0, 0, 0);

        ShotParametersAutoLogged shot = calculator.calculateShot(robotPOse, target, 1000);

        System.out.println(shot.toString());
        assertTrue(shot.wheelSpeed > 0, "Wheel speed should be positive");
        assertTrue(shot.hoodAngle > 0 && shot.hoodAngle < Math.PI / 2, "Hood angle should be between 0 and 90 deg");
    }

    @Test
    public void testVelocityLookaheadLinear() {
        Pose3d robotPose = new Pose3d();
        Translation3d target = new Translation3d(5.0, 0.0, 2.0);

        // Simulate driving steadily TOWARDS the target at 2 m/s on the X axis
        calculator.updateVelocityState(1000.0, 2.0, 0.0, 0.0);
        calculator.updateVelocityState(1020.0, 2.0, 0.0, 0.0);

        ShotParametersAutoLogged movingShot = calculator.calculateShot(robotPose, target, 1020.0);

        ShotCalculator staticCalc = new ShotCalculator();
        staticCalc.updateVelocityState(1020.0, 0.0, 0.0, 0.0);
        ShotParametersAutoLogged staticShot = staticCalc.calculateShot(robotPose, target, 1020.0);
        System.out.printf("MPS Diff: %s\n", staticShot.ballVelocity - movingShot.ballVelocity);

        // Because we are driving towards the target, the virtual target should be CLOSER, requiring less wheel speed
        assertTrue(movingShot.wheelSpeed < staticShot.wheelSpeed, "Moving towards target should reduce required RPM");
    }

    @Test
    public void testTurretInducedVelocityFromRotation() {
        Pose3d robotPose = new Pose3d();
        Translation3d target = new Translation3d(5.0, 0.0, 2.0);

        calculator.updateVelocityState(1000.0, 0.0, 0.0, 2.0);
        calculator.updateVelocityState(1020.0, 0.0, 0.0, 2.0);

        ShotParametersAutoLogged shot = calculator.calculateShot(robotPose, target, 1020.0);

        assertNotEquals(0.0, shot.turretAngle, "Turret should compensate for rotational whip");
    }

    @Test
    public void testMPSToRPM() {
        double mps = 20.0;
        double rpm = ShotCalculator.shooterMPSToRPM(mps);
        assertTrue(rpm > 0, "RPM should be strictly positive for positive MPS");
    }

    @Test
    public void generateBallisticCurveCSV() {
        System.out.println("Distance(m),HoodAngle(deg),WheelSpeed(RPM)");

        for (double dist = 1.0; dist <= 8.0; dist += 0.25) {
            Pose3d robotPose = new Pose3d();
            Translation3d target = new Translation3d(dist, 0.0, 2.0);

            calculator.updateVelocityState(0.0, 0.0, 0.0, 0.0);
            ShotParametersAutoLogged shot = calculator.calculateShot(robotPose, target, 0.0);

            double hoodDeg = Math.toDegrees(shot.hoodAngle);

            System.out.printf("%.2f,%.2f,%.2f%n", dist, hoodDeg, shot.wheelSpeed);
        }
    }
}
