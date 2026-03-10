/* The Deadbolts (C) 2026 */
package org.teamdeadbolts.subsystems.shooter;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

public class ShotCalculatorTest {
    private NetworkTableInstance testInst;
    private ShotCalculator calculator;

    @BeforeEach
    public void setup() {
        calculator = new ShotCalculator();
        testInst = NetworkTableInstance.create();
        testInst.startServer();
    }

    @AfterEach
    void tearDown() {
        if (testInst != null) {
            testInst.close();
        }
    }

    private void runSimulation(String scenarioName, Pose3d robotPose, ChassisSpeeds speeds) {
        System.out.println("=== Scenario: " + scenarioName + " ===");

        double simulationTime = 1.0;

        calculator.updateVelocityState(simulationTime, speeds);

        ShotCalculator.ShotParameters result =
                calculator.calculateShot(robotPose, new Translation3d(), simulationTime, 0.0, Math.PI / 2);

        double distance2d = robotPose.getTranslation().toTranslation2d().getDistance(new Translation2d());

        System.out.printf("Input Distance (2D): %.2f meters%n", distance2d);
        System.out.printf(
                "Input Robot Speeds : Vx=%.2f m/s, Vy=%.2f m/s, Omega=%.2f rad/s%n",
                speeds.vxMetersPerSecond, speeds.vyMetersPerSecond, speeds.omegaRadiansPerSecond);

        System.out.println("Theoretical Outputs:");
        System.out.printf("  -> Hood Angle   : %.2f degrees%n", Units.radiansToDegrees(result.hoodAngle));
        System.out.printf("  -> Turret Angle : %.2f degrees%n", Units.radiansToDegrees(result.turretAngle));
        System.out.printf("  -> Wheel RPM    : %.0f RPM%n", result.wheelSpeed);
        System.out.printf("  -> Ball Velocity: %.2f m/s%n", result.ballVelocity);
        System.out.println("--------------------------------------------------\n");
    }

    @Test
    public void testStationaryClose() {
        Pose3d robotPose = new Pose3d(0.0, 3.5, 0.0, new Rotation3d()); // 2 meters away in Y
        ChassisSpeeds speeds = new ChassisSpeeds(0, 0, 0); // Stationary
        runSimulation("Stationary - Close Range (2m)", robotPose, speeds);
    }

    @Test
    public void testStationaryFar() {
        Pose3d robotPose = new Pose3d(0.0, 0.5, 0.0, new Rotation3d()); // 5 meters away in Y
        ChassisSpeeds speeds = new ChassisSpeeds(0, 0, 0); // Stationary
        runSimulation("Stationary - Far Range (5m)", robotPose, speeds);
    }

    @Test
    public void testMovingForwardTowardsTarget() {
        Pose3d robotPose = new Pose3d(0.0, 0.5, 0.0, new Rotation3d()); // 5 meters away
        ChassisSpeeds speeds = new ChassisSpeeds(0, 3.0, 0);
        runSimulation("Moving Forward Towards Target (3 m/s)", robotPose, speeds);
    }

    @Test
    public void testMovingSidewaysRelativeToTarget() {
        Pose3d robotPose = new Pose3d(0.0, 0.5, 0.0, new Rotation3d()); // 5 meters away
        ChassisSpeeds speeds = new ChassisSpeeds(3.0, 0, 0);
        runSimulation("Moving Sideways / Strafing (3 m/s)", robotPose, speeds);
    }

    @Test
    public void testRotatingInPlace() {
        Pose3d robotPose = new Pose3d(0.0, 2.5, 0.0, new Rotation3d()); // 3 meters away
        ChassisSpeeds speeds = new ChassisSpeeds(0, 0, Math.PI);
        runSimulation("Rotating in Place (180 deg/s)", robotPose, speeds);
    }
}
