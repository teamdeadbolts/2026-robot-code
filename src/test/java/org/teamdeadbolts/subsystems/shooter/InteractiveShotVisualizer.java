/* The Deadbolts (C) 2026 */
package org.teamdeadbolts.subsystems.shooter;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.NetworkTableInstance;
import io.javalin.Javalin;
import java.util.ArrayList;
import java.util.List;
import org.junit.jupiter.api.Tag;
import org.junit.jupiter.api.Test;
import org.teamdeadbolts.constants.ShooterConstants;
import org.teamdeadbolts.utils.tuning.SavedLoggedNetworkNumber;

@Tag("visualizer")
public class InteractiveShotVisualizer {
    private static final SavedLoggedNetworkNumber calcIterations =
            SavedLoggedNetworkNumber.get("Tuning/Shooter/CalcIterations", 30);
    private static final SavedLoggedNetworkNumber minImpactAngle =
            SavedLoggedNetworkNumber.get("Tuning/Shooter/MinImpactAngleDegrees", 50);
    private static final SavedLoggedNetworkNumber maxImpactAngle =
            SavedLoggedNetworkNumber.get("Tuning/Shooter/MaxImpactAngleDegrees", 90);
    private static final SavedLoggedNetworkNumber shootLatancyMs =
            SavedLoggedNetworkNumber.get("Tuning/Shooter/ShootLatancyMs", 0);
    private static final SavedLoggedNetworkNumber timeToKeepVel =
            SavedLoggedNetworkNumber.get("Tuning/Shooter/TimeToKeepVelMs", 1000);
    private static final SavedLoggedNetworkNumber airResistanceMultiplier =
            SavedLoggedNetworkNumber.get("Tuning/Shooter/airResistanceMultiplier", 0.01);

    // 1. Define the Data Structures to communicate with the web UI
    public record SimRequest(
            double rX,
            double rY,
            double rZ,
            double rRot,
            double tX,
            double tY,
            double tZ,
            double vX,
            double vY,
            double vOmega,
            double calcIters,
            double minImpact,
            double maxImpact,
            double latencyMs,
            double timeToKeep,
            double airRes) {}

    public record SimResponse(
            List<Double> xCoords,
            List<Double> yCoords,
            List<Double> zCoords,
            double startX,
            double startY,
            double startZ,
            double targetX,
            double targetY,
            double targetZ,
            double virtX,
            double virtY,
            double virtZ,
            double aimU,
            double aimV,
            double aimW,
            double calcHoodAngle,
            double calcTurretAngle,
            double calcRpm,
            double impactAngle,
            double ballVelocity) {}

    @Test
    public void runInteractiveServer() throws InterruptedException {
        System.out.println("Starting Interactive Shot Visualizer...");

        // Start NetworkTables so SavedLoggedNetworkNumber works
        NetworkTableInstance inst = NetworkTableInstance.getDefault();
        inst.startServer();

        ShotCalculator calculator = new ShotCalculator();

        // Start Web Server
        Javalin app = Javalin.create(config -> {
                    config.showJavalinBanner = false;
                })
                .start(7070);

        System.out.println("==================================================");
        System.out.println(" Server Started Open http://localhost:7070 in your browser.");
        System.out.println("==================================================");

        app.get("/", ctx -> ctx.html(HTML_PAGE));

        app.post("/simulate", ctx -> {
            SimRequest req = ctx.bodyAsClass(SimRequest.class);

            calcIterations.set(req.calcIters());
            minImpactAngle.set(req.minImpact());
            maxImpactAngle.set(req.maxImpact());
            shootLatancyMs.set(req.latencyMs());
            timeToKeepVel.set(req.timeToKeep());
            airResistanceMultiplier.set(req.airRes());

            // Wait a tiny bit for NT subscribers to update

            // Wait a tiny bit for NT subscribers to update
            try {
                Thread.sleep(5);
            } catch (InterruptedException ignored) {
            }

            // Setup State
            double simulationTime = 1.0;
            Pose3d robotPose = new Pose3d(req.rX(), req.rY(), req.rZ(), new Rotation3d(0, 0, req.rRot()));
            Translation3d targetPos = new Translation3d(req.tX(), req.tY(), req.tZ());
            ChassisSpeeds speeds = new ChassisSpeeds(req.vX(), req.vY(), req.vOmega());

            // Run Exact Calculator Math
            calculator.updateVelocityState(simulationTime, speeds);
            ShotCalculator.ShotParameters result =
                    calculator.calculateShot(robotPose, targetPos, simulationTime, 0.0, Math.PI / 2);

            // Extract Turret info
            Pose3d turretPose = robotPose.transformBy(ShooterConstants.SHOOTER_OFFSET);
            double distFromPivot =
                    turretPose.getTranslation().toTranslation2d().getDistance(targetPos.toTranslation2d());
            double pureV0 = result.ballVelocity / (1 + (req.airRes() * distFromPivot));

            double theta = result.rawLaunchAngleRad;
            double phi = result.turretAngle + robotPose.getRotation().getZ();

            // Calculate Vector components
            double u = Math.cos(theta) * Math.cos(phi);
            double v = Math.cos(theta) * Math.sin(phi);
            double w = Math.sin(theta);

            // Calculate the field-relative offset of the turret
            Translation2d robotRelOffset =
                    ShooterConstants.SHOOTER_OFFSET.getTranslation().toTranslation2d();
            Translation2d fieldRelOffset = robotRelOffset.rotateBy(
                    new Rotation2d(robotPose.getRotation().getZ()));

            // Calculate the true linear velocity of the turret (Chassis V + Tangential V)
            double turretVx = speeds.vxMetersPerSecond - (speeds.omegaRadiansPerSecond * fieldRelOffset.getY());
            double turretVy = speeds.vyMetersPerSecond + (speeds.omegaRadiansPerSecond * fieldRelOffset.getX());

            // True environmental velocities applied to the ball
            double vx = (pureV0 * u) + turretVx;
            double vy = (pureV0 * v) + turretVy;
            double vz = pureV0 * w;

            double exitRadius = ShooterConstants.EXIT_RADIUS_METERS;
            double x0 = turretPose.getX() + (exitRadius * u);
            double y0 = turretPose.getY() + (exitRadius * v);
            double z0 = turretPose.getZ() + (exitRadius * w);

            // Simulate Trajectory
            List<Double> xCoords = new ArrayList<>();
            List<Double> yCoords = new ArrayList<>();
            List<Double> zCoords = new ArrayList<>();
            double g = 9.81;

            for (double t = 0; t <= 3.0; t += 0.02) {
                double x = x0 + (vx * t);
                double y = y0 + (vy * t);
                double z = z0 + (vz * t) - (0.5 * g * Math.pow(t, 2));

                xCoords.add(x);
                yCoords.add(y);
                zCoords.add(z);
                if (z < 0) break;
            }

            // Package it all up and send back to the web UI
            SimResponse response = new SimResponse(
                    xCoords,
                    yCoords,
                    zCoords,
                    x0,
                    y0,
                    z0,
                    targetPos.getX(),
                    targetPos.getY(),
                    targetPos.getZ(),
                    result.virtTarget.getX(),
                    result.virtTarget.getY(),
                    result.virtTarget.getZ(),
                    u,
                    v,
                    w,
                    Math.toDegrees(result.hoodAngle),
                    Math.toDegrees(result.turretAngle),
                    result.wheelSpeed,
                    Math.toDegrees(result.impactAngle),
                    result.ballVelocity);

            ctx.json(response);
        });

        while (true) {
            Thread.sleep(10000);
        }
    }

    // 4. The HTML UI embedded directly into the Java file
    private static final String HTML_PAGE = """
        <!DOCTYPE html>
        <html>
        <head>
            <meta charset="UTF-8">
            <title>Interactive Shot Calculator</title>
            <script src="https://cdn.plot.ly/plotly-2.24.1.min.js"></script>
            <style>
                body { margin: 0; display: flex; font-family: sans-serif; background: #111; color: #ddd; }
                #sidebar { width: 350px; padding: 20px; background: #222; height: 100vh; overflow-y: auto; box-sizing: border-box;}
                #plot { flex-grow: 1; height: 100vh; }
                .control-group { margin-bottom: 15px; }
                .control-group label { display: block; font-size: 12px; margin-bottom: 5px; color: #aaa;}

                /* Styling for the new number inputs */
                .control-group input[type="number"] {
                    width: 100%; padding: 5px; box-sizing: border-box;
                    background: #333; color: cyan; border: 1px solid #555;
                    border-radius: 4px; font-weight: bold;
                }

                .val-display { float: right; color: cyan; font-weight: bold; }
                h3 { margin-top: 0; border-bottom: 1px solid #444; padding-bottom: 5px; }
                .stats { background: #000; padding: 10px; border-radius: 5px; border: 1px solid #333; margin-top: 20px; }
                .stats div { margin-bottom: 5px; font-size: 14px; }
                .stats span { color: #0f0; font-weight: bold; }
            </style>
        </head>
        <body>
            <div id="sidebar">
                <h3>Robot Pose</h3>
                <div class="control-group"><label>X Position (m) <span id="rXVal" class="val-display">0</span></label><input type="number" id="rX" min="-5" max="5" step="0.1" value="0" oninput="updateUI()"></div>
                <div class="control-group"><label>Y Position (m) <span id="rYVal" class="val-display">-0.5</span></label><input type="number" id="rY" min="-5" max="5" step="0.1" value="0.0" oninput="updateUI()"></div>
                <div class="control-group"><label>Rotation (rad) <span id="rRotVal" class="val-display">0</span></label><input type="number" id="rRot" min="-3.14" max="3.14" step="0.05" value="0" oninput="updateUI()"></div>

                <h3>Robot Speeds</h3>
                <div class="control-group"><label>Vx (m/s) <span id="vXVal" class="val-display">0</span></label><input type="number" id="vX" min="-5" max="5" step="0.1" value="0" oninput="updateUI()"></div>
                <div class="control-group"><label>Vy (m/s) <span id="vYVal" class="val-display">0</span></label><input type="number" id="vY" min="-5" max="5" step="0.1" value="0" oninput="updateUI()"></div>
                <div class="control-group"><label>Omega (rad/s) <span id="vOmegaVal" class="val-display">0</span></label><input type="number" id="vOmega" min="-3.14" max="3.14" step="0.1" value="0" oninput="updateUI()"></div>

                <h3>Target Position</h3>
                <div class="control-group"><label>Target X (m) <span id="tXVal" class="val-display">3.0</span></label><input type="number" id="tX" min="-10" max="10" step="0.1" value="1.0" oninput="updateUI()"></div>
                <div class="control-group"><label>Target Y (m) <span id="tYVal" class="val-display">5.0</span></label><input type="number" id="tY" min="-10" max="10" step="0.1" value="1.0" oninput="updateUI()"></div>
                <div class="control-group"><label>Target Z (m) <span id="tZVal" class="val-display">2.0</span></label><input type="number" id="tZ" min="0" max="4" step="0.1" value="1.8288" oninput="updateUI()"></div>

                <h3>Tuning & NetworkTables</h3>
                <div class="control-group"><label>Calc Iterations</label><input type="number" id="calcIters" value="30" step="1" onchange="runSimulation()"></div>
                <div class="control-group"><label>Min Impact Angle (°)</label><input type="number" id="minImpact" value="50" step="1" onchange="runSimulation()"></div>
                <div class="control-group"><label>Max Impact Angle (°)</label><input type="number" id="maxImpact" value="90" step="1" onchange="runSimulation()"></div>
                <div class="control-group"><label>Shoot Latency (ms)</label><input type="number" id="latencyMs" value="0" step="1" onchange="runSimulation()"></div>
                <div class="control-group"><label>Time to Keep Vel (ms)</label><input type="number" id="timeToKeep" value="1000" step="100" onchange="runSimulation()"></div>
                <div class="control-group"><label>Air Resistance Multiplier</label><input type="number" id="airRes" value="0.01" step="0.001" onchange="runSimulation()"></div>

                <div class="stats">
                    <div>Hood Angle: <span id="outHood">0.0</span>°</div>
                    <div>Turret Angle: <span id="outTurret">0.0</span>°</div>
                    <div>Wheel RPM: <span id="outRpm">0</span> RPM</div>
                    <div>Ball Velocity: <span id="ballVelocity">0</span> m/s</div>
                    <div>Impact Angle: <span id="impactAngle">0.0</span>°</div>
                </div>
            </div>

            <div id="plot"></div>

            <script>
                const layout = {
                    title: 'Live Shot Trajectory',
                    scene: { aspectmode: 'data', xaxis: {title: 'X'}, yaxis: {title: 'Y'}, zaxis: {title: 'Z'} },
                    paper_bgcolor: '#111', font: {color: 'white'}
                };
                Plotly.newPlot('plot', [], layout);

                function updateUI() {
                    ['rX','rY','rRot','vX','vY','vOmega','tX','tY','tZ'].forEach(id => {
                        document.getElementById(id+'Val').innerText = document.getElementById(id).value;
                    });
                    runSimulation();
                }

                async function runSimulation() {
                    const req = {
                        rX: parseFloat(document.getElementById('rX').value),
                        rY: parseFloat(document.getElementById('rY').value),
                        rZ: 0,
                        rRot: parseFloat(document.getElementById('rRot').value),
                        tX: parseFloat(document.getElementById('tX').value),
                        tY: parseFloat(document.getElementById('tY').value),
                        tZ: parseFloat(document.getElementById('tZ').value),
                        vX: parseFloat(document.getElementById('vX').value),
                        vY: parseFloat(document.getElementById('vY').value),
                        vOmega: parseFloat(document.getElementById('vOmega').value),

                        // Parse the new text boxes
                        calcIters: parseFloat(document.getElementById('calcIters').value),
                        minImpact: parseFloat(document.getElementById('minImpact').value),
                        maxImpact: parseFloat(document.getElementById('maxImpact').value),
                        latencyMs: parseFloat(document.getElementById('latencyMs').value),
                        timeToKeep: parseFloat(document.getElementById('timeToKeep').value),
                        airRes: parseFloat(document.getElementById('airRes').value),
                    };

                    const res = await fetch('/simulate', {
                        method: 'POST',
                        headers: {'Content-Type': 'application/json'},
                        body: JSON.stringify(req)
                    });

                    const data = await res.json();

                    document.getElementById('outHood').innerText = data.calcHoodAngle.toFixed(2);
                    document.getElementById('outTurret').innerText = data.calcTurretAngle.toFixed(2);
                    document.getElementById('outRpm').innerText = data.calcRpm.toFixed(0);
                    document.getElementById('impactAngle').innerText = data.impactAngle.toFixed(2);
                    document.getElementById('ballVelocity').innerText = data.ballVelocity.toFixed(0);

                    const traces = [
                        { x: data.xCoords, y: data.yCoords, z: data.zCoords, mode: 'lines', type: 'scatter3d', name: 'Ball Path', line: {width: 6, color: 'cyan'} },
                        { x: [data.startX], y: [data.startY], z: [data.startZ], mode: 'markers', type: 'scatter3d', name: 'Turret Start', marker: {size: 8, color: 'green'} },
                        { x: [data.targetX], y: [data.targetY], z: [data.targetZ], mode: 'markers', type: 'scatter3d', name: 'Actual Target', marker: {size: 8, color: 'red'} },
                        { x: [data.virtX], y: [data.virtY], z: [data.virtZ], mode: 'markers', type: 'scatter3d', name: 'Virtual Target', marker: {size: 6, color: 'onumber', symbol: 'cross'} },
                        { type: 'cone', x: [data.startX], y: [data.startY], z: [data.startZ], u: [data.aimU], v: [data.aimV], w: [data.aimW], sizemode: 'absolute', sizeref: 0.6, anchor: 'tail', showscale: false, colorscale: [[0, 'yellow'], [1, 'yellow']], name: 'Turret Aim' }
                    ];

                    Plotly.react('plot', traces, layout);
                }

                updateUI();
            </script>
        </body>
        </html>
        """;
}
