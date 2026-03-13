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
    private static final SavedLoggedNetworkNumber linerFilter =
            SavedLoggedNetworkNumber.get("Tuning/Shooter/LinearFilter", 5);

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
            double airRes,
            double linearFilter) {}

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

        NetworkTableInstance inst = NetworkTableInstance.getDefault();
        inst.startServer();

        ShotCalculator calculator = new ShotCalculator();

        Javalin app = Javalin.create(config -> {
                    config.showJavalinBanner = false;
                })
                .start(7070);

        System.out.println("==================================================");
        System.out.println(" Server Started Open http://localhost:7070 in your browser.");
        System.out.println("==================================================");

        app.get("/", ctx -> {
            // Inject the true robot offset into the HTML before sending to the client
            String html = HTML_PAGE
                    .replace(
                            "TURRET_OFFSET_X",
                            String.valueOf(ShooterConstants.SHOOTER_OFFSET
                                    .getTranslation()
                                    .getX()))
                    .replace(
                            "TURRET_OFFSET_Y",
                            String.valueOf(ShooterConstants.SHOOTER_OFFSET
                                    .getTranslation()
                                    .getY()))
                    .replace(
                            "TURRET_OFFSET_Z",
                            String.valueOf(ShooterConstants.SHOOTER_OFFSET
                                    .getTranslation()
                                    .getZ()));
            ctx.html(html);
        });

        app.post("/simulate", ctx -> {
            SimRequest req = ctx.bodyAsClass(SimRequest.class);

            calcIterations.set(req.calcIters());
            minImpactAngle.set(req.minImpact());
            maxImpactAngle.set(req.maxImpact());
            shootLatancyMs.set(req.latencyMs());
            timeToKeepVel.set(req.timeToKeep());
            airResistanceMultiplier.set(req.airRes());

            if (linerFilter.get() != req.linearFilter()) {
                linerFilter.set(req.linearFilter());
                calculator.refresh();
            }

            try {
                Thread.sleep(5);
            } catch (InterruptedException ignored) {
            }

            double currentTime = System.currentTimeMillis() / 1000.0;
            Pose3d robotPose = new Pose3d(req.rX(), req.rY(), req.rZ(), new Rotation3d(0, 0, req.rRot()));
            Translation3d targetPos = new Translation3d(req.tX(), req.tY(), req.tZ());
            ChassisSpeeds speeds = new ChassisSpeeds(req.vX(), req.vY(), req.vOmega());

            calculator.updateVelocityState(currentTime, speeds);
            ShotCalculator.ShotParameters result =
                    calculator.calculateShot(robotPose, targetPos, currentTime, 0.0, Math.PI / 2);

            Pose3d turretPose = robotPose.transformBy(ShooterConstants.SHOOTER_OFFSET);

            double theta = result.rawLaunchAngleRad;
            double phi = result.turretAngle + robotPose.getRotation().getZ();

            double u = Math.cos(theta) * Math.cos(phi);
            double v = Math.cos(theta) * Math.sin(phi);
            double w = Math.sin(theta);

            Translation2d robotRelOffset =
                    ShooterConstants.SHOOTER_OFFSET.getTranslation().toTranslation2d();
            Translation2d fieldRelOffset = robotRelOffset.rotateBy(
                    new Rotation2d(robotPose.getRotation().getZ()));

            double turretVx = speeds.vxMetersPerSecond - (speeds.omegaRadiansPerSecond * fieldRelOffset.getY());
            double turretVy = speeds.vyMetersPerSecond + (speeds.omegaRadiansPerSecond * fieldRelOffset.getX());

            double vx = (result.ballVelocity * u) + turretVx;
            double vy = (result.ballVelocity * v) + turretVy;
            double vz = result.ballVelocity * w;

            double exitRadius = ShooterConstants.EXIT_RADIUS_METERS;
            double x0 = turretPose.getX() + (exitRadius * u);
            double y0 = turretPose.getY() + (exitRadius * v);
            double z0 = turretPose.getZ() + (exitRadius * w);

            List<Double> xCoords = new ArrayList<>();
            List<Double> yCoords = new ArrayList<>();
            List<Double> zCoords = new ArrayList<>();
            double g = 9.81;

            double simX = x0;
            double simY = y0;
            double simZ = z0;
            double dt = 0.02;

            for (double t = 0; t <= 3.0; t += dt) {
                xCoords.add(simX);
                yCoords.add(simY);
                zCoords.add(simZ);

                simX += vx * dt;
                simY += vy * dt;
                simZ += vz * dt;
                vz -= g * dt;

                if (simZ < 0) break;
            }

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

    private static final String HTML_PAGE = """
        <!DOCTYPE html>
        <html>
        <head>
            <meta charset="UTF-8">
            <title>Three.js Shot Visualizer</title>
            <script src="https://cdnjs.cloudflare.com/ajax/libs/three.js/r128/three.min.js"></script>
            <style>
                body { margin: 0; display: flex; font-family: sans-serif; background: #111; color: #ddd; overflow: hidden; }
                #sidebar { width: 350px; padding: 20px; background: #222; height: 100vh; overflow-y: auto; box-sizing: border-box; z-index: 10; border-right: 2px solid #444;}
                #canvas-container { flex-grow: 1; height: 100vh; position: relative; }
                .control-group { margin-bottom: 12px; }
                .control-group label { display: block; font-size: 12px; margin-bottom: 5px; color: #aaa;}
                .control-group input[type="number"] { width: 100%; padding: 5px; box-sizing: border-box; background: #333; color: cyan; border: 1px solid #555; border-radius: 4px; font-weight: bold; }
                .val-display { float: right; color: cyan; font-weight: bold; }
                h3 { margin-top: 0; border-bottom: 1px solid #444; padding-bottom: 5px; font-size: 16px; color: #fff;}
                .stats { background: #000; padding: 10px; border-radius: 5px; border: 1px solid #333; margin-top: 20px; }
                .stats div { margin-bottom: 5px; font-size: 14px; }
                .stats span { color: #0f0; font-weight: bold; }
                #overlay { position: absolute; top: 10px; left: 10px; background: rgba(0,0,0,0.8); padding: 15px; border-radius: 5px; pointer-events: none; color: white; border: 1px solid #555;}
                .key { display: inline-block; background: #eee; color: #000; padding: 2px 6px; border-radius: 3px; font-weight: bold; margin: 2px;}
            </style>
        </head>
        <body>
            <div id="sidebar">
                <h3>Robot Pose (Live)</h3>
                <div class="control-group"><label>X Pos (m) <span id="rXVal" class="val-display">0</span></label><input type="number" id="rX" value="0" disabled></div>
                <div class="control-group"><label>Y Pos (m) <span id="rYVal" class="val-display">0</span></label><input type="number" id="rY" value="0" disabled></div>
                <div class="control-group"><label>Rot (rad) <span id="rRotVal" class="val-display">0</span></label><input type="number" id="rRot" value="0" disabled></div>

                <h3>Drive Kinematics</h3>
                <div class="control-group"><label>Max Speed (m/s)</label><input type="number" id="maxSpeed" value="4.0" step="0.5"></div>
                <div class="control-group"><label>Acceleration (m/s²)</label><input type="number" id="accel" value="8.0" step="0.5"></div>
                <div class="control-group"><label>Max Rot Speed (rad/s)</label><input type="number" id="maxOmega" value="3.14" step="0.5"></div>

                <h3>Target Dimensions</h3>
                <div class="control-group"><label>Center X (m)</label><input type="number" id="tX" step="0.1" value="3.0"></div>
                <div class="control-group"><label>Center Y (m)</label><input type="number" id="tY" step="0.1" value="0.0"></div>
                <div class="control-group"><label>Height (Z) (m)</label><input type="number" id="tZ" step="0.1" value="2.0"></div>
                <div class="control-group"><label>Side Width (m)</label><input type="number" id="tSize" step="0.1" value="1.0"></div>

                <h3>Tuning</h3>
                <div class="control-group"><label>Air Resistance</label><input type="number" id="airRes" value="0.01" step="0.001"></div>
                <div class="control-group"><label>Calc Iterations</label><input type="number" id="calcIters" value="30" step="1"></div>
                <div class="control-group"><label>Min Impact Angle (°)</label><input type="number" id="minImpact" value="50" step="1"></div>
                <div class="control-group"><label>Max Impact Angle (°)</label><input type="number" id="maxImpact" value="90" step="1"></div>
                <div class="control-group"><label>Shoot Latency (ms)</label><input type="number" id="latencyMs" value="0" step="1"></div>
                <div class="control-group"><label>Time to Keep Vel (ms)</label><input type="number" id="timeToKeep" value="1000" step="100"></div>
                <div class="control-group"><label>Linear Filter Size</label><input type="number" id="linearFilter" value="5" step="1"></div>

                <div class="stats">
                    <div>Hood Angle: <span id="outHood">0.0</span>°</div>
                    <div>Turret Angle: <span id="outTurret">0.0</span>°</div>
                    <div>Wheel RPM: <span id="outRpm">0</span> RPM</div>
                    <div>Ball Vel: <span id="ballVelocity">0</span> m/s</div>
                </div>
            </div>

            <div id="canvas-container">
                <div id="overlay">
                    <b style="color: #00BFFF; font-size: 16px;">Click 3D area to lock mouse!</b><br><br>
                    <span class="key">W</span><span class="key">A</span><span class="key">S</span><span class="key">D</span> - Field Relative Drive<br>
                    <span class="key">Q</span><span class="key">E</span> - Rotate Robot<br>
                    <span class="key">MOUSE</span> - Free Orbit Camera<br>
                    <span class="key">SPACE</span> - Shoot Ball<br>
                    <span class="key">ESC</span> - Unlock Mouse
                </div>
            </div>

            <script>
                // --- THREE.JS SETUP ---
                const container = document.getElementById('canvas-container');
                const scene = new THREE.Scene();
                scene.background = new THREE.Color('#111111');

                THREE.Object3D.DefaultUp.set(0, 0, 1);
                const camera = new THREE.PerspectiveCamera(60, container.clientWidth / container.clientHeight, 0.1, 100);

                const renderer = new THREE.WebGLRenderer({ antialias: true });
                renderer.setSize(container.clientWidth, container.clientHeight);
                container.appendChild(renderer.domElement);

                scene.add(new THREE.AmbientLight(0xffffff, 0.6));
                const dirLight = new THREE.DirectionalLight(0xffffff, 0.8);
                dirLight.position.set(5, 5, 10);
                scene.add(dirLight);

                const grid = new THREE.GridHelper(30, 30, 0x444444, 0x222222);
                grid.rotation.x = Math.PI / 2;
                scene.add(grid);

                // --- ROBOT & ENVIRONMENT MESHES ---
                const robotGroup = new THREE.Group();
                scene.add(robotGroup);

                const chassisGeo = new THREE.BoxGeometry(0.7, 0.7, 0.11);
                const chassisMat = new THREE.MeshStandardMaterial({ color: 0x00BFFF, wireframe: false });
                const chassis = new THREE.Mesh(chassisGeo, chassisMat);
                chassis.position.z = 0.055;
                robotGroup.add(chassis);

                const frontGeo = new THREE.BoxGeometry(0.3, 0.1, 0.12);
                const frontMat = new THREE.MeshStandardMaterial({ color: 0xff0000 });
                const frontIndicator = new THREE.Mesh(frontGeo, frontMat);
                frontIndicator.position.set(0.35, 0, 0.06);
                robotGroup.add(frontIndicator);

                // --- Live Turret Pivot ---
                const turretGroup = new THREE.Group();
                // Dynamically injected offsets from Java
                turretGroup.position.set(TURRET_OFFSET_X, TURRET_OFFSET_Y, TURRET_OFFSET_Z);
                robotGroup.add(turretGroup);

                const turretBase = new THREE.Mesh(
                    new THREE.CylinderGeometry(0.2, 0.2, 0.1, 16),
                    new THREE.MeshStandardMaterial({ color: 0x555555 })
                );
                turretBase.rotation.x = Math.PI / 2;
                turretGroup.add(turretBase);

                // --- Hood/Barrel Pivot for Vertical Aiming ---
                const hoodPivot = new THREE.Group();
                hoodPivot.position.set(0, 0, 0.05);
                turretGroup.add(hoodPivot);

                const barrelGeo = new THREE.CylinderGeometry(0.04, 0.04, 0.5);
                const barrelMat = new THREE.MeshStandardMaterial({ color: 0x888888 });
                const barrel = new THREE.Mesh(barrelGeo, barrelMat);
                barrel.rotation.z = -Math.PI / 2;
                barrel.position.set(0.25, 0, 0);
                hoodPivot.add(barrel);

                const targetGeo = new THREE.BoxGeometry(1, 1, 1);
                const targetMat = new THREE.MeshStandardMaterial({ color: 0xff4444, transparent: true, opacity: 0.5 });
                const targetBox = new THREE.Mesh(targetGeo, targetMat);
                scene.add(targetBox);

                const virtTargetMesh = new THREE.Mesh(new THREE.SphereGeometry(0.1), new THREE.MeshBasicMaterial({ color: 0xffa500 }));
                scene.add(virtTargetMesh);

                const lineMat = new THREE.LineDashedMaterial({ color: 0x00ffff, dashSize: 0.2, gapSize: 0.1 });
                let ghostLine = new THREE.Line(new THREE.BufferGeometry(), lineMat);
                scene.add(ghostLine);

                // --- CAMERA & POINTER LOCK STATE ---
                let camYaw = Math.PI;
                let camPitch = 0.4;
                const camRadius = 3.0;
                let isPointerLocked = false;

                container.addEventListener('click', () => {
                    container.requestPointerLock();
                });

                document.addEventListener('pointerlockchange', () => {
                    isPointerLocked = (document.pointerLockElement === container);
                    const overlay = document.getElementById('overlay');
                    if (isPointerLocked) {
                        overlay.style.opacity = '0.3';
                    } else {
                        overlay.style.opacity = '1.0';
                    }
                });

                document.addEventListener('mousemove', (e) => {
                    if (!isPointerLocked) return;

                    const sensitivity = 0.003;
                    camYaw -= e.movementX * sensitivity;
                    camPitch -= e.movementY * sensitivity;
                    camPitch = Math.max(0.05, Math.min(Math.PI / 2 - 0.05, camPitch));
                });

                // --- KINEMATICS & STATE ---
                let state = { x: -4, y: 0, rot: 0, vx: 0, vy: 0, omega: 0 };
                const keys = { w: false, a: false, s: false, d: false, q: false, e: false, space: false };
                let latestSimData = null;
                const activeBalls = [];

                window.addEventListener('keydown', (e) => {
                    const k = e.key.toLowerCase();
                    if(keys.hasOwnProperty(k)) keys[k] = true;
                    if(e.code === 'Space') shootBall();
                });
                window.addEventListener('keyup', (e) => {
                    const k = e.key.toLowerCase();
                    if(keys.hasOwnProperty(k)) keys[k] = false;
                });

                function shootBall() {
                    if (!latestSimData) return;

                    const ballMesh = new THREE.Mesh(
                        new THREE.SphereGeometry(0.12),
                        new THREE.MeshStandardMaterial({ color: 0xffff00 })
                    );
                    scene.add(ballMesh);

                    activeBalls.push({
                        mesh: ballMesh,
                        trajX: latestSimData.xCoords,
                        trajY: latestSimData.yCoords,
                        trajZ: latestSimData.zCoords,
                        frame: 0
                    });
                }

                // --- SERVER COMMUNICATION LOOP ---
                setInterval(async () => {
                    const req = {
                        rX: state.x, rY: state.y, rZ: 0, rRot: state.rot,
                        vX: state.vx, vY: state.vy, vOmega: state.omega,
                        tX: parseFloat(document.getElementById('tX').value),
                        tY: parseFloat(document.getElementById('tY').value),
                        tZ: parseFloat(document.getElementById('tZ').value),
                        calcIters: parseFloat(document.getElementById('calcIters').value),
                        minImpact: parseFloat(document.getElementById('minImpact').value),
                        maxImpact: parseFloat(document.getElementById('maxImpact').value),
                        latencyMs: parseFloat(document.getElementById('latencyMs').value),
                        timeToKeep: parseFloat(document.getElementById('timeToKeep').value),
                        airRes: parseFloat(document.getElementById('airRes').value),
                        linearFilter: parseFloat(document.getElementById('linearFilter').value),
                    };

                    try {
                        const res = await fetch('/simulate', { method: 'POST', headers: {'Content-Type': 'application/json'}, body: JSON.stringify(req) });
                        latestSimData = await res.json();

                        document.getElementById('outHood').innerText = latestSimData.calcHoodAngle.toFixed(2);
                        document.getElementById('outTurret').innerText = latestSimData.calcTurretAngle.toFixed(2);
                        document.getElementById('outRpm').innerText = latestSimData.calcRpm.toFixed(0);
                        document.getElementById('ballVelocity').innerText = latestSimData.ballVelocity.toFixed(0);

                        // Horizontal Turret Aiming
                        turretGroup.rotation.z = latestSimData.calcTurretAngle * (Math.PI / 180);

                        // Vertical Hood Aiming
                        // Calculator sends calcHoodAngle = 90 - launchAngle
                        const launchAngleRad = (90 - latestSimData.calcHoodAngle) * (Math.PI / 180);
                        hoodPivot.rotation.y = -launchAngleRad;

                        virtTargetMesh.position.set(latestSimData.virtX, latestSimData.virtY, latestSimData.virtZ);

                        const pts = [];
                        for(let i=0; i<latestSimData.xCoords.length; i++){
                            pts.push(new THREE.Vector3(latestSimData.xCoords[i], latestSimData.yCoords[i], latestSimData.zCoords[i]));
                        }
                        ghostLine.geometry.setFromPoints(pts);
                        ghostLine.computeLineDistances();
                    } catch (e) { console.error("Sim error", e); }
                }, 50);

                // --- MAIN GAME LOOP ---
                const clock = new THREE.Clock();

                function animate() {
                    requestAnimationFrame(animate);
                    const dt = clock.getDelta();

                    const maxSpd = parseFloat(document.getElementById('maxSpeed').value);
                    const accel = parseFloat(document.getElementById('accel').value);
                    const maxOmg = parseFloat(document.getElementById('maxOmega').value);

                    let inputX = (keys.w ? 1 : 0) - (keys.s ? 1 : 0);
                    let inputY = (keys.a ? 1 : 0) - (keys.d ? 1 : 0);

                    if(inputX !== 0 || inputY !== 0) {
                        const len = Math.sqrt(inputX*inputX + inputY*inputY);
                        inputX /= len; inputY /= len;
                    }

                    const targetVx = inputX * maxSpd;
                    const targetVy = inputY * maxSpd;

                    const moveTowards = (current, target, step) => {
                        if (current < target) return Math.min(current + step, target);
                        if (current > target) return Math.max(current - step, target);
                        return current;
                    };

                    state.vx = moveTowards(state.vx, targetVx, accel * dt);
                    state.vy = moveTowards(state.vy, targetVy, accel * dt);

                    let inputRot = (keys.q ? 1 : 0) - (keys.e ? 1 : 0);
                    const targetOmega = inputRot * maxOmg;
                    state.omega = moveTowards(state.omega, targetOmega, accel * 1.5 * dt);

                    state.x += state.vx * dt;
                    state.y += state.vy * dt;
                    state.rot += state.omega * dt;

                    document.getElementById('rX').value = state.x.toFixed(2);
                    document.getElementById('rXVal').innerText = state.x.toFixed(2);
                    document.getElementById('rY').value = state.y.toFixed(2);
                    document.getElementById('rYVal').innerText = state.y.toFixed(2);
                    document.getElementById('rRot').value = state.rot.toFixed(2);
                    document.getElementById('rRotVal').innerText = state.rot.toFixed(2);

                    robotGroup.position.set(state.x, state.y, 0);
                    robotGroup.rotation.z = state.rot;

                    const lookAtZ = 0.5;
                    const offsetX = Math.cos(camPitch) * Math.cos(camYaw) * camRadius;
                    const offsetY = Math.cos(camPitch) * Math.sin(camYaw) * camRadius;
                    const offsetZ = Math.sin(camPitch) * camRadius;

                    camera.position.x = state.x - offsetX;
                    camera.position.y = state.y - offsetY;
                    camera.position.z = lookAtZ + offsetZ;
                    camera.lookAt(state.x, state.y, lookAtZ);

                    const tX = parseFloat(document.getElementById('tX').value);
                    const tY = parseFloat(document.getElementById('tY').value);
                    const tZ = parseFloat(document.getElementById('tZ').value);
                    const tSize = parseFloat(document.getElementById('tSize').value);

                    targetBox.scale.set(tSize, tSize, tZ);
                    targetBox.position.set(tX, tY, tZ / 2);

                    for (let i = activeBalls.length - 1; i >= 0; i--) {
                        let b = activeBalls[i];
                        if (b.frame < b.trajX.length) {
                            b.mesh.position.set(b.trajX[b.frame], b.trajY[b.frame], b.trajZ[b.frame]);
                            b.frame++;
                        } else {
                            scene.remove(b.mesh);
                            activeBalls.splice(i, 1);
                        }
                    }

                    renderer.render(scene, camera);
                }

                window.addEventListener('resize', () => {
                    camera.aspect = container.clientWidth / container.clientHeight;
                    camera.updateProjectionMatrix();
                    renderer.setSize(container.clientWidth, container.clientHeight);
                });

                animate();
            </script>
        </body>
        </html>
        """;
}
