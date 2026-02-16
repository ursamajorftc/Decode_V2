package org.firstinspires.ftc.teamcode.autos;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.utilities.Datavault;
import org.firstinspires.ftc.teamcode.utilities.TelemetryDebug;
import org.firstinspires.ftc.teamcode.subsystems.Turret;

@Autonomous(name = "Blue Twelve Ball Close", group = "Competition Autos")
public class blueTwelveBallClose extends OpMode {
    public Follower follower; // Pedro Pathing follower instance
    boolean hasShot = false;
    private TelemetryManager panelsTelemetry; // Panels Telemetry instance
    private Timer pathTimer, opmodeTimer;
    private int pathState = 1; // Current autonomous path state (state machine)
    private blueTwelveBallClose.Paths paths; // Paths defined in the Paths class
    private Turret turret;
    private Shooter shooter;
    private Intake intake;
    private TelemetryDebug telemetryDebug;

    @Override
    public void init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(21.738, 122.952, Math.toRadians(144)));

        paths = new Paths(follower); // Build paths

        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.update(telemetry);

        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();

        telemetryDebug = new TelemetryDebug(telemetry);
        turret = new Turret(hardwareMap, follower, false, telemetryDebug);
        shooter = new Shooter(hardwareMap, false, telemetryDebug);
        intake = new Intake(hardwareMap);
    }

    @Override
    public void loop() {
        follower.update(); // Update Pedro Pathing
        shooter.update();
        turret.update();
        pathState = autonomousPathUpdate(); // Update autonomous state machine

        // Log values to Panels and Driver Station
        panelsTelemetry.debug("Path State", pathState);
        panelsTelemetry.debug("X", follower.getPose().getX());
        panelsTelemetry.debug("Y", follower.getPose().getY());
        panelsTelemetry.debug("Heading", follower.getPose().getHeading());
        panelsTelemetry.update(telemetry);
    }

    @Override
    public void start() {
        pathTimer.resetTimer();
        opmodeTimer.resetTimer();
    }

    public int autonomousPathUpdate() {
        switch (pathState) {
            case 1: // 1. DO PATH 1
                follower.followPath(paths.Path1);
                setPathState(2);
                break;

            case 2: // WAIT FOR PATH 1
                if (!follower.isBusy()) {
                    setPathState(3);
                }
                break;

            case 3: // 2. SHOOT THREE BALLS
                shootThreeBalls(pathTimer, true); // true = long shoot (3.5s)
                if (hasShot) {
                    setPathState(4);
                }
                break;

            case 4: // 3. DO PATH 2
                follower.followPath(paths.Path2);
                setPathState(5);
                break;

            case 5: // WAIT FOR PATH 2
                if (!follower.isBusy()) {
                    setPathState(6);
                }
                break;

            case 6: // 4. INTAKE WHILE DOING PATH 3
                intake.intake(0.4);
                follower.followPath(paths.Path3);
                setPathState(7);
                break;

            case 7: // WAIT FOR PATH 3
                if (!follower.isBusy()) {
                    intake.stop();
                    setPathState(8);
                }
                break;

            case 8: // 5. DO PATH 4
                follower.followPath(paths.Path4);
                setPathState(9);
                break;

            case 9: // WAIT FOR PATH 4
                if (!follower.isBusy()) {
                    setPathState(10);
                }
                break;

            case 10: // 6. DO PATH 5
                follower.followPath(paths.Path5);
                setPathState(11);
                break;

            case 11: // WAIT FOR PATH 5
                if (!follower.isBusy()) {
                    setPathState(12);
                }
                break;

            case 12: // 7. ACCELERATE AND AIM WHILE DOING PATH 6
                shooter.accelerate();
                turret.aim();
                follower.followPath(paths.Path6);
                setPathState(13);
                break;

            case 13: // WAIT FOR PATH 6 (Keep Aiming!)
                shooter.accelerate();
                turret.aim();
                if (!follower.isBusy()) {
                    setPathState(14);
                }
                break;

            case 14: // 8. SHOOT THREE BALLS
                shootThreeBalls(pathTimer, false); // false = quick shoot (1.0s)
                if (hasShot) {
                    setPathState(-1); // End of Auto
                }
                break;
        }
        return pathState;
    }

    public void setPathState(int pathState) {
        this.pathState = pathState;
        pathTimer.resetTimer();
        hasShot = false;
    }

    public void shootThreeBalls(Timer pathTimer, boolean longShoot) {
        double time = pathTimer.getElapsedTimeSeconds();
        double duration = longShoot ? 3.5 : 2.0;
        double intakeStartTime = longShoot ? 2.5 : 0.0;

        if (time < duration) {
            // Keep these running for the entire duration
            turret.aim();
            shooter.accelerate();

            // Only intake when the flywheel is (presumably) ready
            if (time >= intakeStartTime) {
                intake.intake();
            } else {
                intake.stop();
            }
        } else {
            // Action is FINALLY done
            intake.stop();
            shooter.stop();
            hasShot = true;
        }

    }
    @Configurable
    public static class Paths {
        public PathChain Path1;
        public PathChain Path2;
        public PathChain Path3;
        public PathChain Path4;
        public PathChain Path13;
        public PathChain Path12;
        public PathChain Path5;
        public PathChain Path6;
        public PathChain Path7;
        public PathChain Path8;
        public PathChain Path9;
        public PathChain Path10;
        public PathChain Path11;

        public Paths(Follower follower) {
            Path1 = follower.pathBuilder().addPath(new BezierLine(new Pose(21.738, 122.952),

                            new Pose(54.759, 98.814))).setLinearHeadingInterpolation(Math.toRadians(144), Math.toRadians(135))

                    .build();

            Path2 = follower.pathBuilder().addPath(new BezierLine(new Pose(54.759, 98.814),

                            new Pose(42.538, 83.517))).setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(180))

                    .build();

            Path3 = follower.pathBuilder().addPath(new BezierLine(new Pose(42.538, 83.517),

                            new Pose(16.641, 84.234))).setConstantHeadingInterpolation(Math.toRadians(180))

                    .build();

            Path4 = follower.pathBuilder().addPath(new BezierLine(new Pose(16.641, 84.234),

                            new Pose(30.807, 84.179))).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(90))

                    .build();

            Path13 = follower.pathBuilder().addPath(new BezierLine(new Pose(30.807, 84.179),

                            new Pose(14.152, 75.724))).setConstantHeadingInterpolation(Math.toRadians(90))

                    .build();

            Path12 = follower.pathBuilder().addPath(new BezierLine(new Pose(14.152, 75.724),

                            new Pose(61.090, 81.490))).setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(145))

                    .build();

            Path5 = follower.pathBuilder().addPath(new BezierLine(new Pose(61.090, 81.490),

                            new Pose(45.938, 59.683))).setLinearHeadingInterpolation(Math.toRadians(145), Math.toRadians(180))

                    .build();

            Path6 = follower.pathBuilder().addPath(new BezierLine(new Pose(45.938, 59.683),

                            new Pose(9.745, 60.110))).setTangentHeadingInterpolation()

                    .build();

            Path7 = follower.pathBuilder().addPath(new BezierCurve(new Pose(9.745, 60.110), new Pose(39.652, 56.893), new Pose(60.083, 80.462))).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(145))

                    .build();

            Path8 = follower.pathBuilder().addPath(new BezierLine(new Pose(60.083, 80.462),

                            new Pose(48.166, 35.255))).setLinearHeadingInterpolation(Math.toRadians(145), Math.toRadians(180))

                    .build();

            Path9 = follower.pathBuilder().addPath(new BezierLine(new Pose(48.166, 35.255),

                            new Pose(9.034, 36.428))).setConstantHeadingInterpolation(Math.toRadians(180))

                    .build();

            Path10 = follower.pathBuilder().addPath(new BezierLine(new Pose(9.034, 36.428),

                            new Pose(60.324, 81.297))).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(120))

                    .build();

            Path11 = follower.pathBuilder().addPath(new BezierLine(new Pose(60.324, 81.297),

                            new Pose(18.276, 69.593))).setLinearHeadingInterpolation(Math.toRadians(120), Math.toRadians(90))

                    .build();
        }
    }
    @Override
    public void stop () {
        Datavault.finalAutoPose = follower.getPose();
        Datavault.turretPosition = turret.getTurretPosition();
    }
}

