package org.firstinspires.ftc.teamcode.autos;

import com.bylazar.configurables.annotations.Configurable;
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

@Autonomous(name = "Red Nine Ball Close", group = "Competition Autos")
public class redNineBallClose extends OpMode {
    public Follower follower; // Pedro Pathing follower instance
    boolean hasShot = false;
    private Timer pathTimer, opmodeTimer;
    private int pathState = 1; // Current autonomous path state (state machine)
    private redNineBallClose.Paths paths; // Paths defined in the Paths class
    private Turret turret;
    private Shooter shooter;
    private Intake intake;
    private TelemetryDebug telemetryDebug;

    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(144-30.033, 134.767, Math.toRadians(36)));

        paths = new Paths(follower); // Build paths


        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();

        telemetryDebug = new TelemetryDebug(telemetry);
        turret = new Turret(hardwareMap, follower, true, telemetryDebug);
        shooter = new Shooter(hardwareMap, true, telemetryDebug);
        intake = new Intake(hardwareMap);
    }

    @Override
    public void loop() {
        follower.update(); // Update Pedro Pathing
        shooter.update();
        turret.update();
        pathState = autonomousPathUpdate(); // Update autonomous state machine

        telemetry.addData("Turret Position", turret.getTurretPosition());
        telemetry.addData("Distance From Target", shooter.getDistanceFromTarget());
        telemetry.addData("Relative Target Angle", turret.getRelativeTargetHeading());
        telemetry.addData("Pitch Servo Position", shooter.getPitch());
        telemetry.addData("Current RPM", shooter.getFlywheelRPM());
        telemetry.addData("Current Position", shooter.getPosition());
        for (TelemetryDebug.watcher w : telemetryDebug.watchers) {
            telemetry.addData(w.getName(), w.getValue());
        }
    }

    @Override
    public void start() {
        pathTimer.resetTimer();
        opmodeTimer.resetTimer();
        turret.resetEncoder();
    }

    public int autonomousPathUpdate() {
        switch (pathState) {
            case 1: // 1. DO PATH 1
                shooter.accelerate(74);
                follower.followPath(paths.Path1);
                setPathState(2);
                break;

            case 2: // WAIT FOR PATH 1
                shooter.accelerate(74);
                if (!follower.isBusy()) {
                    setPathState(3);
                }
                break;

            case 3: // 2. SHOOT THREE BALLS
                shootThreeBalls(pathTimer, false, 74); // true = long shoot (3.5s)
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
                intake.intake();
                follower.followPath(paths.Path3);
                setPathState(7);
                break;

            case 7: // WAIT FOR PATH 3
                if (!follower.isBusy()) {
                    if (pathTimer.getElapsedTimeSeconds() >= 0.5) {
                        intake.stop();
                        setPathState(8);
                    }
                }
                break;

            case 8: // 5. DO PATH 4
                follower.followPath(paths.Path4);
                setPathState(9);
                break;

            case 9: // WAIT FOR PATH 4
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() >= 0.5) {
                    setPathState(10);
                }
                break;

            case 10: // 6. DO PATH 5
                shooter.accelerate(70);
                follower.followPath(paths.Path5);
                setPathState(105);
                break;

            case 105:
                shooter.accelerate(70);
                if (!follower.isBusy()) {
                    setPathState(11);
                }
                break;

            case 11: // WAIT FOR PATH 5
                if (pathTimer.getElapsedTimeSeconds() > 2.0) {
                    shootThreeBalls(pathTimer, false, 70); // false = quick shoot (1.0s)
                } else {
                    shooter.accelerate(70);
                    turret.aimWithoutCompensation();
                }

                if (hasShot) {
                    setPathState(12);
                }
                break;

            case 12: // DO PATH 6
                follower.followPath(paths.Path6);
                setPathState(13);
                break;

            case 13: // WAIT FOR PATH 6 (Keep Aiming!)
                if (!follower.isBusy()) {
                    setPathState(14);
                }
                break;
            case 14:
                intake.intake(0.8);
                follower.followPath(paths.Path7);
                setPathState(15);
                break;
            case 15:
                if (!follower.isBusy()) {
                    if (pathTimer.getElapsedTimeSeconds() >= 0.5) {
                        intake.stop();
                        setPathState(16);
                    }
                }
                break;
            case 16:
                shooter.accelerate(70);
                follower.followPath(paths.Path8);
                setPathState(17);
                break;
            case 17:
                shooter.accelerate(70);
                if (!follower.isBusy()) {
                    setPathState(18);
                }
                break;
            case 18:
                if (pathTimer.getElapsedTimeSeconds() > 2.0) {
                    shootThreeBalls(pathTimer, false, 70); // false = quick shoot (1.0s)
                } else {
                    shooter.accelerate(70);
                    turret.aimWithoutCompensation();
                }

                if (hasShot) {
                    intake.stop();
                    shooter.stop();
                    turret.idle();
                    setPathState(19);
                }
                break;
            case 19:
                if (!follower.isBusy()){
                    setPathState(20);
                }
                break;
            case 20:
                follower.followPath(paths.Path9);
                setPathState(21);
                break;
            case 21:
                if (!follower.isBusy()) {
                    stop();
                    setPathState(-1);
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
        double duration = longShoot ? 3.5 : 2.5;
        double intakeStartTime = longShoot ? 2.5 : 0.75;

        if (time < duration) {
            // Keep these running for the entire duration
            turret.aimWithoutCompensation();
            shooter.accelerate();

            // Only intake when the flywheel is (presumably) ready
            if (time >= intakeStartTime && shooter.isReady()) {
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

    public void shootThreeBalls(Timer pathTimer, boolean longShoot, double distance) {
        double time = pathTimer.getElapsedTimeSeconds();
        double duration = longShoot ? 3.5 : 4;
        double intakeStartTime = longShoot ? 2.5 : 0.75;

        if (time < duration) {
            // Keep these running for the entire duration
            turret.aimWithoutCompensation();
            shooter.accelerate(distance);

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
        public PathChain Path5;
        public PathChain Path6;
        public PathChain Path7;
        public PathChain Path8;
        public PathChain Path9;
        public PathChain Path10;
        public PathChain Path11;


        public Paths(Follower follower) {
            Path1 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(144-30.033, 134.767),

                                    new Pose(144-61, 91)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(36), Math.toRadians(36))

                    .build();

            Path2 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(144-61, 91),

                                    new Pose(144-57, 88)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(36), Math.toRadians(0))

                    .build();

            Path3 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(144-57, 88),
                                    new Pose(144-24, 88)
                            )
                    ).setConstantHeadingInterpolation(Math.toRadians(0))
                    .setBrakingStrength(1.5)
                    .setBrakingStart(0.8)

                    .build();

            Path4 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(144-24, 88),
                                    new Pose(144-33.4, 84),
                                    new Pose(144-17, 80)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(90))

                    .build();

            Path5 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(144-17, 80),
                                    new Pose(144-61, 91)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(36))

                    .build();

            Path6 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(144-61, 91),

                                    new Pose(144-55, 61.5)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(36), Math.toRadians(0))

                    .build();
            Path7 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(144-55, 61.5),

                                    new Pose(144-17, 61.5)
                            )
                    ).setConstantHeadingInterpolation(Math.toRadians(0))
                    .setBrakingStrength(1.5)
                    .setBrakingStart(0.8)
                    .build();
            Path8 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(144-17, 61.5),
                                    new Pose(144-37, 62),
                                    new Pose(144-61, 91)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(36))
                    .build();
            Path9 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(144-61, 91),
                                    new Pose(144-30, 80)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(36), Math.toRadians(90))
                    .build();
        }
    }

    @Override
    public void stop() {
        Datavault.finalPose = follower.getPose();
        Datavault.turretPosition = turret.getTurretPosition();
        cum();
    }

    public void cum () {
        System.out.println("FUCCCCCCCKKKKKKKKKKKKKKKK IMMMMMMMMMMMMMMMM......");
    }
}

