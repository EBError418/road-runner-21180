package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.acmerobotics.roadrunner.*;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import java.lang.Math;

@Autonomous(name = "Auto Near 2026", group = "Concept")
public class AutoNear2026 extends LinearOpMode {

    private MecanumDrive drive;
    private intakeUnit2026 motors;
    private Colored patternDetector;

    private int alliance = 1; // 1 = blue, -1 = red
    private double detectedPattern = 21; // default pattern
    private Vector2d shootPos;
    private double shootHeading;

    // Timing constants for launcher
    private static final int TRIGGER_CLOSE_MS = 300;
    private static final int TRIGGER_OPEN_MS = 950;
    private static final int LAUNCHER_SPINUP_MS = 1450;

    @Override
    public void runOpMode() {
        patternDetector = new Colored(hardwareMap);
        motors = new intakeUnit2026(hardwareMap, "launcher", "intake", "triggerServo");

        Pose2d startPose = new Pose2d(
                6 * Params.HALF_MAT - Params.CHASSIS_HALF_LENGTH,
                alliance * Params.CHASSIS_HALF_WIDTH,
                Math.toRadians(180)
        );

        double shootX = Params.HALF_MAT;
        double shootY = alliance * Params.HALF_MAT;
        shootHeading = Math.toRadians(180) + Math.atan2(6 * Params.HALF_MAT - shootY, 6 * Params.HALF_MAT - shootX);
        shootPos = new Vector2d(shootX, shootY);

        drive = new MecanumDrive(hardwareMap, startPose);

        waitForStart();
        if (opModeIsActive()) runAuto();
    }

    private void runAuto() {
        motors.triggerClose();
        updateDriveAcceleration(false);

        // Move backward and detect AprilTag while approaching shooting position
        Action moveAndDetect = drive.actionBuilder(drive.localizer.getPose())
                .afterDisp(3 * Params.HALF_MAT, new DetectPatternAction())
                .afterDisp(3.1 * Params.HALF_MAT, packet -> {
                    Logging.log("Starting launcher...");
                    motors.startLauncher();
                    return false;
                })
                .strafeToConstantHeading(shootPos)
                .turnTo(shootHeading)
                .build();

        Actions.runBlocking(moveAndDetect);

        shootArtifacts();

        VelConstraint slowConstraint = (pose, path, disp) -> 20.0;

        for (int i = 0; i < 3; i++) {
            int patternIndex = (int) ((detectedPattern + i) % 3);
            Vector2d pickupPos = getPickupPosition(patternIndex);
            Vector2d endPos = new Vector2d(
                    pickupPos.x,
                    pickupPos.y + alliance * (2 + ((pickupPos.x > 0) ? 0 : 1)) * Params.HALF_MAT
            );

            Action collectAndShoot = drive.actionBuilder(drive.localizer.getPose())
                    .strafeToLinearHeading(pickupPos, Math.toRadians(90))
                    .afterTime(0.001, packet -> {
                        Logging.log("Starting intake...");
                        motors.startIntake();
                        return false;
                    })
                    .strafeToConstantHeading(endPos, slowConstraint)
                    .afterTime(0.001, packet -> {
                        Logging.log("Starting launcher...");
                        motors.startLauncher();
                        return false;
                    })
                    .strafeToLinearHeading(shootPos, shootHeading)
                    .build();

            Actions.runBlocking(collectAndShoot);
            shootArtifacts();
            motors.stopIntake();
        }
    }

    /** Handles launching three artifacts with precise timing */
    private void shootArtifacts() {
        Logging.log("Shooting artifacts...");

        if (motors.getLauncherPower() < 0.1) {
            Logging.log("Launcher off — starting it up.");
            motors.startLauncher();
            sleep(LAUNCHER_SPINUP_MS);
        }

        for (int i = 0; i < 3; i++) {
            motors.triggerOpen();
            sleep(TRIGGER_CLOSE_MS);
            motors.triggerClose();

            if (i < 2) {
                motors.startIntake();
                sleep(TRIGGER_OPEN_MS);
            }
        }

        motors.triggerClose();
        motors.stopLauncher();
    }

    /** Picks pickup row based on detected pattern */
    private Vector2d getPickupPosition(int row) {
        return new Vector2d(
                (1 - (row * 2)) * Params.HALF_MAT,
                alliance * (2 * Params.HALF_MAT + Params.CHASSIS_HALF_LENGTH)
        );
    }

    /** Dynamically tunes acceleration/velocity */
    private void updateDriveAcceleration(boolean slowMode) {
        if (slowMode) {
            MecanumDrive.PARAMS.maxWheelVel = 65;
            MecanumDrive.PARAMS.maxProfileAccel = 50;
            MecanumDrive.PARAMS.minProfileAccel = -40;
        } else {
            MecanumDrive.PARAMS.maxWheelVel = 10000;
            MecanumDrive.PARAMS.maxProfileAccel = 10000;
            MecanumDrive.PARAMS.minProfileAccel = -10000;
        }
    }

    /** Action: Detects pattern for a brief duration */
    private class DetectPatternAction implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            for (int i = 0; i < 30; i++) {
                detectedPattern = patternDetector.returnId();
                Logging.log("Pattern detected: %f", detectedPattern);
                telemetry.addData("Limelight", detectedPattern);
                telemetry.update();

                if (detectedPattern > 20 && detectedPattern < 24) break;
                sleep(1);
            }
            return false;
        }
    }
}
