package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "Auto Near 2026", group = "Concept")
public class AutoNear2026 extends LinearOpMode {
    private MecanumDrive drive;
    private intakeUnit2026 motors;
    private Colored patternDetector;

    private int leftOrRight = 1; // 1 for blue, -1 for red
    private double detectedPattern;

    private Pose2d StartPose;
    private Pose2d shootPos;

    @Override
    public void runOpMode() {
        // connect the hardware map to color discrimination system
        patternDetector = new Colored(hardwareMap);
        motors = new intakeUnit2026(hardwareMap, "launcher", "intake", "triggerServo");

        // Initialize drive and vision system
        StartPose = new Pose2d(
                (6 * Params.HALF_MAT - Params.CHASSIS_HALF_LENGTH),
                (leftOrRight * Params.CHASSIS_HALF_WIDTH), Math.toRadians(180.0)
        );

        // define shoot position, calculate shoot angle based on location x, y
        double shootPosX = 1.8 * Params.HALF_MAT;
        double shootPosY = leftOrRight * Params.HALF_MAT;
        double shootPosTheta;
        shootPosTheta = Math.atan2(6 * Params.HALF_MAT - shootPosY, 6 * Params.HALF_MAT - shootPosX);
        shootPos = new Pose2d(shootPosX, shootPosY, shootPosTheta);

        drive = new MecanumDrive(hardwareMap, StartPose);

        telemetry.addLine("Initialized. Waiting for start...");
        telemetry.update();

        waitForStart();

        if (opModeIsActive()) {
            run_auto();
        }
    }

    private void run_auto() {
        motors.triggerClose();
        detectedPattern = 0;

        // Detect April Tag while Moving backwards to detect position
        // Then turn to shoot position angle.
        Action leg1 = drive.actionBuilder(drive.localizer.getPose())
                .strafeToConstantHeading(new Vector2d(shootPos.position.x, shootPos.position.y))
                .afterTime(0.001, new startLauncherAction())
                .turnTo(shootPos.heading)
                .build();

        // detect April Tag while moving backwards to shoot position.
        Actions.runBlocking(
                new ParallelAction(
                        new limeLightCamera(), // detect April Tag, save the detected pattern in detectedPattern.
                        leg1 // moving action
                )
        );

        // shoot preload artifacts
        shootArtifacts();

        // moving to detect patten row of artifacts to pick up
        Action actIntake1 = drive.actionBuilder(drive.localizer.getPose())
                .strafeToLinearHeading(rowChoose(detectedPattern), Math.toRadians(90.0))
                .build();
        Actions.runBlocking(actIntake1);

        // start intake motor
        motors.startIntake();

        //moving forward to pick up artifacts, checking speed
        Action actIntake2 = drive.actionBuilder(drive.localizer.getPose())
                .strafeToConstantHeading(new Vector2d(drive.localizer.getPose().position.x,3 * Params.HALF_MAT))
                .build();
        Actions.runBlocking(actIntake2);

        // stop intake motor
        motors.stopIntake();


        // Move to the right row based on detected pattern
        Action leg3 = drive.actionBuilder(drive.localizer.getPose())
                .strafeToLinearHeading(rowChoose(detectedPattern), Math.toRadians(90))
                .strafeToConstantHeading(new Vector2d(drive.localizer.getPose().position.x,3 * Params.HALF_MAT))
                .strafeToConstantHeading(new Vector2d(drive.localizer.getPose().position.x, -2.5 * Params.HALF_MAT))
                .strafeToLinearHeading(shootPos.position, shootPos.heading)
                .build();
        Action leg4 = drive.actionBuilder(drive.localizer.getPose())
                .strafeToLinearHeading(new Vector2d(-0.75 * Params.HALF_MAT, 4 * Params.HALF_MAT), Math.toRadians(90))
                .build();

    }

    private void shootArtifacts() {
        int waitTimeForTriggerClose = 300;
        int waitTimeForTriggerOpen = 800;

        // start launcher motor if it has not been launched
        if (motors.getLauncherPower() < 0.1) {
            motors.startLauncher();
            sleep(waitTimeForTriggerOpen + 400); // waiting time for launcher motor ramp up
        }

        motors.triggerOpen(); // shoot first
        sleep(waitTimeForTriggerClose);
        motors.triggerClose(); //close trigger to wait launcher motor speed up after first launching

        motors.startIntake(); // start intake motor to move 3rd artifacts into launcher
        sleep(waitTimeForTriggerOpen);// waiting time for launcher motor ramp up
        motors.triggerOpen(); // shoot second
        sleep(waitTimeForTriggerClose);

        motors.triggerClose();
        sleep(waitTimeForTriggerOpen); // waiting time for launcher motor ramp up
        motors.triggerOpen();  // shoot third
        sleep(waitTimeForTriggerClose);

        motors.triggerClose();
        motors.stopIntake();
        motors.stopLauncher();
    }

    private void sortArtifacts(int pattern) {
        switch (pattern) {
            case 21:
                telemetry.addLine("Sorting: GPP");
                break;
            case 22:
                telemetry.addLine("Sorting: PGP");
                break;
            case 23:
                telemetry.addLine("Sorting: PPG");
                break;
            default:
                telemetry.addLine("Sorting: Unknown");
                break;
        }
        telemetry.update();
    }

    // function that chooses the right row based on detected pattern, returns a Vector2d
    private Vector2d rowChoose(double pattern) {
        if (pattern == 0) {
            telemetry.addLine("No pattern detected");
            telemetry.update();
            pattern = 23; // default to PPG if no pattern is detected
        }
        double rowIndex = pattern - 20;
        return new Vector2d(
                -3 * Params.HALF_MAT + rowIndex * 2 * Params.HALF_MAT,
                leftOrRight * 3 * Params.HALF_MAT
        );
    }

    private class limeLightCamera implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            for (int i = 0; i < 100; i++) { // up to ~0.3s
                detectedPattern = patternDetector.returnId();
                telemetry.addData("Detected Pattern", detectedPattern);
                telemetry.update();
                if ((detectedPattern > 20) && (detectedPattern < 24)) {
                    return true;
                }
                sleep(1);
            }
            return false;
        }
    }

    private class startLauncherAction implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            motors.startLauncher();
            return true;
        }
    }

    // Update profile acceleration for MecanumDrive
    private void updateProfileSpeed(double speed) {
        MecanumDrive.PARAMS.maxWheelVel = speed;
    }

}
