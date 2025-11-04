package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;
import androidx.appcompat.widget.VectorEnabledTintResources;

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
    private Vector2d shootPos;
    private double shootHeading;

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
        double shootPosX = 1 * Params.HALF_MAT;
        double shootPosY = leftOrRight * Params.HALF_MAT;
        shootHeading = Math.toRadians(180) + Math.atan2(6 * Params.HALF_MAT - shootPosY, 6 * Params.HALF_MAT - shootPosX);
        shootPos = new Vector2d(shootPosX, shootPosY);

        drive = new MecanumDrive(hardwareMap, StartPose);
        while (!isStarted()) {
            sleep(10);
            telemetry.addLine("Initialized. Waiting for start...");
            detectAprilTeg();
            telemetry.addData("line light", "Detected Pattern = %f", detectedPattern);

            telemetry.update();
        }
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
                .afterDisp(3 * Params.HALF_MAT, new limeLightCamera()) // start detecting after moving 20 inches
                .afterDisp(3.1 * Params.HALF_MAT, new startLauncherAction())
                .strafeToConstantHeading(shootPos)
                //.afterTime(0.001, new startLauncherAction())
                .turnTo(shootHeading)
                .build();

        // detect April Tag while moving backwards to shoot position.
        Actions.runBlocking(leg1);

        // shoot preload artifacts
        shootArtifacts();

        // moving to detect patten row of artifacts to pick up
        for (int pickupIndex = 0; pickupIndex < 3; pickupIndex++ ) {
            Vector2d pickupPos;
            Vector2d pickupEndPos;

            pickupPos = rowChoose(detectedPattern);
            if (pickupIndex >0) // second and third pickup
            {
                // update pickupPos.x

            }

            pickupEndPos = new Vector2d(pickupPos.x, pickupPos.y + leftOrRight * (2 + ((pickupPos.x>0)? 0 : 1))* Params.HALF_MAT );

            Action actIntake1 = drive.actionBuilder(drive.localizer.getPose())
                    .strafeToLinearHeading(pickupPos, Math.toRadians(90.0))
                    .afterTime(0.001, new startIntakeAction())
                    .strafeToConstantHeading(pickupEndPos) // picking up artifacts
                    .afterTime(0.001, new startLauncherAction())
                    .strafeToLinearHeading(shootPos, shootHeading)
                    .build();
            Actions.runBlocking(actIntake1);

            // shoot first picked up artifacts
            shootArtifacts();

            // stop intake motor
            motors.stopIntake();
        }

        // Move to the right row based on detected pattern
        Action leg3 = drive.actionBuilder(drive.localizer.getPose())
                .strafeToLinearHeading(rowChoose(detectedPattern), Math.toRadians(90))
                .strafeToConstantHeading(new Vector2d(drive.localizer.getPose().position.x,3 * Params.HALF_MAT))
                .strafeToConstantHeading(new Vector2d(drive.localizer.getPose().position.x, -2.5 * Params.HALF_MAT))
                .strafeToLinearHeading(shootPos, shootHeading)
                .build();
        Action leg4 = drive.actionBuilder(drive.localizer.getPose())
                .strafeToLinearHeading(new Vector2d(-0.75 * Params.HALF_MAT, 4 * Params.HALF_MAT), Math.toRadians(90))
                .build();

    }

    private void shootArtifacts() {
        int waitTimeForTriggerClose = 300;
        int waitTimeForTriggerOpen = 1200;
        Logging.log("start shooting.");
        // start launcher motor if it has not been launched
        if (motors.getLauncherPower() < 0.1) {
            Logging.log("start launcher motor since it is stopped.");
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
        //motors.stopIntake();
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
            pattern = 21; // default to PPG if no pattern is detected
        }
        double rowIndex = 22 - pattern;
        return new Vector2d(
                (rowIndex * 2 - 1) * Params.HALF_MAT,
                leftOrRight * (2 * Params.HALF_MAT + Params.CHASSIS_HALF_LENGTH)
        );
    }

    private class limeLightCamera implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            for (int i = 0; i < 30; i++) { // up to ~0.3s
                detectedPattern = patternDetector.returnId();
                Logging.log("pattern  = %f", detectedPattern);
                telemetry.addData("line light","Detected Pattern = %f", detectedPattern);
                telemetry.update();
                if ((detectedPattern > 20) && (detectedPattern < 24)) {
                    return false;
                }
                sleep(1);
            }
            return false;
        }
    }

    private class startLauncherAction implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            Logging.log("start launcher motor.");
            motors.startLauncher();
            return false;
        }
    }

    private class startIntakeAction implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            Logging.log("start intake motor.");
            motors.startIntake();
            return false;
        }
    }

    // Update profile acceleration for MecanumDrive
    private void updateProfileSpeed(double speed) {
        MecanumDrive.PARAMS.maxWheelVel = speed;
    }

    public boolean detectAprilTeg() {
        for (int i = 0; i < 30; i++) { // up to ~0.3s
            detectedPattern = patternDetector.returnId();
            Logging.log("pattern  = %f", detectedPattern);
            if ((detectedPattern > 20) && (detectedPattern < 24)) {
                return false;
            }
            sleep(1);
        }
        return false;
    }

}
