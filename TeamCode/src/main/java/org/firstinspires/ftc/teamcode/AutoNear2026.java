package org.firstinspires.ftc.teamcode;

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
                (leftOrRight * Params.CHASSIS_HALF_WIDTH), Math.toRadians(180)
        );
        shootPos = new Pose2d(2*Params.HALF_MAT, leftOrRight * Params.HALF_MAT, Math.toRadians(225));

        drive = new MecanumDrive(hardwareMap, StartPose);

        telemetry.addLine("Initialized. Waiting for start...");
        telemetry.update();

        waitForStart();

        if (opModeIsActive()) {
            run_auto();
        }
    }

    private void run_auto() {
//        motors.startIntake();
        motors.triggerClose();
        // Moving backwards to detect position
        Action leg1 = drive.actionBuilder(drive.localizer.getPose())
                .strafeToConstantHeading(new Vector2d(shootPos.position.x, drive.localizer.getPose().position.y)).build();
        // After detect, go to shoot position
        Action leg2 = drive.actionBuilder(new Pose2d(shootPos.position.x, drive.localizer.getPose().position.y, drive.localizer.getPose().heading.real))
                .strafeToLinearHeading(shootPos.position, shootPos.heading).build();
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
        Actions.runBlocking(leg1);
        Logging.log("Running leg1 complete, x pos: %.2f, y pos: %.2f", drive.localizer.getPose().position.x, drive.localizer.getPose().position.y);
        detectedPattern = 0;
        for (int i = 0; i < 30; i++) { // up to ~0.3s
            detectedPattern = patternDetector.returnId();
            telemetry.addData("Detected Pattern", detectedPattern);
            telemetry.update();
            sleep(1);
            if (detectedPattern != 0) {
                Actions.runBlocking(leg2);
                shootArtifacts();
                Actions.runBlocking(leg3);
                shootArtifacts();
                Actions.runBlocking(leg4);
            } else if (i == 29) {
                Actions.runBlocking(leg2);
                shootArtifacts();
                Actions.runBlocking(leg3);
                shootArtifacts();
                Actions.runBlocking(leg4);
            }
        }
    }

    private void shootArtifacts() {
        telemetry.addLine("Shooting...");
        telemetry.update();
        motors.startLauncher();
        sleep(3000);
        motors.triggerOpen(); // shoot first
        sleep(300);
        motors.triggerClose();
        motors.startIntake();
        sleep(1000);
        motors.triggerOpen(); // shoot second
        sleep(300);
        motors.stopIntake();
        motors.triggerClose();
        sleep(1000);
        motors.triggerOpen();  // shoot third
        sleep(1000);
        motors.stopLauncher();
        motors.triggerClose();
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
        double rowIndex = pattern - 20;
        return new Vector2d(
                -3 * Params.HALF_MAT + rowIndex * 2 * Params.HALF_MAT,
                leftOrRight * 3 * Params.HALF_MAT
        );
    }
}