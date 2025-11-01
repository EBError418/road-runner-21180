package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "Auto Near 2026", group = "Concept")
public class AutoNear2026 extends LinearOpMode {
    private MecanumDrive drive;
    private int leftOrRight = 1; // 1 for blue, -1 for red
    public Pose2d StartPose;
    public Pose2d shootPos = new Pose2d(-Params.HALF_MAT, leftOrRight * Params.HALF_MAT, Math.toRadians(45));

    intakeUnit2026 motors;
    Colored patternDetector = new Colored(hardwareMap);
    // Try a few times to get a valid color reading
//    double detectedColor = 0;
//            for (int i = 0; i < 30 && opModeIsActive(); i++) { // up to ~0.3s
//        detectedColor = patternDetector.returnId();
//        telemetry.addData("Detected Color", detectedColor);
//        telemetry.update();
//        if (detectedColor != 0) break;
//        sleep(10);
//    }
    public double detectedPattern;

    @Override
    public void runOpMode() {
        // Initialize drive and vision system
        StartPose = new Pose2d(
                (-6 * Params.HALF_MAT + Params.CHASSIS_LENGTH / 2),
                (leftOrRight * Params.CHASSIS_HALF_WIDTH),
                180.0
        );
        drive = new MecanumDrive(hardwareMap, StartPose);
        motors = new intakeUnit2026(hardwareMap, "launcher", "intake", "triggerServo");
        telemetry.addLine("Initialized. Waiting for start...");
        telemetry.update();
        waitForStart();
        if (opModeIsActive()) {
            auto();
        }
    }

    private void auto() {
        motors.startIntake();
        double distanceToShootPos = Math.hypot(
                shootPos.position.x - StartPose.position.x,
                shootPos.position.y - StartPose.position.y
        );

        Actions.runBlocking(
                drive.actionBuilder(StartPose)
                        // Move to shooting position
                        .strafeToLinearHeading(shootPos.position, shootPos.heading)
                        .afterDisp(distanceToShootPos, () -> {
                            sortArtifacts((int) detectedPattern);
                            shootArtifacts();
                        })
                        // Move to first artifact position
                        .strafeToLinearHeading(new Vector2d(-3 * Params.HALF_MAT, 3 * Params.HALF_MAT), Math.toRadians(90))
                        // Move forward to collect artifacts
                        .strafeTo(new Vector2d(-3 * Params.HALF_MAT, 3.75 * Params.HALF_MAT))
                        // Move slightly back
                        .strafeTo(new Vector2d(-3 * Params.HALF_MAT, 3 * Params.HALF_MAT))
                        // Return to shooting position
                        .strafeToLinearHeading(shootPos.position, shootPos.heading)
                        .afterDisp(distanceToShootPos, () -> {
                            sortArtifacts((int) detectedPattern);
                            shootArtifacts();
                        })
                        // Move to second artifact position
                        .strafeToLinearHeading(new Vector2d(-1 * Params.HALF_MAT, 2 * Params.HALF_MAT), Math.toRadians(90))
                        // Move forward to collect artifacts
                        .strafeTo(new Vector2d(-1 * Params.HALF_MAT, 3.75 * Params.HALF_MAT))
                        // Move slightly back
                        .strafeTo(new Vector2d(-1 * Params.HALF_MAT, 3 * Params.HALF_MAT))
                        // Return to shooting position
                        .strafeToLinearHeading(shootPos.position, shootPos.heading)
                        .afterDisp(distanceToShootPos, () -> {
                            sortArtifacts((int) detectedPattern);
                            shootArtifacts();
                        })
                        // Empty artifacts
                        .strafeToLinearHeading(new Vector2d(-0.75 * Params.HALF_MAT, 4 * Params.HALF_MAT), Math.toRadians(90))
                        .build()
        );
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
}