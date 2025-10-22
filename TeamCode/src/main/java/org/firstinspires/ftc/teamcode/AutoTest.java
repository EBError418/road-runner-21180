package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "Auto2026", group = "Concept")
public class AutoTest extends LinearOpMode {
    private MecanumDrive drive;

    private int leftOrRight = 1;
    public Pose2d newStartPose;

    // Limelight detector
    Colored patternDetector = new Colored(hardwareMap);
    public Pose2d shootPos = new Pose2d(Params.HALF_MAT, Params.HALF_MAT, Math.toRadians(45));

    intakeUnit2026 motors;

    @Override
    public void runOpMode() {
        // Initialize drive and vision system
        setRobotLocation();
        setStartPoses(leftOrRight);
        drive = new MecanumDrive(hardwareMap, newStartPose);
        motors = new intakeUnit2026(hardwareMap, "launcher", "intake", "trigger");

        telemetry.addLine("Initialized. Waiting for start...");
        telemetry.update();

        waitForStart();

        if (opModeIsActive()) {
            // Try a few times to get a valid color reading
            double detectedColor = 0;
            for (int i = 0; i < 30 && opModeIsActive(); i++) { // up to ~0.3s
                detectedColor = patternDetector.returnId();
                telemetry.addData("Detected Color", detectedColor);
                telemetry.update();
                if (detectedColor != 0) break;
                sleep(10);
            }

            auto(detectedColor);
        }
    }

    private void setRobotLocation() {
        leftOrRight = 1;
    }

    // ↓ Do we need this anymore?
    private double horizontalDistanceFromGoal(double x, double y) {
        return Math.sqrt((Math.pow((x - 1), 2) + Math.pow((y - 1), 2)));
    }

    // ↓ ai code 💔
    // create a function that allows for calibration of shoot position from an array in the format of [goal code, x, y, w, h]
    private void calibrateShootPosition(double[] detection) {
        if (detection == null || detection.length < 5) return;

        double x = detection[1];
        double y = detection[2];
        double w = detection[3];
        double h = detection[4];

//        double shootX = newStartPose.position.x + distanceInHalfMats * Math.cos(angleToGoal);
//        double shootY = newStartPose.position.y + distanceInHalfMats * Math.sin(angleToGoal);
//
//        shootPos = new Pose2d(shootX, shootY, angleToGoal);
    }

    private void setStartPoses(int leftRight) {
        newStartPose = new Pose2d(
                (-6 * Params.HALF_MAT + Params.CHASSIS_LENGTH / 2),
                (leftRight * Params.CHASSIS_HALF_WIDTH),
                0.0
        );
    }

    // ↓ Fix
    private double pixelsToHalfmats(double x, double y) {
        double area = Math.pow(((x +y )/2),2);
        telemetry.addData("Area: ", Double.toString(area));
        return 0;
    }

    private void auto(double detectedPattern) {
        motors.startIntake();

        double distanceToShootPos = Math.hypot(
                shootPos.position.x - newStartPose.position.x,
                shootPos.position.y - newStartPose.position.y
        );

        telemetry.addData("Running Auto", true);
        telemetry.addData("Pattern", detectedPattern);
        telemetry.update();

        Actions.runBlocking(
                drive.actionBuilder(newStartPose)
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
        double[] position = patternDetector.returnPosition();
        calibrateShootPosition(position);
        telemetry.update();
        motors.startLauncher();
        sleep(4000);
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
}