package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "Auto2026", group = "Concept")
public class AutoTest extends LinearOpMode {
    private MecanumDrive drive;
    private Colored patternDetector; // Limelight detector

    private int leftOrRight = 1;
    private Pose2d newStartPose;
    private Vector2d moveToBall;

    private Pose2d shootPos = new Pose2d(Params.HALF_MAT, Params.HALF_MAT, Math.toRadians(45));

    @Override
    public void runOpMode() {
        // Initialize drive and vision system
        patternDetector = new Colored(hardwareMap);
        setRobotLocation();
        setStartPoses(leftOrRight);
        drive = new MecanumDrive(hardwareMap, newStartPose);

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

    private void setStartPoses(int leftRight) {
        newStartPose = new Pose2d(
                (-6 * Params.HALF_MAT + Params.CHASSIS_LENGTH / 2),
                (leftRight * Params.CHASSIS_HALF_WIDTH),
                0.0
        );
    }

//    private void driveToBall(double distance) {
//        moveToBall = new Vector2d(newStartPose.position.x + distance, 0);
//    }

    private double pixelsToHalfmats(double pixels) {
        return Math.sqrt(62320.0 / pixels) / 2;
    }

    private void auto(double detectedPattern) {
        double distanceToShootPos = Math.hypot(
                shootPos.position.x - newStartPose.position.x,
                shootPos.position.y - newStartPose.position.y
        );

        telemetry.addData("Running Auto", true);
        telemetry.addData("Pattern", detectedPattern);
        telemetry.update();

        Actions.runBlocking(
                drive.actionBuilder(newStartPose)
                        .strafeToLinearHeading(shootPos.position, shootPos.heading)
                        .afterDisp(distanceToShootPos, () -> {
                            sortArtifacts((int) detectedPattern);
                            shootArtifacts();
                        })
                        .strafeToLinearHeading(new Vector2d(-3 * Params.HALF_MAT, 3 * Params.HALF_MAT), Math.toRadians(90))
                        .strafeTo(new Vector2d(-3 * Params.HALF_MAT, 3.75 * Params.HALF_MAT))
                        .strafeTo(new Vector2d(-3 * Params.HALF_MAT, 3 * Params.HALF_MAT))
                        .strafeToLinearHeading(shootPos.position, shootPos.heading)
                        .build()
        );
    }

    private void shootArtifacts() {
        telemetry.addLine("Shooting...");
        telemetry.update();
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
