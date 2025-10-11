package org.firstinspires.ftc.teamcode;

import java.io.*;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name="Auto2026", group="Concept")
public class AutoTest extends LinearOpMode {
    private MecanumDrive drive;
    public int leftOrRight = 1;
    Pose2d newStartPose;
    Vector2d moveToBall;

    Pose2d shootPos = new Pose2d(Params.HALF_MAT, Params.HALF_MAT, Math.toRadians(45));

    public void setRobotLocation() {
        leftOrRight = 1;  // side should be either -1 for left or 1 for right
    }

    private void setStartPoses(int leftRight) {
        // road runner variables
        newStartPose = new Pose2d((-6 * Params.HALF_MAT + Params.CHASSIS_LENGTH / 2),(leftRight * Params.CHASSIS_HALF_WIDTH),0.0);
    }

    public void driveToBall(double distance) { // distance should be in half mats
        moveToBall = new Vector2d(newStartPose.position.x + distance, 0);
    }

    public double pixelsToHalfmats(double pixels) { // pixels = area of bounding box
        return Math.sqrt(62320.0 / pixels) / 2; // returns half mats
    }

    @Override
    public void runOpMode() {
        setRobotLocation();
        setStartPoses(leftOrRight);

        driveToBall(pixelsToHalfmats(15590)); // replace with area of bounding box from LL

        drive = new MecanumDrive(hardwareMap, newStartPose);
        waitForStart();   // <-- REQUIRED
        if (opModeIsActive()) {
            auto();
        }
    }

    private void auto() {
        double distanceToShootPos = Math.hypot(
                shootPos.position.x - newStartPose.position.x,
                shootPos.position.y - newStartPose.position.y
        );

        Actions.runBlocking(
                drive.actionBuilder(newStartPose)
                        // Move to shooting position
                        .strafeToLinearHeading(shootPos.position, shootPos.heading)
                        // Shoot
                        .afterDisp(distanceToShootPos, () -> {
                            sortArtifacts(22);

                            shootArtifacts();
                        })
                        // First move to artifacts
                        .strafeToLinearHeading(new Vector2d(-3 * Params.HALF_MAT,3 * Params.HALF_MAT),Math.toRadians(90))
                        // Move forward to collect artifacts
                        .strafeTo(new Vector2d(-3 * Params.HALF_MAT, 3.75 * Params.HALF_MAT))
                        // Move a little back
                        .strafeTo(new Vector2d(-3 * Params.HALF_MAT, 3 * Params.HALF_MAT))
                        // Move to shooting position
                        .strafeToLinearHeading(shootPos.position, shootPos.heading)
                        // Shoot
                        .afterDisp(distanceToShootPos, () -> {
                            sortArtifacts(23);

                            shootArtifacts();
                        })
                        // Second move to artifacts
                        .strafeToLinearHeading(new Vector2d(-1 * Params.HALF_MAT, 3 * Params.HALF_MAT), Math.toRadians(90))
                        // Move forward to collect artifacts
                        .strafeTo(new Vector2d(-1 * Params.HALF_MAT, 3.75 * Params.HALF_MAT))
                        // Move a little back
                        .strafeTo(new Vector2d(-1 * Params.HALF_MAT, 3 * Params.HALF_MAT))
                        // Move to shooting position
                        .strafeToLinearHeading(shootPos.position, shootPos.heading)
                        // Shoot
                        .afterDisp(distanceToShootPos, () -> {
                            sortArtifacts(21);

                            shootArtifacts();
                        })
                        // Empty artifacts
                        .strafeToLinearHeading(new Vector2d(0, 4 * Params.HALF_MAT), Math.toRadians(90))

                        .build()
        );
    }

    private void shootArtifacts() {
        // Shoot balls code here
    }

    private void sortArtifacts(int pattern) {
        // Sort balls code here

        switch (pattern) {
            case 21:
                // Pattern 1 code here
                break;
            case 22:
                // Pattern 2 code here
                break;
            case 23:
                // Pattern 3 code here
                break;
            default:
                // Default case code here
                break;
        }
    }
}
