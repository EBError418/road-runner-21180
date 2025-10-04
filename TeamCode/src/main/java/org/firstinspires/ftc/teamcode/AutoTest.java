package org.firstinspires.ftc.teamcode;

import java.io.*;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name="AutoTest", group="Concept")
public class AutoTest extends LinearOpMode {
    private MecanumDrive drive;
    public int leftOrRight = 1;
    Pose2d newStartPose;
    Vector2d moveForward = new Vector2d(2 * Params.HALF_MAT, 0);
    Vector2d moveToBall;

    Vector2d shootPos = new Vector2d(Params.HALF_MAT, Params.HALF_MAT);

    public void setRobotLocation() {
        leftOrRight = 1;  // side should be either -1 for left or 1 for right
    }

    private void setStartPoses(int leftRight) {
        // road runner variables
        newStartPose = new Pose2d((-6 * Params.HALF_MAT + Params.CHASSIS_LENGTH / 2),(-leftRight * Params.CHASSIS_HALF_WIDTH),0.0);
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

        driveToBall(pixelsToHalfmats(15590)); // replace 100 with area of bounding box from LL

        drive = new MecanumDrive(hardwareMap, newStartPose);
        waitForStart();   // <-- REQUIRED
        if (opModeIsActive()) {
            auto();
        }
    }

    private void auto() {
//        Actions.runBlocking(
//                drive.actionBuilder(newStartPose)
//                        .strafeTo(moveToBall)
//                        .build()
//        );

        Actions.runBlocking(
                drive.actionBuilder(newStartPose)
                        .strafeToLinearHeading(shootPos, Math.toRadians(45))
                        .build()
        );

        // Shoot balls code here


        // End shoot balls code

        Actions.runBlocking(
                drive.actionBuilder(newStartPose)
                        .strafeToLinearHeading(new Vector2d(-4 * Params.HALF_MAT, 2 * Params.HALF_MAT), Math.toRadians(90))
                        .build()
        );

        Actions.runBlocking(
                drive.actionBuilder(newStartPose)
                        .strafeTo(new Vector2d(0, 2 * Params.HALF_MAT))
                        .build()
        );

        Actions.runBlocking(
                drive.actionBuilder(newStartPose)
                        .strafeToLinearHeading(shootPos, Math.toRadians(45))
                        .build()
        );

        // Shoot balls code here


        // End shoot balls code
    }
}
