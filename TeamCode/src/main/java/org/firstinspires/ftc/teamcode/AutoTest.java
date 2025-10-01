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

    public void setRobotLocation(int side) {
        leftOrRight = side;  // side should be either -1 for left or 1 for right
    }

    private void setStartPoses(int leftRight) {
        // road runner variables
        newStartPose = new Pose2d((-6 * Params.HALF_MAT + Params.CHASSIS_LENGTH / 2),(-leftRight * Params.CHASSIS_HALF_WIDTH),0.0);
    }

    public void driveToBall(double distance) { // distance should be in half mats
        moveToBall = new Vector2d(distance * Params.HALF_MAT, 0);
    }

    public double pixelsToHalfmats(double pixels) { // pixels = area of bounding box
        return Math.sqrt(62320.0 / pixels); // returns half mats
    }

    @Override
    public void runOpMode() {
        setRobotLocation(1); // -1 for left or 1 for right

        setStartPoses(leftOrRight);

        driveToBall(pixelsToHalfmats(100)); // replace 100 with area of bounding box from LL

        drive = new MecanumDrive(hardwareMap, newStartPose);
        waitForStart();   // <-- REQUIRED
        if (opModeIsActive()) {
            auto();
        }
    }

    private void auto() {
        Actions.runBlocking(
                drive.actionBuilder(newStartPose)
                        .strafeTo(moveToBall)
                        .build()
        );
    }
}
