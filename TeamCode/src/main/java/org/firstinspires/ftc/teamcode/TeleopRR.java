/*
 * Copyright (c) 2020 OpenFTC Team
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */
 /*
  * PID controller and IMU codes are copied from
  * https://stemrobotics.cs.pdx.edu/node/7268%3Froot=4196.html
  */

package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.List;

/**
 * Hardware config:
 *      imu on control Hub:
 *          "imu"
 *
 *      Four drive motors:
 *          "FrontLeft"
 *          "BackLeft"
 *          "BackRight"
 *          "FrontRight"
 *
 *      Tow slider motors:
 *          "RightSlider"
 *          "LeftSlider"
 *
 *      Tow servo motors:
 *          "ArmServo"
 *          "ClawServo"
 */

@TeleOp(name="Teleop RR", group="Concept")
//@Disabled
public class TeleopRR extends LinearOpMode {
    // Declare OpMode members.
    private final ElapsedTime runtime = new ElapsedTime();

    // chassis
    MecanumDrive drive;

    //claw and arm unit
    private intakeUnit intake;

    // debug flags, turn it off for formal version to save time of logging
    boolean debugFlag = true;

    private DistanceSensor distSensor;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");

        GamePadButtons gpButtons = new GamePadButtons();

        updateProfileAccel(true);

        drive = new MecanumDrive(hardwareMap, Params.currentPose);
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        intake = new intakeUnit(hardwareMap, "Arm", "Wrist",
                "Knuckle", "Finger");

        //intake.setArmModeRunToPosition(intake.getArmPosition());
        //intake.resetArmEncoder();

        // you can use this as a regular DistanceSensor.
        distSensor = hardwareMap.get(DistanceSensor.class, "distance");

        // you can also cast this to a Rev2mDistanceSensor if you want to use added
        // methods associated with the Rev2mDistanceSensor class.
        //Rev2mDistanceSensor sensorTimeOfFlight = (Rev2mDistanceSensor) sensorDistance;

        // bulk reading setting - auto refresh mode
        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        //preset positions used for teleop commands
        Pose2d pickUpSpecimenPos = new Pose2d(-3.9 * Params.HALF_MAT, - 6 * Params.HALF_MAT + Params.CHASSIS_HALF_WIDTH, Math.toRadians(179.9998));
        Vector2d hangSpecimenPos = new Vector2d(- 3.3 * Params.HALF_MAT,  - Params.CHASSIS_HALF_WIDTH + 12.0);
        Vector2d outOfSubPose = new Vector2d(- 5 * Params.HALF_MAT, - 3 * Params.HALF_MAT);
        Vector2d pickupSamplePos = new Vector2d(- Params.HALF_MAT, - 4 * Params.HALF_MAT);

        // Wait for the game to start (driver presses PLAY)
        telemetry.addData("Mode", "waiting for start: %s", (Params.blueOrRed > 0) ? "Blue" : "Red");

        telemetry.update();
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            //gamepad1 buttons
            gpButtons.checkGamepadButtons(gamepad1, gamepad2);

            double maxDrivePower;
            if (gpButtons.speedUp) {
                maxDrivePower = Params.POWER_HIGH;
            } else if (gpButtons.speedDown) {
                maxDrivePower = Params.POWER_LOW;
            } else {
                maxDrivePower = Params.POWER_NORMAL;
            }
            drive.setDrivePowers(new PoseVelocity2d(
                    new Vector2d(
                            -gpButtons.robotDrive * maxDrivePower,
                            -gpButtons.robotStrafe * maxDrivePower
                    ),
                    -gpButtons.robotTurn * maxDrivePower
            ));

            if (gpButtons.armBackwards) {
                intake.setArmPosition(intake.getArmPosition() + 50);
            }

            if (gpButtons.armForwards) {
                intake.setArmPosition(intake.getArmPosition() - 50);
            }

            if (gpButtons.wristLeft) {
                intake.setWristPosition(intake.getWristPosition() + 50);
            }

            if (gpButtons.wristRight) {
                intake.setWristPosition(intake.getWristPosition() - 50);
            }

            if (gpButtons.knuckleUp) {
                intake.setKnucklePosition(intake.getKnucklePosition() - 0.006);
            }

            if (gpButtons.knuckleDown) {
                intake.setKnucklePosition(intake.getKnucklePosition() + 0.006);
            }

            if (gpButtons.fingerClose) {
                intake.setFingerPosition(intake.FINGER_CLOSE);
            }

            if (gpButtons.fingerOpen) {
                intake.setFingerPosition(intake.FINGER_OPEN);
            }

            // Align specimen to the high chamber, get ready for hanging.
            if (gpButtons.SpecimenHangAlign) {
                intake.setArmPosition(intake.ARM_POS_BEFORE_HANG);
                intake.setWristPosition(intake.WRIST_POS_HIGH_CHAMBER);
                intake.setKnucklePosition(intake.KNUCKLE_POS_HIGH_CHAMBER);
            }

            // hanging specimen action, arm back then forward to hang the specimen
            if (gpButtons.SpecimenHangAction) {
                intake.setArmPosition(intake.ARM_POS_HIGH_CHAMBER);
                intake.setWristPosition(intake.WRIST_POS_HIGH_CHAMBER);
                sleep(1000); // waiting arm to back position before flip forward to hanging specimen
                //intake.setArmPosition(intake.ARM_POS_HIGH_CHAMBER_TELEOP);
                //intake.setWristPosition(intake.WRIST_POS_HIGH_CHAMBER);

            }

            // hanging specimen action, arm back then forward to hang the specimen
            //not needed
            if (gamepad1.b) {
                intake.setArmPosition(intake.ARM_POS_BACK);
                intake.setWristPosition(intake.WRIST_POS_HIGH_CHAMBER);
                sleep(1000); // waiting arm to back position before flip forward to hanging specimen
                intake.setArmPosition(intake.ARM_POS_HIGH_CHAMBER_TELEOP);
                intake.setWristPosition(intake.WRIST_POS_HIGH_CHAMBER);

                sleep(1000);
                intake.setArmPosition(intake.ARM_POS_HIGH_CHAMBER_TELEOP - 300);
                sleep(200);

                Actions.runBlocking(
                        drive.actionBuilder(drive.pose)
                                .strafeToConstantHeading(new Vector2d(drive.pose.position.x, drive.pose.position.y - 12.0))
                                .build()
                );

            }

            // pick up specimen from sub and drive to high chamber automatically
            if (gpButtons.SpecimenPickupAction) {
                drive.pose = pickUpSpecimenPos;
                intake.setWristPosition(intake.WRIST_POS_GRAB_SPECIMEN);
                intake.setKnucklePosition(intake.KNUCKLE_POS_PICKUP_SPECIMEN);
                intake.setFingerPosition(intake.FINGER_CLOSE);
                sleep(100);
                intake.setKnucklePosition(intake.KNUCKLE_POS_PICKUP_SPECIMEN + 0.15);
                //intake.setWristPosition(intake.WRIST_POS_HIGH_CHAMBER);
                intake.setArmPosition(intake.ARM_POS_BEFORE_HANG);
                Actions.runBlocking(
                        drive.actionBuilder(pickUpSpecimenPos)
                                .afterTime(0.6, new armReadyToHangHigh())
                                .strafeToConstantHeading(hangSpecimenPos)
                                .build()
                );

                intake.setArmPosition(intake.ARM_POS_HIGH_CHAMBER + 500);
                intake.setKnucklePosition(intake.KNUCKLE_POS_HIGH_CHAMBER);
                double sensorDist = distSensor.getDistance(DistanceUnit.INCH);
                double shiftDelta = sensorDist - Params.HIGH_CHAMBER_DIST;
                shiftDelta = (shiftDelta > 6)? 6 : ((shiftDelta < -6)? -6 : shiftDelta);
                shiftDelta = Range.clip(shiftDelta, -7.0, 7.0);
                Logging.log("drive pose before distance");
                Logging.log(" X position = %2f, Y position = %2f ", drive.pose.position.x, drive.pose.position.y);
                Logging.log("tele Y sensor dist = %2f, shift delta = %2f", sensorDist, shiftDelta);

                Actions.runBlocking(
                        drive.actionBuilder(drive.pose)
                                .strafeToConstantHeading(new Vector2d(drive.pose.position.x + shiftDelta , drive.pose.position.y))
                                .build()
                );

                //sleep(100);


            }

            if(gamepad1.back)
            {
                drive.pose = pickUpSpecimenPos;
                intake.setWristPosition(intake.WRIST_POS_GRAB_SPECIMEN);
                intake.setKnucklePosition(intake.KNUCKLE_POS_PICKUP_SPECIMEN);
                intake.setFingerPosition(intake.FINGER_CLOSE);
                sleep(100);
                intake.setKnucklePosition(intake.KNUCKLE_POS_PICKUP_SPECIMEN + 0.15);
                //intake.setWristPosition(intake.WRIST_POS_HIGH_CHAMBER);
                intake.setArmPosition(intake.ARM_POS_BEFORE_HANG);
                Actions.runBlocking(
                        drive.actionBuilder(pickUpSpecimenPos)
                                .afterTime(0.6, new armReadyToHangHigh())
                                .strafeToConstantHeading(hangSpecimenPos)
                                .build()
                );

                intake.setArmPosition(intake.ARM_POS_HIGH_CHAMBER + 500);
                intake.setKnucklePosition(intake.KNUCKLE_POS_HIGH_CHAMBER);
                double sensorDist = distSensor.getDistance(DistanceUnit.INCH);
                double shiftDelta = sensorDist - Params.HIGH_CHAMBER_DIST;
                shiftDelta = (shiftDelta > 6)? 6 : ((shiftDelta < -6)? -6 : shiftDelta);
                shiftDelta = Range.clip(shiftDelta, -7.0, 7.0);
                Logging.log("drive pose before distance");
                Logging.log(" X position = %2f, Y position = %2f ", drive.pose.position.x, drive.pose.position.y);
                Logging.log("tele Y sensor dist = %2f, shift delta = %2f", sensorDist, shiftDelta);

                Actions.runBlocking(
                        drive.actionBuilder(drive.pose)
                                .strafeToConstantHeading(new Vector2d(drive.pose.position.x + shiftDelta , drive.pose.position.y))
                                .build()
                );

                sleep(700);

                // hanging specimen
                intake.setArmPosition(intake.ARM_POS_HIGH_CHAMBER);
                intake.setWristPosition(intake.WRIST_POS_HIGH_CHAMBER);
            }

            /*
            if (gamepad1.b) {
                intake.setArmPosition(intake.ARM_POS_HIGH_CHAMBER_TELEOP - 200);
                sleep(200);
                Actions.runBlocking(
                        drive.actionBuilder(drive.pose)
                                .strafeToLinearHeading(new Vector2d(drive.pose.position.x, drive.pose.position.y - 12.0), drive.pose.heading.log() - Math.toRadians(15.0))
                                .build()
                );
            }

             */


            // set arm, wrist, finger positions before pickup specimen.
            if (gpButtons.SpecimenPickupAlign) {
                intake.setArmPosition(intake.ARM_POS_GRAB_SPECIMEN);
                intake.setWristPosition(intake.WRIST_POS_GRAB_SPECIMEN);
                intake.setKnucklePosition(intake.KNUCKLE_POS_PICKUP_SPECIMEN);
                intake.setFingerPosition(intake.FINGER_OPEN);
            }

            // move robot to pickup element from the center of field
            if (gpButtons.SubPickupPos) {
                // TODO : update acting by using arm up, wrist down, moving using spline to directly.
                drive.pose = new Pose2d(hangSpecimenPos, 0);
                intake.setFingerPosition(intake.FINGER_OPEN);
                intake.setWristPosition(intake.WRIST_POS_NEUTRAL);
                sleep(400);
                Actions.runBlocking(
                        drive.actionBuilder(new Pose2d(hangSpecimenPos, 0))
                                .strafeToConstantHeading(outOfSubPose)
                                .splineToLinearHeading(new Pose2d(pickupSamplePos, Math.toRadians(90)), Math.toRadians(60))
                                .afterTime(0.6, new armPickupFromSub())
                                .build()
                );
            }

            // set arm and wrist position for picking up at the center of field
            // right bumper of pad1
            if (gpButtons.ArmPickUpPos) {
                intake.setArmPosition(intake.ARM_POS_SUB);
                intake.setWristPosition(intake.WRIST_POS_NEUTRAL);
            }

            // set arm and wrist position for drop off at low bucket.
            if (gpButtons.LowBucketPos) {
                intake.setArmPosition(intake.ARM_POS_LOW_BUCKET);
                intake.setWristPosition(intake.WRIST_POS_NEUTRAL);
                intake.setKnucklePosition(intake.KNUCKLE_POS_LOW_BUCKET);
            }

            // get ready for hanging at end game
            if (gpButtons.EndgameHangingLineup) {
                intake.setArmPosition(intake.ARM_POS_HANGING);
                intake.setKnucklePosition(intake.KNUCKLE_POS_HANGING);
            }

            // hanging robot
            if (gpButtons.EndgameHangingPos) {
                intake.setArmPosition(intake.ARM_POS_DOWN_HANGING);
            }

            drive.updatePoseEstimate();
            Params.currentPose = drive.pose;
            if (debugFlag) {
                // claw arm servo log
                telemetry.addData("Finger", "position %.3f", intake.getFingerPosition());

                telemetry.addData("Knuckle", "position = %.3f", intake.getKnucklePosition());

                telemetry.addData("Wrist", "position %s", intake.getWristPosition());

                telemetry.addData("Arm", "position = %.3f", (double)(intake.getArmPosition()));

                telemetry.addData(" ", " ");

                telemetry.addData("heading", " %.3f", Math.toDegrees(drive.pose.heading.log()));

                telemetry.addData("location", " %s", drive.pose.position.toString());

                telemetry.addData(" --- ", " --- ");

                telemetry.addData("range", String.format("%.01f in", distSensor.getDistance(DistanceUnit.INCH)));


                telemetry.update(); // update message at the end of while loop
            }

        }

        //intake.armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        // The motor stop on their own but power is still applied. Turn off motor.
    }

    //action to set arm and wrist position to pick up from sub
    private class armPickupFromSub implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            intake.setArmPosition(intake.ARM_POS_SUB);
            intake.setWristPosition(intake.WRIST_POS_NEUTRAL);
            return false;
        }
    }

    private class armReadyToHangHigh implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            intake.setArmPosition(intake.ARM_POS_BEFORE_HANG);
            intake.setWristPosition(intake.WRIST_POS_HIGH_CHAMBER);
            return false;
        }
    }

    private void updateProfileAccel(boolean fastMode) {
        if (fastMode) {
            MecanumDrive.PARAMS.minProfileAccel = -40;
            MecanumDrive.PARAMS.maxProfileAccel = 60;
        }
    }
}
