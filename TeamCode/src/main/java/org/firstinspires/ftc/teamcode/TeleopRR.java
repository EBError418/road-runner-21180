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
 *      Tow arm, wrist motors:
 *          "Arm"
 *          "Wrist"
 *
 *      Tow servo motors:
 *          "Knuckle"
 *          "Finger"
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

    private DistanceSensor distSensorHanging;
    private DistanceSensor distSensorPickup;

    int specimenCount = 0;//counter used to update specimen hanging position

    Pose2d pickUpSpecimenPos = new Pose2d(- 3.05 * Params.HALF_MAT, - 3.8 * Params.HALF_MAT, Math.toRadians(180));
    Vector2d hangSpecimenPos = new Vector2d(- 3.3 * Params.HALF_MAT,  0 + specimenCount * 2.0); //shifts left for every specimen hanged
    Vector2d outOfSubPose = new Vector2d(- 5 * Params.HALF_MAT, - 3 * Params.HALF_MAT);
    Vector2d pickupSamplePos = new Vector2d(- Params.HALF_MAT, - 4 * Params.HALF_MAT);

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");

        GamePadButtons gpButtons = new GamePadButtons();

        updateProfileAccel(true);

        drive = new MecanumDrive(hardwareMap, Params.currentPose);
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        intake = new intakeUnit(hardwareMap, "Arm", "Wrist",
                "Knuckle", "Finger");

        // set RunToPosition mode and set power for motors.
        intake.setWristModeRunToPosition(intake.getWristPosition());
        intake.setArmModeRunToPosition(intake.getArmPosition());


        // you can use this as a regular DistanceSensor.
        distSensorHanging = hardwareMap.get(DistanceSensor.class, "distance hanging");
        distSensorPickup = hardwareMap.get(DistanceSensor.class, "distance pickup");

        // bulk reading setting - auto refresh mode
        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        //preset positions used for teleop commands



        // Wait for the game to start (driver presses PLAY)
        telemetry.addData("Mode", "waiting for start.");

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

            if (gpButtons.wristFront) {
                intake.setWristPosition(intake.WRIST_POS_NEUTRAL);
            }

            if (gpButtons.wristBack) {
                intake.setWristPosition(intake.WRIST_POS_GRAB_SAMPLE_BACK);
            }

            if (gpButtons.fingerClose) {
                intake.setFingerPosition(intake.FINGER_CLOSE);
            }

            if (gpButtons.fingerOpen) {
                intake.setFingerPosition(intake.FINGER_OPEN_SUB);
            }

            // Align specimen to the high chamber, get ready for hanging.
            if (gpButtons.SpecimenHangAlign) {
                intake.setArmPosition(intake.ARM_POS_HIGH_CHAMBER_READY);
                intake.setWristPosition(intake.WRIST_POS_HIGH_CHAMBER);
                intake.setKnucklePosition(intake.KNUCKLE_POS_HIGH_CHAMBER);
            }

            // hanging specimen action, lower arm and shift left several inches. GAMEPAD1.Y
            if (gpButtons.SpecimenHangAction) {
                intake.setArmPosition(intake.ARM_POS_HIGH_CHAMBER_TELEOP);
                intake.setWristPosition(intake.WRIST_POS_HIGH_CHAMBER);
                sleep(500); // waiting for hanging success

                // lift arm a little bit before moving left
                intake.setArmPosition(intake.ARM_POS_HIGH_CHAMBER_MOVING_SPECIMEN);
                sleep(200);

                // moving left 6 inch
                Actions.runBlocking(
                        drive.actionBuilder(drive.pose)
                                .strafeToConstantHeading(new Vector2d(drive.pose.position.x, drive.pose.position.y - 6.0))
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

                //intake.setArmPosition(intake.ARM_POS_BEFORE_HANG); // lift arm to during strafe to chambers
                Actions.runBlocking(
                        drive.actionBuilder(pickUpSpecimenPos)
                                .afterTime(0.4, new armReadyToHangHigh())
                                .strafeToLinearHeading(hangSpecimenPos, pickUpSpecimenPos.heading)
                                .build()
                );

                intake.setArmPosition(intake.ARM_POS_HIGH_CHAMBER_READY);
                intake.setKnucklePosition(intake.KNUCKLE_POS_HIGH_CHAMBER);

                adjustPosByDistanceSensor(Params.HIGH_CHAMBER_DIST, 1);

                if (specimenCount <= 8) {
                    specimenCount ++;//update specimen pos
                }

                hangSpecimenPos = new Vector2d(- 3.3 * Params.HALF_MAT,  specimenCount * 2);

                // hanging specimen
                intake.setArmPosition(intake.ARM_POS_HIGH_CHAMBER);
                intake.setWristPosition(intake.WRIST_POS_HIGH_CHAMBER);
                sleep(400);
                intake.setArmPosition(intake.ARM_POS_HIGH_CHAMBER_MOVING_SPECIMEN);
                sleep(200);
                Actions.runBlocking(
                        drive.actionBuilder(drive.pose)
                                //.afterTime(0.05, new armToPickUpPos())
                                .strafeToLinearHeading(new Vector2d(drive.pose.position.x, drive.pose.position.y - 5.0), pickUpSpecimenPos.heading)
                                .build()
                );

                //return to observation zone
                intake.fingerServoOpen();
                sleep(150);
                Actions.runBlocking(
                        drive.actionBuilder(drive.pose)
                                .afterTime(0.05, new armToPickUpPos())
                                .strafeToLinearHeading(pickUpSpecimenPos.position, pickUpSpecimenPos.heading)
                                .build()
                );

                /*
                intake.setArmPosition(intake.ARM_POS_HIGH_CHAMBER_READY - 100);
                sleep(350);
                Actions.runBlocking(
                        drive.actionBuilder(drive.pose)
                                .strafeToConstantHeading(new Vector2d(drive.pose.position.x, drive.pose.position.y - 6.0))
                                .build()
                );

                 */
            }

            // strafe 6 inch to push specimens
            if (gamepad1.b) {
                intake.setArmPosition(intake.ARM_POS_HIGH_CHAMBER_MOVING_SPECIMEN);
                sleep(200);
                Actions.runBlocking(
                        drive.actionBuilder(drive.pose)
                                .strafeToConstantHeading(new Vector2d(drive.pose.position.x, drive.pose.position.y - 6.0))
                                .build()
                );
            }

            // set arm, wrist, finger positions before pickup specimen.
            if (gpButtons.SpecimenPickupAlign) {
                intake.setArmPosition(intake.ARM_POS_GRAB_SPECIMEN);
                intake.setWristPosition(intake.WRIST_POS_GRAB_SPECIMEN);
                intake.setKnucklePosition(intake.KNUCKLE_POS_PICKUP_SPECIMEN);
                intake.setFingerPosition(intake.FINGER_OPEN);
            }

            // move robot to pickup element from the center of field
            //DEPRECATED
            if (gpButtons.SubPickupPos) {
                // TODO : need update for new intake
                /*
                drive.pose = new Pose2d(hangSpecimenPos, Math.toRadians(-180.0));
                intake.setFingerPosition(intake.FINGER_OPEN);
                intake.setArmPosition(intake.ARM_POS_BEFORE_HANG);
                sleep(400);
                Actions.runBlocking(
                        drive.actionBuilder(new Pose2d(hangSpecimenPos, 0))
                                .strafeToConstantHeading(outOfSubPose)
                                .splineToLinearHeading(new Pose2d(pickupSamplePos, Math.toRadians(90)), Math.toRadians(60))
                                .afterTime(0.6, new armPickupFromSub())
                                .build()
                );
                 */
            }

            // set arm and wrist position for picking up at the center of field
            // right bumper of pad2
            if (gpButtons.ArmPickUpPos) {
                intake.setArmPosition(intake.ARM_POS_SUB);
                intake.setWristPosition(intake.WRIST_POS_NEUTRAL);
                intake.setKnucklePosition(intake.KNUCKLE_POS_PICKUP_SAMPLE_READY);
                intake.setFingerPosition(intake.FINGER_OPEN_SUB);
            }

            // set arm and wrist position for drop off at low bucket.
            if (gpButtons.LowBucketPos) {
                intake.setArmPosition(intake.ARM_POS_LOW_BUCKET);
                intake.setWristPosition(intake.WRIST_POS_LOW_BUCKET);
                intake.setKnucklePosition(intake.KNUCKLE_POS_LOW_BUCKET);
            }

            // get ready for hanging at end game
            //gamepad2.x
            if (gpButtons.EndgameHangingLineup) {
                intake.setArmPosition(intake.ARM_POS_BEFORE_HANG);
                intake.setKnucklePosition(intake.KNUCKLE_POS_HANGING);
                intake.setWristPosition(intake.WRIST_POS_NEUTRAL);
            }

            // hanging robot
            //gamepad2.y
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

                telemetry.addData("specimens hanged: %s", specimenCount);

                telemetry.addData(" --- ", " --- ");

                telemetry.addData("hanging distance range", String.format("%.01f in", distSensorHanging.getDistance(DistanceUnit.INCH)));

                telemetry.addData("pickup distance range", String.format("%.01f in", distSensorPickup.getDistance(DistanceUnit.INCH)));


                telemetry.update(); // update message at the end of while loop
            }

        }

        //intake.armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        // The motor stop on their own but power is still applied. Turn off motor.
    }

    //action to set arm and wrist position to pick up from sub
    private class armToPickUpPos implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            //intake.setArmPosition(intake.ARM_POS_GRAB_SPECIMEN);
            //intake.setWristPosition(intake.WRIST_POS_GRAB_SPECIMEN);
            intake.setKnucklePosition(intake.KNUCKLE_POS_AWAY_FROM_SUBMERSIBLE);
            intake.fingerServoOpen();
            return false;
        }
    }

    private class armReadyToHangHigh implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            intake.setArmPosition(intake.ARM_POS_HIGH_CHAMBER_READY);
            intake.setWristPosition(intake.WRIST_POS_HIGH_CHAMBER);
            intake.setKnucklePosition(intake.KNUCKLE_POS_HIGH_CHAMBER - 0.2);
            return false;
        }
    }

    private void updateProfileAccel(boolean fastMode) {
        if (fastMode) {
            MecanumDrive.PARAMS.minProfileAccel = -40;
            MecanumDrive.PARAMS.maxProfileAccel = 60;
        }
    }

    private void adjustPosByDistanceSensor(double aimDistance, int distSensorID) { //distSensorID: 1 for hanging sensor, 2 for pickup sensor
        double sensorDist = 0.0;
        int repeatTimes = 5;

        //DistanceSensor usedDistSensor = (distSensorID == 1)? distSensor : distSensorPickup ;

        for (int i = 1; i <= repeatTimes; i++)
        {
            double sensorReading = (distSensorID == 1)? distSensorHanging.getDistance(DistanceUnit.INCH) : distSensorPickup.getDistance(DistanceUnit.INCH) ;
            sensorDist = sensorDist + sensorReading;
            Logging.log("distance sensor # %s reading repetition # %s reading number = %2f", distSensorID, i, sensorReading);
            sleep(2);
        }
        sensorDist = sensorDist / repeatTimes;


        double shiftDelta = sensorDist - aimDistance;
        shiftDelta = Range.clip(shiftDelta, -7.0, 7.0); // limit adjust distance to +-7.0 inch
        Logging.log("drive pose before distance average number");
        Logging.log("before adjust, sensor distance = %2f, shift delta = %2f", sensorDist, shiftDelta);
        Logging.log(" X position = %2f, Y position = %2f , heading = %sf", drive.pose.position.x, drive.pose.position.y, Math.toDegrees(drive.pose.heading.log()));

        Actions.runBlocking(
                drive.actionBuilder(drive.pose)
                        .strafeToLinearHeading(new Vector2d(drive.pose.position.x + shiftDelta , drive.pose.position.y), pickUpSpecimenPos.heading) // adjust heading also.
                        .build()
        );
        Logging.log(" After adjust: X position = %2f, Y position = %2f , heading = %sf", drive.pose.position.x, drive.pose.position.y, Math.toDegrees(drive.pose.heading.log()));
        Logging.log("after adjust, sensor distance = %2f, aim distance = %2f ", (distSensorID == 1)? distSensorHanging.getDistance(DistanceUnit.INCH) : distSensorPickup.getDistance(DistanceUnit.INCH), aimDistance);
    }
}
