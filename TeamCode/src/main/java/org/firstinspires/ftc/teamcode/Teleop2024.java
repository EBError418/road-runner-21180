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

@TeleOp(name="Teleop 2024", group="Concept")
//@Disabled
public class Teleop2024 extends AutoRightHanging2 {
    // Declare OpMode members.
    private final ElapsedTime runtime = new ElapsedTime();

    // chassis
    MecanumDrive drive;

    //claw and arm unit
    private intakeUnit intake;
    private DistanceSensor distSensorHanging;
    private DistanceSensor distSensorF;

    int specimenCount = 0;//counter used to update specimen hanging position
    int specimenShiftMax = 7; //shift 2 inch for each specimen hanging
    double specimenShiftInch = -7.0; // shift specimen to 7 inch right (-y) after hanging on high chamber
    double specimenShiftEach = 1.5; // shift hanging place shift in Y direct
    // debug flags, turn it off for formal version to save time of logging
    boolean debugFlag = true;

    double initHeading = Math.toRadians(180);

    // avoid program crush when calling turnTo() function for fine heading correction
    double headingAngleCorrection = Math.toRadians(180.0 - 0.1);

    Pose2d pickUpSpecimenWallPos = new Pose2d(- 3.6 * Params.HALF_MAT, - 4.0 * Params.HALF_MAT, initHeading);
    Pose2d specimenLineUpPos = new Pose2d(- 3.3 * Params.HALF_MAT, - 4.0 * Params.HALF_MAT, initHeading);

    Vector2d hangSpecimenPos = new Vector2d(- 3.4 * Params.HALF_MAT,  0.0); //shifts left for every specimen hanged
    Vector2d clearHighChamberPos = new Vector2d(- 3.5 * Params.HALF_MAT, - 3.5 * Params.HALF_MAT);
    Vector2d clearHighChamberForHang = new Vector2d(-3.5 * Params.HALF_MAT, 3 * Params.HALF_MAT);
    Vector2d pickupSamplePos = new Vector2d(- Params.HALF_MAT, - 4 * Params.HALF_MAT);
    Vector2d LowRungPos = new Vector2d(- 0.5 * Params.HALF_MAT, 3 * Params.HALF_MAT);

    int sleepTimeForHangingSpecimen = 500;
    int knuckleSleepTime = 150;
    int sleepHighChamber = 2;

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
        distSensorHanging = hardwareMap.get(DistanceSensor.class, "distanceB");
        distSensorF = hardwareMap.get(DistanceSensor.class, "distanceF");

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

            // moving arm with constraint
            if (gpButtons.armBackwards) {
                intake.setArmPosition(intake.getArmPosition() + 50);

                double knucklePos = intake.getKnucklePosition();
                if (-2800 > intake.getArmPosition() && intake.getArmPosition() > -3700) {
                    intake.setWristPosition(intake.WRIST_BACK);
                    if (knucklePos>intake.KNUCKLE_SIZE_CONSTRAINT) {
                        intake.setKnucklePosition(intake.KNUCKLE_SIZE_CONSTRAINT);
                    }
                }
            }

            // moving arm with constraint
            if (gpButtons.armForwards) {
                intake.setArmPosition(intake.getArmPosition() - 50);

                // limit size to 20 inch at back side.
                double knucklePos = intake.getKnucklePosition();
                if (-2800 > intake.getArmPosition() && intake.getArmPosition() > -3700) {
                    intake.setWristPosition(intake.WRIST_BACK);
                    if (knucklePos>intake.KNUCKLE_SIZE_CONSTRAINT) {
                        intake.setKnucklePosition(intake.KNUCKLE_SIZE_CONSTRAINT);

                    }
                }
            }

            // wrist control: game pad2, left stick
            if (gpButtons.wristLeft) {
                intake.setWristPosition(intake.getWristPosition() + 25);
            }
            if (gpButtons.wristRight) {
                intake.setWristPosition(intake.getWristPosition() - 25);
            }

            // wrist control: game pad2, dpad left/right button
            if (gpButtons.wristFront) {
                double knucklePos = intake.getKnucklePosition();
                // remove interference between wrist and fingers
                if (knucklePos < intake.KNUCKLE_POS_WRIST_CONSTRAINT)
                {
                    intake.setKnucklePosition(intake.KNUCKLE_POS_WRIST_CONSTRAINT);
                    sleep(knuckleSleepTime); // knuckle move out the way
                }
                intake.setWristPosition(intake.WRIST_POS_NEUTRAL);

                /*
                if (knucklePos < intake.KNUCKLE_POS_WRIST_CONSTRAINT)
                {
                    sleep(150); // wait wrist moving to position
                    intake.setKnucklePosition(intake.KNUCKLE_POS_WRIST_CONSTRAINT); // set nuckle back
                }
                 */
            }
            if (gpButtons.wristBack) {
                double knucklePos = intake.getKnucklePosition();
                // remove interference between wrist and fingers
                if (knucklePos < intake.KNUCKLE_POS_WRIST_CONSTRAINT)
                {
                    intake.setKnucklePosition(intake.KNUCKLE_POS_WRIST_CONSTRAINT);
                    sleep(knuckleSleepTime); // knuckle move out the way
                }
                intake.setWristPosition(intake.WRIST_BACK);
            }

            // knuckle control
            if (gpButtons.knuckleUp) {
                intake.setKnucklePosition(intake.getKnucklePosition() - 0.006);
            }

            if (gpButtons.knuckleDown) {
                intake.setKnucklePosition(intake.getKnucklePosition() + 0.006);
            }

            if (gamepad2.start && gamepad2.right_trigger > 0) {
                intake.resetWristEncoder();
            }

            // finger control. game pad2 -> dpad up/down
            if (gpButtons.fingerOpenClose) {
                double tmp = (intake.FINGER_OPEN_SUB + intake.FINGER_CLOSE)/2.0;

                intake.setFingerPosition((intake.getFingerPosition() > tmp)? intake.FINGER_OPEN_SUB : intake.FINGER_CLOSE );
                sleep(150); // avoid multi-times hitting
            }

            if (gpButtons.fingerOpenCloseBack) {
                double tmp = (intake.FINGER_CLOSE_BACK + intake.FINGER_OPEN_BACK)/2.0;
                intake.setFingerPosition((intake.getFingerPosition() > tmp)? intake.FINGER_CLOSE_BACK : intake.FINGER_OPEN_BACK );
                sleep(150);// avoid multi-times hitting
            }

            // Align specimen to the high chamber, get ready for hanging. gamepad1.left_trigger
            if (gpButtons.SpecimenHangAlign) {
                intake.setArmPosition(intake.ARM_POS_HIGH_CHAMBER_READY);
                intake.setWristPosition(intake.WRIST_BACK);
                intake.setKnucklePosition(intake.KNUCKLE_POS_HIGH_CHAMBER);
            }

            // hanging specimen action, lower arm and shift left several inches. GAMEPAD1. left_bumper
            if (gpButtons.SpecimenHangAction) {
                intake.setArmPosition(intake.ARM_POS_HIGH_CHAMBER);
                intake.setWristPosition(intake.WRIST_BACK);
                intake.setKnucklePosition(intake.KNUCKLE_POS_HIGH_CHAMBER);
                sleep(sleepTimeForHangingSpecimen); // waiting for hanging success
                intake.setFingerPosition(intake.FINGER_OPEN_SUB);
                sleep(100);

                // lift arm a little bit before moving left
                intake.setArmPosition(intake.ARM_POS_HIGH_CHAMBER_MOVING_SPECIMEN);
                intake.setKnucklePosition(intake.KNUCKLE_SIZE_CONSTRAINT);
            }

            // move to submarine after hanging specimens, gamepad1.b
            if (gpButtons.SpecimenHangToSub) {
                drive.pose = pickUpSpecimenWallPos;
                intake.setFingerPosition(intake.FINGER_CLOSE);
                sleep(150);
                intake.setKnucklePosition(intake.KNUCKLE_POS_LIFT_FROM_WALL);
                sleep(100); // wait knuckle lift the specimen

                Actions.runBlocking(
                        drive.actionBuilder(drive.pose)
                                .afterTime(0.6, new intakeAct(intake.ARM_POS_HIGH_CHAMBER_READY, intake.WRIST_BACK, intake.KNUCKLE_POS_LIFT_FROM_WALL, intake.FINGER_CLOSE))
                                .strafeToLinearHeading(hangSpecimenPos, initHeading)
                                .turnTo(headingAngleCorrection) // fine correction heading
                                .build()
                );

                intake.setKnucklePosition(intake.KNUCKLE_POS_HIGH_CHAMBER);
                intake.setKnucklePosition(intake.getKnucklePosition() + 0.01);//reset position after collision
                intake.setFingerPosition(intake.getFingerPosition() - 0.01);//reset position after collision

                sleep(sleepHighChamber);
                adjustPosByDistanceSensor(Params.HIGH_CHAMBER_DIST, distSensorHanging);

                if (specimenCount <= specimenShiftMax) {
                    specimenCount ++;//update specimen pos
                }

                // hanging specimen
                intake.setArmPosition(intake.ARM_POS_HIGH_CHAMBER);

                sleep(sleepTimeForHangingSpecimen);

                intake.setKnucklePosition(intake.getKnucklePosition() + 0.01);//reset position after collision

                intake.setArmPosition(intake.ARM_POS_HIGH_CHAMBER_MOVING_SPECIMEN);


                //go to submersible pick up pos
                intake.fingerServoOpen();
                sleep(150);
                intake.setKnucklePosition(intake.KNUCKLE_POS_AWAY_FROM_SUBMERSIBLE);

                Actions.runBlocking(
                        drive.actionBuilder(drive.pose)
                                .strafeToLinearHeading(clearHighChamberPos, Math.toRadians(90))
                                .afterTime(0.15, new armReadyToPickupSample())
                                .strafeToConstantHeading(pickupSamplePos)
                                .build()
                );
            }

            //   move to ascent after hanging specimen. gamepad1.x
            if (gpButtons.SpecimenHangToAscent) {
                drive.pose = pickUpSpecimenWallPos;
                intake.setFingerPosition(intake.FINGER_CLOSE);
                sleep(150);
                intake.setKnucklePosition(intake.KNUCKLE_POS_LIFT_FROM_WALL);
                sleep(100); // wait knuckle lift the specimen

                Actions.runBlocking(
                        drive.actionBuilder(drive.pose)
                                .afterTime(0.6, new intakeAct(intake.ARM_POS_HIGH_CHAMBER_READY, intake.WRIST_BACK, intake.KNUCKLE_POS_LIFT_FROM_WALL, intake.FINGER_CLOSE))
                                .strafeToLinearHeading(hangSpecimenPos, initHeading)
                                .turnTo(headingAngleCorrection) // fine correction heading
                                .build()
                );
                intake.setKnucklePosition(intake.KNUCKLE_POS_HIGH_CHAMBER);
                intake.setKnucklePosition(intake.getKnucklePosition() + 0.01);//reset position after collision
                intake.setFingerPosition(intake.getFingerPosition() - 0.01);//reset position after collision

                sleep(sleepHighChamber);
                adjustPosByDistanceSensor(Params.HIGH_CHAMBER_DIST, distSensorHanging);

                if (specimenCount <= specimenShiftMax) {
                    specimenCount ++;//update specimen pos
                }

                // hanging specimen
                intake.setArmPosition(intake.ARM_POS_HIGH_CHAMBER);

                sleep(sleepTimeForHangingSpecimen);

                intake.setKnucklePosition(intake.getKnucklePosition() + 0.01);//reset position after collision

                intake.setArmPosition(intake.ARM_POS_HIGH_CHAMBER_MOVING_SPECIMEN);

                //go to ascent level 2
                intake.fingerServoOpen();
                sleep(150);
                intake.setKnucklePosition(intake.KNUCKLE_POS_AWAY_FROM_SUBMERSIBLE);
                Actions.runBlocking(
                        drive.actionBuilder(drive.pose)
                                .afterTime(0.15, new armReadyToAscent())
                                .strafeToLinearHeading(clearHighChamberForHang, Math.toRadians(-90))
                                .strafeToConstantHeading(LowRungPos)
                                .build()
                );
            }

            // release specimen and back a little bit after hanging specimen. game pad1.a
            if (gpButtons.SpecimenHangToBack) {
                drive.pose = pickUpSpecimenWallPos;
                intake.setFingerPosition(intake.FINGER_CLOSE);
                sleep(150);

                //intake.setArmPosition(intake.ARM_POS_BEFORE_HANG); // lift arm to during strafe to chambers
                Actions.runBlocking(
                        drive.actionBuilder(drive.pose)
                                .afterTime(0.4, new armReadyToHangHigh())
                                .strafeToLinearHeading(hangSpecimenPos, initHeading)
                                .build()
                );

                //intake.setKnucklePosition(intake.KNUCKLE_POS_HIGH_CHAMBER);
                intake.setKnucklePosition(intake.getKnucklePosition() + 0.01);//reset position after collision
                intake.setFingerPosition(intake.getFingerPosition() - 0.01);//reset position after collision

                sleep(sleepHighChamber);
                adjustPosByDistanceSensor(Params.HIGH_CHAMBER_DIST, distSensorHanging);

                if (specimenCount <= specimenShiftMax) {
                    specimenCount ++;//update specimen pos
                }

                // hanging specimen
                intake.setArmPosition(intake.ARM_POS_HIGH_CHAMBER);

                sleep(sleepTimeForHangingSpecimen);

                intake.setKnucklePosition(intake.getKnucklePosition() + 0.01);//reset position after collision

                intake.setArmPosition(intake.ARM_POS_HIGH_CHAMBER_MOVING_SPECIMEN);

                //go to back
                intake.fingerServoOpen();
                sleep(150);
                intake.setKnucklePosition(intake.KNUCKLE_POS_AWAY_FROM_SUBMERSIBLE);
                Actions.runBlocking(
                        drive.actionBuilder(drive.pose)
                                .strafeToConstantHeading(new Vector2d(drive.pose.position.x - 10, drive.pose.position.y))
                                .build()
                );
            }

            // cycling specimen from wall, gamepad1.y
            if (gpButtons.SpecimenCycleWall) {
                drive.pose = pickUpSpecimenWallPos;

                // close finger
                intake.setFingerPosition(intake.FINGER_CLOSE);
                sleep(150);
                intake.setKnucklePosition(intake.KNUCKLE_POS_LIFT_FROM_WALL);
                sleep(100); // wait knuckle lift the specimen

                // strafe to high chamber for hanging specimen
                Actions.runBlocking(
                        drive.actionBuilder(drive.pose)
                                // flip arm to high chamber position, back knuckle to avoid hitting chamber during strafing
                                .afterTime(0.6, new intakeAct(intake.ARM_POS_HIGH_CHAMBER_READY, intake.WRIST_BACK, intake.KNUCKLE_POS_LIFT_FROM_WALL, intake.FINGER_CLOSE))
                                // shift 1.5 inch for each specimen on high chamber
                                .strafeToLinearHeading(hangSpecimenPos, initHeading)
                                .turnTo(headingAngleCorrection) // fine correction heading
                                .build()
                );
                intake.setKnucklePosition(intake.KNUCKLE_POS_HIGH_CHAMBER);

                intake.setKnucklePosition(intake.getKnucklePosition() + 0.01);//reset position after collision
                intake.setFingerPosition(intake.getFingerPosition() - 0.01);//reset position after collision

                sleep(sleepHighChamber);
                adjustPosByDistanceSensor(Params.HIGH_CHAMBER_DIST, distSensorHanging);

                if (specimenCount <= specimenShiftMax) {
                    specimenCount ++;//update specimen pos
                }

                // hanging specimen
                intake.setArmPosition(intake.ARM_POS_HIGH_CHAMBER);

                sleep(sleepTimeForHangingSpecimen);

                intake.setKnucklePosition(intake.getKnucklePosition() + 0.01);//reset position after collision

                intake.setArmPosition(intake.ARM_POS_HIGH_CHAMBER_MOVING_SPECIMEN);

                Logging.log(" after y shift pos: X position = %2f, Y position = %2f , heading = %sf", drive.pose.position.x, drive.pose.position.y, Math.toDegrees(drive.pose.heading.log()));
                Logging.log(" finger position = %2f ", intake.getFingerPosition());

                //return to wall to pickup next specimen
                intake.fingerServoOpen();
                sleep(150);
                intake.setKnucklePosition(intake.KNUCKLE_POS_AWAY_FROM_SUBMERSIBLE);
                Actions.runBlocking(
                        drive.actionBuilder(drive.pose)
                                .afterTime(0.15, new armToPickUpWallPos())
                                // left high chamber
                                .strafeTo(new Vector2d(drive.pose.position.x - 1, drive.pose.position.y))
                                .strafeToLinearHeading(specimenLineUpPos.position, initHeading)
                                .turnTo(headingAngleCorrection) // fine correction heading
                                .build()
                );

                // adjust wall distance by distance sensor
                //adjustPosByDistanceSensor(Params.SPECIMEN_PICKUP_DIST, distSensorF);

            }

            // pickup specimen and move to high chamber
            if ( gamepad1.right_trigger > 0) {
                drive.pose = pickUpSpecimenWallPos;
                intake.setFingerPosition(intake.FINGER_CLOSE);
                sleep(150);
                intake.setKnucklePosition(intake.KNUCKLE_POS_LIFT_FROM_WALL);
                sleep(100); // wait knuckle lift the specimen
                Actions.runBlocking(
                        drive.actionBuilder(drive.pose)
                                .afterTime(0.6, new intakeAct(intake.ARM_POS_HIGH_CHAMBER_READY, intake.WRIST_BACK, intake.KNUCKLE_POS_LIFT_FROM_WALL, intake.FINGER_CLOSE))
                                .strafeToLinearHeading(hangSpecimenPos, initHeading)
                                .build()
                );

                intake.setKnucklePosition(intake.KNUCKLE_POS_HIGH_CHAMBER);
                sleep(sleepHighChamber);
                adjustPosByDistanceSensor(Params.HIGH_CHAMBER_DIST, distSensorHanging);
            }

            // pickup specimen from wall positions, gamepad2.right_trigger or left_trigger
            if (gpButtons.SpecimenPickupWallPos) {
                pickupFromWallActions();
            }

            // set arm and wrist position for picking up at the center of field. gamepad2.right_bumper
            // right bumper of pad2
            if (gpButtons.PickupSampleIntakePos) {
                intake.setArmPosition(intake.ARM_POS_SUB);
                //intake.setFingerPosition(intake.FINGER_OPEN_BACK);
                intake.setKnucklePosition(intake.KNUCKLE_POS_PICKUP_SAMPLE);
                sleep(knuckleSleepTime); // remove wrist and fingers interference
                intake.setWristPosition(intake.WRIST_BACK);
            }

            // set arm and wrist position for drop off at low bucket. gamepad2.left_bumper
            if (gpButtons.LowBucketPos) {
                intake.setArmPosition(intake.ARM_POS_LOW_BUCKET);
                intake.setKnucklePosition(intake.KNUCKLE_POS_LOW_BUCKET);
                sleep(knuckleSleepTime); // remove wrist and fingers interference
                intake.setWristPosition(intake.WRIST_BACK);
            }

            // get ready for hanging at end game
            //gamepad2.x
            if (gpButtons.EndgameHangingLineup) {
                intake.setKnucklePosition(intake.KNUCKLE_POS_HANGING);
                sleep(knuckleSleepTime); // remove wrist and fingers interference
                intake.setArmPosition(intake.ARM_POS_BEFORE_HANG);
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

                telemetry.addData("back distance range", String.format("%.01f in", distSensorHanging.getDistance(DistanceUnit.INCH)));

                telemetry.addData("front distance range", String.format("%.01f in", distSensorF.getDistance(DistanceUnit.INCH)));

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
            intake.fingerServoOpen();
            intake.setArmPosition(intake.ARM_POS_GRAB_SPECIMEN_READY);
            intake.setKnucklePosition(intake.KNUCKLE_POS_PICKUP_SPECIMEN_ready);
            sleep(knuckleSleepTime);
            intake.setWristPosition(intake.WRIST_POS_NEUTRAL);

            return false;
        }
    }

    //action to set arm and wrist position to pick up from sub
    private class armToPickUpWallPos implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            pickupFromWallActions();

            return false;
        }
    }


    private class armReadyToHangHigh implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            intake.setArmPosition(intake.ARM_POS_HIGH_CHAMBER_READY);
            intake.setKnucklePosition(intake.KNUCKLE_POS_WRIST_CONSTRAINT);
            sleep(knuckleSleepTime);
            intake.setWristPosition(intake.WRIST_BACK);
            sleep(knuckleSleepTime);
            intake.setKnucklePosition(intake.KNUCKLE_POS_HIGH_CHAMBER);

            return false;
        }
    }

    private class armReadyToPickupSample implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            intake.setArmPosition(intake.ARM_POS_GRAB_SAMPLE);
            intake.setKnucklePosition(intake.KNUCKLE_POS_PICKUP_SAMPLE_READY);
            sleep(knuckleSleepTime);
            intake.setWristPosition(intake.WRIST_BACK);
            return false;
        }
    }

    private class armReadyToAscent implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            intake.setArmPosition(intake.ARM_POS_BEFORE_HANG);
            intake.setKnucklePosition(intake.KNUCKLE_POS_HANGING);
            sleep(knuckleSleepTime);
            intake.setWristPosition(intake.WRIST_POS_NEUTRAL);
            return false;
        }
    }

    //action for arm, wrist, finger position control during driving

    private class intakeAct implements Action {

        /**
         *
         * @param armPos:  the target position for arm motor
         * @param wristPos: wrist position
         * @param knucklePos: knuckle position
         * @param fingerPos: finger position
         */
        public intakeAct(int armPos, int wristPos, double knucklePos, double fingerPos) {
            arm = armPos;
            wrist = wristPos;
            knuckle = knucklePos;
            finger = fingerPos;
        }

        private final int arm;
        private final int wrist;
        private final double knuckle;
        private final double finger;

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {

            if (Params.NO_CATION != arm) {
                intake.setArmPosition(arm);
            }

            if (Params.NO_CATION != wrist) {
                intake.setWristPosition(wrist);
            }

            if (Params.NO_CATION != (int)(knuckle)){
                intake.setKnucklePosition(knuckle);
            }

            if (Params.NO_CATION != (int)(finger)){
                intake.setFingerPosition(finger);
            }
            return false;
        }
    }


    private void updateProfileAccel(boolean fastMode) {
        if (fastMode) {
            MecanumDrive.PARAMS.minProfileAccel = -40;
            MecanumDrive.PARAMS.maxProfileAccel = 60;
        }
    }

    /**
     * Set intake positions for picking up specimen from wall
     */
    private void pickupFromWallActions() {
        intake.fingerServoOpen();
        sleep(knuckleSleepTime);
        intake.setArmPosition(intake.ARM_POS_GRAB_SPECIMEN_WALL);
        intake.setKnucklePosition(intake.KNUCKLE_POS_PICKUP_SPECIMEN_WALL);
        sleep(knuckleSleepTime);
        sleep(knuckleSleepTime); // double sleep time since we have enough spare time here
        intake.setWristPosition(intake.WRIST_POS_NEUTRAL);
    }

    private void adjustPosByDistanceSensor(double aimDistance, DistanceSensor distSensorID) { //distSensorID: 1 for hanging sensor, 2 for pickup sensor
        double sensorDist = 0.0;
        int repeatTimes = 5;

        for (int i = 1; i <= repeatTimes; i++)
        {
            double sensorReading = distSensorID.getDistance(DistanceUnit.INCH) ;
            sensorDist = sensorDist + sensorReading;
            Logging.log("distance sensor reading repetition # %s reading number = %2f", i, sensorReading);
            sleep(2);
        }
        sensorDist = sensorDist / repeatTimes;


        double shiftDelta = sensorDist - aimDistance;
        shiftDelta = Range.clip(shiftDelta, -10.0, 10.0); // limit adjust distance to +-7.0 inch


        if (sensorDist > 300.0 ) {
            shiftDelta = 3.0;
        }

        if (aimDistance > 10) // wall distance is bigger than 10, robot need move to -x direction.
        {
            shiftDelta = -shiftDelta;
        }

        Logging.log("drive pose before distance average number");
        Logging.log("before adjust, sensor distance = %2f, shift delta = %2f", sensorDist, shiftDelta);
        Logging.log(" X position = %2f, Y position = %2f , heading = %sf", drive.pose.position.x, drive.pose.position.y, Math.toDegrees(drive.pose.heading.log()));

        Actions.runBlocking(
                drive.actionBuilder(drive.pose)
                        .strafeToLinearHeading(new Vector2d(drive.pose.position.x + shiftDelta , drive.pose.position.y), initHeading) // adjust heading also.
                        .build()
        );
        Logging.log(" After adjust: X position = %2f, Y position = %2f , heading = %sf", drive.pose.position.x, drive.pose.position.y, Math.toDegrees(drive.pose.heading.log()));
        Logging.log("after adjust, sensor distance = %2f, aim distance = %2f ", distSensorID.getDistance(DistanceUnit.INCH), aimDistance);
    }
}
