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

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Rotation2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
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

@TeleOp(name="Teleop 2025", group="Concept")
//@Disabled
public class Teleop2025 extends AutoRightHanging2 {
    // chassis
    //MecanumDrive drive;

    //claw and arm unit
    //private intakeUnit intake;

    //private DistanceSensor distSensorB;
    //private DistanceSensor distSensorF;

    int specimenCount = 0;//counter used to update specimen hanging position
    int specimenShiftMax = 7; //shift 2 inch for each specimen hanging
    double specimenShiftInch = -7.0; // shift specimen to 7 inch right (-y) after hanging on high chamber
    double specimenShiftEach = 1.6; // shift hanging place shift in Y direct
    // debug flags, turn it off for formal version to save time of logging
    boolean debugFlag = true;

    double initHeading = Params.startPose.heading.toDouble();

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");

        double imu_heading;
        GamePadButtons gpButtons = new GamePadButtons();

        updateProfileAccel(false); //more faster driving

        drive = new MecanumDrive(hardwareMap, Params.currentPose);
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        imu = drive.lazyImu.get();

        Vector2d pickupSpecimen = new Vector2d(Params.pickupSpecimenX, -4.0 * Params.HALF_MAT);
        pickupSpecimenLineup = new Vector2d(Params.pickupSpecimenLineupX + 1.0, -4.0 * Params.HALF_MAT);
        hangSpecimenPos = new Vector2d(Params.hangingSpecimenX - 3.0, -0.2 * Params.HALF_MAT);


        intake = new intakeUnit(hardwareMap, "Arm", "Wrist",
                "Knuckle", "Finger");

        // set RunToPosition mode and set power for motors.
        intake.setWristModeRunToPosition(intake.getWristPosition());
        intake.setArmModeRunToPosition(intake.getArmPosition());


        // you can use this as a regular DistanceSensor.
        distSensorB = hardwareMap.get(DistanceSensor.class, "distanceB");
        distSensorF = hardwareMap.get(DistanceSensor.class, "distanceF");

        // bulk reading setting - auto refresh mode
        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

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

            // reset wrist motor position
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

            // hanging specimen action, lower arm and shift left several inches. GAMEPAD1.left_bumper
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

            // cycling specimen from wall, gamepad1.y
            if (gpButtons.SpecimenCycleWall) {
                specimenCount = 0;
                // Reset IMU if it has not been reset at the beginning of autonomous
                if (!Params.imuReseted) {
                    imu.resetYaw();
                    Params.imuReseted = true;
                }
                imu_heading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

                drive.pose = new Pose2d(pickupSpecimen, imu_heading - Math.PI); // pickup Position

                for (int i = 0; i < specimenShiftMax; i++) { // cycling specimen
                    // close finger
                    intake.setFingerPosition(intake.FINGER_CLOSE);
                    sleep(150);

                    imu_heading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

                    Logging.log("teleop cycle # %s specimen pick up before update imu heading: %2f dead wheel heading: %2f", i, Math.toDegrees(imu_heading), Math.toDegrees(drive.pose.heading.log()));
                    //drive.pose = new Pose2d(drive.pose.position, imu_heading - Math.PI); // pickup Position
                    Logging.log("teleop cycle # %s specimen pick up after replace by imu heading: %2f dead wheel heading: %2f", i, Math.toDegrees(imu_heading), Math.toDegrees(drive.pose.heading.log()));

                    Actions.runBlocking(
                            drive.actionBuilder(drive.pose)
                                    .turnTo(headingAngleCorrection) // fine correct heading
                                    .build()
                    );
                    imu_heading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
                    Logging.log("teleop cycle # %s specimen pick up after update imu heading: %2f dead wheel heading: %2f", i, Math.toDegrees(imu_heading), Math.toDegrees(drive.pose.heading.log()));

                    intake.setKnucklePosition(intake.KNUCKLE_POS_LIFT_FROM_WALL);
                    sleep(100); // wait knuckle lift the specimen

                    if (gamepad1.x) {
                        break;
                    } // quit specimen cycling if gamepad1.x

                    Actions.runBlocking(
                            drive.actionBuilder(drive.pose)
                                    // flip arm to high chamber position, back knuckle to avoid hitting chamber during strafing
                                    .afterTime(0.1, new intakeAct(intake.ARM_POS_HIGH_CHAMBER_READY, intake.WRIST_BACK, Params.NO_CATION, Params.NO_CATION))
                                    //.afterTime(1., new intakeAct(Params.NO_CATION, Params.NO_CATION, intake.KNUCKLE_POS_HIGH_CHAMBER, Params.NO_CATION))
                                    // shift 1.5 inch for each specimen on high chamber
                                    .strafeToLinearHeading(new Vector2d(hangSpecimenPos.x, hangSpecimenPos.y + specimenShiftEach * specimenCount), initHeading)
                                    // get knuckle ready for hanging
                                    //.afterTime(0.001, new intakeAct(Params.NO_CATION, Params.NO_CATION, intake.KNUCKLE_POS_HIGH_CHAMBER, Params.NO_CATION))
                                    .turnTo(headingAngleCorrection) // fine correct heading
                                    .build()
                    );

                    intake.setKnucklePosition(intake.KNUCKLE_POS_HIGH_CHAMBER);

                    imu_heading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

                    Logging.log("teleop cycle # %s specimen hang before update imu heading: %2f dead wheel heading: %2f", i, Math.toDegrees(imu_heading), Math.toDegrees(drive.pose.heading.log()));
                    //drive.pose = new Pose2d(drive.pose.position, imu_heading - Math.PI); // pickup Position
                    Actions.runBlocking(
                            drive.actionBuilder(drive.pose)
                                    .turnTo(headingAngleCorrection) // fine correct heading
                                    .build()
                    );
                    imu_heading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
                    Logging.log("teleop cycle # %s specimen hang after update imu heading: %2f dead wheel heading: %2f", i, Math.toDegrees(imu_heading), Math.toDegrees(drive.pose.heading.log()));

                    //adjust pos using distance sensor
                    adjustPosByDistanceSensor(Params.HIGH_CHAMBER_DIST, distSensorB, drive);

                    if (gamepad1.x) {
                        break;
                    } // quit specimen cycling if gamepad1.x

                    if (specimenCount <= specimenShiftMax) {
                        specimenCount++;//update specimen pos
                    }

                    // hanging specimen
                    intake.setArmPosition(intake.ARM_POS_HIGH_CHAMBER);
                    sleep(sleepTimeForHangingSpecimen);

                    intake.setArmPosition(intake.ARM_POS_HIGH_CHAMBER_MOVING_SPECIMEN);
                    sleep(200);

                    if (gamepad1.x) {
                        break;
                    } // quit specimen cycling if gamepad1.x

                    // if it is the last specimen, quit after hanging without y shift
                    if (specimenShiftMax -1 == i) {
                        intake.fingerServoOpen();
                        intake.setKnucklePosition(intake.KNUCKLE_POS_AWAY_FROM_SUBMERSIBLE);
                        //  back 10 inch before stop
                        Actions.runBlocking(
                                drive.actionBuilder(drive.pose)
                                        .strafeTo(new Vector2d(drive.pose.position.x - 10, drive.pose.position.y)) // line up
                                        .build()
                        );
                        break;
                    }

                    //  y shift
                    Actions.runBlocking(
                            drive.actionBuilder(drive.pose)
                                    .strafeTo(new Vector2d(drive.pose.position.x, drive.pose.position.y + specimenShiftInch)) // line up
                                    .build()
                    );
                    Logging.log(" after y shift pos: X position = %2f, Y position = %2f , heading = %sf", drive.pose.position.x, drive.pose.position.y, Math.toDegrees(drive.pose.heading.log()));

                    //return to wall to pickup next specimen
                    intake.fingerServoOpen();
                    intake.setKnucklePosition(intake.KNUCKLE_POS_AWAY_FROM_SUBMERSIBLE);
                    sleep(200);

                    if (gamepad1.x) {
                        break;
                    } // quit specimen cycling if gamepad1.x

                    Actions.runBlocking(
                            drive.actionBuilder(drive.pose)
                                    .afterTime(0.1, new intakeAct(intake.ARM_POS_GRAB_SPECIMEN_WALL, intake.WRIST_POS_NEUTRAL, Params.NO_CATION, Params.NO_CATION))
                                    .afterTime(0.6, new intakeAct(Params.NO_CATION, intake.WRIST_POS_NEUTRAL, intake.KNUCKLE_POS_PICKUP_SPECIMEN_WALL, Params.NO_CATION))
                                    .strafeToLinearHeading(pickupSpecimenLineup, initHeading) // line up
                                    .turnTo(headingAngleCorrection) // fine correct heading
                                    .build()
                    );

                    // adjust wall distance by distance sensor
                    adjustPosByDistanceSensor(Params.SPECIMEN_PICKUP_DIST, distSensorF, drive);
                    if (gamepad1.x) {
                        break;
                    } // quit specimen cycling if gamepad1.x
                }
            }

            // pickup specimen and move to high chamber
            if ( gamepad1.right_trigger > 0) {
                // Reset IMU if it has not been reset at the beginning of autonomous
                if (!Params.imuReseted) {
                    imu.resetYaw();
                    Params.imuReseted = true;
                }
                imu_heading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
                drive.pose = new Pose2d(pickupSpecimen, imu_heading - Math.PI);
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
                adjustPosByDistanceSensor(Params.HIGH_CHAMBER_DIST, distSensorB, drive);
            }

            // pickup specimen from wall positions, gamepad2.right_trigger
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

                telemetry.addData("Arm", "position = %s", intake.getArmPosition());

                telemetry.addData(" ", " ");

                telemetry.addData("heading", " %.3f", Math.toDegrees(drive.pose.heading.log()));

                telemetry.addData("heading - IMU ", " %.3f", imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES) + 180.0);

                telemetry.addData("location", " %s", drive.pose.position.toString());

                telemetry.addData("specimens hanged:", " %s", specimenCount);

                telemetry.addData(" --- ", " --- ");

                telemetry.addData("back distance range", " %2f", distSensorB.getDistance(DistanceUnit.INCH));

                telemetry.addData("front distance range", " %2f", distSensorF.getDistance(DistanceUnit.INCH));

                telemetry.update(); // update message at the end of while loop
            }

        }
        //intake.armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        // The motor stop on their own but power is still applied. Turn off motor.
    }

}
