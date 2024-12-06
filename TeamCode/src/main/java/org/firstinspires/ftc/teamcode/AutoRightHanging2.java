/* Copyright (c) 2023 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 *
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
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.Arrays;
import java.util.Collections;
import java.util.List;
import java.util.Vector;

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
 *      Servo motors:
 *          "FingerServo"
 *          "WristServo"
 *          "SwitchServo"
 *
 *      One cameras:
 *          "Webcam 1"
 */

@Autonomous(name="Right v2", group="Concept")
//@Disabled
public class AutoRightHanging2 extends LinearOpMode {
    /**
     * Robot Start location: "1" - right side; "-1" - left side.
     */
    public int leftOrRight = 1;

    // Declare OpMode members.
    private final ElapsedTime runtime = new ElapsedTime();
    private intakeUnit intake;
    private MecanumDrive drive;

    Pose2d newStartPose;

    /**
     * Set robot starting position, and blueOrRed, frontOrBack variables:
     * @param leftRight : the value of robot location in the field.
     *                      -1 for left, -1 for left,
     */
    private void setStartPoses(int leftRight) {
        // road runner variables
        newStartPose = new Pose2d((-6 * Params.HALF_MAT + Params.CHASSIS_LENGTH / 2),(-leftRight * Params.CHASSIS_HALF_WIDTH),Math.toRadians(180));
    }
    private DistanceSensor distSensor;

    private DistanceSensor distSensorPickup;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        Logging.log("Status - Initialized");

        setStartPoses(leftOrRight);

        // init drive with road runner
        drive = new MecanumDrive(hardwareMap, newStartPose);
        Params.startPose = newStartPose; // init storage pose.
        Params.currentPose = newStartPose;; // init storage pose

        intake = new intakeUnit(hardwareMap, "Arm", "Wrist", "Knuckle","Finger");
        //intake.resetArmEncoder();

        // set RunToPosition mode and set power for motors.
        intake.setWristModeRunToPosition(intake.getWristPosition());
        intake.setArmModeRunToPosition(intake.getArmPosition());


        intake.setFingerPosition(intake.FINGER_CLOSE);

        // you can use this as a regular DistanceSensor.
        distSensor = hardwareMap.get(DistanceSensor.class, "distanceHanging");
        distSensorPickup = hardwareMap.get(DistanceSensor.class, "distance pickup");

        while (!isStarted()) {
            sleep(10);
            telemetry.addData( "FTC 2024 - ", "Wait for starting ");

            telemetry.addData("deviceName", distSensor.getDeviceName() );

            telemetry.addData("range", String.format("%.01f in", distSensor.getDistance(DistanceUnit.INCH)));

            telemetry.addData("Arm", "position = %d", intake.getArmPosition());

            telemetry.addData(" ---- ", " ---  ");

            telemetry.addData("Arm calibration - -    ", Params.armCalibrated? "Pass!" :" Failed !!!!!");

            telemetry.update();
        }

        // bulk reading setting - auto refresh mode
        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);

        waitForStart();

        Logging.log("Before start wrist pos: %s", intake.getWristPosition());
        Logging.log("Before start finger pos: %2f", intake.getFingerPosition());
        Logging.log("Before start arm pos: %s", intake.getArmPosition());

        runtime.reset();

        // run until the end of the match (driver presses STOP)
        if (opModeIsActive()) {
            if (Params.armCalibrated) {
                autonomousCore();

                Logging.log("Autonomous time - total Run Time: " + runtime);
            } else {
                telemetry.addData("Arm calibration: ---", "need to be done before starting!");
                telemetry.update();
                sleep(4000);
            }
            Params.currentPose = drive.pose; // save current position
        }

    }

    private void autonomousCore() {

        Logging.log("Status - Start auto core");
        Logging.log("Auto start wrist pos: %s", intake.getWristPosition());

        //new stuff for 2024-2025 season
        //hang specimen
        Vector2d firstHighChamberPos = new Vector2d(-3.1 * Params.HALF_MAT, newStartPose.position.y);

        //grab
        Vector2d firstSamplePosition = new Vector2d(- 3.3 * Params.HALF_MAT, - 4.35 * Params.HALF_MAT);
        Vector2d secondSamplePosition = new Vector2d(- 3.3 * Params.HALF_MAT, firstSamplePosition.y - 10);

        Vector2d obsZone = new Vector2d(- 4 * Params.HALF_MAT, - 3 * Params.HALF_MAT);
        Vector2d hangSpecimenPos = new Vector2d(-3.2 * Params.HALF_MAT, firstHighChamberPos.y);
        Pose2d pickUpSpecimenPos = new Pose2d(- 3.3 * Params.HALF_MAT, - 3.8 * Params.HALF_MAT, Math.toRadians(180));
        Vector2d pickUpSpecimenPos1 = new Vector2d(- 3.3 * Params.HALF_MAT, - 3.2 * Params.HALF_MAT);
        Vector2d pickUpSpecimenPos2 = new Vector2d(- 3.3 * Params.HALF_MAT, - 3.2 * Params.HALF_MAT);
        Vector2d pickUpSpecimenPos3 = new Vector2d(- 3.3 * Params.HALF_MAT, - 3.2 * Params.HALF_MAT);

        List<Vector2d> pickUpSpecimen;
        pickUpSpecimen = Arrays.asList(pickUpSpecimenPos1, pickUpSpecimenPos2, pickUpSpecimenPos3);



        //TODO: fix power ramp
        if (leftOrRight == 1) { // right side auto
            //Go to position for arm flip and hang on high chamber
            Logging.log(" Start position: X position = %2f, Y position = %2f, Heading = %2f", drive.pose.position.x, drive.pose.position.y, Math.toDegrees(drive.pose.heading.log()));
            Actions.runBlocking(
                    drive.actionBuilder(newStartPose)
                            .afterTime(0.01, new armToReadyHangAct())
                            .strafeTo(firstHighChamberPos)
                            .build()
            );
            Logging.log("After ready hang pos wrist pos: %s", intake.getWristPosition());

            // adjust robot position by distance sensor. Temp comment out for saving time
            //adjustPosByDistanceSensor(Params.HIGH_CHAMBER_DIST);
            Logging.log("Distance sensor reading for hanging preload specimen: %2f", distSensor.getDistance(DistanceUnit.INCH));

            //hanging action
            intake.setArmPosition(intake.ARM_POS_HIGH_CHAMBER);
            sleep(300);

            //release specimen and lower arm to clear high chamber
            intake.setFingerPosition(intake.FINGER_OPEN);
            intake.setArmPosition(intake.ARM_POS_GRAB_SAMPLE_BACK + 400);
            intake.setKnucklePosition(intake.KNUCKLE_POS_HIGH_CHAMBER - 0.2);

            sleep(200);
            //Go to pick up first sample on mat
            Actions.runBlocking(
                    drive.actionBuilder(drive.pose)
                            .afterTime(1.6, new armToPickUpPos()) // lower arm during spline moving
                            .strafeToLinearHeading(firstSamplePosition, newStartPose.heading)//go to first sample position
                            .turnTo(newStartPose.heading) // fine adjust heading
                            .afterTime(0.01, new fingerCloseEnRouteAct())//grab first sample
                            .afterTime(0.1, new intakeAct(intake.ARM_POS_DROP_SAMPLE, intake.WRIST_POS_NEUTRAL, intake.KNUCKLE_POS_DROP_SAMPLE, true))//go back for drop sample
                            .waitSeconds(0.1) // wait finger close before moving to second sample
                            .strafeToLinearHeading(secondSamplePosition, newStartPose.heading)//go to second sample position
                            .waitSeconds(0.35) // waiting arm reaching position to drop off first sample.
                            .afterTime(0.01, new armToPickUpPos()) //arm to grab second sample when open finger
                            .afterTime(1.65, new fingerCloseEnRouteAct())//grab second sample
                            .afterTime(1.75, new intakeAct(intake.ARM_POS_DROP_SAMPLE, intake.WRIST_POS_NEUTRAL, intake.KNUCKLE_POS_DROP_SAMPLE, true))
                            // 3.0s is the total time for arm flipping from obz to pickup second sample,
                            // then flipping back to obz to drop off second sample.
                            .afterTime(3.0, new fingerOpenEnRouteAct())
                            .waitSeconds(2.5) // may need adjust according test results. increase it if the second sample hit the first specimen
                            .build()
            );

            Logging.log("after 2nd sample drop heading: %2f", Math.toDegrees(drive.pose.heading.log()));


            //loop for hanging specimen
            //for (int j = 2; j <= 4; j++) {
            int j = 0;
            for (Vector2d specimenPos : pickUpSpecimen) {
                j++;
                Logging.log("Move to pickup specimen # %s from Obz.", j);

                //pickup specimen
                Actions.runBlocking(
                        drive.actionBuilder(drive.pose)
                                .afterTime(0.6, new intakeAct(intake.ARM_POS_GRAB_SPECIMEN, intake.WRIST_POS_GRAB_SPECIMEN, intake.KNUCKLE_POS_PICKUP_SPECIMEN_ready, false))
                                //.strafeToLinearHeading(pickUpSpecimenPos.position, newStartPose.heading)
                                .strafeToLinearHeading(specimenPos, newStartPose.heading)
                                .turnTo(newStartPose.heading)
                                .afterTime(0.001, new logPos()) // log drive position when pickup specimen
                                .afterTime(0.01, new intakeAct(intake.ARM_POS_GRAB_SPECIMEN, intake.WRIST_POS_GRAB_SPECIMEN, intake.KNUCKLE_POS_PICKUP_SPECIMEN, false))
                                .build()
                );
                strafeToSpecimenPickup(0.9);
                Actions.runBlocking(
                        drive.actionBuilder(drive.pose)
                                .afterTime(0.01, new fingerCloseEnRouteAct())//grab specimen
                                .afterTime(0.2, new intakeAct(intake.ARM_POS_HIGH_CHAMBER_READY, intake.WRIST_POS_HIGH_CHAMBER, intake.KNUCKLE_POS_HIGH_CHAMBER - 0.2, true)) //specimen will hit submersible
                                .waitSeconds(0.8)
                                .afterTime(1.1, new armToReadyHangAct())//get arm in position ready for hanging specimen
                                .strafeToLinearHeading(new Vector2d(hangSpecimenPos.x, hangSpecimenPos.y - 1.2 * j), newStartPose.heading) // shift 1.2 inch for each specimen on high chamber
                                .build()
                );


                //adjust to sensor dist
                adjustPosByDistanceSensor(Params.HIGH_CHAMBER_DIST, 1);

                //put specimen on high chamber
                intake.setArmPosition(intake.ARM_POS_HIGH_CHAMBER);
                sleep(200);

                //release specimen and lower arm to clear high chamber
                intake.setFingerPosition(intake.FINGER_OPEN);
                intake.setArmPosition(intake.ARM_POS_GRAB_SAMPLE_BACK + 400);
                intake.setKnucklePosition(intake.KNUCKLE_POS_HIGH_CHAMBER - 0.2);
                sleep(200);
            }

            Logging.log("hanging specimen complete");

            // parking
            Actions.runBlocking(
                    drive.actionBuilder(drive.pose)
                            .afterTime(0.3, new intakeAct(intake.ARM_POS_DROP_SAMPLE, intake.WRIST_POS_NEUTRAL, intake.KNUCKLE_POS_DROP_SAMPLE, true))
                            .strafeToLinearHeading(obsZone, Math.toRadians(-135))
                            .build()
            );


            //sleep(18000);
            /* start for second specimen
            //back to observation zone for next specimen
            intake.setArmPosition(intake.ARM_POS_HIGH_CHAMBER - 700);
            intake.setKnucklePosition(intake.KNUCKLE_POS_HIGH_CHAMBER + 0.15);
            sleep(300);
            Actions.runBlocking(
                    drive.actionBuilder(drive.pose)
                            .setReversed(true)
                            .afterTime(1.0, new armToPickupSpecimen()) // get arm ready for second specimen pick up
                            .strafeToLinearHeading(specimenLineUpPos.position, specimenLineUpPos.heading) // spline to side wall for second specimen
                            //.strafeTo(new Vector2d(specimenLineUpPos.position.x, specimenLineUpPos.position.y))
                            //.turnTo(pickUpSpecimenPos.heading) // fine turn heading before pickup first specimen
                            .strafeToLinearHeading(pickUpSpecimenPos.position, specimenLineUpPos.heading)
                            .build()
            );

            Logging.log("after 2nd specimen pick up heading: %2f", Math.toDegrees(drive.pose.heading.log()));
            //pickup second specimen
            intake.setKnucklePosition(intake.KNUCKLE_POS_HIGH_CHAMBER - 0.032);
            sleep(100); //200
            intake.setFingerPosition(intake.FINGER_CLOSE);
            sleep(100); // 200
            intake.setArmPosition(intake.ARM_POS_BACK);
            sleep(200);

            //strafe to hanging second specimen
            Actions.runBlocking(
                    drive.actionBuilder(drive.pose)
                            // forward arm during strafe
                            //.afterTime(1.9, new intakeAct(intake.ARM_POS_HIGH_CHAMBER, intake.WRIST_POS_HIGH_CHAMBER, 0)) // forward arm to hang first specimen
                            //.strafeToLinearHeading(hangSpecimenPos, 0)
                            //.strafeToLinearHeading(new Vector2d(hangSpecimenPos.x - 0.1 * Params.HALF_MAT, hangSpecimenPos.y + Params.CHASSIS_HALF_WIDTH), 0)
                            .strafeToLinearHeading(new Vector2d(hangSpecimenPos.x - 0.05 * Params.HALF_MAT, hangSpecimenPos.y - Params.CHASSIS_HALF_WIDTH + 8.0), 0)
                            .build()
            );

            // back arm after hanging the second specimen\=
            // -            sensorDist = distSensor.getDistance(DistanceUnit.INCH);
            shiftDelta = sensorDist - Params.HIGH_CHAMBER_DIST;
            shiftDelta = (shiftDelta > 2)? 2 : ((shiftDelta < -2)? -2 : shiftDelta);
            Logging.log("drive pose before distance");
            Logging.log(" X position = %2f, Y position = %2f ", drive.pose.position.x, drive.pose.position.y);
            Logging.log("third specimen sensor dist = %2f, shift delta = %2f", sensorDist, shiftDelta);

            Actions.runBlocking(
                    drive.actionBuilder(drive.pose)
                            .strafeToConstantHeading(new Vector2d(drive.pose.position.x + shiftDelta , drive.pose.position.y))
                            .build()
            );
            Logging.log("after adjust X position = %2f, Y position = %2f ", drive.pose.position.x, drive.pose.position.y);

            sleep(800);
            intake.setFingerPosition(intake.FINGER_OPEN);
            intake.setArmPosition(intake.ARM_POS_HIGH_CHAMBER - 700);
            intake.setKnucklePosition(intake.KNUCKLE_POS_HIGH_CHAMBER + 0.15);

            // back to obs zone for parking
            Actions.runBlocking(
                    drive.actionBuilder(drive.pose)
                            .setReversed(true) // for parking
                            .strafeTo(parkObz)
                            .build()
            );
            intake.setArmPosition(intake.ARM_POS_GRAB_SAMPLE);
            intake.setFingerPosition(intake.FINGER_SPECIMEN_CLOSE);
            intake.setKnucklePosition(intake.KNUCKLE_POS_HANGING);
            */
        }
        Logging.log("X position = %2f, Y position = %2f", drive.pose.position.x, drive.pose.position.y);
    }

    //action for arm flip to hang
    private class armToReadyHangAct implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            intake.setArmPosition(intake.ARM_POS_HIGH_CHAMBER_READY);
            intake.setWristPosition(intake.WRIST_POS_HIGH_CHAMBER);
            intake.setKnucklePosition(intake.KNUCKLE_POS_HIGH_CHAMBER);
            return false;
        }
    }

    //action for arm to pick up neutral sample
    private class armToPickUpPos implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            intake.setKnucklePosition(intake.KNUCKLE_POS_PICKUP_SAMPLE_BACK);
            sleep(100);
            intake.setArmPosition(intake.ARM_POS_GRAB_SAMPLE_BACK);
            intake.setWristPosition(intake.WRIST_POS_GRAB_SAMPLE_BACK);
            intake.setFingerPosition(intake.FINGER_OPEN);
            return false;
        }
    }


    private class fingerOpenEnRouteAct implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            intake.setFingerPosition(intake.FINGER_OPEN);
            return false;
        }
    }

    private class fingerCloseEnRouteAct implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            intake.setFingerPosition(intake.FINGER_CLOSE);
            return false;
        }
    }

    //action for arm to pick up specimen
    private class armToPickupSpecimen implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            intake.setArmPosition(intake.ARM_POS_GRAB_SPECIMEN);
            intake.setWristPosition(intake.WRIST_POS_GRAB_SPECIMEN);
            return false;
        }
    }

    private class armToBackAct implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            intake.setArmPosition(intake.ARM_POS_BACK);
            intake.setWristPosition(intake.WRIST_POS_HIGH_CHAMBER);
            return false;
        }
    }

    private class logPos implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            Logging.log("X position = %2f, Y position = %2f, Heading = %2f", drive.pose.position.x, drive.pose.position.y, Math.toDegrees(drive.pose.heading.log()));

            telemetry.addData("pickup distance range = ", String.format("%.01f in", distSensorPickup.getDistance(DistanceUnit.INCH)));


            return false;
        }
    }

    //action for arm, wrist, finger position control during driving
    private class intakeAct implements Action {
        public intakeAct(double armPos, double wristPos, double knucklePos, boolean fingerPos) {
            arm = (int)armPos;
            wrist = (int)wristPos;
            knuckle = knucklePos;
            finger = fingerPos;
        }

        private final int arm;
        private final int wrist;
        private final double knuckle;
        private final boolean finger;

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {

            if (arm < 0) {
                intake.setArmPosition(arm);
            }

            if (wrist >= 0) {
                intake.setWristPosition(wrist);
            }

            if (knuckle > 0) {
                intake.setKnucklePosition(knuckle);
            }

            if (finger) {
                intake.fingerServoClose();
            } else {
                intake.fingerServoOpen();
            }
            return false;
        }
    }

    private void updateProfileAccel(boolean slowMode) {
        if (slowMode) {
            MecanumDrive.PARAMS.minProfileAccel = -25;
            MecanumDrive.PARAMS.maxProfileAccel = 40;
        } else {
            MecanumDrive.PARAMS.minProfileAccel = -35;
            MecanumDrive.PARAMS.maxProfileAccel = 60;
        }
    }

    private void adjustPosByDistanceSensor(double aimDistance, int distSensorID) { //distSensorID: 1 for hanging sensor, 2 for pickup sensor
        double sensorDist = 0.0;
        int repeatTimes = 5;

        //DistanceSensor usedDistSensor = (distSensorID == 1)? distSensor : distSensorPickup ;

        for (int i = 1; i <= repeatTimes; i++)
        {
            double sensorReading = (distSensorID == 1)? distSensor.getDistance(DistanceUnit.INCH) : distSensorPickup.getDistance(DistanceUnit.INCH) ;
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
                        .strafeToLinearHeading(new Vector2d(drive.pose.position.x + shiftDelta , drive.pose.position.y), newStartPose.heading) // adjust heading also.
                        .build()
        );
        Logging.log(" After adjust: X position = %2f, Y position = %2f , heading = %sf", drive.pose.position.x, drive.pose.position.y, Math.toDegrees(drive.pose.heading.log()));
        Logging.log("after adjust, sensor distance = %2f, aim distance = %2f ", distSensor.getDistance(DistanceUnit.INCH), aimDistance);
    }

    // strafe robot left or right to detect specimen by distance sensor.
    private void strafeToSpecimenPickup(double strafePower) {
        double sensorDist = 0.0;
        double robotPosY = drive.pose.position.y;

        sensorDist = distSensor.getDistance(DistanceUnit.INCH);

        // continue strafing when sensor does not detect specimen and moving distance less than 6 inch
        while ((sensorDist > 20.0) && (Math.abs(drive.pose.position.y - robotPosY) < 6.0))
        {
            // give power to strafe left/right
            drive.setDrivePowers(new PoseVelocity2d(
                    new Vector2d(
                            0.0,
                            strafePower // set strafe power and strafe direction
                    ),
                    0.0
            ));
            sensorDist = distSensor.getDistance(DistanceUnit.INCH);
            Logging.log("distance sensor reading number = %2f", sensorDist);
        }
        // stop robot while sensor detect the specimen
        drive.setDrivePowers(new PoseVelocity2d(
                new Vector2d(
                        0.0,
                        0.0
                ),
                0.0
        ));

        Logging.log("drive pose after strafe.");
        Logging.log(" X position = %2f, Y position = %2f , heading = %sf", drive.pose.position.x, drive.pose.position.y, Math.toDegrees(drive.pose.heading.log()));

        //adjust y position
        adjustPosByDistanceSensor(Params.SPECIMEN_PICKUP_DIST, 2);

        Logging.log("after adjust, sensor distance = %2f ", distSensor.getDistance(DistanceUnit.INCH));
    }

}