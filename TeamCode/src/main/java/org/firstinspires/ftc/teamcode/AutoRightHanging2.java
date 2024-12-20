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
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.Arrays;
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
 *      Servo motors:
 *          "FingerServo"
 *          "WristServo"
 *          "SwitchServo"
 *
 *      One cameras:
 *          "Webcam 1"
 */

@Autonomous(name=" A - 4 specimens", group="Concept")
//@Disabled
public class AutoRightHanging2 extends LinearOpMode {
    /**
     * Robot Start location: "1" - right side; "-1" - left side.
     */
    public int leftOrRight = 1; // -1: left; 1: right
    public int sleepTimeForHangingSpecimen = 500;
    public int knuckleSleepTime = 150;
    public int sleepHighChamber = 2;

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
        newStartPose = new Pose2d((-6 * Params.HALF_MAT + Params.CHASSIS_LENGTH / 2), (-leftRight * (Params.CHASSIS_HALF_WIDTH - 2.0)), Math.toRadians(180));
    }
    private DistanceSensor distSensorB;
    private DistanceSensor distSensorF;


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

        // set RunToPosition mode and set power for motors.
        intake.setWristModeRunToPosition(intake.getWristPosition());
        intake.setArmModeRunToPosition(intake.getArmPosition());

        intake.setIntakeAutoInit();

        // you can use this as a regular DistanceSensor.
        distSensorB = hardwareMap.get(DistanceSensor.class, "distanceB");
        distSensorF = hardwareMap.get(DistanceSensor.class, "distanceF");

        while (!isStarted()) {
            sleep(10);
            telemetry.addData( "FTC 2024 - ", "Wait for starting ");

            telemetry.addData("deviceName", distSensorB.getDeviceName() );
            telemetry.addData("back range", " = %2f", distSensorF.getDistance(DistanceUnit.INCH));


            telemetry.addData("deviceName", distSensorF.getDeviceName() );
            telemetry.addData("front range", " = %2f", distSensorF.getDistance(DistanceUnit.INCH));

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
        Vector2d firstSamplePosition = new Vector2d(- 3.2 * Params.HALF_MAT, - 4.1 * Params.HALF_MAT);

        // adjust x for second sample a little bit according to testing
        Vector2d secondSamplePosition = new Vector2d(- 3.2 * Params.HALF_MAT - 1, firstSamplePosition.y - 10.5);

        Vector2d obsZone = new Vector2d(- 4.3 * Params.HALF_MAT, - 3.5 * Params.HALF_MAT);
        Vector2d hangSpecimenPos;

        // avoid program crush when calling turnTo() function for fine heading correction
        double headingAngleCorrection = Math.toRadians(180.0 - 0.1);

        //wall positions
        Vector2d specimenWallLineUp = new Vector2d(Params.pickupSpecimenLineupX, - 3.8 * Params.HALF_MAT);


        if (leftOrRight == 1) { // right side auto
            //Go to position for arm flip and hang on high chamber
            Logging.log(" Start position: X position = %2f, Y position = %2f, Heading = %2f", drive.pose.position.x, drive.pose.position.y, Math.toDegrees(drive.pose.heading.log()));
            Actions.runBlocking(
                    drive.actionBuilder(newStartPose)
                            .afterTime(0.01, new armToReadyHangAct())
                            .strafeTo(firstHighChamberPos)
                            .build()
            );

            // adjust robot position by distance sensor. Temp comment out for saving time
            adjustPosByDistanceSensor(Params.HIGH_CHAMBER_DIST, distSensorB);

            // Adjust hanging specimen position.x according to the preload specimen hanging position
            // after adjusted by distance sensor
            hangSpecimenPos = new Vector2d(drive.pose.position.x, firstHighChamberPos.y);
            Params.hangingSpecimenX = drive.pose.position.x; // restore hang position for teleop.
            Logging.log("Distance sensor reading for hanging preload specimen: %2f", distSensorB.getDistance(DistanceUnit.INCH));
            Logging.log(" hangSpecimenPos: X position = %2f, Y position = %2f", hangSpecimenPos.x, hangSpecimenPos.y);

            //hanging action
            intake.setArmPosition(intake.ARM_POS_HIGH_CHAMBER);
            sleep(400);

            //release specimen and lower arm to clear high chamber
            intake.setFingerPosition(intake.FINGER_OPEN);
            intake.setArmPosition(intake.ARM_POS_HIGH_CHAMBER_MOVING_SPECIMEN);
            intake.setKnucklePosition(intake.KNUCKLE_POS_AWAY_FROM_SUBMERSIBLE);

            sleep(200);
            //Go to pick up first sample on mat
            Actions.runBlocking(
                    drive.actionBuilder(drive.pose)
                            .afterTime(1.3, new armToPickUpPos()) // lower arm during spline moving
                            .strafeToLinearHeading(firstSamplePosition, newStartPose.heading)//go to first sample position
                            .turnTo(headingAngleCorrection) // fine adjust heading
                            .build()
            );
                            //.afterTime(0.25, new intakeAct(intake.FINGER_CLOSE))//grab first sample
                            // flip arm after 150 ms when finger closed.
                            //.afterTime(0.4, new intakeAct(intake.ARM_POS_DROP_SAMPLE, intake.WRIST_POS_NEUTRAL, intake.KNUCKLE_SIZE_CONSTRAINT, intake.FINGER_CLOSE))//go back for drop sample
                            //.waitSeconds(0.4) // wait finger close before moving to second sample
            intake.fingerServoClose();
            sleep(150);
            intake.setKnucklePosition(intake.KNUCKLE_SIZE_CONSTRAINT);
            intake.setArmPosition(intake.ARM_POS_DROP_SAMPLE);
            intake.setWristPosition(intake.WRIST_POS_NEUTRAL);

            // strafe to second sample during flip arm to drop first sample
            Actions.runBlocking(
                    drive.actionBuilder(drive.pose)
                            .strafeToLinearHeading(secondSamplePosition, newStartPose.heading)//go to second sample position
                            .turnTo(headingAngleCorrection)
                            .build()
            );
            intake.setKnucklePosition(intake.KNUCKLE_POS_PICKUP_SAMPLE_BACK);
            sleep(500); // waiting arm reaching position to drop off first sample.
            intake.setArmPosition(intake.ARM_POS_GRAB_SAMPLE_BACK);
            intake.setWristPosition(intake.WRIST_BACK);
            intake.fingerServoOpen();
            sleep(200);
            intake.setKnucklePosition(intake.KNUCKLE_SIZE_CONSTRAINT); // flip knuckle before lift arm due to size limitation

            sleep(1050); // wait arm flip back to pickup second sample
            intake.setKnucklePosition(intake.KNUCKLE_POS_PICKUP_SAMPLE_BACK); // flip knuckle before lift arm due to size limitation
            sleep(400);
            intake.fingerServoClose();
            sleep(150); // wait finger close to grab second sample
            intake.setKnucklePosition(intake.KNUCKLE_SIZE_CONSTRAINT); // flip knuckle before lift arm due to size limitation
            sleep(150);
           // moving during arm flip for wall case, starting flip arm here
            intake.setArmPosition(intake.ARM_POS_GRAB_SPECIMEN_WALL); // flip arm to grab specimen position before drop off second sample
            intake.setWristPosition(intake.WRIST_POS_NEUTRAL);
            sleep(850);
            intake.setKnucklePosition(intake.KNUCKLE_POS_PICKUP_SPECIMEN_WALL);
            // waiting arm flip to front to drop off second specimen
            sleep(150);

            Logging.log("after 2nd sample drop heading: %2f", Math.toDegrees(drive.pose.heading.log()));

            // pick up specimen from wall, loop to hang specimen
            for (int j = 1; j < 4; j++) {

                Logging.log(" Start moving to wall to pickup # %s specimen. ", j);

                Actions.runBlocking(
                        drive.actionBuilder(drive.pose)
                                .afterTime(0.1, new intakeAct(intake.ARM_POS_GRAB_SPECIMEN_WALL, intake.WRIST_POS_NEUTRAL, intake.KNUCKLE_POS_PICKUP_SPECIMEN_WALL, Params.NO_CATION))
                                .strafeToLinearHeading(specimenWallLineUp, newStartPose.heading) // line up
                                // drop off second sample
                                .turnTo(headingAngleCorrection) // fine correct heading
                                .build()
                );
                intake.fingerServoOpen(); // drop off the second sample
                // using distance sensor to move robot to correct position for pickup specimen from wall
                adjustPosByDistanceSensor(Params.SPECIMEN_PICKUP_DIST, distSensorF);
                if (j == 1) { // only restore once
                    Params.pickupSpecimenX = drive.pose.position.x; // restore X position for teleop.
                    Params.pickupSpecimenLineupX = drive.pose.position.x + 2.5; // leave 2.5 inch gap to avoid hitting the specimen
                    // adjust wall pickup position.x according to distance sensor to speedup next pickup.
                    specimenWallLineUp = new Vector2d(Params.pickupSpecimenLineupX, -3.8 * Params.HALF_MAT);
                    Logging.log(" After adjust of specimenWallLineUp: X position = %2f, Y position = %2f", specimenWallLineUp.x, specimenWallLineUp.y);
                }
                // start picking up actions
                intake.fingerServoClose();
                sleep(150); // waiting finger close
                intake.setKnucklePosition(intake.KNUCKLE_POS_LIFT_FROM_WALL);
                sleep(100); // wait knuckle lift the specimen

                Logging.log(" Start moving to high chamber to hang # %s specimen. ", j);
                // strafe to high chamber for hanging specimen
                Actions.runBlocking(
                        drive.actionBuilder(drive.pose)
                                // flip arm to high chamber position, back knuckle to avoid hitting chamber during strafing
                                .afterTime(0.6, new intakeAct(intake.ARM_POS_HIGH_CHAMBER_READY, intake.WRIST_BACK, Params.NO_CATION, Params.NO_CATION))
                                // shift 1.5 inch for each specimen on high chamber
                                .strafeToLinearHeading(new Vector2d(hangSpecimenPos.x, hangSpecimenPos.y - 1.2 * j), newStartPose.heading)
                                // get knuckle ready for hanging
                                .afterTime(0.001, new intakeAct(Params.NO_CATION, Params.NO_CATION, intake.KNUCKLE_POS_HIGH_CHAMBER, Params.NO_CATION))
                                .turnTo(headingAngleCorrection) // fine correct heading
                                .build()
                );

                //adjust pos using distance sensor
                adjustPosByDistanceSensor(Params.HIGH_CHAMBER_DIST, distSensorB);

                //hang specimen
                intake.setArmPosition(intake.ARM_POS_HIGH_CHAMBER);
                sleep(400);
                intake.setArmPosition(intake.ARM_POS_HIGH_CHAMBER_MOVING_SPECIMEN);
                intake.fingerServoOpen(); // release specimen
                intake.setKnucklePosition(intake.KNUCKLE_POS_AWAY_FROM_SUBMERSIBLE);
                sleep(200);
            }

            Logging.log("hanging specimen complete, start parking");

            // parking
            Actions.runBlocking(
                    drive.actionBuilder(drive.pose)
                            .afterTime(0.2, new intakeAct(intake.ARM_POS_OBZ_PARKING, intake.WRIST_POS_NEUTRAL, intake.KNUCKLE_POS_LIFT_FROM_WALL, intake.FINGER_CLOSE))
                            .strafeToLinearHeading(obsZone, Math.toRadians(-130))
                            .build()
            );
            intake.setKnucklePosition(intake.KNUCKLE_POS_HIGH_CHAMBER);
        }
    } // end auto_core()

    //action for arm flip to hang specimen on high chamber
    private class armToReadyHangAct implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            intake.setArmPosition(intake.ARM_POS_HIGH_CHAMBER_READY);
            intake.setWristPosition(intake.WRIST_BACK);
            intake.setKnucklePosition(intake.KNUCKLE_POS_HIGH_CHAMBER);
            return false;
        }
    }

    //action for arm to pick up neutral sample during auto
    private class armToPickUpPos implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            intake.setKnucklePosition(intake.KNUCKLE_POS_PICKUP_SAMPLE_BACK);
            sleep(100);
            intake.setArmPosition(intake.ARM_POS_GRAB_SAMPLE_BACK);
            intake.setWristPosition(intake.WRIST_BACK);
            intake.setFingerPosition(intake.FINGER_OPEN);
            return false;
        }
    }

    private class logPos implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            Logging.log("pick up specimen X position = %2f, Y position = %2f, Heading = %2f", drive.pose.position.x, drive.pose.position.y, Math.toDegrees(drive.pose.heading.log()));

            return false;
        }
    }

    //action for arm, wrist, finger position control during driving
    class intakeAct implements Action {
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

        // moving arm only with input a integer value.
        public intakeAct(int armPos) {
            arm = armPos;
        }

        // moving finger only with a double value.
        public intakeAct(double fingerPos) {
            finger = fingerPos;
        }

        private int arm = Params.NO_CATION;
        private int wrist = Params.NO_CATION;
        private double knuckle = Params.NO_CATION;
        private double finger = Params.NO_CATION;

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

    //action to set arm and wrist position to pick up from sub
    class armToPickUpWallPos implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            pickupFromWallActions();

            return false;
        }
    }

    void adjustPosByDistanceSensor(double aimDistance, DistanceSensor distSensorID) {
        double sensorDist = 0.0;
        int repeatTimes = 5;

        for (int i = 1; i <= repeatTimes; i++)
        {
            double sensorReading = distSensorID.getDistance(DistanceUnit.INCH);
            sensorDist = sensorDist + sensorReading;
            Logging.log("distance sensor reading repetition # %s reading number = %2f", i, sensorReading);
            sleep(2);
        }
        sensorDist = sensorDist / repeatTimes;

        double shiftDelta = sensorDist - aimDistance;

        // something wrong when reading distance sensor
        if (sensorDist > 50.0 ) {
            shiftDelta = 3.0;
        }

        if (aimDistance > 10) // wall distance is bigger than 10, robot need move to -x direction.
        {
            shiftDelta = -shiftDelta;
        }
        shiftDelta = Range.clip(shiftDelta, -10.0, 10.0); // limit adjust distance to +-10.0 inch
        Logging.log("drive pose before distance average number");
        Logging.log("before adjust, sensor distance = %2f, shift delta = %2f", sensorDist, shiftDelta);
        Logging.log(" X position = %2f, Y position = %2f , heading = %sf", drive.pose.position.x, drive.pose.position.y, Math.toDegrees(drive.pose.heading.log()));

        // adjust when shiftDelta big enough than 0.2 inch to same some time
        if (Math.abs(shiftDelta) > 0.2) {
            Actions.runBlocking(
                    drive.actionBuilder(drive.pose)
                            .strafeToLinearHeading(new Vector2d(drive.pose.position.x + shiftDelta, drive.pose.position.y), newStartPose.heading) // adjust heading also.
                            .build()
            );
            Logging.log(" After adjust: X position = %2f, Y position = %2f , heading = %sf", drive.pose.position.x, drive.pose.position.y, Math.toDegrees(drive.pose.heading.log()));
            Logging.log("after adjust, sensor distance = %2f, aim distance = %2f ", distSensorID.getDistance(DistanceUnit.INCH), aimDistance);
        }
        else {
            Logging.log("No adjust by distance sensor ");
        }
    }

    /**
     * Set intake positions for picking up specimen from wall
     */
    void pickupFromWallActions() {
        intake.fingerServoOpen();
        sleep(knuckleSleepTime);
        intake.setArmPosition(intake.ARM_POS_GRAB_SPECIMEN_WALL);
        intake.setKnucklePosition(intake.KNUCKLE_POS_PICKUP_SPECIMEN_WALL);
        sleep(knuckleSleepTime);
        sleep(knuckleSleepTime); // double sleep time since we have enough spare time here
        intake.setWristPosition(intake.WRIST_POS_NEUTRAL);
    }

    void updateProfileAccel(boolean slowMode) {
        if (slowMode) {
            MecanumDrive.PARAMS.minProfileAccel = -25;
            MecanumDrive.PARAMS.maxProfileAccel = 40;
        } else {
            MecanumDrive.PARAMS.minProfileAccel = -40;
            MecanumDrive.PARAMS.maxProfileAccel = 60;
        }
    }
}