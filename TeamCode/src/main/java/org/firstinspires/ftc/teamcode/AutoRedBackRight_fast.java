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
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

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

@Autonomous(name="Red Back Right - 10 sec", group="Concept")
@Disabled
public class AutoRedBackRight_fast extends LinearOpMode {

    private final double NO_ACT = 100000;

    double BUCKET_SHIFT = 2.0;

    double whiteDropShift;
    /** 1 for Red Front, 2 for Red back, 3 for Blue Front, and 4 for Blue back
     */
    public int startLoc = 1;
    /** blue: 1; red: -1
     */
    private int blueOrRed = 1; // blue: 1; red: -1
    /** front: 1; back -1
     */
    private int frontOrBack = 1; // front: 1; back -1
    /**
     * Parking location: "1" - right of backdrop; "-1" - left of backdrop.
     */
    public int leftOrRight = -1; // -1) means te robot parks near the blue side no matter the autonomous, right(1) is the opposite
    /**
     * blue: 1,2,3; red: 4,5,6
     */
    private int desiredTagNum = 0; // blue: 1,2,3; red: 4,5,6
    private int desiredTagWhite = 0; // blue: 1,2,3; red: 4,5,6

    private int checkStatus = 1;

    double dropOffAngle = -Math.PI/2 - 0.04;// * blueOrRed; // 0.04 for turn orientation control
    double pickupAngle1 = Math.PI/2;

    // USE LATER: boolean debug_flag = true;

    // Declare OpMode members.
    private final ElapsedTime runtime = new ElapsedTime();
    private intakeUnit intake;
    private MecanumDrive drive;

    // camera and sleeve color
    ObjectDetection.PropSide propLocation = ObjectDetection.PropSide.UNKNOWN;
    ObjectDetection propDetect;
    OpenCvCamera camera;
    String webcamName = "Webcam 1";
    boolean isCameraInstalled = true;
    // sensing april tag tools
    private AprilTagTest tag = null;

    // road runner variables
    Pose2d startPose;

    Vector2d vDropWhite;

    /**
     * Set robot starting location on the field:
     * 1 for Red Front, 2 for Red back, 3 for Blue Front, and 4 for Blue back
     */
    public void setRobotLocation() {
        startLoc = 2;
        leftOrRight = 1;
    }

    /**
     * Set robot starting position, and blueOrRed, frontOrBack variables:
     * @param startLocation : the value of robot location in the field.
     *                      1 for Red Front, 2 for Red back,
     *                      3 for Blue Front, and 4 for Blue back
     */
    private void setStartPoses(int startLocation) {
        // road runner variables
        switch(startLocation) {
            case 1: // red front
                blueOrRed = -1;
                frontOrBack = 1;
                break;

            case 2: // red back
                blueOrRed = -1;
                frontOrBack = -1;
                break;

            case 3: //  blue front
                blueOrRed = 1;
                frontOrBack = 1;
                break;

            case 4: //  blue back
                blueOrRed = 1;
                frontOrBack = -1;
                break;
        }
        startPose = new Pose2d((6 * Params.HALF_MAT - Params.CHASSIS_HALF_LENGTH - Params.CHASSIS_START_EXTRA) * blueOrRed,
                //Params.HALF_MAT + (3 * Params.HALF_MAT - Params.CHASSIS_HALF_WIDTH) * frontOrBack,
                Params.HALF_MAT + 2 * Params.HALF_MAT * frontOrBack,
                Math.toRadians(90.0 + 90.0 * blueOrRed));
    }

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        Logging.log("Status - Initialized");

        Servo droneServo = hardwareMap.get(Servo.class, "Drone");
        droneServo.setPosition(Params.DRONE_START);

        setRobotLocation();

        setStartPoses(startLoc);

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

        propDetect = new ObjectDetection();

        if (blueOrRed < 0) {
            propDetect.setColorFlag(ObjectDetection.ColorS.RED);
        } else {
            propDetect.setColorFlag(ObjectDetection.ColorS.BLUE);
        }

        if (isCameraInstalled) {
            camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(
                    WebcamName.class, webcamName), cameraMonitorViewId);

            camera.setPipeline(propDetect);

            camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
                @Override
                public void onOpened() {
                    camera.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
                    Logging.log("Start stream to detect sleeve color.");
                }

                @Override
                public void onError(int errorCode) {
                    Logging.log("Start stream error.");
                    telemetry.addData("Start stream to detect sleeve color.", "error");
                    telemetry.update();
                }
            });
        }

        // init drive with road runner
        Params.startPose = startPose; // init storage pose.
        Params.blueOrRed = blueOrRed;
        Params.deadWheelOn = MecanumDrive.PARAMS.useDeadWheel;

        intake = new intakeUnit(hardwareMap, "Arm", "Wrist",
                "Finger");
        //intake.setArmModeRunToPosition(intake.getArmPosition());

        intake.fingerServoOpen();

        runtime.reset();
        while ((ObjectDetection.PropSide.UNKNOWN == propLocation) &&
                ((runtime.seconds()) < 3.0)) {
            propLocation = propDetect.getPropPos();
        }

        int spikeMarkLoc = 1; // 1 for left, 2 for center, and 3 for right

        while (!isStarted()) {
            propLocation = propDetect.getPropPos();
            sleep(10);
            telemetry.addData( ((blueOrRed >0)? "Blue - " : "Red - "),((frontOrBack >0)? "front" : "back"));

            telemetry.addData("Detected Prop location: ", propLocation);
            telemetry.addData("Arm", "position = %d", intake.getArmPosition());
            telemetry.addData(" ---- ", " ---  ");

            telemetry.addData("Arm calibration - -    ", Params.armCalibrated? "Pass!" :" Failed !!!!!");


            telemetry.update();
        }

        switch (propLocation) {
            case LEFT:
                spikeMarkLoc = 1;
                break;
            case CENTER:
            case UNKNOWN:
                spikeMarkLoc = 2;
                break;
            case RIGHT:
                spikeMarkLoc = 3;
                break;
        }
        desiredTagNum = spikeMarkLoc + ((blueOrRed > 0)? 0 : 3); // blue: 1,2,3; red: 4,5,6
        checkStatus = desiredTagNum * frontOrBack;

        if ((2 == desiredTagNum) || (1 ==desiredTagNum)) {
            desiredTagWhite = 3;
            whiteDropShift = BUCKET_SHIFT;

        }
        else if(3 ==desiredTagNum) {
            desiredTagWhite = 1;
            whiteDropShift = -BUCKET_SHIFT;
        }
        else if((5 ==desiredTagNum) || (6 ==desiredTagNum)) {
            desiredTagWhite = 4;
            whiteDropShift = -BUCKET_SHIFT;
        }
        else { //(4 ==desiredTagNum)
            desiredTagWhite = 6;
            whiteDropShift = BUCKET_SHIFT;
        }
        //desiredTagWhite = (desiredTagNum < 3.5)? 2 : 5; // temp center for safe

        // bulk reading setting - auto refresh mode
        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);

        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        if (opModeIsActive()) {
            drive = new MecanumDrive(hardwareMap, startPose);
            tag = new AprilTagTest(drive, hardwareMap, desiredTagNum, webcamName);

            Logging.log("checkStatus = %d, desiredTagNum = %d", checkStatus, desiredTagNum);
            Logging.log("frontOrBack = %d, blueOrRed = %d", frontOrBack, blueOrRed);

            if (Params.armCalibrated) {
                intake.autonomousInit();
                autonomousCore();

                Params.currentPose = drive.pose; // storage end pose of autonomous
                //intake.parkingPositions(); // Motors are at intake positions at the beginning of Tele-op
                intake.fingerStop();
                sleep(1000);

                camera.closeCameraDevice(); // cost too times at the beginning to close camera about 300 ms
                Logging.log("Autonomous time - total Run Time: " + runtime);
            }
            else {
                telemetry.addData("Arm calibration: ---", "need to be done before starting!");
                telemetry.update();
                sleep(4000);
            }
        }
    }

    private void autonomousCore() {
        autoCore();
        //intake.armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }

    private void autoCore() {

        double pickup1_delta_x = blueOrRed * 1.5;
        double pickup2_delta_x = 4;
        double pickup2_delta_y = 0.25;
        double dropYellow_delta_x = 0.0;
        double dropYellow_delta_y = -1.5;
        double dropWhite_delta_y = -0.5;

        double xDelta = 0.0;
        double yDelta = 0.0;
        double purpleAngle = startPose.heading.toDouble();

        double pickWhiteReady_x = blueOrRed * 3.0 * Params.HALF_MAT + BUCKET_SHIFT;
        double pickWhiteReady_y = 3.5 * Params.HALF_MAT - 5.0;

        double startTurn2ndDrop_x = blueOrRed * Params.HALF_MAT; // under gate

        double pickup1_alpha_x = 0;
        double pickup1_alpha_y = 0;

        switch (checkStatus) {
            case -5:
                xDelta = -7; // avoid stamp on purple for case 5
                yDelta = -8;
                purpleAngle = startPose.heading.toDouble() + Math.PI / 10;

                pickup1_delta_x = -1;
                pickup2_delta_x = -1.0;
                dropYellow_delta_x = -0.5 ;
                break;

            case -2:
                xDelta = 8.0; // avoid stamp on purple for case 5
                yDelta = -7;
                purpleAngle = startPose.heading.toDouble() - Math.PI / 15;

                pickup1_delta_x = 0.5;
                pickup2_delta_x = 3.5;
                dropYellow_delta_x = 1;
                break;
            case -1:
                xDelta = 12;
                yDelta = -17.5;
                purpleAngle = startPose.heading.toDouble() - Math.PI / 6;

                pickup1_delta_x = -0.5;
                pickup2_delta_x = 4.5;
                dropYellow_delta_x = 0;

                pickup1_alpha_x = 3;
                pickup1_alpha_y = -2;

                pickWhiteReady_x = 18;
                pickWhiteReady_y = 3.5 * Params.HALF_MAT - 2;
                pickupAngle1 = Math.toRadians(70);
                break;
            case -6:
                xDelta = -12;
                yDelta = -17.5;
                purpleAngle = startPose.heading.toDouble() - blueOrRed * Math.PI / 8;

                pickWhiteReady_y = 3.5 * Params.HALF_MAT - 6;
                pickup1_alpha_y = 1;

                pickup1_delta_x = -3;
                pickup2_delta_x = -4;
                dropYellow_delta_x = 1;

                pickWhiteReady_x = blueOrRed * 2.0 * Params.HALF_MAT + BUCKET_SHIFT;;
                break;
            case -3:
                xDelta = 9;
                yDelta = -2;
                purpleAngle = startPose.heading.toDouble() - Math.PI / 3.5;

                pickup1_delta_x = 1.0;
                pickup2_delta_x = 4.5;
                pickup2_delta_y = 0.5;
                dropYellow_delta_x = 1;
                break;
            case -4:
                xDelta = -7;
                yDelta = -3.5;
                purpleAngle = startPose.heading.toDouble() + Math.PI / 3.5;

                pickup1_delta_x = -1;
                pickup2_delta_x = -1.5;
                dropYellow_delta_x = 0;
                break;
        }
        pickWhiteReady_x = pickWhiteReady_x + pickup1_delta_x;

        Vector2d vParkPos = new Vector2d(blueOrRed * 3 * Params.HALF_MAT - 2 * leftOrRight * Params.HALF_MAT, -4 * Params.HALF_MAT + 2);
        Vector2d vBackdrop = new Vector2d(blueOrRed * 3 * Params.HALF_MAT + dropYellow_delta_x, -4 * Params.HALF_MAT);
        Vector2d vAprilTag = null;

        if (blueOrRed > 0) {
            vAprilTag = new Vector2d(vBackdrop.x + (2 - desiredTagNum) * Params.BACKDROP_SIDEWAYS, vBackdrop.y);
        } else {
            vAprilTag = new Vector2d(vBackdrop.x + (5 - desiredTagNum) * Params.BACKDROP_SIDEWAYS, vBackdrop.y);
        }

        double adjustInchY = ((1 == checkStatus) || (6 == checkStatus))? -3 : 0; // avoid stamp on purple
        Vector2d vCheckingAprilTagPose = new Vector2d(vAprilTag.x, vAprilTag.y + Params.HALF_MAT + adjustInchY);
        Vector2d vCheckingAprilTag2nd = new Vector2d(vAprilTag.x - (desiredTagWhite - desiredTagNum) * 6, vAprilTag.y + Params.HALF_MAT + adjustInchY);

        // yellow pixel is in the right bucket.
        Vector2d vDropYellow = new Vector2d(vAprilTag.x + BUCKET_SHIFT, vAprilTag.y + 3.0 + dropYellow_delta_y);
        vDropWhite = new Vector2d(vCheckingAprilTag2nd.x + whiteDropShift, vAprilTag.y + 1 + dropWhite_delta_y);

        Vector2d vDropPurple = null;

        Vector2d vStartTurn2ndDrop = new Vector2d(startTurn2ndDrop_x, vCheckingAprilTagPose.y + 0.5 *Params.HALF_MAT); // far enough to avoid step on purple

        vDropPurple = new Vector2d(blueOrRed * 3 * Params.HALF_MAT + xDelta, startPose.position.y + yDelta);

        Pose2d pPickWhite1Ready = new Pose2d(pickWhiteReady_x, pickWhiteReady_y, pickupAngle1);
        Vector2d pickWhite5 = new Vector2d(pickWhiteReady_x + pickup1_alpha_x, pickWhiteReady_y + 11 + pickup1_alpha_y);
        Vector2d vPickup2 = new Vector2d(startTurn2ndDrop_x + pickup2_delta_x, pickWhite5.y + pickup2_delta_y);

        Vector2d vOneWaitPosition = new Vector2d(vCheckingAprilTagPose.x, pickWhite5.y - Params.HALF_MAT);

        Logging.log("check status = %d, xDelta = %.2f, yDelta = %.2f ", checkStatus, xDelta, yDelta);
        logVector("Back drop pose", vBackdrop);
        logVector("April tag", vAprilTag);

        logRobotHeading("robot drive: before strafe");
        logVector("robot drive: start position", startPose.position);
        Logging.log("Autonomous time - robot start moving time: " + runtime);

        // move to drop purple, hitting the ball
        double wristPos = ((2 == checkStatus) || (5 == checkStatus))? intake.WRIST_POS_INTAKE  : intake.WRIST_POS_DROP_PURPLE;
        double armPos = ((2 == checkStatus) || (5 == checkStatus))? intake.ARM_POS_PUSH_PROP : intake.ARM_POS_DROP_PURPLE;
        double waitSec = (1 == checkStatus)? 0 : 0.7;
        waitSec = (6 == checkStatus)? 0.3 : waitSec;

        Actions.runBlocking(
                drive.actionBuilder(drive.pose)
                            .afterTime(0.001, new CloseCamera()) // close camera
                            .afterTime(0.1, new intakeUnitActions(armPos, wristPos, NO_ACT))
                            .strafeTo(vDropPurple)
                            .waitSeconds(waitSec)  // wait for arm to be in place
                            .turnTo(purpleAngle)
                            .build()
        );


        logVector("robot drive: drop purple pose", drive.pose.position);
        logVector("robot drive: drop purple pose required", vDropPurple);
        logRobotHeading("robot drive: drop purple heading -");
        Logging.log("Autonomous time - ready to drop purple time: " + runtime);

        // drop off the purple pixel by arm and wrist actions
        if ((2 == checkStatus) || (5 == checkStatus)) {
            intake.setArmPosition(intake.ARM_POS_DROP_PURPLE);
            intake.setWristPosition(intake.WRIST_POS_DROP_PURPLE);
            sleep(200);
        }
        dropPurpleAction();
        intake.underTheBeam(); // avoid to hitting rigging bin
        sleep(200);

        Logging.log("Autonomous time - after drop purple time: " + runtime);

        // move back to drop yellow and white, lift arm after through gate

                Actions.runBlocking(
                        drive.actionBuilder(drive.pose)
                                .afterTime(0.01, new TurnOnCamera()) // turn on camera for April Tag checking
                                //.strafeTo(vCheckingAprilTagPose)
                                .strafeToLinearHeading(vCheckingAprilTagPose, dropOffAngle)
                                .afterTime(0, new intakeUnitActions(intake.ARM_POS_CAMERA_READ, NO_ACT, NO_ACT))
                                .build()
                );

        Logging.log("Autonomous time - checking camera time: " + runtime);
        logVector("robot drive: drive.pose check april tag", drive.pose.position);
        logVector("robot drive: check april tag required", vCheckingAprilTagPose);
        logRobotHeading("robot drive: check april tag before fine correction");

        // fine correct angle before checking April Tag
        Actions.runBlocking(
                drive.actionBuilder(drive.pose)
                        .turnTo(dropOffAngle)
                        .build()
        );
        logRobotHeading("robot drive: check april tag after fine correction");

        Logging.log("Autonomous - Start April tag detect");
        Pose2d aprilTagPose = tag.updatePoseAprilTag(desiredTagNum);
        logVector("robot drive: april tag location from camera", aprilTagPose.position);
        logVector("robot drive: drop yellow pose required before adjust", vDropYellow);

        // if can not move based on April tag, move by road runner encode.
        if (tag.targetFound) {
            // adjust yellow drop-off position according to april tag location info from camera
            vDropYellow = new Vector2d(drive.pose.position.x - aprilTagPose.position.x + BUCKET_SHIFT,
                    drive.pose.position.y - aprilTagPose.position.y + Params.AUTO_DISTANCE_TO_TAG);
            logVector("robot drive: drop yellow pose required after april tag adjust", vDropYellow);
        }
        else {
            Logging.log("Can not found required AprilTag to drop yellow pixel");
        }

       // intake.readyToDropYellow(intake.ARM_POS_DROP_YELLOW_BACK);
        if ((5 == checkStatus) || (2 == checkStatus)) {
            sleep(100); // wait arm down to the position
        }
        intake.setWristPosition(intake.WRIST_POS_DROP_YELLOW_BACK);

        // shift to AprilTag
        Actions.runBlocking(
                drive.actionBuilder(drive.pose)
                        .strafeTo(vDropYellow)
                        .build()
        );
        logVector("robot drive: drive.pose drop yellow", drive.pose.position);
        logVector("robot drive: drop yellow required", vDropYellow);
        logRobotHeading("robot drive: drop yellow heading -");

        // drop pixel
        //dropYellowAction(); // arm is at high position
        Logging.log("Autonomous time - after drop off yellow time: " + runtime);

        logVector("Before fine turn required", vStartTurn2ndDrop);


        // parking
        intake.setArmPosition(intake.ARM_POS_READY_FOR_HANG + 200);
        Actions.runBlocking(
                drive.actionBuilder(drive.pose)
                        .strafeTo(vParkPos)
                        .build()
        );
        logVector("robot drive: drive.pose parking", drive.pose.position);
        logVector("robot drive: parking required", vParkPos);
    }

    private void dropPurpleAction() {
        // 1. arm and wrist at correct positions
        //intake.readyToDropPurple();
        //sleep(200);

        // 2. open switch
        intake.setWristPosition(intake.WRIST_POS_DROP_PURPLE);
        intake.setWristServoPosition(intake.WRIST_SNAP_POSITION);
        sleep(500);
    }

    /*
    private void dropYellowAction(){
        intake.setArmServoPosition(intake.SWITCH_RIGHT_RELEASE);
        sleep(300);

        // move to drop white
        if ((1 == checkStatus) || (4 == checkStatus)) {
            intake.setArmServoPosition(intake.SWITCH_RIGHT_CLOSE_POS);
            intake.dropWhitePositions();
            sleep(100);
            Actions.runBlocking(
                    drive.actionBuilder(drive.pose)
                            .strafeTo(new Vector2d(vDropWhite.x, drive.pose.position.y - 2.0))
                            .turnTo(dropOffAngle)
                            .build()
            );
        }
        intake.setWristServoPosition(intake.SWITCH_LEFT_RELEASE);
        sleep(200);
        intake.setArmCountPosition(intake.getArmPosition() - 500);
        sleep(200);
    }

     */

    private void dropWhiteAction(){
        // release both
        intake.fingerServoOpen();
        sleep((1 == checkStatus)? 400 : 1200); // save a little time for case 1
        intake.setArmPosition(intake.ARM_POS_READY_FOR_HANG);
        sleep(200);
    }

    private void logVector(String sTag, Vector2d vXY) {
        String vectorName = vXY.toString();
        Logging.log("%s: %s", sTag, vectorName);
    }

    private void logRobotHeading(String sTag) {
        Logging.log("%s drive.pose: %.2f", sTag, Math.toDegrees(drive.pose.heading.log()));
        //Logging.log("%s imu: %.2f", sTag, drive.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES) - Math.toDegrees(startPose.heading.log()));
    }

    private class TurnOnCamera implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            tag.initAprilTag();
            return false;
        }
    }

    private class CloseCamera implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            camera.closeCameraDevice();
            return false;
        }
    }

    private class intakeUnitActions implements Action {
        public intakeUnitActions(double setArmPos, double setWristPos, double setFingerPos) {
            armPos = (int)setArmPos;
            wristPos = setWristPos;
            fingerPos = setFingerPos;
        }

        private final int armPos;
        private final double wristPos;
        private final double fingerPos;

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {

            if (armPos < NO_ACT) {
                intake.setArmPosition(armPos);
            }

            if (wristPos < NO_ACT) {
                intake.setWristPosition(wristPos);
            }

            if (fingerPos < NO_ACT) {
                if (intake.FINGER_STOP_POS == fingerPos) {
                    intake.fingerStop();
                }
                else if(intake.FINGER_INTAKE_POS == fingerPos) {
                    intake.fingerIntake();
                }
                else if (intake.FINGER_OUTTAKE_POS == fingerPos){
                    intake.fingerOuttake();
                }
            }

            return false;
        }
    }

    private class pickupWhiteAct implements Action {
        public pickupWhiteAct(int setArmPos) {
            armPos = setArmPos;
        }

        private final int armPos;

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            //intake.intakePositions(armPos);
            return false;
        }
    }

    private class recordDrivePosition implements Action {
        public recordDrivePosition(String inputStr) {
            drive.updatePoseEstimate();
            str = inputStr;
        }
        private String str = null;

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            logVector(str, drive.pose.position);
            logRobotHeading(str);
            return false;
        }
    }

}
