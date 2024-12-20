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

import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

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

@TeleOp(name="Teleop Sliders", group="Concept")
@Disabled
public class TeleopSliders extends LinearOpMode {
    // Declare OpMode members.
    private final ElapsedTime runtime = new ElapsedTime();

    // chassis
    MecanumDrive mecanum;

    //claw and arm unit
    private intakeUnit4Servo intake;

    private SlidersWith2Motors slider;

    private Servo DroneServo;

    // debug flags, turn it off for formal version to save time of logging
    boolean debugFlag = true;

    private AprilTagTest tag = null;

    final double DESIRED_DISTANCE = 3.0;


    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");

        GamePadButtonsSliders gpButtons = new GamePadButtonsSliders();

        mecanum = new MecanumDrive(hardwareMap, Params.currentPose);
        mecanum.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //tag = new AprilTagTest(mecanum, hardwareMap, 0, "Webcam 1");

        //tag.initAprilTag();

        intake = new intakeUnit4Servo(hardwareMap, "ArmLeft", "ArmRight", "Wrist",
                "Finger");

        slider = new SlidersWith2Motors();


        slider.init(hardwareMap, "sliderRight", "sliderLeft");

        slider.resetEncoders(); // should move this reset code to auto finally.

        //slider.setCountPosition(slider.getPosition());
        //slider.runToPosition();

        // bulk reading setting - auto refresh mode
        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

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
            mecanum.setDrivePowers(new PoseVelocity2d(
                    new Vector2d(
                            -gpButtons.robotDrive * maxDrivePower,
                            -gpButtons.robotStrafe * maxDrivePower
                    ),
                    -gpButtons.robotTurn * maxDrivePower
            ));

            if (gpButtons.sliderUp){
                slider.setInchPosition(slider.getInchPosition() + 4.0);
            }

            if (gpButtons.sliderDown){
                slider.setInchPosition(slider.getInchPosition() - 4.0);
            }

            if (gpButtons.sliderMax ){
                slider.setInchPosition(slider.FOUR_STAGE_SLIDER_MAX_POS);
            }

            if (gpButtons.sliderMin){
                slider.setInchPosition(slider.SLIDER_MIN_POS);
            }

            if (gpButtons.armForward) {
                intake.setArmPosition(intake.getArmLeftPosition() - 0.008);
            }

            if (gpButtons.armBackward) {
                intake.setArmPosition(intake.getArmRightPosition() + 0.008);
            }

            if (gpButtons.wristUp) {
                intake.setWristPosition(intake.getWristPosition() + 0.008);
            }

            if (gpButtons.wristDown) {
                intake.setWristPosition(intake.getWristPosition() - 0.008);
            }

            //finger is on continuous mode
            if (gpButtons.fingerClose) {
                intake.setFingerPosition(0.35);
            }

            if (gpButtons.fingerStop) {
                intake.setFingerPosition(0.5);
            }

            if (gpButtons.fingerOpen) {
                intake.setFingerPosition(0.9);
            }

            if (gpButtons.pickupPos) {
                intake.setFingerPosition(0.65);
                intake.setWristPosition(intake.WRIST_POS_GRAB_SAMPLE);
                intake.setArmPosition(0.432);
                slider.setCountPosition(0);
            }

            if (gpButtons.highestPos) {
                intake.setFingerPosition(0.35);
                intake.setWristPosition(intake.WRIST_POS_GRAB_SAMPLE);
                intake.setArmPosition(1.0);
                slider.setCountPosition(-2000);
            }

            mecanum.updatePoseEstimate();
            Params.currentPose = mecanum.pose;
            if (debugFlag) {
                // claw arm servo log
                telemetry.addData("Finger", "position %.3f", intake.getFingerPosition());

                telemetry.addData("Wrist", "position %.3f", intake.getWristPosition());

                telemetry.addData("ArmLeft", "position = %.3f", (double)(intake.getArmLeftPosition()));
                telemetry.addData("ArmRight", "position = %.3f", (double)(intake.getArmRightPosition()));

                telemetry.addData("sliderLeft", "position = %s", slider.LeftSliderMotor.getCurrentPosition());
                telemetry.addData("sliderRight", "position = %s", slider.RightSliderMotor.getCurrentPosition());

                //telemetry.addData("Finger", "position %.3f", intake.getFingerPosition());

                //telemetry.addData("switch Right", "position %.3f", intake.getSwitchRightPosition());

                //telemetry.addData("switch Left", "position %.3f", intake.getSwitchLeftPosition());

                telemetry.addData(" ", " ");



                telemetry.addData("heading", " %.3f", Math.toDegrees(mecanum.pose.heading.log()));

                telemetry.addData("location", " %s", mecanum.pose.position.toString());

                //telemetry.addData("motor velocity = ", " %.3f", mecanum.leftFront.getVelocity());

                telemetry.update(); // update message at the end of while loop


            }

        }

        //intake.armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        // The motor stop on their own but power is still applied. Turn off motor.
    }

    private void logVector(String sTag, Vector2d vXY) {
        String vectorName = vXY.toString();
        Logging.log("%s: %s", sTag, vectorName);
    }

    /*
    private void moveByAprilTag_new(int tagNum) {
        intake.dropPositions();
        sleep(300); // make sure arm is out of camera sight

        Pose2d aprilTagPose = tag.updatePoseAprilTag_new(tagNum);

        // if can not move based on April tag, moved by road runner.
        if (tag.targetFound) {
            mecanum.updatePoseEstimate();
            mecanum.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            Logging.log("yaw in new = %.2f", Math.toDegrees(aprilTagPose.heading.log()));
            logVector("robot drive: distance from camera to april tag", aprilTagPose.position);

            // adjust yellow drop-off position according to april tag location info from camera
            Vector2d desiredMove = new Vector2d(mecanum.pose.position.x - aprilTagPose.position.x,
                    mecanum.pose.position.y - aprilTagPose.position.y);
            logVector("robot drive: before move to tag pose", mecanum.pose.position);

            logVector("robot drive: move to tag pose required", desiredMove);
            logRobotHeading("before moving to april tag");
            // shift to AprilTag
            Actions.runBlocking(
                    mecanum.actionBuilder(mecanum.pose)
                            .strafeToLinearHeading(desiredMove, mecanum.pose.heading.toDouble() + aprilTagPose.heading.toDouble())
                            .build()
            );
            logRobotHeading("after moving to april tag");


            mecanum.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
    }


    private void lineWithAprilTag(int tagNum) {
        intake.dropPositions();
        sleep(300); // make sure arm is out of camera sight

        /*  // do back when switch to drop automatically.
        Actions.runBlocking(
                mecanum.actionBuilder(mecanum.pose)
                        .lineToY(mecanum.pose.position.y + 1.5 * Params.HALF_MAT)
                        .build()
        );



        //sleep(850);

        // if can not move based on April tag, moved by road runner.
        if (tag.targetFound) {
            mecanum.updatePoseEstimate();
            mecanum.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            Logging.log("tag target found.");
            logVector("robot drive: distance from camera to april tag", aprilTagPose.position);

            // adjust yellow drop-off position according to april tag location info from camera
            Vector2d desiredMove = new Vector2d(mecanum.pose.position.x - aprilTagPose.position.x,
                    mecanum.pose.position.y + 6.5 * Params.HALF_MAT);
            logVector("robot drive: before move to pose", mecanum.pose.position);
            //intake.armMotor.setPower(0.5);
            intake.underTheBeam();
            Actions.runBlocking(
                    mecanum.actionBuilder(mecanum.pose)
                            .strafeToLinearHeading(new Vector2d(desiredMove.x, mecanum.pose.position.y), -Math.PI / 2)
                            .strafeTo(desiredMove)
                            .turn(-Math.PI / 2 * Params.blueOrRed)
                            .build()
            );
            logVector("robot drive: move to tag pose required", desiredMove);
            logRobotHeading("after moving to april tag");
            //intake.armMotor.setPower(1);
            mecanum.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        } else {
            Logging.log("new command: april tag not detected");
        }
    }
    */

    private void logRobotHeading(String sTag) {
        Logging.log("%s drive.pose: %.2f", sTag, Math.toDegrees(mecanum.pose.heading.log()));
        //Logging.log("%s imu: %.2f", sTag, mecanum.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES) - Math.toDegrees(Params.startPose.heading.log()));
    }

    private void moveForward(double moveDistance) {
        intake.underTheBeam();
        sleep(300); // make sure arm is out of camera sight

        mecanum.updatePoseEstimate();
        mecanum.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        logVector("robot drive: current drive pose", mecanum.pose.position);
        logRobotHeading("before moving to back area");

        double turnAngle = -mecanum.pose.heading.log() - Math.PI / 2.0;
        turnAngle = (Math.abs(turnAngle) > Math.PI)? (turnAngle - Math.signum(turnAngle) * 2 * Math.PI) :  turnAngle;
        // shift to AprilTag
        Actions.runBlocking(
                mecanum.actionBuilder(mecanum.pose)
                        .turn(turnAngle)
                        .lineToYLinearHeading(mecanum.pose.position.y - moveDistance, -Math.PI / 2.0)
                        .build()
        );
        logRobotHeading("after moving to back area");
        logVector("robot drive: arrive back area, drive pose", mecanum.pose.position);
        mecanum.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    private void moveBack(double moveDistance) {
        mecanum.updatePoseEstimate();
        mecanum.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        logVector("robot drive: current drive pose", mecanum.pose.position);
        logRobotHeading("before moving to back area");

        // shift to AprilTag
        Actions.runBlocking(
                mecanum.actionBuilder(mecanum.pose)
                        .lineToYLinearHeading(mecanum.pose.position.y + moveDistance, -Math.PI / 2.0)
                        .build()
        );
        logRobotHeading("after moving to back area");
        logVector("robot drive: arrive back area, drive pose", mecanum.pose.position);
        mecanum.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
}
