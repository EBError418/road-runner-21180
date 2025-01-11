/* Copyright (c) 2017 FIRST. All rights reserved.
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
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;


/**
 * This is NOT an opmode.
 *
 * This hardware class assumes the following device names have been configured on the robot:
 * Note:  All names are case sensitive.
 * Motors type: motors for arm and wrist, finger.
 *
 * 1. Arm servo motor: ArmServo
 * 2. wrist servo motor: wristServo
 */
public class intakeUnit
{
    //private
    HardwareMap hardwareMap =  null;

    // arm servo motor variables
    private DcMotor armMotor = null;
    private DcMotor wristMotor = null;

    //wrist servo
    private Servo knuckleServo = null;

    //finger servo
    public Servo fingerServo = null;

    //finger
    final double FINGER_OPEN_SUB = 0.5;
    final double FINGER_OPEN = FINGER_OPEN_SUB;
    final double FINGER_CLOSE = 0.81; // must bigger than 0 for action builder working
    final double FINGER_OPEN_BACK = 0.4;
    final double FINGER_CLOSE_BACK = 0.2;

    //knuckle
    final double KNUCKLE_MIN = 0.11;
    final double KNUCKLE_MAX = 0.9;

    final double KNUCKLE_POS_AUTO_INIT = 0.15 - 0.03;
    final double KNUCKLE_POS_LIFT_FROM_WALL = 0.15  - 0.04;
    final double KNUCKLE_POS_AWAY_FROM_SUBMERSIBLE = 0.21 - 0.03;
    final double KNUCKLE_POS_PICKUP_SPECIMEN_ready = 0.2 - 0.03;
    final double KNUCKLE_POS_WRIST_CONSTRAINT = 0.3;
    final double KNUCKLE_POS_HANGING = 0.20  - 0.03; // end game hanging
    final double KNUCKLE_POS_PICKUP_SPECIMEN_WALL = 0.203  - 0.03;
    final double KNUCKLE_SIZE_CONSTRAINT = 0.25;  // limit the maximum back reach to 20 inch
    final double KNUCKLE_POS_PICKUP_SPECIMEN = 0.358 - 0.03;
    final double KNUCKLE_POS_HIGH_CHAMBER = 0.358 - 0.03;
    final double KNUCKLE_POS_PICKUP_SAMPLE_READY = 0.408 - 0.03; // approaching submersible
    final double KNUCKLE_POS_PICKUP_SAMPLE_BACK = 0.52 - 0.03; // pickup sample during auto
    final double KNUCKLE_POS_PICKUP_SAMPLE_RUBBER = 0.584;
    final double KNUCKLE_POS_LOW_BUCKET = 0.581 - 0.03;
    final double KNUCKLE_POS_PICKUP_SAMPLE = 0.73 - 0.03;


    //new stuff: wrist
    final int WRIST_MIN = -50;
    final int WRIST_MAX= 320;
    final int WRIST_INIT = 270;
    final int WRIST_BACK = 270; // pickup sample during auto
    final int WRIST_POS_NEUTRAL = 0;

    //arm
    int ARM_MIN = -3870;
    int ARM_MAX = 100;
    int ARM_INIT = -3860;
    int ARM_POS_GRAB_SAMPLE_BACK = -3850;
    int ARM_POS_DROP_SAMPLE = -650; // drop off sample during during auto. Need adjust to make sure fingers do not touch ground.
    int ARM_POS_HIGH_CHAMBER_READY = -2890;
    int ARM_POS_HIGH_CHAMBER_MOVING_SPECIMEN = -3150; // moving specimen left/right

    int ARM_POS_GRAB_SAMPLE = -500; // pickup sample during teleop
    int ARM_POS_HIGH_CHAMBER = -3475;//-2967;//2490;
    int ARM_POS_LOW_BUCKET = -1853;
    int ARM_POS_LEFT_PARKING = -500;
    int ARM_POS_OBS_ZONE = -430;
    int ARM_POS_BACK = -3000;
    int ARM_POS_BEFORE_HANG = -2130; // ready for hanging robot during end game
    int ARM_POS_GRAB_SPECIMEN = -170; //-240, grab specimen from ground
    int ARM_POS_SUB = -500;
    int ARM_POS_DOWN_HANGING = 100;
    int ARM_POS_GRAB_SPECIMEN_WALL = -360; // grab specimen from wall
    int ARM_POS_OBZ_PARKING = -100;

    /**
     * Init slider motors hardware, and set their behaviors.
     * @param hardwareMap the Hardware Mappings.
     */
    public intakeUnit(HardwareMap hardwareMap, String armServoName, String wristMotorName, String knuckleServoName,
                      String fingerServoName) {
        // Save reference to Hardware map
        this.hardwareMap = hardwareMap;

        Logging.log("init motors for finger, wrist and arm.");

        fingerServo = hardwareMap.get(Servo.class, fingerServoName);
        //fingerServo.setDirection(Servo.Direction.REVERSE);
        knuckleServo = hardwareMap.get(Servo.class, knuckleServoName);
        knuckleServo.setDirection(Servo.Direction.FORWARD);

        sleep(100);

        /* init arm motor, set mode to encode mode */
        armMotor = hardwareMap.get(DcMotor.class, armServoName);
        armMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //setArmModeRunToPosition(getArmPosition());

        /* init wrist motor, set mode to encode mode */
        wristMotor = hardwareMap.get(DcMotor.class, wristMotorName);
        Logging.log("before init wrist pos: %s", wristMotor.getCurrentPosition());
        wristMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        wristMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    //finger servo
    public void fingerServoOpen() {
        fingerServo.setPosition(FINGER_OPEN);
    }
    public void fingerServoClose() {
        fingerServo.setPosition(FINGER_CLOSE);
    }
    public void setFingerPosition(double fingerpos){
        fingerServo.setPosition(fingerpos);
    }

    /**
     * set the target position of knuckle servo
     * @param knucklePos the target position value for knuckle servo
     */
    public void setKnucklePosition(double knucklePos){
        knucklePos = Range.clip(knucklePos, KNUCKLE_MIN, KNUCKLE_MAX);
        knuckleServo.setPosition(knucklePos);
    }

    /**
     * set the target position of wrist motor
     * @param wristPos the target position value for wrist motor
     */
    public void setWristPosition(int wristPos) {
        wristPos = Range.clip(wristPos, WRIST_MIN, WRIST_MAX);
        wristMotor.setTargetPosition(wristPos);
    }


    /* Reset wrist motor encode*/
    public void resetWristEncoder() {
        wristMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Logging.log("after calibration wrist pos: %s", wristMotor.getCurrentPosition());
        setWristModeRunToPosition(0);
        Logging.log("after movement wrist pos: %s", wristMotor.getCurrentPosition());
    }

    /* set */
    public void setWristModeRunToPosition(int wristPos) {
        setWristPosition(wristPos);
        wristMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        wristMotor.setPower(0.3);
    }

    /**
     * set the target position of arm servo motor
     * @param armPos the target position value for arm servo motor
     */
    public void setArmPosition(double armPos) {
        armPos = Range.clip(armPos, ARM_MIN, ARM_MAX);
        armMotor.setTargetPosition((int)(armPos));
    }

    public void setArmModeRunToPosition(int armPos) {
        setArmPosition(armPos);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armMotor.setPower(0.95);
    }

    public void resetArmEncoder() {
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setArmModeRunToPosition(0);
    }

    public void setIntakeAutoInit() {
        setArmPosition(ARM_INIT);
        setWristPosition(WRIST_INIT);
        setKnucklePosition(KNUCKLE_POS_AUTO_INIT);
        setFingerPosition(FINGER_CLOSE);
    }

    /**
     * Get the arm motor current position value
     * @return the current arm motor position value
     */
    public int getArmPosition() {
        return armMotor.getCurrentPosition();
    }

    /**
     * Get the wrist servo motor current position value
     * @return the current wrist servo motor position value
     */
    public int getWristPosition() {
        return wristMotor.getCurrentPosition();
    }

    public double getFingerPosition() {
        return fingerServo.getPosition();
    }

    public double getKnucklePosition() { return knuckleServo.getPosition(); }

    private void sleep(long milliseconds) {
        try {
            Thread.sleep(milliseconds);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
    }
}