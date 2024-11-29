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

    //new stuff: wrist
    final double WRIST_POS_DELTA = 0.0;
    final double WRIST_POS_GRAB_SAMPLE = WRIST_POS_DELTA + 0.36;
    final int WRIST_POS_GRAB_SAMPLE_BACK = 270;
    final int WRIST_POS_HIGH_CHAMBER = 270;
    final double WRIST_POS_LOW_BUCKET = WRIST_POS_DELTA + 0.626;
    final double WRIST_POS_PARKING = WRIST_POS_DELTA + 0.1;
    final double WRIST_POS_OBS_ZONE = WRIST_POS_DELTA + 0.309;
    final int WRIST_POS_GRAB_SPECIMEN = 0;//0.302;
    final int WRIST_POS_NEUTRAL = 0;

    //finger
    final double FINGER_CLOSE = 0.01; // must bigger than 0 for action builder working
    final double FINGER_SPECIMEN_CLOSE = 0.1;
    final double FINGER_OPEN = 0.5;

    //arm
    int ARM_POS_DELTA = -3920;
    int ARM_POS_GRAB_SAMPLE = -255;
    int ARM_POS_GRAB_SAMPLE_BACK = -3742;
    int ARM_POS_HIGH_CHAMBER = -3310;//-2967;//2490;
    int ARM_POS_HIGH_CHAMBER_READY = ARM_POS_HIGH_CHAMBER + 500;
    int ARM_POS_HIGH_CHAMBER_TELEOP = ARM_POS_HIGH_CHAMBER;
    int ARM_POS_LOW_BUCKET = -2020;
    int ARM_POS_PARKING = ARM_POS_DELTA + 2050;
    int ARM_POS_OBS_ZONE = -430;
    int ARM_POS_BACK = ARM_POS_DELTA + 1000;
    int ARM_POS_BEFORE_HANG = -2419;
    int ARM_POS_GRAB_SPECIMEN = -240; //-540;
    int ARM_POS_SUB = ARM_POS_DELTA + 2925;
    int ARM_POS_HANGING = ARM_POS_DELTA + 1820;
    int ARM_POS_DOWN_HANGING = ARM_POS_DELTA + 4150;
    int ARM_POS_DROP_SAMPLE = -360;

    //knuckle
    final double KNUCKLE_POS_PICKUP_SPECIMEN = 0.182;
    final double KNUCKLE_POS_HANGING = 0.85;
    final double KNUCKLE_POS_LOW_BUCKET = 0.500;
    final double KNUCKLE_POS_HIGH_CHAMBER = 0.383;
    final double KNUCKLE_POS_PICKUP_SAMPLE = 0.350;
    final double KNUCKLE_POS_PICKUP_SAMPLE_BACK = 0.514;
    final double KNUCKLE_POS_DROP_SAMPLE = 0.628;


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
        knuckleServo = hardwareMap.get(Servo.class, knuckleServoName);
        knuckleServo.setDirection(Servo.Direction.FORWARD);

        sleep(100);

        /* init arm motor, set mode to encode mode */
        armMotor = hardwareMap.get(DcMotor.class, armServoName);
        armMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        setArmModeRunToPosition(getArmPosition());

        /* init wrist motor, set mode to encode mode */
        wristMotor = hardwareMap.get(DcMotor.class, wristMotorName);
        Logging.log("before init wrist pos: %s", wristMotor.getCurrentPosition());
        wristMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        setWristModeRunToPosition(getWristPosition());
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
        knucklePos = Range.clip(knucklePos, 0.1, 0.85);
        knuckleServo.setPosition(knucklePos);
    }

    /**
     * set the target position of wrist motor
     * @param wristPos the target position value for wrist motor
     */
    public void setWristPosition(int wristPos) {
        wristPos = Range.clip(wristPos, 0, 270);
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
        armMotor.setTargetPosition((int)(armPos));
    }

    public void setArmModeRunToPosition(int armPos) {
        setArmPosition(armPos);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armMotor.setPower(0.9);
    }

    public void resetArmEncoder() {
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setArmModeRunToPosition(0);
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