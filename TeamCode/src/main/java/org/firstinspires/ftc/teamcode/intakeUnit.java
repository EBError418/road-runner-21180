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

    //wrist servo
    private Servo wristServo = null;

    final double SWITCH_LEFT_CLOSE_POS = 0.21;
    final double WRIST_SNAP_POSITION = 0.03;

    //finger servo
    public Servo fingerServo = null;

    //new stuff
    final double WRIST_POS_DELTA = 0.0;
    final double WRIST_POS_GRAB_SAMPLE = WRIST_POS_DELTA + 0.355;
    final double WRIST_POS_HIGH_CHAMBER = WRIST_POS_DELTA + 0.454;
    final double WRIST_POS_LOW_BUCKET = WRIST_POS_DELTA + 0.626;
    final double WRIST_POS_PARKING = WRIST_POS_DELTA + 0.1;
    final double WRIST_POS_OBS_ZONE = WRIST_POS_DELTA + 0.309;
    final double WRIST_POS_GRAB_SPECIMEN = WRIST_POS_DELTA + 0.327;
    final double WRIST_POS_SUB = WRIST_POS_DELTA + 0.583;
    final double WRIST_POS_HANGING = WRIST_POS_DELTA + 0.1;

    //finger
    final double FINGER_CLOSE = 0.0;
    final double FINGER_OPEN = 0.5;

    //arm
    final int ARM_POS_DELTA = 100;
    int ARM_POS_GRAB_SAMPLE = ARM_POS_DELTA + 3585;
    int ARM_MIN_COUNT_POS = 3820;
    int ARM_MAX_COUNT_POS = 0;
    int ARM_POS_AFTER_HANG = 135;
    int ARM_POS_HIGH_CHAMBER = ARM_POS_DELTA + 2780;//2490;
    int ARM_POS_LOW_BUCKET = ARM_POS_DELTA + 1858;
    int ARM_POS_PARKING = ARM_POS_DELTA + 2050;
    int ARM_POS_OBS_ZONE = ARM_POS_DELTA + 3500;
    int ARM_POS_BACK = ARM_POS_DELTA + 1000;
    int ARM_POS_BEFORE_HANG = ARM_POS_HIGH_CHAMBER - 100;
    int ARM_POS_GRAB_SPECIMEN = ARM_POS_DELTA + 3300;
    int ARM_POS_SUB = ARM_POS_DELTA + 2925;
    int ARM_POS_HANGING = ARM_POS_DELTA + 1820;
    int ARM_POS_DOWN_HANGING = ARM_POS_DELTA + 4150;


    /**
     * Init slider motors hardware, and set their behaviors.
     * @param hardwareMap the Hardware Mappings.
     *
     * @param wristServoName the name string for wrist servo motor
     *
     */
    public intakeUnit(HardwareMap hardwareMap, String armServoName, String wristServoName,
                      String fingerServoName) {
        // Save reference to Hardware map
        this.hardwareMap = hardwareMap;

        Logging.log("init motors for finger, wrist and arm.");

         fingerServo = hardwareMap.get(Servo.class, fingerServoName);

        wristServo = hardwareMap.get(Servo.class, wristServoName);
        wristServo.setDirection(Servo.Direction.FORWARD);
        sleep(200);

        armMotor = hardwareMap.get(DcMotor.class, armServoName);
        armMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        //setArmModeRunToPosition(0);

        //resetArmPositions(Params.armIntakeCount_InitFront);
    }

    public void setWristServoPosition(double wristPos){
        wristPos = Range.clip(wristPos, WRIST_SNAP_POSITION, SWITCH_LEFT_CLOSE_POS);
        wristServo.setPosition(wristPos);
    }

    public void fingerServoOpen() {
        fingerServo.setPosition(FINGER_OPEN);
    }

    /**
     * set the target position of wrist servo motor
     * @param wristPos the target position value for wrist servo motor
     */
    public void setWristPosition(double wristPos) {
        wristServo.setPosition(wristPos);
    }

    public void setFingerPosition(double fingerpos){
        fingerServo.setPosition(fingerpos);
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

    // release arm motor
    public void releaseArmMotor() {
        setArmPosition(armMotor.getCurrentPosition());
        armMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        armMotor.setPower(0);
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
    public double getWristPosition() {
        return wristServo.getPosition();
    }

    public double getFingerPosition() {
        return fingerServo.getPosition();
    }

    private void sleep(long milliseconds) {
        try {
            Thread.sleep(milliseconds);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
    }
    /*
    public void resetArmPositions(int intakePos) {
        ARM_POS_INTAKE = intakePos;
        ARM_MIN_COUNT_POS = ARM_POS_INTAKE - 3320; //0;
        ARM_MAX_COUNT_POS = ARM_POS_INTAKE + 100; //3620;
        ARM_POS_AUTO = ARM_POS_INTAKE - 3240; //80;
        ARM_POS_READY_FOR_HANG = ARM_POS_INTAKE - 1760; // 1800
        ARM_POS_CAMERA_READ = ARM_POS_INTAKE - 1060; //2500;
        ARM_POS_MOVING_PIXEL_ON_BOARD = ARM_POS_INTAKE - 520;
        ARM_POS_UNDER_BEAM = ARM_POS_INTAKE - 260; //3100;
        ARM_POS_DROP_PURPLE = ARM_POS_INTAKE - 130; //3380;
        ARM_POS_PUSH_PROP = ARM_POS_INTAKE - 100;
        ARM_POS_INTAKE5 = ARM_POS_INTAKE - 126;

        //NEW STUFF
        //ARM_POS_GRAB_SAMPLE = 3676;
        ARM_POS_BEFORE_HANG = 973;
        ARM_POS_AFTER_HANG = 135;
        ARM_POS_HIGH_CHAMBER = 2570;
    }
    */
}