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
    private Servo armServo = null;


    final double FINGER_INTAKE_POS = 0;
    final double FINGER_STOP_POS = 0.5;
    final double FINGER_OUTTAKE_POS = 1.0;

    //wrist servo
    private Servo wristServo = null;
    final double SWITCH_RIGHT_CLOSE_POS = 0.27;
    final double ARM_FORE_POSITION = 0.36;

    final double SWITCH_LEFT_CLOSE_POS = 0.21;
    final double WRIST_SNAP_POSITION = 0.03;

    //finger servo
    public Servo fingerServo = null;
    final double FINGER_CLOSE_POS = 0.2;  // Minimum rotational position
    final double FINGER_OPEN_POS = 0.95; // Maximum rotational position
    final double WRIST_POS_DROP_TURN = 0.30;
    final double WRIST_POS_DROP_PURPLE = 0.35;
    final double WRIST_POS_DROP_YELLOW_BACK = 0.36;

    final double WRIST_POS_DROP_YELLOW = 0.39;
    final double WRIST_POS_DROP_WHITE = 0.42;
    final double WRIST_POS_DROP = 0.46;
    final double WRIST_POS_INTAKE5 = 0.455;
    final double WRIST_POS_INTAKE = 0.475;
    final double WRIST_POS_REACH_FORWARD = 0.5;

    // slider motor variables
    //public DcMotor sliderOneMotor = null;
    //public DcMotor sliderTwoMotor = null;
    int ARM_POS_INTAKE = Params.armIntakeCount_InitFront;
    int ARM_MIN_COUNT_POS;
    int ARM_MAX_COUNT_POS;
    int ARM_POS_AUTO;
    int ARM_POS_HANG;
    int ARM_POS_READY_FOR_HANG;
    int ARM_POS_DROP;
    int ARM_POS_CAMERA_READ;
    int ARM_POS_DROP_WHITE;
    int ARM_POS_DROP_YELLOW;
    int ARM_POS_DROP_YELLOW_BACK;
    int ARM_POS_UNDER_BEAM;
    int ARM_POS_MOVING_PIXEL_ON_BOARD;
    int ARM_POS_DROP_PURPLE;
    int ARM_POS_PUSH_PROP;
    int ARM_POS_INTAKE2;
    int ARM_POS_INTAKE3;
    int ARM_POS_INTAKE4;
    int ARM_POS_INTAKE5;
    int ARM_POS_INTAKE_WHITE2;


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
        //switchRightServo = hardwareMap.get(Servo.class, rightSwitchName);
        //switchRightServo.setPosition(SWITCH_RIGHT_CLOSE_POS);

        //switchLeftServo = hardwareMap.get(Servo.class, leftSwitchName);
        //switchLeftServo.setPosition(SWITCH_LEFT_CLOSE_POS);

        fingerServo = hardwareMap.get(Servo.class, fingerServoName);
        //fingerStop();

        wristServo = hardwareMap.get(Servo.class, wristServoName);
        wristServo.setDirection(Servo.Direction.FORWARD);
        sleep(200);

        armServo = hardwareMap.get(Servo.class, armServoName);

        /*
        sliderOneMotor = hardwareMap.get(DcMotor.class, sliderOneMotorName);
        sliderOneMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        sliderTwoMotor = hardwareMap.get(DcMotor.class, sliderOneMotorName);
        sliderTwoMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

         */

        //resetArmPositions(Params.armIntakeCount_InitFront);
    }

    public void setArmServoPosition(double armPos) {
        armPos = Range.clip(armPos, SWITCH_RIGHT_CLOSE_POS, ARM_FORE_POSITION);
        armServo.setPosition(armPos);
    }

    public void setWristServoPosition(double wristPos){
        wristPos = Range.clip(wristPos, WRIST_SNAP_POSITION, SWITCH_LEFT_CLOSE_POS);
        wristServo.setPosition(wristPos);
    }

    public void fingerServoOpen() {
        fingerServo.setPosition(FINGER_OPEN_POS);
    }

    public void fingerServoClose() {
        fingerServo.setPosition(FINGER_CLOSE_POS);
    }

    /*
    public void extendSlide(int amount) {
        sliderOneMotor.setTargetPosition(sliderOneMotor.getCurrentPosition() + amount);
        sliderOneMotor.setTargetPosition(sliderTwoMotor.getCurrentPosition() - amount);
    }

    public void setCountPosition(int sliderMotorPosition) {
        sliderMotorPosition = Range.clip(sliderMotorPosition, 0, 1000);
        sliderOneMotor.setTargetPosition(sliderMotorPosition);
        sliderTwoMotor.setTargetPosition(sliderMotorPosition);
    }

     */

    public void armServoForward() {
        setArmServoPosition(ARM_FORE_POSITION);
    }

    public void wristServoSnapSpecimen(){
        setWristServoPosition(WRIST_SNAP_POSITION);
    }

    public void switchServoClose() {
        setArmServoPosition(SWITCH_RIGHT_CLOSE_POS);
        setWristServoPosition(SWITCH_LEFT_CLOSE_POS);
    }

    /**
     * set the target position of wrist servo motor
     * @param wristPos the target position value for wrist servo motor
     */
    public void setWristPosition(double wristPos) {
        wristPos = Range.clip(wristPos, FINGER_CLOSE_POS, FINGER_OPEN_POS);
        wristServo.setPosition(wristPos);
    }

    public void setFingerPosition(double fingerpos){
        fingerServo.setPosition(fingerpos);
    }
    /**
     * set the wrist servo motor position to open the wrist
     */
    public void wristUp() {
        setWristPosition(wristServo.getPosition() + 0.001);
    }

    /**
     * set the wrist servo motor position to close the wrist
     */
    public void wristDown() {
        setWristPosition(wristServo.getPosition() - 0.001);
    }

    // Finger servo control methods.
    public void fingerIntake() {
        //fingerServo.setPosition(FINGER_INTAKE_POS);
    }
    public void fingerStop() {
        //fingerServo.setPosition(FINGER_STOP_POS);
    }
    public void fingerOuttake() {
        //fingerServo.setPosition(FINGER_OUTTAKE_POS);
    }

    /**
     * set the target position of arm servo motor
     * @param armPos the target position value for arm servo motor
     */
    public void setArmPosition(double armPos) {
        armServo.setPosition(armPos);
    }

    public void setArmModeRunToPosition(double armPos) {
        setArmPosition(armPos);
        //sliderOneMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //sliderOneMotor.setPower(0.9);
    }
    public void resetArmEncoder() {
        //sliderOneMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setArmModeRunToPosition(0);

        //Logging.log("Arm Motor mode = %s",  sliderOneMotor.getMode());
        //Logging.log("Arm Motor curr position = %d",  sliderOneMotor.getCurrentPosition());
        //Logging.log("Arm Motor target position = %d",  sliderOneMotor.getTargetPosition());
    }

    /*
    public void armUp() {
        setArmCountPosition(sliderOneMotor.getCurrentPosition() + 5);
    }

    public void armDown() {
        setArmCountPosition(sliderOneMotor.getCurrentPosition() - 5);
    }

    public void armLiftAcc() {
        setArmCountPosition(sliderOneMotor.getCurrentPosition() + 50);
    }

    public void armDownAcc() {
        setArmCountPosition(sliderOneMotor.getCurrentPosition() - 50);
    }

    public void hangingRobot() {
        setArmCountPosition(ARM_POS_HANG);
    }

    // auto setting positions
    /*
    public void intakePositions(int armPosition) {
        setArmCountPosition(armPosition);
        wristServo.setPosition(WRIST_POS_INTAKE);
        switchServoClose();
        fingerIntake();
    }

    public void intakeWhite2Positions() {
        setArmCountPosition(ARM_POS_INTAKE_WHITE2);
        wristServo.setPosition(WRIST_POS_INTAKE5);
        switchServoClose();
        fingerIntake();
    }

    public void parkingPositions() {
        setArmCountPosition(ARM_POS_INTAKE);
        wristServo.setPosition(WRIST_POS_INTAKE);
        switchServoClose();
        fingerStop();
    }



    public void dropPositions() {
        setArmCountPosition(ARM_POS_DROP);
        wristServo.setPosition(WRIST_POS_DROP);
        switchServoClose();
        fingerStop();
    }

    public void dropWhitePositions() {
        setArmCountPosition(ARM_POS_DROP_WHITE);
        wristServo.setPosition(WRIST_POS_DROP_WHITE);
        switchServoClose();
        fingerStop();
    }

     */

    public void autonomousInit() {

    }

    /*
    public void readyToDropPurple() {
        setArmCountPosition(ARM_POS_DROP_PURPLE);
        wristServo.setPosition(WRIST_POS_DROP_PURPLE);
        switchServoClose();
        fingerStop();
    }



    public void pushPropPose() {
        setArmCountPosition(ARM_POS_PUSH_PROP);
        wristServo.setPosition(WRIST_POS_INTAKE);
        switchServoClose();
    }
    */

    public void readyToDropYellow(int armPosition){
        setArmPosition(armPosition);
        wristServo.setPosition(WRIST_POS_DROP_YELLOW);
        switchServoClose();
    }

    public void readyToDropWhite(int armPosition){
        setArmPosition(armPosition);
        wristServo.setPosition(WRIST_POS_DROP_WHITE);
        switchServoClose();
    }

    public void underTheBeam(){
        setArmPosition(ARM_POS_UNDER_BEAM);
        wristServo.setPosition(WRIST_POS_DROP_PURPLE);
    }

    public void movingPixelPosition() {
        setArmPosition(ARM_POS_MOVING_PIXEL_ON_BOARD);
        wristServo.setPosition(WRIST_POS_REACH_FORWARD);
    }

    public void underTheBeamIntake(){
        setArmPosition(ARM_POS_UNDER_BEAM);
        wristServo.setPosition(WRIST_POS_INTAKE);
    }
    /**
     * Get the arm servo motor current position value
     * @return the current arm servo motor position value
     */
    public double getArmPosition() {
        return armServo.getPosition();
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



    /*
    public double getSwitchRightPosition() {
        return switchRightServo.getPosition();
    }
    public double getSwitchLeftPosition() {
        return switchLeftServo.getPosition();
    }

     */


    private void sleep(long milliseconds) {
        try {
            Thread.sleep(milliseconds);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
    }

    public void resetArmPositions(int intakePos) {
        ARM_POS_INTAKE = intakePos;
        ARM_MIN_COUNT_POS = ARM_POS_INTAKE - 3320; //0;
        ARM_MAX_COUNT_POS = ARM_POS_INTAKE + 100; //3620; 
        ARM_POS_AUTO = ARM_POS_INTAKE - 3240; //80;
        ARM_POS_HANG = ARM_POS_INTAKE - 3000; //500;
        ARM_POS_READY_FOR_HANG = ARM_POS_INTAKE - 1760; // 1800
        ARM_POS_DROP = ARM_POS_INTAKE - 1000; //2550;
        ARM_POS_CAMERA_READ = ARM_POS_INTAKE - 1060; //2500;
        ARM_POS_DROP_WHITE = ARM_POS_INTAKE - 750; //2800;
        ARM_POS_DROP_YELLOW = ARM_POS_INTAKE - 635; //2800;
        ARM_POS_MOVING_PIXEL_ON_BOARD = ARM_POS_INTAKE - 520;
        ARM_POS_DROP_YELLOW_BACK = ARM_POS_INTAKE - 500;
        ARM_POS_UNDER_BEAM = ARM_POS_INTAKE - 260; //3100;
        ARM_POS_DROP_PURPLE = ARM_POS_INTAKE - 130; //3380;
        ARM_POS_PUSH_PROP = ARM_POS_INTAKE - 100;
        ARM_POS_INTAKE2 = ARM_POS_INTAKE - 29;
        ARM_POS_INTAKE3 = ARM_POS_INTAKE - 58;
        ARM_POS_INTAKE4 = ARM_POS_INTAKE - 87;
        ARM_POS_INTAKE_WHITE2 = ARM_POS_INTAKE - 116;
        ARM_POS_INTAKE5 = ARM_POS_INTAKE - 126;
    }
}