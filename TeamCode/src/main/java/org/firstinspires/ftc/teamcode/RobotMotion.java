package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class RobotMotion {
    HardwareMap hardwareMap =  null;
    DcMotor motorLeftFront = null;
    DcMotor motorLeftBack = null;
    DcMotor motorRightFront = null;
    DcMotor motorRightBack = null;

    public RobotMotion(HardwareMap hwMap) {
        hardwareMap = hwMap;
        motorLeftFront = hardwareMap.dcMotor.get("left_front");
        motorLeftBack = hardwareMap.dcMotor.get("left_back");
        motorRightFront = hardwareMap.dcMotor.get("right_front");
        motorRightBack = hardwareMap.dcMotor.get("right_back");

        motorLeftFront.setDirection(DcMotor.Direction.FORWARD);
        motorLeftBack.setDirection(DcMotor.Direction.FORWARD);
        motorRightFront.setDirection(DcMotor.Direction.REVERSE);
        motorRightBack.setDirection(DcMotor.Direction.REVERSE);

        motorLeftFront.setPower(0);
        motorLeftBack.setPower(0);
        motorRightFront.setPower(0);
        motorRightBack.setPower(0);

        motorLeftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorLeftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorRightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorRightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void moveRobotForward(double power) {
        motorLeftFront.setPower(power);
        motorLeftBack.setPower(power);
        motorRightFront.setPower(power);
        motorRightBack.setPower(power);
    }

    public void strafeRight(double power) {
        motorLeftFront.setPower(power);
        motorLeftBack.setPower(-power);
        motorRightFront.setPower(-power);
        motorRightBack.setPower(power);
    }

    public void strafeLeft(double power) {
        motorLeftFront.setPower(-power);
        motorLeftBack.setPower(power);
        motorRightFront.setPower(power);
        motorRightBack.setPower(-power);
    }

    public void turnRight(double power) {
        motorLeftFront.setPower(power);
        motorLeftBack.setPower(power);
        motorRightFront.setPower(-power);
        motorRightBack.setPower(-power);
    }

    public void turnLeft(double power) {
        motorLeftFront.setPower(-power);
        motorLeftBack.setPower(-power);
        motorRightFront.setPower(power);
        motorRightBack.setPower(power);
    }

    public void stopRobot() {
        motorLeftFront.setPower(0);
        motorLeftBack.setPower(0);
        motorRightFront.setPower(0);
        motorRightBack.setPower(0);
    }
}
