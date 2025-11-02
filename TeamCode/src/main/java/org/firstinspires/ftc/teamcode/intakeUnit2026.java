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
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;


/**
 * This is NOT an opmode.
 *
 * This hardware class assumes the following device names have been configured on the robot:
 * Note:  All names are case sensitive.
 * Motors type: motors for intake and launcher.
 *
 * 1. The intake motor: intake
 * 2. The launcher motor: launcher
 */
public class intakeUnit2026
{
    double intakePower = -0.86;
    double farPower = 0.98; //Power for launching from far triangle
    double closePower = 0.52;//0.5 ; //Power for launching from close triangle(x=1, y=1)
    double trigger_close = 0.05;
    double trigger_open = 0.4;


    HardwareMap hardwareMap;
    private final DcMotor intakeMotor;
    private final DcMotor launcherMotor;
    private Servo triggerServo = null;


    public intakeUnit2026(HardwareMap hardwareMap, String launcher, String intake, String trigger) {
        // Save reference to Hardware map
        this.hardwareMap = hardwareMap;

        // Define and Initialize Motors
        intakeMotor = hardwareMap.get(DcMotor.class, intake);
        launcherMotor = hardwareMap.get(DcMotor.class, launcher);

        /*
        The motor is to do its best to run at targeted velocity.
        An encoder must be affixed to the motor in order to use this mode. This is a PID mode.
         */
        launcherMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //launcherMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // trigger servo, initial with closed position
        triggerServo = hardwareMap.get(Servo.class, trigger);
        triggerServo.setPosition(0.0);
    }

    public void startIntake() {
        intakeMotor.setPower(intakePower);
    }

    public void stopIntake() {
        intakeMotor.setPower(0.0);
    }

    public void startLauncher() {
        launcherMotor.setPower(closePower);
    }

    // always close trigger when launcher stops to avoid artifact stuck between launcher wheels.
    public void stopLauncher() {
        launcherMotor.setPower(0.0);
        triggerClose();
    }

    public  double getLauncherPower() {
        return launcherMotor.getPower();
    }

    public  int getLauncherPos() {
        return launcherMotor.getCurrentPosition();
    }

    /*
    Close trigger servo during intake
     */
    public void triggerClose() {
        triggerServo.setPosition(trigger_close);
    }

    /*
    Open trigger servo before launch
     */
    public void triggerOpen() {
        triggerServo.setPosition(trigger_open);
    }

    /*
    Get trigger servo position for display during debug/testing
     */
    public double getTriggerPosition() {
        return triggerServo.getPosition();
    }
}