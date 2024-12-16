package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.Gamepad;

/**
 * Game pad buttons design:
 * Game Pad1:
 *  Left stick:
 *      left-right: strafe robot
 *      up-down:    drive root
 *  Right stick:
 *      left-right: turn robot
 *  dpad:
 *      left:       robot movement speed down
 *      right:      robot movement speed up
 *      up:         normal auto pickup the 3rd cone from cone stack
 *      down:       normal auto pickup the 4th and 5th cone from cone stack
 *  X:              robot movement speed down (same as dpad_left)
 *  B:              robot movement speed down (same as dpad_right)
 *  Left Bumper:    normal auto pickup cone from ground
 *  Right bumper:   normal auto drop off cone
 *  Left trigger:   auto pick up cone from cone base then move to high junction, slider lifted to high junction
 *  Right trigger:  auto drop off cone on high junction then move to cone base
 *  Y:              Teapot function - drop off cone on high junction then pick up cone from base, then move to high junction
 *  Back:           Back to cone base just at the beginning of Teleop
 *  A:              open claw to drop off cone manually.*
 * Game pad2:
 *  dpad:
 *      Up:         close the claw to pick up cone
 *      Down:       open claw to drop off cone manually
 *  Right stick:
 *                  left-right: manually control slider up and down
 *  X:              move slider to wall position
 *  A:              move slider to low junction position
 *  B:              move slider to medium junction position
 *  Y:              move slider to high junction position
 *  Right bumper:   move slider to ground junction position
 *  Right trigger:  move slider to ground position
 *  Left stick:
 *      left-right: manually control arm position
 *      up:         move arm to pick up position
 *      down:       move arm to drop off position
 *
 */
public class GamePadButtons {
    //game pad setting
    public float robotDrive;
    public float robotStrafe;
    public float robotTurn;
    public boolean speedDown;
    public boolean speedUp;
    public boolean wristLeft;
    public boolean wristRight;
    public boolean speedCtrl;
    public boolean wristFront;
    public boolean wristBack;

    public boolean sliderUp;
    public float sliderUpDown;
    public boolean sliderDown;

    public boolean armForwards;
    public boolean armBackwards;

    public boolean knuckleUp;
    public boolean knuckleDown;

    public boolean fingerOpenCloseBack;
    public boolean fingerOpenClose;

    public boolean SpecimenHangAlign;
    public boolean SpecimenHangAction;
    public boolean quitCycle;

    //public boolean SubPickupPos;
    public boolean PickupSampleIntakePos;
    public boolean LowBucketPos;
    public boolean EndgameHangingLineup;
    public boolean EndgameHangingPos;

    public boolean SpecimenPickupWallPos;
    public boolean SpecimenCycleWall;

    public void checkGamepadButtons(@NonNull Gamepad gamepad1, @NonNull Gamepad gamepad2) {
        //game pad 1 buttons

        //driving
        /*
        robotDrive = gamepad1.left_stick_y;
        robotStrafe = gamepad1.left_stick_x;
        robotTurn = gamepad1.right_stick_x;
         */

        robotDrive = gamepad1.left_stick_y * gamepad1.left_stick_y * gamepad1.left_stick_y;
        robotStrafe = gamepad1.left_stick_x * gamepad1.left_stick_x * gamepad1.left_stick_x;
        robotTurn = gamepad1.right_stick_x * gamepad1.right_stick_x * gamepad1.right_stick_x;

        //speed controls
        speedCtrl = gamepad1.right_bumper || gamepad2.back;
        speedDown = speedCtrl;

        //finger
        fingerOpenClose = gamepad2.dpad_up;
        fingerOpenCloseBack = gamepad2.dpad_down;

        //knuckle
        knuckleUp = gamepad2.left_stick_y < -0.3;
        knuckleDown = gamepad2.left_stick_y > 0.3;

        //wrists
        wristLeft = gamepad2.right_trigger > 0  &&  gamepad2.a;
        wristRight = gamepad2.right_trigger > 0  &&  gamepad2.b;
        wristBack = gamepad2.dpad_left;
        wristFront = gamepad2.dpad_right;

        //arm
        armForwards = gamepad2.right_stick_y > 0;
        armBackwards = gamepad2.right_stick_y < 0;

        //slider
        sliderUp = gamepad2.right_stick_y > 0;
        sliderDown = gamepad2.right_stick_y < 0;
        sliderUpDown  = gamepad2.right_stick_y;

        //specimen presets
        SpecimenHangAlign = gamepad1.left_trigger > 0;
        SpecimenHangAction = gamepad1.left_bumper;

        //alternative after hanging procedures
        SpecimenCycleWall = gamepad1.y;
        quitCycle = gamepad1.x;

        PickupSampleIntakePos =  gamepad2.right_bumper;

        LowBucketPos = gamepad2.left_bumper;
        EndgameHangingLineup = gamepad2.x;
        EndgameHangingPos = gamepad2.y;

        SpecimenPickupWallPos =  gamepad2.left_trigger > 0;

    }
}
