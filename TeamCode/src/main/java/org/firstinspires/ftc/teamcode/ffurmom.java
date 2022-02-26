package org.firstinspires.ftc.teamcode;
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

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.exception.RobotCoreException;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;


/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="Ffurmom", group="Linear Opmode")

public class ffurmom  extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotorEx leftFront = null;
    private DcMotorEx rightRear = null;
    private DcMotorEx rightFront = null;
    private DcMotorEx leftRear = null;
    private CRServo rightDuck = null;
    private CRServo leftDuck = null;
    private CRServo capstone = null;
    private Servo stopper = null;
    private DcMotor intake = null;
    private DcMotor linearslideleft = null;
    private DcMotor linearslideright = null;
    private DcMotor actuator = null;
    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        leftFront  = hardwareMap.get(DcMotorEx.class, "frontLeft");
        rightRear = hardwareMap.get(DcMotorEx.class, "backRight");
        rightFront  = hardwareMap.get(DcMotorEx.class, "frontRight");
        leftRear = hardwareMap.get(DcMotorEx.class, "backLeft");
        // dw = hardwareMap.get(CRServo.class, "intake");
        leftDuck = hardwareMap.get(CRServo.class, "leftDuck");
        rightDuck = hardwareMap.get(CRServo.class, "rightDuck");
        capstone = hardwareMap.get(CRServo.class, "capstone");
        stopper = hardwareMap.get(Servo.class, "stopper");
        intake = hardwareMap.get(DcMotor.class, "intake");

//        stopper = hardwareMap.get(Servo.class, "stopper");
        linearslideleft = hardwareMap.get(DcMotor.class, "linearslideleft");
        linearslideright = hardwareMap.get(DcMotor.class, "linearslideright");
        actuator = hardwareMap.get(DcMotor.class, "actuator");
//        DcMotorEx intake=hardwareMap.get(DcMotorEx.class, "intake");
        int county=0;
        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        linearslideleft.setDirection(DcMotorSimple.Direction.REVERSE);
        linearslideright.setDirection(DcMotorSimple.Direction.REVERSE);



        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();
        // stopper.setPosition(0);
        // run until the end of the match (driver presses STOP)
        leftFront.setDirection(DcMotor.Direction.REVERSE);
        leftRear.setDirection(DcMotor.Direction.REVERSE);
        rightRear.setDirection(DcMotor.Direction.FORWARD);
        rightFront.setDirection(DcMotor.Direction.FORWARD);
        //linearSlide.setDirection(DcMotorSimple.Direction.FORWARD);
        int power=0;
        boolean lsout=false;
        boolean position=false;
        while (opModeIsActive()) {


            // Setup a variable for each drive wheel to save power level for telemetry
            int targetflip=0;
            // Choose to drive using either Tank Mode, or POV Mode
            // Comment out the method that's not used.  The default below is POV.

            // POV Mode uses left stick to go forward, and right stick to turn.
            // - This uses basic math to combine motions and is easier to drive straight.

            // Tank Mode uses one stick to control each wheel.
            // - This requires no math, but it is hard to drive forward slowly and keep straight.
            // leftPower  = -gamepad1.left_stick_y ;
            // rightPower = -gamepad1.right_stick_y ;
            double y = -gamepad1.left_stick_y; // Remember, this is reversed!
            double x = gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
            double rx = gamepad1.right_stick_x;

            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio, but only when
            // at least one is out of the range [-1, 1]
            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower = (y + x + rx) / denominator;
            double backLeftPower = (y - x + rx) / denominator;
            double frontRightPower = (y - x - rx) / denominator;
            double backRightPower = (y + x - rx) / denominator;

//            leftFront.setPower(frontLeftPower);
//            leftRear.setPower(backLeftPower);
//            rightFront.setPower(frontRightPower);
//            rightRear.setPower(backRightPower);
            if (gamepad2.a){
                leftFront.setPower(1);
            }
            if (gamepad2.b){
                rightFront.setPower(1);
            }
            if (gamepad2.x){
                leftRear.setPower(1);
            }

            if(gamepad2.right_trigger > 0)
                intake.setPower(gamepad2.right_trigger);
            else if(gamepad2.left_trigger > 0)
                intake.setPower(-gamepad2.left_trigger);
            else{

            }

            if(gamepad2.right_bumper){
                stopper.setPosition(1);
            }
            else if(gamepad2.left_bumper){
                stopper.setPosition(0);
            }

            while (gamepad2.a){
                capstone.setPower(1);
            }
            while (gamepad2.b){
                capstone.setPower(-1);
            }


            if(gamepad2.dpad_up){
                leftDuck.setPower(-1);
            }
            else if(gamepad2.dpad_down){
                leftDuck.setPower(1);
            }
            if(gamepad2.dpad_left){
                rightDuck.setPower(-1);
            }

            else if(gamepad2.dpad_right){
                rightDuck.setPower(1);
            }
            if(gamepad2.left_bumper){
                leftDuck.setPower(0);
                rightDuck.setPower(0);
            }

            if(gamepad1.dpad_up){
                leftDuck.setPower(-1);
            }
            else if(gamepad1.dpad_down){
                leftDuck.setPower(1);
            }
            if(gamepad1.dpad_left){
                rightDuck.setPower(-1);
            }
            else if(gamepad1.dpad_right){
                rightDuck.setPower(1);
            }
            if(gamepad1.left_bumper){
                leftDuck.setPower(0);
                rightDuck.setPower(0);
            }


        }
    }
}

