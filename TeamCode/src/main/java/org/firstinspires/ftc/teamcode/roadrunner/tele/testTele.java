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

package org.firstinspires.ftc.teamcode.roadrunner.tele;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.HardwareClasses.testHardware;


/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When a selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="testTele", group="Linear Opmode")

public class testTele extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();
    private testHardware robot = new testHardware();

    @Override
    public void runOpMode() throws InterruptedException {

        robot.init(hardwareMap);
        waitForStart();
        runtime.reset();

        boolean lastButtonRotate = false;
        boolean toggleRotate = false;
        boolean lastButtonForClaw = false;
        boolean toggleClaw = false;

        while (opModeIsActive()) {

            // Variable Declarations

            double verticalComponent = -1 * gamepad1.left_stick_y;
            double lateralComponent = gamepad1.left_stick_x;
            double turnComponent = gamepad1.right_stick_x;
            double slideMoveUp = gamepad2.right_trigger;
            double slideMoveDown = gamepad2.left_trigger;
            boolean intakeArmOut = gamepad2.left_bumper;
            boolean intakeArmIn = gamepad2.right_bumper;
            boolean claw = gamepad2.a;
            boolean driverClaw = gamepad1.a;
            boolean rotateClaw = gamepad2.b;
            boolean releases = gamepad2.x;
            boolean slowFront = gamepad2.dpad_up;
            boolean slowBack = gamepad2.dpad_down;
            boolean slowLeft = gamepad2.dpad_left;
            boolean slowRight = gamepad2.dpad_right;
            boolean slowFront1 = gamepad1.dpad_up;
            boolean slowBack1 = gamepad1.dpad_down;
            boolean slowLeft1 = gamepad1.dpad_left;
            boolean slowRight1 = gamepad1.dpad_right;
            double slowRotateLeft = gamepad1.left_trigger;
            double slowRotateRight = gamepad1.right_trigger;

            double SPEED_MULTIPLIER = 0.9;

            //Drivetrain Movements
            double normalizingFactor = Math.max(Math.abs(verticalComponent)
                    + Math.abs(lateralComponent) + Math.abs(turnComponent), 1);

            double fl = SPEED_MULTIPLIER * (verticalComponent + lateralComponent + turnComponent) / normalizingFactor;
            double fr = SPEED_MULTIPLIER * (verticalComponent - lateralComponent - turnComponent) / normalizingFactor;
            double bl = SPEED_MULTIPLIER * (verticalComponent - lateralComponent + turnComponent) / normalizingFactor;
            double br = SPEED_MULTIPLIER * (verticalComponent + lateralComponent - turnComponent) / normalizingFactor;

            //Slow Movements - DPAD - for both driver and gunner

            if (slowFront || slowFront1) {
                fl = 0.35;
                fr = 0.35;
                bl = 0.35;
                br = 0.35;
            } else if (slowBack || slowBack1) {
                fl = 0.35 * -1;
                fr = 0.35 * -1;
                bl = 0.35 * -1;
                br = 0.35 * -1;
            } else if (slowLeft || slowLeft1) {
                fl = -1 * 0.5;
                fr = 0.5;
                bl = 0.5;
                br = -1 * 0.5;
            } else if (slowRight || slowRight1) {
                fl = 0.5;
                fr = -1 * 0.5;
                bl = -1 * 0.5;
                br = 0.5;
            }

            //Slow Rotation - for driver only

            if (slowRotateLeft > 0){
                fl = -0.3;
                fr = 0.3;
                bl = -0.3;
                br = 0.3;
            }if (slowRotateRight > 0){
                fl = 0.3;
                fr = -0.3;
                bl = 0.3;
                br = -0.3;
            }

            //Claw, with toggle

            if ((claw || driverClaw) && !lastButtonForClaw){
                if (toggleClaw){
                    robot.claw.setPosition(1);
                    toggleClaw = false;
                }
                else{
                    robot.claw.setPosition(0);
                    toggleClaw = true;
                }
                lastButtonForClaw = true;
            }if (!(claw||driverClaw)) lastButtonForClaw = false;

            //Release Mechanism, hold and release

            if (releases) robot.release.setPosition(1);
            else robot.release.setPosition(0.15);

            //Claw Rotation, with toggle

            if (rotateClaw && !lastButtonRotate) {
                if (toggleRotate){
                    robot.clawRotate.setPosition(0);
                    toggleRotate = false;
                }
                else{
                    robot.clawRotate.setPosition(1);
                    toggleRotate = true;
                }
                lastButtonRotate = true;
            }if (!rotateClaw) lastButtonRotate = false;

            //Intake Arm

            if (intakeArmOut) robot.intakeArm.setPower(-0.3);
            else if (intakeArmIn) robot.intakeArm.setPower(0.3);
            else robot.intakeArm.setPower(0);

            //Slide Movements

            if (slideMoveDown > 0) {
                robot.rightSlide.setPower(slideMoveDown * 0.95);
                robot.leftSlide.setPower(slideMoveDown);
            } else if (slideMoveUp > 0) {
                robot.rightSlide.setPower(-0.95 * slideMoveUp);
                robot.leftSlide.setPower(-1 * slideMoveUp);
            } else {
                robot.rightSlide.setPower(0);
                robot.leftSlide.setPower(0);
            }

            //Power assignments for Drivetrain (after all processing)

            robot.leftFrontDrive.setPower(fl);
            robot.rightFrontDrive.setPower(fr);
            robot.leftBackDrive.setPower(bl);
            robot.rightBackDrive.setPower(br);

            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Left Bumper", intakeArmOut);
            telemetry.addData("Right Bumper", intakeArmIn);
            telemetry.addData("lastButtonForClaw", lastButtonForClaw);
            telemetry.addData("lastPressedClaw",toggleClaw);
            telemetry.update();
        }
    }
}