package org.firstinspires.ftc.teamcode.roadrunner.tele;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.HardwareClasses.testHardware;
import org.firstinspires.ftc.teamcode.MecanumDrive;
@TeleOp(group = "drive", name = "compTele")
public class fieldCentricTeleTest extends LinearOpMode {
//     double forwardHeading = Math.toRadians(180);
    private final testHardware robot = new testHardware();
    boolean isResetRequested;
    @Override
    public void runOpMode() throws InterruptedException {
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0,0,0));
        robot.init(hardwareMap);

        waitForStart();
        boolean lastButtonRotate = false;
        boolean toggleRotate = false;
        boolean lastButtonForClaw = false;
        boolean toggleClaw = false;

        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());


        if (isStopRequested()) return;
        while (opModeIsActive() && !isStopRequested()) {

            // Create a vector from the gamepad x/y inputs
            Vector2d input = new Vector2d(
                    gamepad1.left_stick_x,
                    gamepad1.left_stick_y
            );
            // Then, rotate that vector by the inverse of that heading
            double robotHeading = drive.pose.heading.toDouble();
            double xComp = input.x;
            double yComp = -1 * input.y;
            double inputHeading = Math.atan2(yComp, xComp);
            double inputMagnitude = Math.sqrt(xComp*xComp + yComp*yComp);
            inputHeading -= robotHeading;
            isResetRequested = gamepad1.y;

            double finalX = inputMagnitude * Math.cos(inputHeading);
            double finalY = inputMagnitude * Math.sin(inputHeading);
            double turnComponent = gamepad1.right_stick_x;
            double SPEED_MULTIPLIER = 0.92;
            double normalizingFactor = Math.max(Math.abs(finalY)
                    + Math.abs(finalX) + Math.abs(turnComponent), 1);

            if (isResetRequested){
                drive.pose = new Pose2d(new Vector2d(0,0), 0);
            }

            double fl = SPEED_MULTIPLIER * (finalY + finalX + turnComponent) / normalizingFactor;
            double fr = SPEED_MULTIPLIER * (finalY - finalX - turnComponent) / normalizingFactor;
            double bl = SPEED_MULTIPLIER * (finalY - finalX + turnComponent) / normalizingFactor;
            double br = SPEED_MULTIPLIER * (finalY + finalX - turnComponent) / normalizingFactor;

            drive.updatePoseEstimate();

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
            boolean gunnerAuto = gamepad2.y;
            double slowRotateLeft = gamepad1.left_trigger;
            double slowRotateRight = gamepad1.right_trigger;


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

            if ((claw) && !lastButtonForClaw){
                if (toggleClaw){
                    robot.claw.setPosition(1);
                    toggleClaw = false;
                }
                else{
                    robot.claw.setPosition(0);
                    toggleClaw = true;
                }
                lastButtonForClaw = true;
            }if (!(claw||driverClaw)) lastButtonForClaw =  false;

            //Driver's Claw Automation

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

            if (intakeArmOut) robot.intakeArm.setPower(-0.65);
            else if (intakeArmIn) robot.intakeArm.setPower(0.65);
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

            robot.leftFrontDrive.setPower(fl);
            robot.leftBackDrive.setPower(bl);
            robot.rightFrontDrive.setPower(fr);
            robot.rightBackDrive.setPower(br);
            telemetry.addData("heading", robotHeading);
            telemetry.addData("input heading", inputHeading);
            telemetry.addData("x-comp", input.x);
            telemetry.addData("y-comp", input.y);
            telemetry.addData("Right slide current position: ", robot.rightSlide.getCurrentPosition());

            telemetry.update();



        }
    }
}