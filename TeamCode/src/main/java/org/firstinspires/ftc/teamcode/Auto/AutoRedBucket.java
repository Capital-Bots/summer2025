package org.firstinspires.ftc.teamcode.Auto;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.AngularVelConstraint;
import com.acmerobotics.roadrunner.MinVelConstraint;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.HardwareClasses.testHardware;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.Arrays;

@Autonomous(name="BucketAuto", group="RedSide")
public class AutoRedBucket extends LinearOpMode {
    public class Lift {
        private DcMotorEx lift;
        private DcMotorEx lift2;

        public Lift(HardwareMap hardwareMap) {
            lift = hardwareMap.get(DcMotorEx.class, "rightSlide");
            lift2 = hardwareMap.get(DcMotorEx.class, "leftSlide");
            lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            lift2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            lift.setDirection(DcMotorSimple.Direction.FORWARD);
            lift2.setDirection(DcMotorSimple.Direction.REVERSE);
        }
        public class LiftUp implements Action {
            private boolean initialized = false;
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    lift.setPower(1);
                    lift2.setPower(1);
                    initialized = true;
                }
                double pos = lift.getCurrentPosition();
                packet.put("liftPos", pos);
                if (pos < 11594) {
                    return true;
                } else {
                    lift.setPower(0);
                    lift2.setPower(0);
                    return false;
                }
            }
        }
        public Action liftUp() {
            return new LiftUp();
        }

        public class LiftDown implements Action {
            private boolean initialized = false;
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    lift.setPower(-1);
                    lift2.setPower(-1);
                    initialized = true;
                }
                double pos = lift.getCurrentPosition();
                packet.put("liftPos", pos);
                if (pos > 100.0) {
                    return true;
                } else {
                    lift.setPower(0);
                    return false;
                }
            }
        }
        public Action liftDown() {
            return new LiftDown();
        }
    }

    private final testHardware robot = new testHardware();
    private ElapsedTime runtime = new ElapsedTime();

    public void runOpMode() throws InterruptedException {
        Pose2d initialPose = new Pose2d(-34, -62, Math.toRadians(90));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);
        robot.init(hardwareMap);
        Lift lift = new Lift(hardwareMap);
        Vector2d basketPos = new Vector2d(-56, -60);

        waitForStart();
        robot.clawRotate.setPosition(1);
        robot.claw.setPosition(0);
        double slideInitPos = robot.rightSlide.getCurrentPosition();

        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        telemetry.addData("Right slide current position: ", robot.rightSlide.getCurrentPosition());

        if (!isStopRequested() && opModeIsActive()) {
            TrajectoryActionBuilder action = drive.actionBuilder(initialPose)
                    .strafeTo(new Vector2d(-34, -42))
                    .waitSeconds(0.5)
                    .turn(Math.toRadians(-45))
                    .waitSeconds(0.5)
                    .strafeTo(basketPos)
                    .waitSeconds(0.5);
            TrajectoryActionBuilder action2 = action.endTrajectory().fresh()
                    .waitSeconds(1)
                    .strafeTo(new Vector2d(-55,-62), new MinVelConstraint(Arrays.asList(
                            new TranslationalVelConstraint(9),
                            new AngularVelConstraint(Math.PI / 2)
                    )));
            TrajectoryActionBuilder midAction = action2.endTrajectory().fresh()
                    .strafeTo(basketPos);
            TrajectoryActionBuilder action3 = midAction.endTrajectory().fresh()
                    .strafeToLinearHeading(new Vector2d(-42, -51), Math.toRadians(90));
            TrajectoryActionBuilder waitingAction = drive.actionBuilder(drive.pose)
                    .waitSeconds(0.5);
            TrajectoryActionBuilder action4 = action3.endTrajectory().fresh()
                    .strafeTo(basketPos)
                    .waitSeconds(0.5);
            TrajectoryActionBuilder action5 = action4.endTrajectory().fresh()
                    .strafeTo(new Vector2d(-55,-63));
            //drop preload
//                    .turn(Math.toRadians(45))
//                    .strafeTo(new Vector2d(-48.5, -32)) //grab first sample
//                    .strafeTo(basketPos)
//                    .turn(Math.toRadians(-45))
//                    //drop sample
//                    .turn(Math.toRadians(45))
//                    .strafeTo(new Vector2d(-57, -32))
//                    .strafeTo(basketPos)
//                    .turn(Math.toRadians(-45))
//                    //drop sample
//                    .turn(Math.toRadians(45))
////                THIRD SAMPLE
////                .lineToY(-26)
////                .turn(Math.toRadians(90))
////                .lineToX(-60)
////                .turn(Math.toRadians(-90))
////                .strafeTo(basketPos)
////                .turn(Math.toRadians(-45))
//                    //drop sample
//                    .splineToLinearHeading(new Pose2d(-22, 0, Math.toRadians(-180)), Math.PI/2);
            lift.lift.setPower(.65);
            lift.lift2.setPower(.65);
            Actions.runBlocking(new SequentialAction(action.build(), action2.build()));
            lift.lift.setPower(0);
            lift.lift2.setPower(0);
//            while (robot.rightSlide.getCurrentPosition() > (slideInitPos - 11594)) {
//                robot.rightSlide.setPower(-1);
//                robot.leftSlide.setPower(-1);
//            }
//            robot.rightSlide.setPower(0);
//            robot.leftSlide.setPower(0);


            Servo release = hardwareMap.get(Servo.class, "release");
            robot.release.setPosition(0.999);
            sleep(500);
            Actions.runBlocking(midAction.build());
            lift.lift.setPower(-1);
            lift.lift2.setPower(-1);
            Actions.runBlocking(new SequentialAction(waitingAction.build(), action3.build(), waitingAction.build()));
            double i = runtime.milliseconds();
            while (runtime.milliseconds() < i + 425) robot.intakeArm.setPower(0.5);
            robot.intakeArm.setPower(0);
            sleep(750);
            robot.claw.setPosition(0.8);
            robot.release.setPosition(0.15);
            sleep(750);
            robot.clawRotate.setPosition(0);
            sleep(750);

            double f = runtime.milliseconds();
            while (runtime.milliseconds() < f + 1000) robot.intakeArm.setPower(-0.5);

            robot.claw.setPosition(0);
            robot.intakeArm.setPower(0);
            sleep(500);
            robot.clawRotate.setPosition(1);
            sleep(500);
            lift.lift.setPower(1);
            lift.lift2.setPower(1);
            Actions.runBlocking(action4.build());
            sleep(1000);
            robot.rightFrontDrive.setPower(-0.4);
            robot.rightBackDrive.setPower(-0.4);
            sleep(1000);
            robot.rightFrontDrive.setPower(0);
            robot.rightBackDrive.setPower(0);
            robot.leftSlide.setPower(0);
            robot.rightSlide.setPower(0);
            sleep(200);
            robot.release.setPosition(1);
            sleep(1000);
            robot.rightFrontDrive.setPower(0.5);
            robot.leftFrontDrive.setPower(0.5);
            robot.rightBackDrive.setPower(0.5);
            robot.leftBackDrive.setPower(0.5);
            sleep(1000);
        }
    }
}


//MEEP MEEP
//This is the associated MeepMeep with the above code. To run it, switch over to the "MeepMeepTesting"
//branch and run "myClass".
//Vector2d basketPos = new Vector2d(-56, -56);
//        Vector2d releasePos = new Vector2d(-58,-58);
//        Vector2d firstPiecePos = new Vector2d(-48.5,-33.25);
//        Vector2d secondPiecePos = new Vector2d(-57,-33.25);
////        Vector2d thirdPiecePos = new Vector2d()
//        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
//                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
//                .setColorScheme(new ColorSchemeRedLight())
//                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
//                .setDimensions(17.5, 14.5)
//                .setStartPose(new Pose2d(-34, -63, Math.toRadians(90)))
//                .build();
//        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(-34, -63, Math.toRadians(90)))
//                .strafeTo(basketPos)
//                .waitSeconds(1)
//                .turn(Math.toRadians(-45))
//                .waitSeconds(1)
//                .strafeTo(releasePos)
//                .waitSeconds(1)
//                //release preload
//                .strafeTo(basketPos)
//                .waitSeconds(1)
//                .strafeToSplineHeading(firstPiecePos, Math.toRadians(90)) //grab first sample
//                .waitSeconds(1)
//                .strafeToSplineHeading(basketPos, Math.toRadians(45))
//                .waitSeconds(1)
//                .strafeToSplineHeading(releasePos, Math.toRadians(45))
//                .waitSeconds(1)
//                .strafeToSplineHeading(basketPos, Math.toRadians(45))
//                .waitSeconds(1)
//                .strafeToSplineHeading(secondPiecePos, Math.toRadians(90))
//                .waitSeconds(1)
//                //drop sample
//                .strafeToSplineHeading(basketPos, Math.toRadians(45))
//                .waitSeconds(1)
//                .strafeToSplineHeading(releasePos, Math.toRadians(45))
//                .waitSeconds(1)
//                .strafeToSplineHeading(basketPos, Math.toRadians(45))
//                .waitSeconds(1)
//                //drop sample
////                THIRD SAMPLE
//                .splineToLinearHeading(new Pose2d(-55, -36, Math.toRadians(90)), Math.PI/2)
//                .waitSeconds(1)
//                .splineToLinearHeading(new Pose2d(-34, -9, Math.toRadians(90)), Math.PI/2)
//                .waitSeconds(1)
//                .splineToLinearHeading(new Pose2d(-25, -12, Math.toRadians(90)), Math.PI/2)
//                .waitSeconds(1)