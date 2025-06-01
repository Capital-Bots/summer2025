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
import com.acmerobotics.roadrunner.Trajectory;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.acmerobotics.roadrunner.ftc.Encoder;
import com.acmerobotics.roadrunner.ftc.OverflowEncoder;
import com.acmerobotics.roadrunner.ftc.RawEncoder;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.HardwareClasses.testHardware;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.Arrays;

@Autonomous(name="NotBucketAuto", group="RedSide")
public class AutoRedNotBucket extends LinearOpMode {
    private final testHardware robot = new testHardware();
    private ElapsedTime runtime = new ElapsedTime();

    public void runOpMode() throws InterruptedException {
        Pose2d initialPose = new Pose2d(-34, -62, Math.toRadians(90));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);
        robot.init(hardwareMap);
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
                    .lineToY(50);
            Actions.runBlocking(action.build());
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