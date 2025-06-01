package org.firstinspires.ftc.teamcode.roadrunner.tele;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
@Disabled
@TeleOp
public class AprilTagTest extends LinearOpMode{

    @Override
    public void runOpMode() throws InterruptedException {

        //Create the tag processor, and tell it to draw the axes, cube, outline, and ID
        //Set tag family and library to current
        AprilTagProcessor tagProcessor = new AprilTagProcessor.Builder()
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setDrawTagID(true)
                .setDrawTagOutline(true)
                .setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                .setTagLibrary(AprilTagGameDatabase.getCenterStageTagLibrary())
                .build();

        //Connect camera to tag processor through vision portal
        VisionPortal visionPortal = new VisionPortal.Builder()
                .addProcessor(tagProcessor)
                .setCamera(hardwareMap.get(WebcamName.class,"Webcam 1"))
                .setCameraResolution(new Size(640,480))
                .build();

        waitForStart();

        while (!isStopRequested()&& opModeIsActive()){
            //In case of not detecting all tags, use a try-catch loop
            try {
                if (tagProcessor.getDetections().size() > 0) {
                    AprilTagDetection tag1 = tagProcessor.getDetections().get(0);
                    AprilTagDetection tag2 = tagProcessor.getDetections().get(1);
                    AprilTagDetection tag3 = tagProcessor.getDetections().get(2);

                    //First one detected
                    telemetry.addData("x1", tag1.ftcPose.x);
                    telemetry.addData("y1", tag1.ftcPose.y);
                    telemetry.addData("z1", tag1.ftcPose.z);
                    telemetry.addData("roll1", tag1.ftcPose.roll);
                    telemetry.addData("pitch1", tag1.ftcPose.pitch);
                    telemetry.addData("yaw1", tag1.ftcPose.yaw);

                    //Second tag detected
                    telemetry.addData("x2", tag2.ftcPose.x);
                    telemetry.addData("y2", tag2.ftcPose.y);
                    telemetry.addData("z2", tag2.ftcPose.z);
                    telemetry.addData("roll2", tag2.ftcPose.roll);
                    telemetry.addData("pitch2", tag2.ftcPose.pitch);
                    telemetry.addData("yaw2", tag2.ftcPose.yaw);

                    //Third tag detected
                    telemetry.addData("x3", tag3.ftcPose.x);
                    telemetry.addData("y3", tag3.ftcPose.y);
                    telemetry.addData("z3", tag3.ftcPose.z);
                    telemetry.addData("roll3", tag3.ftcPose.roll);
                    telemetry.addData("pitch3", tag3.ftcPose.pitch);
                    telemetry.addData("yaw3", tag3.ftcPose.yaw);
                }
            } catch(Exception e) {}
            telemetry.update();
        }
    }
}

