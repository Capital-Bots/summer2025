package org.firstinspires.ftc.teamcode.roadrunner.tele;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.HardwareClasses.testHardware;
@Disabled
@TeleOp(name="PIDTesting", group="Linear Opmode")
public class PIDTesting extends LinearOpMode{
    private testHardware robot = new testHardware();
    private ElapsedTime timer = new ElapsedTime();
    double integralSum = 0;
    double Kp = 0.5;
    double Ki = 0;
    double Kd = 0;
    double Kf = 0;
    DcMotorEx motor;
    double armPosition = 0;
    double position = 0;

    @Override
    public void runOpMode(){
        motor = hardwareMap.get(DcMotorEx.class, "intakeArm");
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        PIDFController controller = new PIDFController(0,0,0,0);

        waitForStart();
        timer.reset();
        controller.setPIDF(Kp, Ki, Kd, Kf);
        controller.setSetPoint(100);
        double armVel = 0;
        while (!isStopRequested() && opModeIsActive() && timer.milliseconds()<6000){
            armPosition = motor.getCurrentPosition();
            armVel = controller.calculate(armPosition);
            motor.setVelocity(armVel);
            telemetry.addData("Positional Error: " , controller.getPositionError());
            telemetry.addData("Positional Error Tolerance: ", controller.getTolerance()[0]);
            telemetry.addData("Slide Encoder Val", armPosition);
            telemetry.addData("At point? : ", controller.atSetPoint());
            telemetry.addData("Current P: ", controller.getP());
            telemetry.addData("Current I: ", controller.getI());
            telemetry.addData("Current D: ", controller.getD());
            telemetry.update();
        }
        motor.setPower(0);
    }
}