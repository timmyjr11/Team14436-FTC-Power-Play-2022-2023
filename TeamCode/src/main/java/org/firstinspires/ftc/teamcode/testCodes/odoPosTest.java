package org.firstinspires.ftc.teamcode.testCodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.util.Encoder;
@Config
@Autonomous
public class odoPosTest extends LinearOpMode {

    private final FtcDashboard dash = FtcDashboard.getInstance();
    Encoder left;
    Encoder right;
    Encoder front;
    DcMotorEx frontRight;
    DcMotorEx backRight;
    DcMotorEx leftEncoder;

    @Override
    public void runOpMode() throws InterruptedException {
        left = new Encoder(hardwareMap.get(DcMotorEx.class, "leftEncoder"));
        right = new Encoder(hardwareMap.get(DcMotorEx.class, "frontRight"));
        front = new Encoder(hardwareMap.get(DcMotorEx.class, "backRight"));
        frontRight = hardwareMap.get(DcMotorEx.class, "frontRight");
        backRight = hardwareMap.get(DcMotorEx.class, "backRight");
        leftEncoder = hardwareMap.get(DcMotorEx.class, "leftEncoder");

        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        telemetry = new MultipleTelemetry(telemetry, dash.getTelemetry());


        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {
            telemetry.addData("Left encoder velo", left.getCorrectedVelocity());
            telemetry.addData("Left encoder pos", left.getCurrentPosition());
            telemetry.addData("Left encoder direction", left.getDirection());
            telemetry.addLine("");
            telemetry.addData("Right encoder velo", right.getCorrectedVelocity());
            telemetry.addData("Right encoder pos", right.getCurrentPosition());
            telemetry.addData("Right encoder direction", right.getDirection());
            telemetry.addLine("");
            telemetry.addData("Front encoder velo", front.getCorrectedVelocity());
            telemetry.addData("Front encoder pos", front.getCurrentPosition());
            telemetry.addData("Front encoder direction", front.getDirection());
            telemetry.update();
        }
    }
}