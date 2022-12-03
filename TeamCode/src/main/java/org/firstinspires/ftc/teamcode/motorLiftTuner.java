package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.outoftheboxrobotics.photoncore.PhotonCore;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.PoseStorage;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

import java.util.ArrayList;

@Config
@TeleOp
public class motorLiftTuner extends LinearOpMode {
    FtcDashboard dashboard = FtcDashboard.getInstance();

    DcMotorEx blackLift;
    DcMotorEx blueLift;

    public static double bluePower = 0;
    public static double blackPower = 0;

    @Override
    public void runOpMode() throws InterruptedException {

        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        blueLift = hardwareMap.get(DcMotorEx.class, "blueLift");
        blackLift = hardwareMap.get(DcMotorEx.class, "blackLift");

        blueLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        blackLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        blueLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        blackLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        blackLift.setDirection(DcMotorSimple.Direction.REVERSE);


        waitForStart();

        while(opModeIsActive() && !isStopRequested()) {
            blueLift.setPower(bluePower);
            blackLift.setPower(blackPower);
        }
    }
}
