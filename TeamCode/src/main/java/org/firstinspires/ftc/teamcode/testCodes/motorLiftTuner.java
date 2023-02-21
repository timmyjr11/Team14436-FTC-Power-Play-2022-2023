package org.firstinspires.ftc.teamcode.testCodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

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
            blackLift.setPower(bluePower);
            telemetry.addData("blue lift", blueLift.getCurrentPosition());
            telemetry.addData("black lift", blackLift.getCurrentPosition());
            telemetry.update();
        }
    }
}
