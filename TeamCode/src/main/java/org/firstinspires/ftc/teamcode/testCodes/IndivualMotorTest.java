package org.firstinspires.ftc.teamcode.testCodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Config
@Autonomous
@Disabled
public class IndivualMotorTest extends LinearOpMode {

    DcMotorEx frontRight;
    DcMotorEx frontLeft;
    DcMotorEx backRight;
    DcMotorEx backLeft;

    public static double frontLeftPower = 0;
    public static double backLeftPower = 0;
    public static double frontRightPower = 0;
    public static double backRightPower = 0;

    FtcDashboard dashboard;

    @Override
    public void runOpMode() throws InterruptedException {
        frontLeft = hardwareMap.get(DcMotorEx.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotorEx.class, "frontRight");
        backLeft = hardwareMap.get(DcMotorEx.class,"backLeft");
        backRight = hardwareMap.get(DcMotorEx.class, "backRight");

        dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());


        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode((DcMotor.RunMode.RUN_WITHOUT_ENCODER));
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        waitForStart();

        while(opModeIsActive() && !isStopRequested()) {
            frontLeft.setPower(frontLeftPower);
            frontRight.setPower(frontRightPower);
            backLeft.setPower(backLeftPower);
            backRight.setPower(backRightPower);

            telemetry.addData("Front Right", frontRight.getPower());
            telemetry.addData("Front Left", frontLeft.getPower());
            telemetry.addData("Back Left", backLeft.getPower());
            telemetry.addData("Back Right", backRight.getPower());
            telemetry.update();
        }
    }
}
