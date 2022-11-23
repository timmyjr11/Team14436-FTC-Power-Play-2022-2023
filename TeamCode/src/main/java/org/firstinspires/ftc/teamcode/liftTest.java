package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp
public class liftTest extends LinearOpMode {
    DcMotorEx blueLift;
    DcMotorEx blackLift;

    @Override
    public void runOpMode() throws InterruptedException {
        blackLift = hardwareMap.get(DcMotorEx.class, "blackLift");
        blueLift = hardwareMap.get(DcMotorEx.class, "blueLift");

        blueLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        blackLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        blueLift.setDirection(DcMotorSimple.Direction.REVERSE);


        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {

            if (gamepad1.dpad_up) {
                blackLift.setPower(0.5);
                blueLift.setPower(0.5);
            } else if (gamepad1.dpad_down) {
                blackLift.setPower(-0.5);
                blueLift.setPower(-0.5);
            } else {
                blueLift.setPower(0);
                blackLift.setPower(0);
            }
        }
    }
}
