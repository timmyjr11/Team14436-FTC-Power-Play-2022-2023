package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp
public class robot extends LinearOpMode {
    DcMotor frontLeft;
    DcMotor frontRight;
    DcMotor backLeft;
    DcMotor backRight;

    @Override
    public void runOpMode() {
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");

        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();

        while(opModeIsActive()) {
            frontLeft.setPower(-gamepad1.left_stick_y);
            frontRight.setPower(-gamepad1.left_stick_y);
            backLeft.setPower(-gamepad1.left_stick_y);
            backRight.setPower(-gamepad1.left_stick_y);

            frontLeft.setPower(gamepad1.right_stick_x);
            frontRight.setPower(-gamepad1.right_stick_x);
            backLeft.setPower(gamepad1.right_stick_x);
            backRight.setPower(-gamepad1.right_stick_x);
            theGenreOfTim();
        }
    }

    private void theGenreOfTim(){
        telemetry.addLine("Ello");
        telemetry.addData("anything", frontLeft.getPower());
        telemetry.addData("rob",frontRight.getPower());
        telemetry.addData("tim is a LOSER",backLeft.getPower());
        telemetry.addData("tim is a poopy head",backRight.getPower());
        telemetry.update();

    }

}
