package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.ArrayList;

@TeleOp
public class firstRobotTest extends LinearOpMode {
    DcMotorEx frontRight;
    DcMotorEx frontLeft;
    DcMotorEx backRight;
    DcMotorEx backLeft;
    DcMotorEx blueLift;
    DcMotorEx blackLift;
    Servo blueServo;
    Servo blackServo;

    // Track width:
    // Wheel Radius:

    double power;

    ArrayList<Boolean> booleanArray = new ArrayList<>();
    int booleanIncrementer = 0;
    boolean a1Pressed;


    @Override
    public void runOpMode() throws InterruptedException {
        frontLeft = hardwareMap.get(DcMotorEx.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotorEx.class, "frontRight");
        backLeft = hardwareMap.get(DcMotorEx.class,"backLeft");
        backRight = hardwareMap.get(DcMotorEx.class, "backRight");
        blackLift = hardwareMap.get(DcMotorEx.class, "blackLift");
        blueLift = hardwareMap.get(DcMotorEx.class, "blueLift");
        blackServo = hardwareMap.get(Servo.class, "blackServo");
        blueServo = hardwareMap.get(Servo.class, "blueServo");

        blueLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        blackLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        blackLift.setDirection(DcMotorSimple.Direction.REVERSE);

        blueLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        blackLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode((DcMotor.RunMode.RUN_WITHOUT_ENCODER));
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        blueServo.setDirection(Servo.Direction.REVERSE);

        blueServo.setPosition(0);
        blackServo.setPosition(0);

        waitForStart();

        while(opModeIsActive() && !isStopRequested()) {
            setPower(gamepad1.left_stick_x,-gamepad1.left_stick_y, gamepad1.right_stick_x);

            if (gamepad1.dpad_up) {
                blackLift.setPower(1);
                blueLift.setPower(1);
            } else if (gamepad1.dpad_down) {
                blackLift.setPower(-1);
                blueLift.setPower(-1);
            } else {
                blueLift.setPower(0);
                blackLift.setPower(0);
            }

            if (a1Pressed) {
                if (blackServo.getPosition() == 0 && blueServo.getPosition() == 0) {
                    blueServo.setPosition(0.75);
                    blackServo.setPosition(0.75);
                } else if (blackServo.getPosition() == 0.75 && blueServo.getPosition() == 0.75) {
                    blueServo.setPosition(0);
                    blackServo.setPosition(0);
                }
            }
            a1Pressed = ifPressed(gamepad1.a);
        }
    }

    private void setPower(double y, double x, double rot) {
        double frontLeftPower = x + y + rot;
        double backLeftPower = x - y + rot;
        double frontRightPower = x - y - rot;
        double backRightPower = x + y - rot;

        if (gamepad1.right_bumper) {
            power = 0.5;
        } else if (gamepad1.left_bumper) {
            power = 0.25;
        } else {
            power = 1;
        }

        //Sets the power of the motors to the motors, which allows variable speed and movement in every direction
        frontLeft.setPower(frontLeftPower * power);
        frontRight.setPower(frontRightPower * power);
        backLeft.setPower(backLeftPower * power);
        backRight.setPower(backRightPower * power);

        telemetry.addData("Front Right", frontRightPower);
        telemetry.addData("Front Left", frontLeftPower);
        telemetry.addData("Back Left", backLeftPower);
        telemetry.addData("Back Right", backRightPower);
        telemetry.addData("X", x);
        telemetry.addData("y", y);
        telemetry.addData("rot", rot);
        telemetry.update();


    }

    //Function used to allow toggling
    private boolean ifPressed(boolean button) {
        boolean output = false;
        if (booleanArray.size() == booleanIncrementer) {
            booleanArray.add(false);
        }

        boolean buttonWas = booleanArray.get(booleanIncrementer);

        //noinspection PointlessBooleanExpression
        if (button != buttonWas && button == true) {
            output = true;
        }
        booleanArray.set(booleanIncrementer, button);

        booleanIncrementer = booleanIncrementer + 1;
        return output;
    }
}
