package org.firstinspires.ftc.teamcode.teachingCode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
@Disabled
@TeleOp
public class Anything_You_Want extends LinearOpMode {

    DcMotorEx backShooter;
    DcMotorEx frontShooter;
    DcMotorEx convey;
    DcMotorEx intake;

    Servo rightPivot;
    Servo leftPivot;
    Servo armPivot;
    Servo Gripper;
    Servo stopper;
    Servo tapper;
    @Override
    public void runOpMode() throws InterruptedException {
        backShooter = hardwareMap.get(DcMotorEx.class,"backShooter");
        frontShooter = hardwareMap.get(DcMotorEx.class,"frontShooter");
        convey = hardwareMap.get(DcMotorEx.class,"convey");
        intake = hardwareMap.get(DcMotorEx.class,"intake");

        rightPivot = hardwareMap.get(Servo.class,"rightPivot");
        leftPivot = hardwareMap.get(Servo.class,"leftPivot");
        armPivot = hardwareMap.get(Servo.class,"armPivot");
        Gripper = hardwareMap.get(Servo.class,"Gripper");
        stopper = hardwareMap.get(Servo.class,"stopper");
        tapper = hardwareMap.get(Servo.class,"tapper");

        frontShooter.setDirection(DcMotorSimple.Direction.REVERSE);
        backShooter.setDirection(DcMotorSimple.Direction.REVERSE);
        convey.setDirection(DcMotorSimple.Direction.REVERSE);
        intake.setDirection(DcMotorSimple.Direction.REVERSE);
        leftPivot.setDirection(Servo.Direction.REVERSE);
        frontShooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backShooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        waitForStart();

        while(opModeIsActive()){
            if(gamepad1.a){
                if(armPivot.getPosition()==1){
                    armPivot.setPosition(0);
                } else if(armPivot.getPosition()==0){
                    armPivot.setPosition(1);
                }
            }
        }


    }
}
