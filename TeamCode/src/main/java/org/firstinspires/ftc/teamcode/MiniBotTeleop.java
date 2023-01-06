package org.firstinspires.ftc.teamcode;
import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp(name="MiniTeleop")

public class MiniBotTeleop extends LinearOpMode{

    minibot robot = new minibot();


    private double speedControl = 0.5;
    private double turnControl = 0.5;
    @Override


    public void runOpMode()
    {
        robot.init(hardwareMap,this);


        waitForStart();

        while (opModeIsActive()) {




            telemetry.update();


            if (gamepad1.right_stick_y != 0 || gamepad1.right_stick_x != 0) {
                robot.motorLeft.setPower((gamepad1.right_stick_y - gamepad1.left_stick_x * 0.35) * speedControl);
                robot.motorRight.setPower((-gamepad1.right_stick_y - gamepad1.left_stick_x * 0.35) * speedControl);
                robot.motorFront.setPower((-gamepad1.right_stick_x - gamepad1.left_stick_x * 0.35) * speedControl);
                robot.motorBack.setPower((gamepad1.right_stick_x - gamepad1.left_stick_x * 0.35) * speedControl);
            } else if (gamepad1.left_stick_x != 0 || gamepad1.left_stick_y != 0) {
                robot.motorLeft.setPower(-gamepad1.left_stick_x * turnControl);
                robot.motorRight.setPower(-gamepad1.left_stick_x * turnControl);
                robot.motorFront.setPower(-gamepad1.left_stick_x * turnControl);
                robot.motorBack.setPower(-gamepad1.left_stick_x * turnControl);
            } else {

                robot.motorLeft.setPower(0);
                robot.motorRight.setPower(0);
                robot.motorFront.setPower(0);
                robot.motorBack.setPower(0);
            }


            if (gamepad1.dpad_up) speedControl = 1.0;
            if (gamepad1.dpad_left) speedControl = 0.75;
            if (gamepad1.dpad_right) speedControl = 0.5;
            if (gamepad1.dpad_down) speedControl = 0.25;


            if (gamepad1.dpad_up) turnControl = 0.75;
            if (gamepad1.dpad_left) turnControl = 0.5;
            if (gamepad1.dpad_right) turnControl = 0.35;
            if (gamepad1.dpad_down) turnControl = 0.25;



            if (gamepad2.right_stick_y<0) robot.arm.setPower(gamepad2.right_stick_y * 0.5);
            else if (gamepad2.right_stick_y>0) robot.arm.setPower(gamepad2.right_stick_y * 0.25);
            else if (gamepad2.b) robot.arm.setPower(-0.4);
            else robot.arm.setPower(0);





        }}}