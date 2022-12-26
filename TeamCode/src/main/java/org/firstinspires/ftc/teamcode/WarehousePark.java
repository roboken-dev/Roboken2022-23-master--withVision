package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name="WarehousePark")

public class WarehousePark extends LinearOpMode {


    minibot robot = new minibot();


    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap, this);
        waitForStart();

        robot.encoderSideDrive(0.25,20,5,this);
    }}