package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import pkg3939.Robot3939;

/**
 * Created by maryjane on 11/6/2018.
 * after 1st meet
 * mecanum wheels
 * Hi, edit
 */

@TeleOp(name="HolonomicGyro", group="Iterative Opmode")
//@Disabled
public class HolonomicGyro extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();

    //Declaration of the motors and servos goes here
    Robot3939 robot = new Robot3939();

    public static final boolean earthIsFlat = true;

    @Override //when init is pressed
    public void runOpMode(){

        //Naming, Initialization of the hardware, use this deviceName in the robot controller phone
        //use the name of the object in the code
        robot.initMotors(hardwareMap);
        robot.initServos(hardwareMap);
        robot.initIMU(hardwareMap);
        robot.setFront(hardwareMap);

        robot.useEncoders(false);

        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {
            double LX = gamepad1.left_stick_x, LY = gamepad1.left_stick_y, RX = gamepad1.right_stick_x;

            //forks
            robot.setForks(gamepad1.a);
            robot.setClaw(gamepad1.b);

            robot.setSpeed(gamepad1.left_bumper, gamepad1.right_bumper);

            double angle = robot.getAngle();
            if(angle < 0)
                angle += 360;

            //robot drive
            double coords[] = robot.returnOffsetAngle(LX, LY, angle);
            LX = coords[0];
            LY = coords[1];

            robot.drive(LX,LY,-RX);

            telemetry.addData("Drive", "Holonomic");
            telemetry.addData("Global Heading", angle);
            telemetry.addData("speed", robot.speed);
            telemetry.update();
        }
    }
}
