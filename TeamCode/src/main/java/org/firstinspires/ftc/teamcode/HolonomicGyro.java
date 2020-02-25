package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

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

    public static final double NEW_P = 10.0;//2.5
    public static final double NEW_I = 0.0;//0.1
    public static final double NEW_D = 0.0;//0.2
    public static final double NEW_F = 0.0;//0.0

    public void mySleep(double time) {//seconds
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < time)) {

        }
    }

     public void setSlides(double power, int constant) {
        if (opModeIsActive()) {
            robot.slides.setTargetPosition(robot.slides.getCurrentPosition() + constant);
            //robot.rightSlides.setTargetPosition(robot.rightSlides.getCurrentPosition() + constant);

            //robot.slides.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            //robot.rightSlides.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            robot.slides.setPower(power);
            //robot.rightSlides.setPower(power);

            //robot.slides.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            while (robot.slides.isBusy()) {
                //wait till motor finishes working
//                robot.drive(gamepad1.left_stick_x,
//                        gamepad1.left_stick_y,
//                        gamepad1.right_stick_x);
//                telemetry.addLine("Slides Extending");
//                telemetry.update();
            }

//            //robot.rightSlides.setPower(-0.4);
//            //
////            robot.slides.setPower(0);
////            robot.rightSlides.setPower(0);
        }
    }


    @Override //when init is pressed
    public void runOpMode() {
        robot.initMotors(hardwareMap);
        robot.initServos(hardwareMap);
        robot.setFront(hardwareMap);
        //robot.initIMU(hardwareMap);
        robot.useEncoders(false);//don't need encoders for teleop
        robot.initLinearSlides(hardwareMap);


        // re-read coefficients and verify change.
        PIDFCoefficients pidNew = new PIDFCoefficients(NEW_P, NEW_I, NEW_D, NEW_F);
        //robot.slides.setPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION, pidNew);

        PIDFCoefficients pidModified = robot.slides.getPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION);


        boolean yHeld = false;
        boolean y2Held = false;
        boolean useNormal = true;
        boolean initIMU = true;
        boolean xHeld = false;
        boolean capstoneUp = false;

        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {
                //forks
                double LX = gamepad1.left_stick_x, LY = -gamepad1.left_stick_y, RX = -gamepad1.right_stick_x;

                robot.hookFoundation(gamepad1.a);//pressing a changes claw position, up to down, or vice versa
                robot.setSpeed(gamepad1.left_bumper, gamepad1.right_bumper);
                if (!robot.slidesDown())
                    robot.setHinge(gamepad2.b);
                robot.setStoneArm(gamepad2.a);

                if (gamepad2.right_bumper && !robot.slidesDown())
                    setSlides(1, -100);
                else if (gamepad2.left_bumper){
                    setSlides(1, 100);
//                    robot.useSlideEncoders(true);
//                    robot.setSlides(-1);
//                    mySleep(0.2);
//                    robot.useSlideEncoders(false);
//                    robot.setSlides(-0.3);
                }
                    //setSlides(-1, -70);
                else if (gamepad2.dpad_up && robot.slidesDown())
                    setSlides(-1, -220);
                else if (gamepad2.dpad_down) {
                    robot.slides.setPower(0);
                    //robot.rightSlides.setPower(0);
                }

                //elevate slightly to move stone under bridge
                if (!y2Held && gamepad2.y) {
                    y2Held = true;
                    robot.useSlideEncoders(false);
                    robot.slides.setPower(-0.5);
                    //robot.rightSlides.setPower(-0.5);
                    mySleep(0.1);
                    robot.slides.setPower(-0.3);
                    //robot.rightSlides.setPower(-0.3);
                } else if (!gamepad2.y) {
                    y2Held = false;
                }

                if(!xHeld && gamepad1.x) {
                    xHeld = true;
                    robot.capstoneUp = !robot.capstoneUp;
                } else if(!gamepad1.x)
                    xHeld = false;


                if(robot.capstoneUp) {
                    robot.capstoneUp();
                } else
                    robot.capstoneDown();

                //press to switch between Gyro and Normal Drive
                if (!yHeld && gamepad1.y) {
                    yHeld = true;
                    useNormal = !useNormal;
                } else if (!gamepad1.y)
                    yHeld = false;

                if (!useNormal) {
                    if (initIMU) {
                        robot.initIMU(hardwareMap);
                        initIMU = false;
                    }
                    double angle = robot.getAngle();
                    if (angle < 0)
                        angle += 360.0;

                    //robot drive
                    double coords[] = robot.getComponents(LX, LY, angle);
                    double newLX = coords[0];
                    double newLY = coords[1];

                    robot.drive(newLX, newLY, -RX);
                } else {
                    if(gamepad1.left_trigger > 0.1)
                        robot.drive(gamepad1.left_stick_x*0.3, gamepad1.left_stick_y*0.3, gamepad1.right_stick_x*0.3);
                    else
                        robot.drive(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x);
                }

                //telemetry.addData("Global Heading", robot.getAngle());
                telemetry.addData("LX", gamepad1.left_stick_x);
                telemetry.addData("LY", gamepad1.left_stick_y);
                telemetry.addData("RX", gamepad1.right_stick_x);

                telemetry.addData("slide position", robot.slides.getCurrentPosition());
                telemetry.addData("P,I,D (modified)", "%.04f, %.04f, %.04f",
                    pidModified.p, pidModified.i, pidModified.d);

                telemetry.addData("capstone", robot.capstone.getPosition());
                telemetry.addData("left servo", robot.servoLeft.getPosition());
                telemetry.addData("right servo", robot.servoRight.getPosition());
                telemetry.addData("hinge", robot.hinge.getPosition());
                telemetry.addData("stoneArm", robot.stoneArm.getPosition());

                telemetry.update();
        }
    }
}


/*
double angle = robot.getAngle();
            if(angle < 0)
                angle += 360.0;

            //robot drive
            double coords[] = robot.getComponents(LX, LY, angle);
            double newLX = coords[0];
            double newLY = coords[1];

            robot.drive(newLX,newLY,RX);
 */
