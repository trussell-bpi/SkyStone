//package org.firstinspires.ftc.teamcode;
//
//import com.qualcomm.robotcore.eventloop.opmode.Disabled;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.util.ElapsedTime;
//
//import pkg3939.Robot3939;
//
///**
// * Created by maryjane on 11/6/2018.
// * after 1st meet
// * mecanum wheels
// * Hi, edit
// */
//
//@TeleOp(name="Holonomic", group="Iterative Opmode")
//@Disabled
//public class Holonomic extends LinearOpMode {
//    private ElapsedTime runtime = new ElapsedTime();
//
//    //Declaration of the motors and servos goes here
//    Robot3939 robot = new Robot3939();
//
//    public static final boolean earthIsFlat = true;
//
//    public void setSlides(double power, int constant) {
//        if (opModeIsActive()) {
//            robot.slides.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//            robot.rightSlides.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//
//            robot.slides.setTargetPosition(robot.slides.getCurrentPosition() + constant);
//            robot.rightSlides.setTargetPosition(robot.rightSlides.getCurrentPosition() + constant);
//
//            robot.slides.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            robot.rightSlides.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//
//            robot.slides.setPower(power);
//            robot.rightSlides.setPower(power);
//
//            runtime.reset();
//
//            while (robot.slides.isBusy() || robot.rightSlides.isBusy()) {
//                //wait till motor finishes working
//                robot.drive(gamepad1.left_stick_x,
//                        gamepad1.left_stick_y,
//                        gamepad1.right_stick_x);
//                telemetry.addLine("Slides Extending");
//                telemetry.update();
//                if (runtime.seconds() > 1.2)
//                    break;
//            }
//            telemetry.addLine("Extended");
//            telemetry.update();
//
//            robot.slides.setPower(0);
//            robot.rightSlides.setPower(0);
//
//            robot.slides.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//            robot.rightSlides.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//
//            robot.slides.setPower(-0.25);
//            robot.rightSlides.setPower(-0.25);
//            //
////            robot.slides.setPower(0);
////            robot.rightSlides.setPower(0);
//        }
//    }
//
//    @Override //when init is pressed
//    public void runOpMode() {
//        robot.initMotors(hardwareMap);
//        robot.initServos(hardwareMap);
//        robot.setFront(hardwareMap);
//        //robot.initIMU(hardwareMap);
//        robot.useEncoders(false);//don't need encoders for teleop
//        robot.initLinearSlides(hardwareMap);
//
//        boolean driver = true;
//        boolean yHeld = false;
//        boolean dUpHeld = false;
//
//        waitForStart();
//        runtime.reset();
//
//        while (opModeIsActive()) {
//            //forks
//            robot.hookFoundation(gamepad1.a);//pressing a changes claw position, up to down, or vice versa
//            robot.setSpeed(gamepad1.left_bumper, gamepad1.right_bumper);
//
//            if(!robot.slidesDown())
//                robot.setHinge(gamepad2.b);
//            robot.setStoneArm(gamepad2.a);
//            if (gamepad2.right_bumper && !robot.slidesDown())
//                setSlides(0.5, 20);
//            else if (gamepad2.left_bumper)
//                setSlides(1, -50);
//            else if (gamepad2.y && robot.slidesDown())
//                setSlides(1, -225);
//            else if (gamepad2.x) {
//                robot.slides.setPower(0);
//                robot.rightSlides.setPower(0);
//            }
//
//            if (!yHeld && gamepad1.y) {
//                yHeld = true;
//                driver = !driver;
//            } else if (!gamepad1.y) {
//                yHeld = false;
//            }
//
//            if(!dUpHeld && gamepad2.dpad_up) {
//                dUpHeld = true;
//                driver = !driver;
//            } else if(gamepad2.dpad_up)
//                dUpHeld = false;
//
//            if (driver)
//                robot.drive(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x);
//            else
//                robot.drive(gamepad2.left_stick_x * 0.6,
//                        gamepad2.left_stick_y * 0.6,
//                        gamepad2.right_stick_x * 0.6);
//
//                telemetry.addData("Drive", "Holonomic");
//                //telemetry.addData("Global Heading", robot.getAngle());
//                telemetry.addData("LX", gamepad1.left_stick_x);
//                telemetry.addData("LY", gamepad1.left_stick_y);
//                telemetry.addData("RX", gamepad1.right_stick_y);
//
//                telemetry.addData("speed", robot.speed);
//                telemetry.addData("left servo", robot.servoLeft.getPosition());
//                telemetry.addData("right servo", robot.servoRight.getPosition());
//                telemetry.addData("hinge", robot.hinge.getPosition());
//                telemetry.addData("stoneArm", robot.stoneArm.getPosition());
//
//                telemetry.update();
//            }
//        }
//    }
