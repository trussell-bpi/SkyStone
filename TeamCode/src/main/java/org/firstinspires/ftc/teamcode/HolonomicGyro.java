package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import com.qualcomm.robotcore.hardware.PIDFCoefficients;
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

    public static final int SWING_HEIGHT = 1000;//height at which we can swing the hinge
    public static final int MAX_HEIGHT = 2205;//max ticks
    public static final int MAX_STACK_COUNT = 9;//number of stones max
    public static final int MINIMUM_HEIGHT = 0;//ticks

    public static final int STAGE1 = 0;
    public static final int STAGE2 = 400;
    public static final int STAGE3 = 630;
    public static final int STAGE4 = 870;
    public static final int STAGE5 = 1110;
    public static final int STAGE6 = 1370;
    public static final int STAGE7 = 1600;
    public static final int STAGE8 = 1840;
    public static final int STAGE9 = 2090;

    public static int STAGE = 0;

    public void mySleep(double time) {//seconds
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < time)) {

        }
    }

     public void setSlides(double power, int constant) {
        if (opModeIsActive()) {
            int tickDifference = Math.abs(robot.slides.getCurrentPosition() - constant);
            robot.slides.setTargetPosition(constant);

            robot.slides.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            if(constant == MINIMUM_HEIGHT && tickDifference < 60) {
                robot.slides.setPower(0.3);
            } else
                robot.slides.setPower(power);


            while (robot.slides.isBusy()) {
                //wait till motor finishes working
                robot.drive(gamepad1.left_stick_x,
                        gamepad1.left_stick_y,
                        gamepad1.right_stick_x);
                telemetry.addLine("Slides Extending");
                telemetry.update();
            }
        }
     }

    public void runToStage(int targetStage) {
        switch(targetStage) {
            case 0:
                setSlides(1, MINIMUM_HEIGHT);
                break;
            case 1:
                setSlides(1, STAGE1);
                break;
            case 2:
                setSlides(1, STAGE2);
                break;
            case 3:
                setSlides(1, STAGE3);
                break;
            case 4:
                setSlides(1, STAGE4);
                break;
            case 5:
                setSlides(1, STAGE5);
                break;
            case 6:
                setSlides(1, STAGE6);
                break;
            case 7:
                setSlides(1, STAGE7);
                break;
            case 8:
                setSlides(1, STAGE8);
                break;
            case 9:
                setSlides(1, STAGE9);
                break;
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

        boolean yHeld = false;
        boolean y2Held = false;
        boolean x2Held = false;
        boolean RB2Held = false;
        boolean LB2Held = false;
        boolean useNormal = true;
        boolean initIMU = true;
        boolean xHeld = false;
        boolean capstoneUp = false;


        waitForStart();

        //robot.slides.setVelocityPIDFCoefficients(10, 0, 0, 0);

        //robot.slides.setPositionPIDFCoefficients(10);


        //int motorIndex = ((DcMotorEx)lift);

        PIDFCoefficients pidRUNUSINGENCODERS = robot.slides.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);

        PIDFCoefficients pidRUNTOPOSITION = robot.slides.getPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION);

        runtime.reset();

        while (opModeIsActive()) {
                //forks
                double LX = gamepad1.left_stick_x, LY = -gamepad1.left_stick_y, RX = -gamepad1.right_stick_x;

                robot.hookFoundation(gamepad1.a);//pressing a changes claw position, up to down, or vice versa
                robot.setSpeed(gamepad1.left_bumper, gamepad1.right_bumper);

                if (robot.slides.getCurrentPosition() > SWING_HEIGHT)
                    robot.setHinge(gamepad2.b);

                robot.setStoneArm(gamepad2.a);

                    //setSlides(-1, -70);
                if (gamepad2.dpad_up && robot.slidesDown())
                    setSlides(1, MAX_HEIGHT);
                else if (gamepad2.dpad_down) {
                    setSlides(1,0);
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
                    if(gamepad1.left_trigger > 0.1) {
                        robot.useEncoders(true);
                        robot.drive(gamepad1.left_stick_x * 0.3, gamepad1.left_stick_y * 0.3, gamepad1.right_stick_x * 0.3);
                    } else {
                        robot.useEncoders(false);
                        robot.drive(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x);
                    }
                }

                //increment stage count
                if (!RB2Held && gamepad2.right_bumper) {
                    RB2Held = true;
                    STAGE++;

                } else if (!gamepad2.right_bumper) {
                    RB2Held = false;
                }

                //decrement stage count
                if (!LB2Held && gamepad2.left_bumper) {
                    LB2Held = true;
                    STAGE--;

                } else if (!gamepad2.left_bumper) {
                    LB2Held = false;
                }

                STAGE = Range.clip(STAGE, 0, MAX_STACK_COUNT);


                //go to position(STAGE)
                if(!x2Held && gamepad2.x) {
                    x2Held = true;
                    runToStage(STAGE);
                } if(!gamepad2.x) {
                    x2Held = false;
                }

                if (!y2Held && gamepad2.y) {
                    y2Held = true;
                    STAGE = 0;
                    setSlides(1, MINIMUM_HEIGHT);
                } else if (!gamepad2.y) {
                    y2Held = false;
                }

                //manually fine tune slide height
                if(gamepad2.right_trigger > 0.1) {
                    if(MAX_HEIGHT - robot.slides.getCurrentPosition() > gamepad2.right_trigger * 30)
                        setSlides(1, (int)(robot.slides.getCurrentPosition() + gamepad2.right_trigger * 100));
                } else if(gamepad2.left_trigger > 0.1 && (robot.slides.getCurrentPosition() > 10)) {
                    if(robot.slides.getCurrentPosition() - MINIMUM_HEIGHT > gamepad2.left_trigger * 50)
                        setSlides(0.5, (int)(robot.slides.getCurrentPosition() - gamepad2.left_trigger * 100));
                }




                //telemetry.addData("Global Heading", robot.getAngle());
                telemetry.addData("LX", gamepad1.left_stick_x);
                telemetry.addData("LY", gamepad1.left_stick_y);
                telemetry.addData("RX", gamepad1.right_stick_x);

                telemetry.addData("slide position", robot.slides.getCurrentPosition());
                telemetry.addData("P,I,D (modified)", "%.04f, %.04f, %.04f. %.04f",
                    pidRUNUSINGENCODERS.p, pidRUNUSINGENCODERS.i, pidRUNUSINGENCODERS.d, pidRUNUSINGENCODERS.f);

                telemetry.addData("P,I,D (modified)", "%.04f, %.04f, %.04f. %.04f",
                    pidRUNTOPOSITION.p, pidRUNTOPOSITION.i, pidRUNTOPOSITION.d, pidRUNTOPOSITION.f);

                telemetry.addData("current stage number", STAGE);
                telemetry.addData("capstone", robot.capstone.getPosition());
                telemetry.addData("left servo", robot.servoLeft.getPosition());
                telemetry.addData("right servo", robot.servoRight.getPosition());
                telemetry.addData("hinge", robot.hinge.getPosition());
                telemetry.addData("stoneArm", robot.stoneArm.getPosition());

                telemetry.update();
        }
    }
}