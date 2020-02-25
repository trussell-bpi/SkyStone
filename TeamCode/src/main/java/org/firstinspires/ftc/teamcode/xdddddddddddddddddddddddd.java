package org.firstinspires.ftc.teamcode;
import java.lang.Math;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;


@TeleOp(name="xdddddddddddddddddddddddd", group="Iterative Opmode")
@Disabled
public class xdddddddddddddddddddddddd extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();

    private DcMotor RF, RR, LF, LR;
    private Servo FCL, FCR, Cap, Hinge, Claw;
    double RFPower, RRPower, LFPower, LRPower;
    boolean aHeld=false;
    private boolean foundationDown = false;

    @Override
    public void runOpMode() {
        RF= hardwareMap.dcMotor.get("Right Front Motor");
        RR= hardwareMap.dcMotor.get("Right Rear Motor");
        LF= hardwareMap.dcMotor.get("Left Front Motor");
        LR= hardwareMap.dcMotor.get("Left Rear Motor");

        FCL = hardwareMap.servo.get("Foundation Claw Left");
        FCR = hardwareMap.servo.get("Foundation Claw Right");
        Cap = hardwareMap.servo.get("Capstone Placer");
        Hinge = hardwareMap.servo.get("Hinge");
        Claw = hardwareMap.servo.get("Claw");

        RF.setDirection(DcMotor.Direction.FORWARD);
        RR.setDirection(DcMotor.Direction.FORWARD);
        LF.setDirection(DcMotor.Direction.REVERSE);
        LR.setDirection(DcMotor.Direction.REVERSE);

        RF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        LF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        LR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        waitForStart();
        while (opModeIsActive()) {
            double LX =gamepad1.left_stick_x;
            double LY=gamepad1.left_stick_y;
            double RX=gamepad1.right_stick_x;
            if (Math.abs(LX) > .1||Math.abs(LY)>.1||Math.abs(RX)>.1) {
                LFPower = LY - LX + RX;
                RFPower = LY + LX - RX;
                RRPower = LY - LX - RX;
                LRPower = LY + LX + RX;
            } else{
                LFPower = 0;
                RFPower = 0;
                RRPower = 0;
                LRPower = 0;
            }

            double maxPower=Math.max(LFPower,Math.max(RFPower,Math.max(RRPower,LRPower)));

            if (maxPower>1) {
                LFPower=LFPower/maxPower;
                RFPower=RFPower/maxPower;
                RRPower=RRPower/maxPower;
                LRPower=LRPower/maxPower;
            }

            LF.setPower(LFPower);
            RF.setPower(RFPower);
            RR.setPower(RRPower);
            LR.setPower(LRPower);

            if (!aHeld & gamepad1.a) {
                foundationDown=!foundationDown;

            } else if(!gamepad1.a) {
                aHeld=false;
            }
            if (foundationDown) {
                FCR.setPosition(.5);
                FCL.setPosition(.5);
            } else {
                FCR.setPosition(0);
                FCL.setPosition(0);
            }
        }
    }
}
