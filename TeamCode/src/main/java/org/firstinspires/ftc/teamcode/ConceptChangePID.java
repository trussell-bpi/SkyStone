package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import pkg3939.Robot3939;
/**
 * Created by tom on 9/26/17.
 * This assumes that you are using a REV Robotics Expansion Hub
 * as your DC motor controller.  This op mode uses the extended/enhanced
 * PID-related functions of the DcMotorEx class.  The REV Robotics Expansion Hub
 * supports the extended motor functions, but other controllers (such as the
 * Modern Robotics and Hitechnic DC Motor Controllers) do not.
 */

@Autonomous(name="Concept: Change PID", group = "Concept")
public class ConceptChangePID extends LinearOpMode {

    // our DC motor.
    DcMotorEx slides;
//    Robot3939 robot = new Robot3939();

    public static final double NEW_P = 10.0;//2.5
    public static final double NEW_I = 0.0;//0.1
    public static final double NEW_D = 0.0;//0.2
    public static final double NEW_F = 0.0;//0.0

    public void setSlides(double power, int constant) {
        if (opModeIsActive()) {
            slides.setTargetPosition(slides.getCurrentPosition() + constant);

            slides.setPower(power);

            while (slides.isBusy()) {

            }
        }
    }

    public void runOpMode() {
        // get reference to DC motor.
        // since we are using the Expansion Hub,
        // cast this motor to a DcMotorEx object.
        slides = (DcMotorEx)hardwareMap.get(DcMotor.class, "slides");
        slides.setDirection(DcMotorSimple.Direction.FORWARD);

        slides.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        slides.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        // wait for start command.
        waitForStart();

        // get the PID coefficients for the RUN_USING_ENCODER  modes.
        PIDFCoefficients pidOrig = slides.getPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION);

        // change coefficients using methods included with DcMotorEx class.
        PIDFCoefficients pidNew = new PIDFCoefficients(NEW_P, NEW_I, NEW_D, NEW_F);
        slides.setPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION, pidNew);

        // re-read coefficients and verify change.
        PIDFCoefficients pidModified = slides.getPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION);

        // display info to user.
        if(opModeIsActive()) {
            telemetry.addData("Runtime", "%.03f", getRuntime());
            telemetry.addData("P,I,D (orig)", "%.04f, %.04f, %.0f",
                    pidOrig.p, pidOrig.i, pidOrig.d);
            telemetry.addData("P,I,D (modified)", "%.04f, %.04f, %.04f",
                    pidModified.p, pidModified.i, pidModified.d);
            telemetry.update();

            setSlides(1, 100);
            while(opModeIsActive()) {

            }
        }
    }
}