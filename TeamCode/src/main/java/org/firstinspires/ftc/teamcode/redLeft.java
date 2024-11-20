package org.firstinspires.ftc.teamcode;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import java.util.Timer;
import java.util.TimerTask;


import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

// scoring x cord =
// scoring y cord =

//taking brick x cord = 27
@Autonomous
public class redLeft extends LinearOpMode {

    private DcMotor armMotor = null;
    private DcMotor liftMotor = null;

    private CRServo Claw = null;
    private Servo Turn = null;


    public static final int ARM_COUNTS_PER_MOTOR_REV = 1996;
    public static final double ARM_DEGREES_PER_COUNT = 360.0 / ARM_COUNTS_PER_MOTOR_REV;

    public double armTargetAngle = 65.0;

    @Override
    public void runOpMode() throws InterruptedException {



        armMotor = hardwareMap.get(DcMotor.class, "Arm");
        liftMotor = hardwareMap.get(DcMotor.class, "Lift");

        Claw = hardwareMap.get(CRServo.class, "Claw");
        Turn = hardwareMap.get(Servo.class, "Turn");

        armMotor.setTargetPosition(360);

        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        setArmPosition(armTargetAngle);
        Turn.setPosition(0.01);

        Claw.setPower(-0.4);
        sleep(50);


        telemetry.addLine("Waiting for Start");
        telemetry.update();
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);



        resetRuntime();
        waitForStart();
        if (opModeIsActive()) {
            //setArmPosition(65)
            TrajectorySequence traj = drive.trajectorySequenceBuilder(new Pose2d())
                    .forward(28)
                    .turn(Math.toRadians(-50))
                    .addDisplacementMarker(()->{
                        setArmPosition(100);
                        Turn.setPosition(0.32);
                        telemetry.addLine("I am inside the marker");
                        telemetry.update();
                    })
                    .lineToLinearHeading(new Pose2d(9,20,Math.toRadians(-50)))
                    .addDisplacementMarker(()->{
                        Turn.setPosition(0);
                    })
                    .forward(4)
                    .addTemporalMarker(7,()->{
                        liftMotor.setPower(-0.6);
                    })
                    .addTemporalMarker(10,()->{
                        liftMotor.setPower(0);
                        Claw.setPower(0.1);
                    })
                    .back(5)
                    .waitSeconds(5)
                    /*
                    .forward(4)
                    .lineToLinearHeading(new Pose2d(5,22,Math.toRadians(-50)))
                    .addDisplacementMarker(()->{
                        Claw.setPower(0.3);
                        Timer timer = new Timer();
                        timer.schedule(new TimerTask() {
                            @Override
                            public void run() {
                                Claw.setPower(0);
                            }
                        }, 75);
                    })
                    .forward(2)


                    */
                    .build();
            drive.followTrajectorySequence(traj);
            telemetry.addLine("This is after turn");
            telemetry.addData("Time", getRuntime());
            telemetry.update();



        }
    }
    public void setArmPosition(double angle) {
        int targetPosition = (int) (angle / ARM_DEGREES_PER_COUNT);
        armMotor.setTargetPosition(targetPosition);
        armMotor.setPower(0.2);
    }

    public void setLiftPosition (double motorTargetPos) {
        int value = (int) motorTargetPos;
        liftMotor.setTargetPosition(value);
        liftMotor.setPower(0.7);
    }

}
