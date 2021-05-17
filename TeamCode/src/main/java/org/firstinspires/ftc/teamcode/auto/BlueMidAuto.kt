package org.firstinspires.ftc.teamcode.auto

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.hardware.DcMotor
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive
import org.firstinspires.ftc.teamcode.util.Intake
import org.firstinspires.ftc.teamcode.util.WobbleGoal
import java.lang.Math.toRadians

class BlueMidAuto: LinearOpMode() {
    override fun runOpMode(){
        val drive = SampleMecanumDrive(hardwareMap)
        val wobbleGoal = WobbleGoal.standard(hardwareMap)

        val intake = Intake.standard(hardwareMap)
        val shooterMotor = hardwareMap.dcMotor["shooter1"]
        shooterMotor.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        val shooterMotor2 = hardwareMap.dcMotor["shooter2"]
        shooterMotor2.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        val shooterServo = hardwareMap.servo["pinball"]
        val flap = hardwareMap.servo["Flap"]


        // init stuff
        wobbleGoal.liftArm()
        wobbleGoal.grab()
        flap.position = 0.00 //High goal: 0.00      PS: 0.05
        shooterServo.position = 0.00

        waitForStart()

        val box = 0

        val startPose = Pose24(-64.0, -24.0, toRadians(180.0))
        drive.poseEstimate = startPose

        val rightPowerShotTrajectory = drive.trajectoryBuilder(startPose, true)
                .splineToConstantHeading(Vector2d(-3.0, 12.5), toRadians(0.0))
                .build()
        val midPowerShotTrajectory = drive.trajectoryBuilder(rightPowerShotTrajectory.end(), true)
                .strafeLeft(8.0)
                .build()
        val leftPowerShotTrajectory = drive.trajectoryBuilder(midPowerShotTrajectory.end(), true)
                .strafeRight(7.0)
                .build()
        drive.followTrajectory(rightPowerShotTrajectory)
        drive.followTrajectory(midPowerShotTrajectory)
        drive.followTrajectory(leftPowerShotTrajectory)

        val trajectory: Trajectory

        when (box){
            0 -> {
                trajectory = drive.followTrajectory(drive.trajectoryBuilder(leftPowerShotTrajectory.end(), toRadians(0.0))
                        .splineToLinearHeading(Pose2d(12.0, -41.0, toRadians(90.0)), toRadians(-90.0))
                        .splineto
                        .build())
                drive.followTrajectory(trajcetory)
            }
            1 ->{
                trajectory = drive.followTrajectory(drive.trajectoryBuilder(leftPowerShotTrajectory.end(), toRadians(0.0))
                        .splineToConstantHeading(Vector2d(16.0, 36.0), toRadians(90.0))
                        .build())
                drive.followTrajectory(trajectory)
            }
            2 ->{

            }

            val backup = drive.trajectoryBuilder(trajectory.end(), false)
                    .splineToLinearHeading(Pose2d(-38.0, 26.0, toRadians(0.0)), toRadians(180.0))
                    .build()
            drive.followTrajectory(backup)
        }
    }
}