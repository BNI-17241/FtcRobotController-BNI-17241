package org.firstinspires.ftc.teamcode.Competition.Decode.Spark17241.Controls.Auto.paths;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import  com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Competition.Decode.Spark17241.pedroPathing.Constants;

@Autonomous(name = "Pathing Tester", group = "Lab")
public class pathingTester extends OpMode {

    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;

    // Used Visualizer to determine pose
    private final Pose startPose = new Pose(56, 8, Math.toRadians(90)); // Start Pose of robot (Red Far Launch Zone).
    private final Pose scorePose = new Pose(90, 90, Math.toRadians(50)); // Red Goal Scoring Pose of robot.
    private final Pose parkHomePose = new Pose(50, 40, Math.toRadians(180)); // Red Home

    private Path scorePreload;
    private PathChain goPark, scorePickup1;

    public enum pathingState { START, SCORE_PRELOAD, PARK_HOME, READY }
    pathingState pathState = pathingState.READY;






    @Override
    public void loop() {
        // These loop the movements of the robot, these must be called continuously in order to work
        follower.update();
        autonomousPathUpdate();
        // Feedback to Driver Hub for debugging
        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.update();
    }
    /** This method is called once at the init of the OpMode. **/
    @Override
    public void init() {
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();
        follower = Constants.createFollower(hardwareMap);
        buildPaths();
        follower.setStartingPose(startPose);
    }
    /** This method is called continuously after Init while waiting for "play". **/
    @Override
    public void init_loop() {}
    /** This method is called once at the start of the OpMode.
     * It runs all the setup actions, including building paths and starting the path system **/
    @Override
    public void start() {
        opmodeTimer.resetTimer();
        setPathState(pathingState.SCORE_PRELOAD);
    }
    /** We do not use this because everything should automatically disable **/
    @Override
    public void stop() {}


    // Build our Paths using the various Poses

    public void buildPaths() {
        /* From Start and To Score Preload Path. We are using a BezierLine. */
        scorePreload = new Path(new BezierLine(startPose, scorePose));
        scorePreload.setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading());

        /* From Score and Tp Park Home  Path.  We are using a single path with a BezierLine. */
        goPark = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, parkHomePose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), parkHomePose.getHeading())
                .build();


    }


    public void autonomousPathUpdate() {
        switch (pathState) {
            case START:
                follower.followPath(scorePreload);
                setPathState(pathingState.SCORE_PRELOAD);
                break;

            case SCORE_PRELOAD:
            /* You could check for
            - Follower State: "if(!follower.isBusy()) {}"
            - Time: "if(pathTimer.getElapsedTimeSeconds() > 1) {}"
            - Robot Position: "if(follower.getPose().getX() > 36) {}"
            */
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if(!follower.isBusy()) {

                    /* NEED TO INSERT CODE TO SCORE PRELOAD ARTIFACTS */

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                    follower.followPath(goPark,true);
                    setPathState(pathingState.READY);
                }
                break;
            case READY:
                break;
        }
    }
    /** These change the states of the paths and actions. It will also reset the timers of the individual switches **/
    public void setPathState(pathingState pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

}
