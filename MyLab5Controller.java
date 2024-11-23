// MyLab5Controller.java
/*
 * MyLab5Controller Class Definition
 * File: MyLab5Controller.java
 * Date: 15th Oct 2022
 * Description: Simple Controller based on the Lab4 controller (2022)
 * Author: Terry Payne (trp@liv.ac.uk)
 */

import com.cyberbotics.webots.controller.Supervisor;
import com.cyberbotics.webots.controller.Camera;
import com.cyberbotics.webots.controller.Display;

public class MyLab5Controller {

  public static void main(String[] args) {

    Supervisor robot = new Supervisor();
    int timeStep = (int) Math.round(robot.getBasicTimeStep());
    
    // set up display
    Display odometry_display = robot.getDisplay("odometryDisplay");
    
    //get robot's and goal pose
    Pose robot_pose = new Pose(0.0, 0.0, 0.0);
    Pose goal_pose = new Pose(2.75, -3.26, 0);
    
    PioneerProxSensors1 prox_sensors = new PioneerProxSensors1(robot, "sensor_display", robot_pose);
    PioneerNav2 nav = new PioneerNav2(robot, robot_pose, prox_sensors);
    
    //Get camera
    Camera camera = robot.getCamera("camera");
    if (camera != null)
      camera.enable(timeStep);

    

    double time_elapsed = 0;
    double target_time = 0;
    double robot_velocity = 0.3;
    // double distanceTraveledThisLoop = 0.0
    boolean noObstacle = true;
    double totalDistance = 0.0;
    double distance = 0.0;
    
    // define schedule
    PioneerNav2.MoveState[] schedule = { PioneerNav2.MoveState.BUG2 };
    int schedule_index = -1; // we increment before selecting the current action
    PioneerNav2.MoveState state = PioneerNav2.MoveState.BUG2; // current state

    Pose start = nav.get_real_pose();

    //double distance = 0.0; // Initialize the distance variable for BUG2

  while (robot.step(timeStep) != -1) {
      //Testing out the front proximity sensors
      robot_pose = nav.get_real_pose();
      prox_sensors.set_pose(robot_pose);
      prox_sensors.paint();  // Render sensor Display

    //Find distance travelled
    Pose currentPose = nav.get_real_pose();
    double distanceX = currentPose.getX() - start.getX();
    double distanceY = currentPose.getY() - start.getY();
    double distanceTraveledThisLoop = Math.sqrt(distanceX * distanceX + distanceY * distanceY);
    totalDistance += distanceTraveledThisLoop;
    start = currentPose;

    state = nav.getState();

    if (time_elapsed > target_time) {
        time_elapsed = 0;
        schedule_index = (schedule_index + 1) % schedule.length;
        state = schedule[schedule_index];

        //Manage states between folllow wall and bug 2
        if (state == PioneerNav2.MoveState.FOLLOW_WALL) {
            target_time = 0; // always refresh!
            nav.follow_wall(robot_velocity, 0.3, true);
            System.out.println("FOLLOW WALL");
        } else if (state == PioneerNav2.MoveState.FORWARD) {
            target_time = nav.forward(0.8, robot_velocity);
        } else if (state == PioneerNav2.MoveState.ARC) {
            target_time = nav.arc(Math.PI/2.0, 0.0, robot_velocity);
        } else if (state == PioneerNav2.MoveState.STOP) {
            nav.stop();
            target_time = 60 * 1000; // Pause for 1 minute
        } else if (state == PioneerNav2.MoveState.BUG2) {
            start = robot_pose;
            distance += distanceTraveledThisLoop;
            
            //Check if there is an obstacle ahead, 3 and 4 are front sensors
            if (prox_sensors.get_value(3) < 0.5 || prox_sensors.get_value(4) < 0.5) {
                // Switch to FOLLOW_WALL state if an obstacle is detected
                state = PioneerNav2.MoveState.FOLLOW_WALL;
                //System.out.println("Switching to FOLLOW WALL due to obstacle in BUG 2");
                continue; // Go to the next iteration to start FOLLOW_WALL logic immediately
            }

            // If no obstacle and still in BUG2
            if (noObstacle) {
                if (distance > 0.5) {
                    // Any additional logic for BUG2
                }
                nav.createLine(robot_pose, goal_pose);
                nav.bug2(goal_pose);
               // System.out.println("Executing BUG 2");
            }
        }
    } else {
        time_elapsed += timeStep; // Increment time elapsed
    }
    
    //Display information to the odometry display
    if (odometry_display != null) {
        odometry_display.setColor(0xFFFFFF);          // White
        odometry_display.fillRectangle(0,0,
                odometry_display.getWidth(),
                odometry_display.getHeight());
    
        odometry_display.setColor(0x000000);          // Black
           // font size = 18, with antialiasing
        odometry_display.drawText("Robot State", 1, 1);

        
        odometry_display.drawText("Total Distance " + totalDistance, 1,20);
        Pose true_pose = nav.get_real_pose();
        odometry_display.drawText("True Pose: "+true_pose, 1, 40);
      }

      //Pose true_pose = nav.get_real_pose();
      //System.out.println("Action: " + display_action + " \t" + "True Pose: "+true_pose);

}
    // Enter here exit cleanup code.
  }
}
