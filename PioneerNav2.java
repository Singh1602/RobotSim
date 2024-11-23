// PioneerNav2.java
/*
 * PioneerNavigation Class Definition
 * File: PioneerNav2.java
 * Date: 18th Oct 2022
 * Description: Simple Navigation Class support (2022)
 * Author: Terry Payne (trp@liv.ac.uk)
 */
 
import com.cyberbotics.webots.controller.Motor;
import com.cyberbotics.webots.controller.Supervisor;
import com.cyberbotics.webots.controller.Node;

public class PioneerNav2 {

  public static enum MoveState {
    STOP,
    FORWARD,
    ARC, 
    FOLLOW_WALL,
    BUG2
  };

  private Supervisor robot;   // reference to the robot
  private Node robot_node;    // reference to the robot node
  private Pose robot_pose;    // the robots percieved pose
  private Motor left_motor;
  private Motor right_motor;
  private double prev_error;
  private double total_error;
  private PioneerProxSensors1 prox_sensors;
  private double max_vel;
  
  private MoveState state;

  private final double WHEEL_RADIUS = 0.0957; // in meters - found using CONFIGURE 
  private final double AXEL_LENGTH = 0.323;   // in meters - found using CONFIGURE

  private double gradient, yIntercept;


  // ==================================================================================
  // Constructor
  // ==================================================================================
  public PioneerNav2(Supervisor robot, Pose init_pose, PioneerProxSensors1 ps) {
    this.prox_sensors = ps;  // reference to proximity sensors {
    
    this.prev_error = 0;
    this.total_error = 0;
    this.robot = robot;    
    this.robot_node = this.robot.getSelf();   // reference to the robot node
    this.state = MoveState.STOP;

    // enable motors
    this.left_motor = robot.getMotor("left wheel");
    this.right_motor = robot.getMotor("right wheel");
    this.left_motor.setPosition(Double.POSITIVE_INFINITY);
    this.right_motor.setPosition(Double.POSITIVE_INFINITY);
    this.max_vel = this.left_motor.getMaxVelocity() - 0.1; // Fudge: just under max vel
    
    int timeStep = (int) Math.round(robot.getBasicTimeStep());

    // set up pose
    this.robot_pose = new Pose(init_pose.getX(), init_pose.getY(), init_pose.getTheta());

    // Initialise motor velocity
    this.left_motor.setVelocity(0.0);
    this.right_motor.setVelocity(0.0);   
    this.gradient = 0.0;
    this.yIntercept = 0.0;
  } 
  

  // The following method only works if in supervisor mode
  public Pose get_real_pose() {
    if (this.robot_node == null)
      return new Pose(0,0,0);
      
    double[] realPos = robot_node.getPosition();
    double[] rot = this.robot_node.getOrientation(); // 3x3 Rotation matrix as vector of length 9
    double theta1 = Math.atan2(-rot[0], rot[3]);
    double halfPi = Math.PI/2;
    double theta2 = theta1 + halfPi;
    if (theta1 > halfPi)
        theta2 = -(3*halfPi)+theta1;
    
    return new Pose(realPos[0], realPos[1], theta2);
  }

  public int forward(double target_dist, double robot_linearvelocity) {
    double wheel_av = (robot_linearvelocity/this.WHEEL_RADIUS);
    double target_time = target_dist/robot_linearvelocity;
    
    this.left_motor.setVelocity(wheel_av);
    this.right_motor.setVelocity(wheel_av);
    this.state = MoveState.FORWARD;
        
    // return target_time as millisecs          
    return (int) (1000.0*target_time);
  }

  public int arc(double icr_angle, double icr_r, double icr_omega) {
    double target_time = icr_angle / icr_omega;

    // Calculate each wheel velocity around ICR
    double vl = icr_omega * (icr_r - (this.AXEL_LENGTH / 2));
    double vr = icr_omega * (icr_r + (this.AXEL_LENGTH / 2));
        
    double leftwheel_av = (vl/this.WHEEL_RADIUS);
    double rightwheel_av = (vr/this.WHEEL_RADIUS);
        
    this.left_motor.setVelocity(leftwheel_av);
    this.right_motor.setVelocity(rightwheel_av);
    this.state = MoveState.ARC;

    // return target_time as millisecs          
    return (int) (1000.0*target_time);
  }  
  public void stop() {
    this.left_motor.setVelocity(0.0);
    this.right_motor.setVelocity(0.0);
    this.state = MoveState.STOP;
  }
  
  public MoveState getState() {
    return this.state;
  }
  
  public void set_velocity(double base, double control) {
    // base gives the velocity of the wheels in m/s
    // control is an adjustment on the main velocity
    double base_av = (base/this.WHEEL_RADIUS);
    double lv = base_av;
    double rv = base_av;
    
    if (control != 0) {
      double control_av = (control/this.WHEEL_RADIUS);
      // Check if we exceed max velocity and compensate
      double correction = 1;
      lv = base_av - control_av;
      rv = base_av + control_av;

      if (lv > this.max_vel) {
        correction = this.max_vel / lv;
        lv = lv * correction;
        rv = rv * correction;
      }
           
      if (rv > this.max_vel) {
        correction = this.max_vel / rv;
        lv = lv * correction;
        rv = rv * correction;
      }

    }
    this.left_motor.setVelocity(lv);
    this.right_motor.setVelocity(rv);
  }

  //PID code from labs 
    private double pid(double error) {
    double kp = 0.9; // proportional weight (may need tuning)
    double kd = 3.5; // differential weight (may need tuning)
    double ki = 0.0; // integral weight (may need tuning)
    
    double prop = error;
    double diff = error - this.prev_error;
    this.total_error += error;

    double control = (kp * prop) + (ki * this.total_error) + (kd * diff);
    this.prev_error = error;
    
    return control;
  }

  //Code from labs, follows wall either on the right or left side of the obstacle
    public void follow_wall(double robot_linearvelocity, double set_point, boolean right) {
    int direction_coeff = 1;
    double error;
    double control;
    double wall_dist;
   
    if (right) direction_coeff = -1;  // invert the values for the control
    
    if (Math.min(this.prox_sensors.get_value(1),
          Math.min(this.prox_sensors.get_value(2),
            Math.min(this.prox_sensors.get_value(3),
              Math.min(this.prox_sensors.get_value(4),
                Math.min(this.prox_sensors.get_value(5),
                  this.prox_sensors.get_value(6)))))) < set_point) 
      this.set_velocity(robot_linearvelocity/3, -0.2*direction_coeff);

    else {
      if (!right) wall_dist = Math.min(this.prox_sensors.get_value(1),
                                       this.prox_sensors.get_value(0));
      else wall_dist = Math.min(this.prox_sensors.get_value(7),
                                this.prox_sensors.get_value(8));
      // Running aproximately parallel to the wall
      if (wall_dist < this.prox_sensors.get_maxRange()) {
        error = wall_dist - set_point;
        control = this.pid(error);
        // adjust for right wall
        this.set_velocity(robot_linearvelocity, control*direction_coeff);
      } else {
        // No wall, so turn
        this.set_velocity(robot_linearvelocity, 0.08*direction_coeff);
      }
    }
    this.state = MoveState.FOLLOW_WALL;
  
  }

  //Bug 2 implementation
  public void bug2(Pose goal) {
    Pose currentLocation = get_real_pose();

    //Calculate angle required to face the goal
    double orientationNeeded = Math.atan2(goal.getY() - currentLocation.getY(), goal.getX() - currentLocation.getX());

    //Normalise the angle to make sure it is between -pi and pi
    double theta = orientationNeeded - currentLocation.getTheta();
      while (theta > Math.PI)
        theta -= 2 * Math.PI;
      while (theta < -Math.PI)
        theta += 2 * Math.PI;
        
      if (Math.abs(theta) < Math.toRadians(5)) {
          set_velocity(0.3, 0); // Move forward
      } else {
        //Changing orientation of the robot
        if (theta < 0) {
            set_velocity(0, -0.3); // Turn right
        } else {
            set_velocity(0, 0.3); // Turn left
        }
      }
  }

  //Create a line from start pose to goal pose
  public void createLine(Pose starting, Pose goal) {
    gradient = (goal.getY() - starting.getY()) / (goal.getX() - starting.getX());
    yIntercept = starting.getY() - gradient * starting.getX();
  }

  // Check if a pose is on the line of the line created in above method
  public boolean checkLine(Pose pose) {
    double onTarget = gradient * pose.getX() + yIntercept;
    return Math.abs(pose.getY() - onTarget) <= 0.3;
  }
}    