/**
 * @author Russell Toris - rctoris@wpi.edu
 * @author Lars Kunze - l.kunze@cs.bham.ac.uk
 * @author Raffaello Bonghi - raffaello.bonghi@officinerobotiche.it
 */

/**
 * A navigator can be used to add click-to-navigate options to an object. If
 * withOrientation is set to true, the user can also specify the orientation of
 * the robot by clicking at the goal position and pointing into the desired
 * direction (while holding the button pressed).
 *
 * @constructor
 * @param options - object with following keys:
 *   * ros - the ROSLIB.Ros connection handle
 *   * tfClient (optional) - the TF client
 *   * robot_pose (optional) - the robot topic or TF to listen position
 *   * serverName (optional) - the action server name to use for navigation, like '/move_base'
 *   * actionName (optional) - the navigation action name, like 'move_base_msgs/MoveBaseAction'
 *   * rootObject (optional) - the root object to add the click listeners to and render robot markers to
 *   * withOrientation (optional) - if the Navigator should consider the robot orientation (default: false)
 */
NAV2D.Navigator = function (options) {
  var that = this;
  options = options || {};
  var ros = options.ros;
  var tfClient = options.tfClient || null;

  var pathClient = options.pathClient || null;
  var pathTopic = options.pathTopic || "/move_base/NavfnROS/plan";

  var robot_pose = options.robot_pose || "/robot_pose";
  var serverName = options.serverName || "/move_base";
  var actionName = options.actionName || "move_base_msgs/MoveBaseAction";
  var navGoalTopic = options.navGoalTopic || "/move_base_simple/goal";
  var withOrientation = options.withOrientation || false;
  var use_image = options.image;
  this.rootObject = options.rootObject || new createjs.Container();

  this.goalMarker = null;

  // setup the actionlib client
  var actionClient = new ROSLIB.ActionClient({
    ros: ros,
    actionName: actionName,
    serverName: serverName,
  });

  // Trace path shape for tracking the robot path
  var tracePath = new ROS2D.TraceShape({
    strokeSize: 0.1,
    strokeColor: createjs.Graphics.getRGB(255, 0, 0), // Red color for trace
    maxPoses: 30 // Limit the trace to 100 poses
  });
  this.rootObject.addChild(tracePath);

  // send a goal to the navigation stack
  var simpleNavGoal = new ROSLIB.Topic({
    ros: ros,
    name: navGoalTopic,
    messageType: "geometry_msgs/PoseStamped",
  });

  // Path shape for drawing the navigation path
  var pathShape = new ROS2D.PathShape({
    strokeSize: 0.2,
    strokeColor: createjs.Graphics.getRGB(0, 0, 255) // Blue color for path
  });
  this.rootObject.addChild(pathShape);

  // Subscribe to the path topic to receive path updates
  var pathListener = new ROSLIB.Topic({
    ros: ros,
    name: pathTopic,
    messageType: "nav_msgs/Path",
    throttle_rate: 500,
  });

  pathListener.subscribe(function (path) {
    // Set the path in pathShape, which will draw it on the rootObject
    pathShape.setPath(path);
  });

  /**
   * Draw Goal
   */
  // Array to store PoseArray markers
  var poseArrayMarkers = [];
  // Timer variable for timeout
  var poseArrayTimeout = null;

  // Subscribe to /pose_array topic to receive PoseArray messages
  var poseArrayListener = new ROSLIB.Topic({
    ros: ros,
    name: '/pose_array',
    messageType: 'geometry_msgs/PoseArray',
    throttle_rate: 500
  });

  // Callback to draw each pose from PoseArray
  poseArrayListener.subscribe(function (poseArrayMsg) {
    // Clear existing markers
    poseArrayMarkers.forEach(marker => that.rootObject.removeChild(marker));
    poseArrayMarkers = [];

    // Iterate through each pose in PoseArray
    poseArrayMsg.poses.forEach((pose, index) => {
      var isFirstElement = index === 0;
      var poseMarker = new ROS2D.NavigationArrow({
        size: isFirstElement ? 20 : 10, // Dua kali lipat ukuran untuk elemen pertama
        strokeSize: 1,
        fillColor: isFirstElement 
          ? createjs.Graphics.getRGB(255, 0, 0, 0.66) // Warna merah untuk elemen pertama
          : createjs.Graphics.getRGB(0, 255, 0, 0.66), // Warna hijau untuk elemen lainnya
        pulse: false
      });

      // Set marker position and orientation
      poseMarker.x = pose.position.x;
      poseMarker.y = -pose.position.y;
      poseMarker.rotation = stage.rosQuaternionToGlobalTheta(pose.orientation);

      // Adjust marker scale based on map scale
      poseMarker.scaleX = 1.0 / stage.scaleX;
      poseMarker.scaleY = 1.0 / stage.scaleY;

      // Add marker to rootObject and poseArrayMarkers array
      that.rootObject.addChild(poseMarker);
      poseArrayMarkers.push(poseMarker);
    });

    // Reset the timeout timer each time a new message is received
    clearTimeout(poseArrayTimeout);
    poseArrayTimeout = setTimeout(() => {
      // If no new message arrives within 2 seconds, remove all markers
      poseArrayMarkers.forEach(marker => that.rootObject.removeChild(marker));
      poseArrayMarkers = [];
    }, 2000);
  });


  // ----------------------------


  /**
   * Send a goal to the navigation stack with the given pose.
   *
   * @param pose - the goal pose
   */
  function sendGoal(pose) {

    // // create a goal
    // var goal = new ROSLIB.Goal({
    //   actionClient: actionClient,
    //   goalMessage: {
    //     target_pose: {
    //       header: {
    //         frame_id: "map",
    //       },
    //       pose: pose,
    //     },
    //   },
    // });
    // goal.send();

    // that.currentGoal = goal;

    // Create a goal using the navigation topic simple goal
    var goal = new ROSLIB.Topic({
      ros: ros,
      name: navGoalTopic,
      messageType: "geometry_msgs/PoseStamped",
    });

    var goalPose = new ROSLIB.Message({
      header: {
        frame_id: "map",
      },
      pose: pose,
    });

    // send the goal using ROSLIB
    simpleNavGoal.publish(goalPose);

    // create a marker for the goal
    if (that.goalMarker === null) {
      if (use_image && ROS2D.hasOwnProperty("ImageNavigator")) {
        that.goalMarker = new ROS2D.ImageNavigator({
          size: 2.5,
          image: use_image,
          alpha: 0.7,
          pulse: true,
        });
      } else {
        that.goalMarker = new ROS2D.NavigationArrow({
          size: 15,
          strokeSize: 1,
          fillColor: createjs.Graphics.getRGB(255, 64, 128, 0.66),
          pulse: false,
        });
      }
      that.rootObject.addChild(that.goalMarker);
    }
    that.goalMarker.x = pose.position.x;
    that.goalMarker.y = -pose.position.y;
    that.goalMarker.rotation = stage.rosQuaternionToGlobalTheta(
      pose.orientation
    );
    that.goalMarker.scaleX = 1.0 / stage.scaleX;
    that.goalMarker.scaleY = 1.0 / stage.scaleY;

    goal.on("result", function () {
      that.rootObject.removeChild(that.goalMarker);
    });
  }

  /**
   * Cancel the currently active goal.
   */
  this.cancelGoal = function () {
    console.log("Cancel Goals");
    if (typeof that.currentGoal !== "undefined") {
      console.log(that.currentGoal);
      that.currentGoal.cancel();
    }
  };

  // get a handle to the stage
  var stage;
  if (that.rootObject instanceof createjs.Stage) {
    stage = that.rootObject;
  } else {
    stage = that.rootObject.getStage();
  }

  // marker for the robot
  var robotMarker = null;
  if (use_image && ROS2D.hasOwnProperty("NavigationImage")) {
    robotMarker = new ROS2D.NavigationImage({
      size: 40,
      image: use_image,
      pulse: true,
    });
  } else {
    robotMarker = new ROS2D.NavigationArrow({
      size: 25,
      strokeSize: 1,
      fillColor: createjs.Graphics.getRGB(255, 128, 0, 0.66),
      pulse: true,
    });
  }

  /* TO DRAW LINES OF PATH */
  

  // wait for a pose to come in first
  robotMarker.visible = false;
  this.rootObject.addChild(robotMarker);
  var initScaleSet = false;

  var updateRobotPosition = function (pose, orientation) {
    // update the robots position on the map
    robotMarker.x = pose.x;
    robotMarker.y = -pose.y;
    console.log(initScaleSet);
    if (!initScaleSet) {
      robotMarker.scaleX = 1.0 / stage.scaleX;
      robotMarker.scaleY = 1.0 / stage.scaleY;
      initScaleSet = true;
    }
    // change the angle
    robotMarker.rotation = stage.rosQuaternionToGlobalTheta(orientation);
    // Set visible
    robotMarker.visible = true;

    tracePath.addPose(new ROSLIB.Pose({
      position: new ROSLIB.Vector3({ x: pose.x, y: pose.y, z: 0 }),
      orientation: new ROSLIB.Quaternion({ x: 0, y: 0, z: 0, w: 1 })
    }));
  };

  if (tfClient !== null) {
    tfClient.subscribe(robot_pose, function (tf) {
      updateRobotPosition(tf.translation, tf.rotation);
    });
  } else {
    // setup a listener for the robot pose
    var poseListener = new ROSLIB.Topic({
      ros: ros,
      name: robot_pose,
      messageType: "geometry_msgs/Pose",
      throttle_rate: 100,
    });
    poseListener.subscribe(function (pose) {
      //console.log(pose.position);
      //console.log(pose.orientation);
      console.log(that.rootObject);
      console.log(robotMarker);
      console.log("add robot marker");
      updateRobotPosition(pose.position, pose.orientation);
    });
  }

  if (withOrientation === false) {
    // setup a double click listener (no orientation)
    this.rootObject.addEventListener("dblclick", function (event) {
      // convert to ROS coordinates
      var coords = stage.globalToRos(event.stageX, event.stageY);
      var pose = new ROSLIB.Pose({
        position: new ROSLIB.Vector3(coords),
      });
      // send the goal
      sendGoal(pose);
    });
  } else {
    // withOrientation === true
    // setup a click-and-point listener (with orientation)
    var position = null;
    var positionVec3 = null;
    var thetaRadians = 0;
    var thetaDegrees = 0;
    var orientationMarker = null;
    var mouseDown = false;
    var xDelta = 0;
    var yDelta = 0;

    var mouseEventHandler = function (event, mouseState) {
      if (mouseState === "down") {
        // get position when mouse button is pressed down
        position = stage.globalToRos(event.stageX, event.stageY);
        positionVec3 = new ROSLIB.Vector3(position);
        mouseDown = true;
      } else if (mouseState === "move") {
        // remove obsolete orientation marker
        that.rootObject.removeChild(orientationMarker);

        if (mouseDown === true) {
          // if mouse button is held down:
          // - get current mouse position
          // - calulate direction between stored <position> and current position
          // - place orientation marker
          var currentPos = stage.globalToRos(event.stageX, event.stageY);
          var currentPosVec3 = new ROSLIB.Vector3(currentPos);

          if (use_image && ROS2D.hasOwnProperty("ImageNavigator")) {
            orientationMarker = new ROS2D.ImageNavigator({
              size: 2.5,
              image: use_image,
              alpha: 0.7,
              pulse: false,
            });
          } else {
            orientationMarker = new ROS2D.NavigationArrow({
              size: 25,
              strokeSize: 1,
              fillColor: createjs.Graphics.getRGB(0, 255, 0, 0.66),
              pulse: true,
            });
          }

          xDelta = currentPosVec3.x - positionVec3.x;
          yDelta = currentPosVec3.y - positionVec3.y;

          thetaRadians = Math.atan2(xDelta, yDelta);

          thetaDegrees = thetaRadians * (180.0 / Math.PI);

          if (thetaDegrees >= 0 && thetaDegrees <= 180) {
            thetaDegrees += 270;
          } else {
            thetaDegrees -= 90;
          }

          orientationMarker.x = positionVec3.x;
          orientationMarker.y = -positionVec3.y;
          orientationMarker.rotation = thetaDegrees;
          orientationMarker.scaleX = 1.0 / stage.scaleX;
          orientationMarker.scaleY = 1.0 / stage.scaleY;

          that.rootObject.addChild(orientationMarker);
        }
      } else if (mouseDown) {
        // mouseState === 'up'
        // if mouse button is released
        // - get current mouse position (goalPos)
        // - calulate direction between stored <position> and goal position
        // - set pose with orientation
        // - send goal
        mouseDown = false;

        var goalPos = stage.globalToRos(event.stageX, event.stageY);

        var goalPosVec3 = new ROSLIB.Vector3(goalPos);

        xDelta = goalPosVec3.x - positionVec3.x;
        yDelta = goalPosVec3.y - positionVec3.y;

        thetaRadians = Math.atan2(xDelta, yDelta);

        if (thetaRadians >= 0 && thetaRadians <= Math.PI) {
          thetaRadians += (3 * Math.PI) / 2;
        } else {
          thetaRadians -= Math.PI / 2;
        }

        var qz = Math.sin(-thetaRadians / 2.0);
        var qw = Math.cos(-thetaRadians / 2.0);

        var orientation = new ROSLIB.Quaternion({ x: 0, y: 0, z: qz, w: qw });

        var pose = new ROSLIB.Pose({
          position: positionVec3,
          orientation: orientation,
        });
        
        // send the goal
        sendGoal(pose);
      }
    };

    this.rootObject.addEventListener("stagemousedown", function (event) {
      mouseEventHandler(event, "down");
    });

    this.rootObject.addEventListener("stagemousemove", function (event) {
      mouseEventHandler(event, "move");
    });

    this.rootObject.addEventListener("stagemouseup", function (event) {
      mouseEventHandler(event, "up");
    });
  }
};
