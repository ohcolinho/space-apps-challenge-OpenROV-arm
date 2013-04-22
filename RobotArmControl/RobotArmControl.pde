/*
Robot Arm Control with Leap motion
Written by Colin Ho
Created: April 20 2013
Last updated: April 21 2013

This processing sketch lets you control a 3 DOF robot arm (with shoulder tilt and pan, and elbow pan) 
with a Leap Motion controller
 
requires:
Leap Motion Library - https://github.com/heuermh/leap-motion-processing
Arduino/Firmata library (modified to fix errors seen here http://arduino.cc/forum/index.php?topic=122177.0) - https://github.com/pardo-bsso/processing-arduino 
 
This Robot Arm Control Sketch for Processing is free software: you can redistribute it and/or
modify it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or(at your option) any later version.
 
You should have received a copy of the GNU General Public License
along with this sketch.  If not, see <http://www.gnu.org/licenses/>.
 
*/


import java.util.Map;
import java.util.concurrent.ConcurrentMap;
import java.util.concurrent.ConcurrentHashMap;
import processing.serial.*;
import cc.arduino.*;

import com.leapmotion.leap.Controller;
import com.leapmotion.leap.Finger;
import com.leapmotion.leap.Frame;
import com.leapmotion.leap.Hand;
import com.leapmotion.leap.Tool;
import com.leapmotion.leap.Vector;
import com.leapmotion.leap.processing.LeapMotion;

LeapMotion leapMotion;
Arduino arduino;

ConcurrentMap<Integer, Integer> fingerColors;
ConcurrentMap<Integer, Integer> toolColors;
ConcurrentMap<Integer, Vector> fingerPositions;
ConcurrentMap<Integer, Vector> toolPositions;
Vector handPosition;
Vector sphereCenter;
float sphereRadius;
float[] armPosition;
static float LEAP_WIDTH = 200.0; // in mm
static float LEAP_HEIGHT = 700.0; // in mm
int postilt;
int pos1;
int pos2;

void setup()
{
  size(16*50, 9*50);
  background(20);
  frameRate(30);

  //println(Arduino.list());
  //firmata serial comms
  arduino = new Arduino(this, Arduino.list()[0], 57600); //your offset may vary

  arduino.pinMode(9, 4);  //tilt
  arduino.pinMode(10, 4);  //shoulder
  arduino.pinMode(11, 4);  //elbow

  //initialize Leap motion variables
  ellipseMode(CENTER);
  leapMotion = new LeapMotion(this);
  fingerColors = new ConcurrentHashMap<Integer, Integer>();
  toolColors = new ConcurrentHashMap<Integer, Integer>();
  fingerPositions = new ConcurrentHashMap<Integer, Vector>();
  toolPositions = new ConcurrentHashMap<Integer, Vector>();
  handPosition = new Vector();
  sphereCenter = new Vector();
}

void draw()
{
  fill(20);
  rect(0, 0, width, height);

  // X Z is the planar coordinate frame
  float handX = handPosition.getX();  //varies from 
  float handY = handPosition.getY();  //varies from 
  float handZ = handPosition.getZ();  //varies from 
  //println("hand X " + handX + "\n hand Y " + handY +"\n hand Z " + handZ );

  //get arm position
  armPosition = armPosition(handX, handY, handZ, 100, 150);
  println("arm position: \nthetaTilt: " + armPosition[0] + "\ntheta1: " + armPosition[1] + "\ntheta2: " + armPosition[2]);
  postilt = (int) map(armPosition[0], -90, 90, 0, 180);
  pos1 = (int)armPosition[1];
  pos2 = (int)armPosition[2];
  println("servo position: \nposTilt: " + postilt + "\npos1: " + pos1 + "\npos2: " + pos2);
  //write to arduino
  arduino.analogWrite(9, postilt);
  arduino.analogWrite(10, pos1);
  arduino.analogWrite(11, pos2);

  //##show position
  //color depth
  float colory = map(handY, 30, 400, 0, 255);
  //println("colory " + colory);
  color dot = color(255-colory, 0, colory);
  fill(dot);
  ellipse(leapToScreenX(handX), leapToScreenZ(handZ), 24.0, 24.0);

  //draw arm links
  stroke(125);
  line(width/2, height/2, leapToScreenX(handX), leapToScreenZ(handZ));


  //text label
  textSize(10);
  fill(0, 255, 153);
  String texts = "x: " + (int)leapToScreenX(handX) + "\n" + "z: " + (int)leapToScreenZ(handZ);
  text(texts, leapToScreenX(handX)+20, leapToScreenZ(handZ));
}

//compute arm position servo angle positions given a xyz from hand position
//inputs are x,y,z of hand from leap, len is length of robot arm in mm
float[] armPosition(float x, float y, float z, float len, float planeH)
{
  float c = sqrt(pow(x, 2) + pow(z, 2) + pow(y-planeH, 2));

  float thetaTilt = asin((y-planeH)/c);// in rad
  float theta2 = 2*asin(c/(2*len));  //in rad

  float thetaC1 = 90;
  if (x>0) {
    thetaC1 = 180-abs(degrees(atan(z/x)));
  }
  else {
    thetaC1 = degrees(abs(atan(z/x)));
  }

  //float thetaC1 = atan((z)/x);
  //float thetaC2 = (PI-theta2)/2;
  //float theta1 = thetaC1+thetaC2;

  float[] armPos = {
    constrain(degrees(thetaTilt), -90.0, 90.0), thetaC1, constrain(degrees(theta2), 0, 180)
    };
    //println("thetaTilt: " + armPos[0] + " theta1: " + armPos[1] + " theta2: " + armPos[2]);
    return armPos;
}

void onFrame(final Controller controller)
{
  Frame frame = controller.frame();
  //for all fingers in the frame
  for (Finger finger : frame.fingers())
  {
    int fingerId = finger.id();  //grab the finger id
    color c = color(random(0, 255), random(0, 255), random(0, 255));  //generate random color
    fingerColors.putIfAbsent(fingerId, c);  //if there isn't a color put color into fingerColors at fingerId
    fingerPositions.put(fingerId, finger.tipPosition());
  }
  for (Tool tool : frame.tools())
  {
    int toolId = tool.id();
    color c = color(random(0, 255), random(0, 255), random(0, 255));
    toolColors.putIfAbsent(toolId, c);
    toolPositions.put(toolId, tool.tipPosition());
  }
  //if there is a hand
  if (!frame.hands().empty()) {
    //get first hand
    Hand hand = frame.hands().get(0);
    handPosition = hand.palmPosition();
    //println("hand position: x: " + handPosition.getX() + " y: " + handPosition.getY() + " z: " + handPosition.getZ());
  }
}

//take float of x (from -250 to 250), and convert to pixel position (0 to width)
float leapToScreenX(float x)
{

  float c = width / 2.0;
  if (x > 0.0)
  {
    return lerp(c, width, x/LEAP_WIDTH);
  }
  else
  {
    return lerp(c, 0.0, -x/LEAP_WIDTH);
  }
}


float leapToScreenZ(float z)
{
  float c = height / 2.0;
  if (z > 0.0)
  {
    return lerp(c, height, z/LEAP_WIDTH);
  }
  else
  {
    return lerp(c, 0.0, -z/LEAP_WIDTH);
  }
}
float leapToScreenY(float y)
{
  return lerp(height, 0.0, y/LEAP_HEIGHT);
}

