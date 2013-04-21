/*

    Leap Motion library for Processing.
    Copyright (c) 2012-2013 held jointly by the individual authors.

    This file is part of Leap Motion library for Processing.

    Leap Motion library for Processing is free software: you can redistribute it and/or
    modify it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    Leap Motion library for Processing is distributed in the hope that it will be
    useful, but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with Leap Motion library for Processing.  If not, see
    <http://www.gnu.org/licenses/>.

*/
import java.util.Map;
import java.util.concurrent.ConcurrentMap;
import java.util.concurrent.ConcurrentHashMap;

import com.leapmotion.leap.Controller;
import com.leapmotion.leap.Finger;
import com.leapmotion.leap.Frame;
import com.leapmotion.leap.Hand;
import com.leapmotion.leap.Tool;
import com.leapmotion.leap.Vector;
import com.leapmotion.leap.processing.LeapMotion;

LeapMotion leapMotion;

ConcurrentMap<Integer, Integer> fingerColors;
ConcurrentMap<Integer, Integer> toolColors;
ConcurrentMap<Integer, Vector> fingerPositions;
ConcurrentMap<Integer, Vector> toolPositions;
Vector handPosition;


void setup()
{
  size(16*50, 9*50);
  background(20);
  frameRate(30);
  ellipseMode(CENTER);

  leapMotion = new LeapMotion(this);
  fingerColors = new ConcurrentHashMap<Integer, Integer>();
  toolColors = new ConcurrentHashMap<Integer, Integer>();
  fingerPositions = new ConcurrentHashMap<Integer, Vector>();
  toolPositions = new ConcurrentHashMap<Integer, Vector>();
  handPosition = new Vector();
}

void draw()
{
  fill(20);
  rect(0, 0, width, height);
  
  //Hand positions

 
  
  for (Map.Entry entry : fingerPositions.entrySet())
  {
    Integer fingerId = (Integer) entry.getKey();
    //println("Finger " + fingerId);
    Vector position = (Vector) entry.getValue();
    //println("Finger Vector: X: " + position.getX() + " Y: " + position.getY() + " Z: " + position.getZ());
    fill(fingerColors.get(fingerId));
    noStroke();
    ellipse(leapToScreenX(position.getX()), leapToScreenY(position.getY()), 24.0, 24.0);
  }
  /*
  for (Map.Entry entry : toolPositions.entrySet())
  {
    Integer toolId = (Integer) entry.getKey();
    Vector position = (Vector) entry.getValue();
    fill(toolColors.get(toolId));
    noStroke();
    ellipse(leapToScreenX(position.getX()), leapToScreenY(position.getY()), 24.0, 24.0);
  }
  */
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
    println("hand position: x: " + handPosition.getX() + " y: " + handPosition.getY() + " z: " + handPosition.getZ());
  }  

  // todo:  clean up expired finger/toolIds
}

static float LEAP_WIDTH = 200.0; // in mm
static float LEAP_HEIGHT = 700.0; // in mm

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

float leapToScreenY(float y)
{
  return lerp(height, 0.0, y/LEAP_HEIGHT);
}
