---
layout: post
title: Lab 3- Communicate Something with Haply
subtitle: Trying to communicate three words with the Haply.
cover-img: /assets/img/lab4/PID.png
thumbnail-img: /assets/img/lab4/PID.png
share-img: /assets/img/lab4/PID.png
tags: [haptics, coursework, labs]
---
This lab focused on getting familiar with controlled actuation by tuning a PID controller. We were provided with skeleton code. We were asked to record our thoughts for each of the steps given to us.

1. Run the sample code and try out the P controller. How does it feel? What does changing the P parameter do? Do you notice any problems?
    <iframe width="560" height="315" src="https://www.youtube.com/embed/snPZVaaJAj0" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture" allowfullscreen></iframe>
    It feels like the end effector is being moved towards the target position. If you pull it away from the target, it springs back towards it. As the P value is increased, the force with which it is pulled towards the target increases. There is oscillation in many cases when it gets to the target because it overshoots and then tries to get back to the target.

2. Add the D component to your controller. How does this change the behavior of the haply? Are there any problems?
    <iframe width="560" height="315" src="https://www.youtube.com/embed/cXgyL-ATQdk" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture" allowfullscreen></iframe>
    On my first run adding the D component, there was increased oscillation along the path to the target position, not just around the target position. Next I changed smoothing to 0.9 to see if I could understand the effects of the D component better. This decreased the oscillation but did add lag. Appart from the lag, just the P and D components together seem to tune the controller fairly well. The lower the derivative value, the wider the oscillations. When the derivative is higher the oscillations were more violent but also located more narrowly around the target position.

3. Add the I component to your controller. How does this change the behaviour of the Haply? Can you create a stable system that reaches the target?
    <iframe width="560" height="315" src="https://www.youtube.com/embed/fSkPxxnk1Cc" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture" allowfullscreen></iframe>
    The I component when I first added it caused my controller to overshoot the target position (see video above). In order to tune my controller, I followed the steps from the <a href="https://ctms.engin.umich.edu/CTMS/index.php?example=Introduction&section=ControlPID">University of Michigan tutorial</a> that we went through for our readings. It took me a lot of fiddling, but I was able to get a controller that reaches the target both accurately and quickly.
    <iframe width="560" height="315" src="https://www.youtube.com/embed/yJqmOrrFOV4" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture" allowfullscreen></iframe>

4. Implement path tracking, specifically, replace the random position by a position following a track over time (e.g., a circle or square). What happens when you hold the handle? How is it affected by the PID parameters?
    <iframe width="560" height="315" src="https://www.youtube.com/embed/L8_tWcMoeKg" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture" allowfullscreen></iframe>
    I used <a href="https://stackoverflow.com/questions/34842502/processing-how-do-i-make-an-object-move-in-a-circular-path">this stack overflow post</a> as reference for the math of how to make the point move in a circular path. I ended up adjusting the radius by hand to make sure I had something of a size the haply could follow. I hardcoded my own point for the same reason. When I hold the handle, the haply moves my hand in a circular motion following the target. The behavior of the haply is affected in a similar way to how it was affected by the PID parameters when the target was not a moving point. The P value still controls how much force is used to push the end effector towards the target, the D value controls whether the oscillation is narrow or wide, and the I value helps smooth everything out. I had to clear the integrator value very frequently to avoid integrator windup.

5. Play with the controller update rates, and with introducing delays. How does this change the system? What happens if you sample faster or slower? What happens if it's random?
    <iframe width="560" height="315" src="https://www.youtube.com/embed/1GSIEFFFoAo" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture" allowfullscreen></iframe>
    I found new PID settings in the last question (0.04, 0.01, 0.76). The looptime parameter makes things more smooth if it is smaller and less smooth as it gets larger. The sample changes how often you are updating based on feedback, so this makes a lot of sense. When you update very often, you do smaller corrections more often, and when you update less often you need to do larger corrections which causes a type of oscillation. If the update rate is random it increases the roughness.


See my code below:
{% highlight processing%}
/**
 **********************************************************************************************************************
 * @file       sketch_2_Hello_Wall.pde
 * @author     Steve Ding, Colin Gallacher, Antoine Weill--Duflos
 * @version    V1.0.0
 * @date       09-February-2021
 * @brief      PID example with random position of a target
 **********************************************************************************************************************
 * @attention
 *
 *
 **********************************************************************************************************************
 */
 
  /* library imports *****************************************************************************************************/ 
import processing.serial.*;
import static java.util.concurrent.TimeUnit.*;
import java.util.concurrent.*;
import controlP5.*;
/* end library imports *************************************************************************************************/  


/* scheduler definition ************************************************************************************************/ 
private final ScheduledExecutorService scheduler      = Executors.newScheduledThreadPool(1);
/* end scheduler definition ********************************************************************************************/ 

ControlP5 cp5;



/* device block definitions ********************************************************************************************/
Board             haplyBoard;
Device            widgetOne;
Mechanisms        pantograph;

byte              widgetOneID                         = 5;
int               CW                                  = 0;
int               CCW                                 = 1;
boolean           renderingForce                     = false;
/* end device block definition *****************************************************************************************/



/* framerate definition ************************************************************************************************/
long              baseFrameRate                       = 120;
/* end framerate definition ********************************************************************************************/ 



/* elements definition *************************************************************************************************/

/* Screen and world setup parameters */
float             pixelsPerMeter                      = 4000.0;
float             radsPerDegree                       = 0.01745;

/* pantagraph link parameters in meters */
float             l                                   = 0.07;
float             L                                   = 0.09;


/* end effector radius in meters */
float             rEE                                 = 0.003;

/* circle parameters */
PVector center;
float angle;
float radius;

/* generic data for a 2DOF device */
/* joint space */
PVector           angles                              = new PVector(0, 0);
PVector           torques                             = new PVector(0, 0);
PVector           oldangles                              = new PVector(0, 0);
PVector           diff                              = new PVector(0, 0);


/* task space */
PVector           posEE                               = new PVector(0, 0);
PVector           fEE                                 = new PVector(0, 0); 

/* device graphical position */
PVector           deviceOrigin                        = new PVector(0, 0);

/* World boundaries reference */
final int         worldPixelWidth                     = 1000;
final int         worldPixelHeight                    = 650;

float x_m,y_m;

// used to compute the time difference between two loops for differentiation
long oldtime = 0;
// for changing update rate
int iter = 0;

/// PID stuff

float P = 0.0;
// for I
float I = 0;
float cumerrorx = 0;
float cumerrory = 0;
// for D
float oldex = 0.0f;
float oldey = 0.0f;
float D = 0;

//for exponential filter on differentiation    
float diffx = 0;
float diffy = 0;
float buffx = 0;
float buffy = 0;
float smoothing = 0.90;

float xr = 0;
float yr = 0;

// checking everything run in less than 1ms
long timetaken= 0;

// set loop time in usec (note from Antoine, 500 is about the limit of my computer max CPU usage)
int looptime = 500;


/* graphical elements */
PShape pGraph, joint, endEffector;
PShape wall;
PShape target;
PFont f;

boolean cleanUp=false;
/* end elements definition *********************************************************************************************/ 



/* setup section *******************************************************************************************************/
void setup(){
  /* put setup code here, run once: */
  
  /* screen size definition */
  size(1000, 700);
  
  /*path tracking setup */
  center= new PVector(width/2, height/2);
  PVector point= new PVector(1, 1);
  float deltaX= center.x-point.x;
  float deltaY= center.y-point.y;
  angle= atan2(deltaX, deltaY);
  
  radius= dist(center.x, center.y, point.x, point.y);
  ellipseMode(RADIUS);
  
  /* GUI setup */
    smooth();
  cp5 = new ControlP5(this);
  cp5.addTextlabel("Prop")
                    .setText("Gain for P(roportional)")
                    .setPosition(0,0)
                    .setColorValue(color(255,0,0))
                    .setFont(createFont("Georgia",20))
                    ;
  cp5.addKnob("P")
               .setRange(0,2)
               .setValue(0)
               .setPosition(50,25)
               .setRadius(50)
               .setDragDirection(Knob.VERTICAL)
               ;
  cp5.addTextlabel("Int")
                    .setText("Gain for I(ntegral)")
                    .setPosition(0,125)
                    .setColorValue(color(255,0,0))
                    .setFont(createFont("Georgia",20))
                    ;
  cp5.addKnob("I")
               .setRange(0,2)
               .setValue(0)
               .setPosition(50,150)
               .setRadius(50)
               .setDragDirection(Knob.VERTICAL)
               ;
  cp5.addTextlabel("Deriv")
                    .setText("Gain for D(erivative)")
                    .setPosition(0,250)
                    .setColorValue(color(255,0,0))
                    .setFont(createFont("Georgia",20))
                    ;
  cp5.addKnob("D")
               .setRange(0,4)
               .setValue(0)
               .setPosition(50,275)
               .setRadius(50)
               .setDragDirection(Knob.VERTICAL)
               ; 
  cp5.addTextlabel("Deriv filt")
                    .setText("Exponential filter for Diff")
                    .setPosition(0,375)
                    .setColorValue(color(255,0,0))
                    .setFont(createFont("Georgia",20))
                    ;  
  cp5.addSlider("smoothing")
     .setPosition(10,400)
     .setSize(200,20)
     .setRange(0,1)
     .setValue(0.8)
     ;
  cp5.addTextlabel("Loop time")
                    .setText("Loop time")
                    .setPosition(0,420)
                    .setColorValue(color(255,0,0))
                    .setFont(createFont("Georgia",20))
                    ;  
  cp5.addSlider("looptime")
     .setPosition(10,450)
     .setWidth(200)
     .setRange(250,4000) // values can range from big to small as well
     .setValue(500)
     .setNumberOfTickMarks(16)
     .setSliderMode(Slider.FLEXIBLE)
     ;
  cp5.addButton("RandomPosition")
     .setValue(0)
     .setPosition(10,500)
     .setSize(200,50)
     ;
  cp5.addButton("ResetIntegrator")
     .setValue(0)
     .setPosition(10,560)
     .setSize(200,50)
     ;
  cp5.addButton("ResetDevice")
     .setValue(0)
     .setPosition(10,620)
     .setSize(200,50)
     ;

  /* device setup */
  
  /**  
   * The board declaration needs to be changed depending on which USB serial port the Haply board is connected.
   * In the base example, a connection is setup to the first detected serial device, this parameter can be changed
   * to explicitly state the serial port will look like the following for different OS:
   *
   *      windows:      haplyBoard = new Board(this, "COM10", 0);
   *      linux:        haplyBoard = new Board(this, "/dev/ttyUSB0", 0);
   *      mac:          haplyBoard = new Board(this, "/dev/cu.usbmodem1411", 0);
   */ 
  haplyBoard          = new Board(this, "/dev/cu.usbmodem14101", 0);
  widgetOne           = new Device(widgetOneID, haplyBoard);
  pantograph          = new Pantograph();
  
  widgetOne.set_mechanism(pantograph);
  
  widgetOne.add_actuator(1, CCW, 2);
  widgetOne.add_actuator(2, CW, 1);
 
  widgetOne.add_encoder(1, CCW, 241, 10752, 2);
  widgetOne.add_encoder(2, CW, -61, 10752, 1);
  
  widgetOne.device_set_parameters();
    
  
  /* visual elements setup */
  background(0);
  deviceOrigin.add(worldPixelWidth/2, 0);
  
  /* create pantagraph graphics */
  create_pantagraph();
  
  
  target = createShape(ELLIPSE, 0,0, 20, 20);
  target.setStroke(color(0));
  
  /* setup framerate speed */
  frameRate(baseFrameRate);
    f = createFont("Arial",16,true); // STEP 2 Create Font
  
  /* setup simulation thread to run at 1kHz */ 
  thread("SimulationThread");
}
/* end setup section ***************************************************************************************************/

public void RandomPosition(int theValue) {
      xr = random(-0.5,0.5);
    yr = random(-0.5,0.5);
}
public void ResetIntegrator(int theValue) {
    cumerrorx= 0;
    cumerrory= 0;
}
public void ResetDevice(int theValue) {
    widgetOne.device_set_parameters();

}


/* Keyboard inputs *****************************************************************************************************/

/// Antoine: this is specific to qwerty keyboard layout, you may want to adapt

void keyPressed() {
  if (key == 'q') {
    P += 0.01;
  } else if (key == 'a') {
    P -= 0.01;
  }
  else if (key == 'w') {
    I += 0.00001;
  }
  else if (key == 's') {
    I -= 0.00001;
  }
  else if (key == 'e') {
    D += 0.1;
  }
  else if (key == 'd') {
    D -= 0.1;
  }
  else if (key == 'r') {
    looptime += 100;
  }
  else if (key == 'f') {
    looptime -= 100;
  }
    else if (key == 't') {
    smoothing += 0.01;
  }
  else if (key == 'g') {
    smoothing -= 0.01;
  }
  else if (key == ' ') {
    cumerrorx= 0;
    cumerrory= 0;
  }
  else if (key == 'i') {
    widgetOne.device_set_parameters();
  }
  else if (key == 'b') {
    xr = random(-0.5,0.5);
    yr = random(-0.5,0.5);
  }
}


/* draw section ********************************************************************************************************/
void draw(){
  /* put graphical code here, runs repeatedly at defined framerate in setup, else default at 60fps: */
  if(renderingForce == false){
    background(255); 
    update_animation(angles.x*radsPerDegree, angles.y*radsPerDegree, posEE.x, posEE.y);
    
    
  }
}
/* end draw section ****************************************************************************************************/

/* exit code created by Juliette */
void exit() {
  cleanUp = true;
  widgetOne.set_device_torques(new float[]{0, 0});
  widgetOne.device_write_torques();
  super.exit();
}
/* end exit section */

int noforce = 0;
long timetook = 0;
long looptiming = 0;
/* simulation section **************************************************************************************************/
public void SimulationThread(){
while(cleanUp == false) {
    long starttime = System.nanoTime();
    long timesincelastloop=starttime-timetaken;
    iter+= 1;
    // we check the loop is running at the desired speed (with 10% tolerance)
    if(timesincelastloop >= looptime*1000*1.1) {
      float freq = 1.0/timesincelastloop*1000000.0;
        println("caution, freq droped to: "+freq + " kHz");
    }
    else if(iter >= 1000) {
      float freq = 1000.0/(starttime-looptiming)*1000000.0;
       println("loop running at "  + freq + " kHz");
       iter=0;
       looptiming=starttime;
    }
    
    timetaken=starttime;
    
    renderingForce = true;
    
    if(haplyBoard.data_available()){
      /* GET END-EFFECTOR STATE (TASK SPACE) */
      widgetOne.device_read_data();
      
      noforce = 0;
      angles.set(widgetOne.get_device_angles());
    
      posEE.set(widgetOne.get_device_position(angles.array()));

      posEE.set(device_to_graphics(posEE)); 
      float x= center.x + cos(angle)*radius/4;
      float y= center.y + sin(angle)*radius/4 +50;
      x_m = x-500; 
      y_m = y;//mouseY;
      //angle+= PI/120;
 // Torques from difference in endeffector and setpoint, set gain, calculate force
      float xE = pixelsPerMeter * posEE.x;
      float yE = pixelsPerMeter * posEE.y;
      long timedif = System.nanoTime()-oldtime;

      float dist_X = x_m-xE;
      cumerrorx += dist_X*timedif*0.000000001;
      float dist_Y = y_m-yE;
      cumerrory += dist_Y*timedif*0.000000001;
      //println(dist_Y*k + " " +dist_Y*k);
      // println(timedif);
      if(timedif > 0) {
        buffx = (dist_X-oldex)/timedif*1000*1000;
        buffy = (dist_Y-oldey)/timedif*1000*1000;            

        diffx = smoothing*diffx + (1.0-smoothing)*buffx;
        diffy = smoothing*diffy + (1.0-smoothing)*buffy;
        oldex = dist_X;
        oldey = dist_Y;
        oldtime=System.nanoTime();
      }
    
    // Forces are constrained to avoid moving too fast
  
      fEE.x = constrain(P*dist_X,-4,4) + constrain(I*cumerrorx,-4,4) + constrain(D*diffx,-8,8);

      
      fEE.y = constrain(P*dist_Y,-4,4) + constrain(I*cumerrory,-4,4) + constrain(D*diffy,-8,8); 


      if(noforce==1)
      {
        fEE.x=0.0;
        fEE.y=0.0;
      }
    widgetOne.set_device_torques(graphics_to_device(fEE).array());
    //println(f_y);
      /* end haptic wall force calculation */
      
    }
    
    
    
    widgetOne.device_write_torques();
  
  
    renderingForce = false;
    long timetook=System.nanoTime()-timetaken;
    if(timetook >= 1000000) {
    println("Caution, process loop took: " + timetook/1000000.0 + "ms");
    }
    else {
      while(System.nanoTime()-starttime < looptime*1000) {
      //println("Waiting");
      }
    }
    
  }
}

/* end simulation section **********************************************************************************************/


/* helper functions section, place helper functions here ***************************************************************/
void create_pantagraph(){
  float lAni = pixelsPerMeter * l;
  float LAni = pixelsPerMeter * L;
  float rEEAni = pixelsPerMeter * rEE;
  
  pGraph = createShape();
  pGraph.beginShape();
  pGraph.fill(255);
  pGraph.stroke(0);
  pGraph.strokeWeight(2);
  
  pGraph.vertex(deviceOrigin.x, deviceOrigin.y);
  pGraph.vertex(deviceOrigin.x, deviceOrigin.y);
  pGraph.vertex(deviceOrigin.x, deviceOrigin.y);
  pGraph.vertex(deviceOrigin.x, deviceOrigin.y);
  pGraph.endShape(CLOSE);
  
  joint = createShape(ELLIPSE, deviceOrigin.x, deviceOrigin.y, rEEAni, rEEAni);
  joint.setStroke(color(0));
  
  endEffector = createShape(ELLIPSE, deviceOrigin.x, deviceOrigin.y, 2*rEEAni, 2*rEEAni);
  endEffector.setStroke(color(0));
  strokeWeight(5);
  
}


PShape create_wall(float x1, float y1, float x2, float y2){
  x1 = pixelsPerMeter * x1;
  y1 = pixelsPerMeter * y1;
  x2 = pixelsPerMeter * x2;
  y2 = pixelsPerMeter * y2;
  
  return createShape(LINE, deviceOrigin.x + x1, deviceOrigin.y + y1, deviceOrigin.x + x2, deviceOrigin.y+y2);
}




void update_animation(float th1, float th2, float xE, float yE){
  background(255);
  pushMatrix();
  float lAni = pixelsPerMeter * l;
  float LAni = pixelsPerMeter * L;
  
  xE = pixelsPerMeter * xE;
  yE = pixelsPerMeter * yE;
  
  th1 = 3.14 - th1;
  th2 = 3.14 - th2;
    
  pGraph.setVertex(1, deviceOrigin.x + lAni*cos(th1), deviceOrigin.y + lAni*sin(th1));
  pGraph.setVertex(3, deviceOrigin.x + lAni*cos(th2), deviceOrigin.y + lAni*sin(th2));
  pGraph.setVertex(2, deviceOrigin.x + xE, deviceOrigin.y + yE);
  
  shape(pGraph);
  shape(joint);
  float[] coord;
  
  
  translate(xE, yE);
  shape(endEffector);
  popMatrix();
  arrow(xE,yE,fEE.x,fEE.y);
  textFont(f,16);                  // STEP 3 Specify font to be used
  fill(0);                         // STEP 4 Specify font color 
  
    float x= center.x + cos(angle)*radius/4;
    float y= center.y + sin(angle)*radius/4 +50;
    
    angle+= PI/120;
    x_m = x; 
      //println(x_m + " " + mouseX);")
    y_m = y;//mouseY;
  pushMatrix();
  translate(x_m, y_m);
  shape(target);
  popMatrix();
  
}


PVector device_to_graphics(PVector deviceFrame){
  return deviceFrame.set(-deviceFrame.x, deviceFrame.y);
}


PVector graphics_to_device(PVector graphicsFrame){
  return graphicsFrame.set(-graphicsFrame.x, graphicsFrame.y);
}

void arrow(float x1, float y1, float x2, float y2) {
  x2=x2*10.0;
  y2=y2*10.0;
  x1=x1+500;
  x2=-x2+x1;
  y2=y2+y1;

  line(x1, y1, x2, y2);
  pushMatrix();
  translate(x2, y2);
  float a = atan2(x1-x2, y2-y1);
  rotate(a);
  line(0, 0, -10, -10);
  line(0, 0, 10, -10);
  popMatrix();
} 

/* end helper functions section ****************************************************************************************/

{% endhighlight %}