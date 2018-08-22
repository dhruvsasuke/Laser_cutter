#ifndef CONFIG_H
#define CONFIG_H
#define AMS1 (1)
#define AMS2 (2)
#define HG7881 (3) // HG7881 Stepper Driver
#define pi 3.14

#define CONTROLLER AMS1


#define VERSION        (1)  // firmware version
#define BAUD           (9600)  // How fast is the Arduino talking?
#define MAX_BUF        (64)  // What is the longest message Arduino can store?
#define STEPS_PER_TURN (400)  // depends on your stepper motor.  most are 200.
#define MIN_STEP_DELAY (50.0)
#define MAX_FEEDRATE   (1000000.0/MIN_STEP_DELAY)
#define MIN_FEEDRATE   (0.01)


// for arc directions
#define ARC_CW          (1)
#define ARC_CCW         (-1)
// Arcs are split into many line segments.  How long are the segments?
#define MM_PER_SEGMENT  (10)
#include<Stepper.h>

Stepper m1(400,8,9),m2(400,10,11);

//------------------------------------------------------------------------------
// METHODS
//------------------------------------------------------------------------------
void m1step(int dir){m1.step(dir);}
void m2step(int dir){m2.step(dir);}
void disable(){}
void setup_controller(){}


#endif


//------------------------------------------------------------------------------
// GLOBALS
//------------------------------------------------------------------------------

char  buffer[MAX_BUF];  // where we store the message until we get a newline
int   sofar;            // how much is in the buffer
float px, py;      // location

// speeds
float fr =     0;  // human version
long  step_delay;  // machine version

// settings
char mode_abs=1;   // absolute mode?


void pause(long ms) {
  delay(ms/1000);
  delayMicroseconds(ms%1000);  // delayMicroseconds doesn't work for values > ~16k.
}


/**
 * Set the feedrate (speed motors will move)
 * @input nfr the new speed in steps/second
 */
void feedrate(float nfr) {
  if(fr==nfr) return;  // same as last time?  quit now.

  if(nfr>MAX_FEEDRATE || nfr<MIN_FEEDRATE) {  // don't allow crazy feed rates
    Serial.print(F("New feedrate must be greater than "));
    Serial.print(MIN_FEEDRATE);
    Serial.print(F("steps/s and less than "));
    Serial.print(MAX_FEEDRATE);
    Serial.println(F("steps/s."));
    return;
  }
  step_delay = 1000000.0/nfr;
  fr = nfr;
}


void position(float npx,float npy) {
  // here is a good place to add sanity tests
  px=npx;
  py=npy;
}

//void line(float newx,float newy) {
//  long i;
//  long over= 0;
//  
//  long dx  = newx-px;
//  long dy  = newy-py;
//  int dirx = dx>0?1:-1;
//  int diry = dy>0?-1:1;  // because the motors are mounted in opposite directions
//  dx = abs(dx);
//  dy = abs(dy);
//
//  if(dx>dy) {
//    over = dx/2;
//    for(i=0; i<dx; ++i) {
//      m1step(dirx);
//      over += dy;
//      if(over>=dx) {
//        over -= dx;
//        m2step(diry);
//      }
//      pause(step_delay);
//    }
//  } else {
//    over = dy/2;
//    for(i=0; i<dy; ++i) {
//      m2step(diry);
//      over += dx;
//      if(over >= dy) {
//        over -= dy;
//        m1step(dirx);
//      }
//      pause(step_delay);
//    }
//  }
//  Serial.print(px);
//  Serial.print(" ");
//  Serial.print(py);
//  Serial.println();
//  px = newx;
//  py = newy;
//}


void line(float x2, float y2){
  int xsteps, ysteps,count=0;
  float xlc = 1,ylc = 1;
  bool xdir = x2 > px, ydir = y2 > py;
  xsteps=abs(x2-px)/xlc;
  ysteps=abs(y2-py)/ylc;
  //Serial.println("Printa karao bhosadpappu");
  px=x2;
  py=y2;
  if(xsteps>ysteps){
    while(count<ysteps){
      m2.step(1);
      m1.step((int)((float)xsteps/(float)ysteps));
      //py+=ylc*(ydir?1:-1);
      //px+=(xlc)*(int)((float)ysteps/(float)(xsteps))*(xdir?1:-1);
      Serial.print(px);
      Serial.print(" ");
      Serial.print(py);
      Serial.print(";");
      Serial.println();
      count++;
    }
  }
  else{
    while(count<xsteps){
      m1.step(1);
      m2.step((int)((float)ysteps/(float)xsteps));
      //px+=xlc*(xdir?1:-1);
      //py+=(ylc)*(int)((float)xsteps/(float)(ysteps))*(ydir?1:-1);
      Serial.print(px);
      Serial.print("\t");
      Serial.print(py);
      //Serial.print(";");
      Serial.println();
      count++;
    }    
  }
}


// returns angle of dy/dx as a value from 0...2PI
float atan3(float dy,float dx) {
  float a = atan2(dy,dx);
  if(a<0) a = (PI*2.0)+a;
  return a;
}


// This method assumes the limits have already been checked.
// This method assumes the start and end radius match.
// This method assumes arcs are not >180 degrees (PI radians)
// cx/cy - center of circle
// x/y - end position
// dir - ARC_CW or ARC_CCW to control direction of arc
//void arc(float cx,float cy,float x,float y,float dir) {
//  // get radius
//  float dx = px - cx;
//  float dy = py - cy;
//  float radius=sqrt(dx*dx+dy*dy);
//
//  // find angle of arc (sweep)
//  float angle1=atan3(dy,dx);
//  float angle2=atan3(y-cy,x-cx);
//  float theta=angle2-angle1;
//  
//  if(dir>0 && theta<0) angle2+=2*PI;
//  else if(dir<0 && theta>0) angle1+=2*PI;
//  
//  theta=angle2-angle1;
//  
//  // get length of arc
//  // float circ=PI*2.0*radius;
//  // float len=theta*circ/(PI*2.0);
//  // simplifies to
//  float len = abs(theta) * radius;
//
//  int i, segments = ceil( len * MM_PER_SEGMENT );
// 
//  float nx, ny, angle3, scale;
//
//  for(i=0;i<segments;++i) {
//    // interpolate around the arc
//    scale = ((float)i)/((float)segments);
//    
//    angle3 = ( theta * scale ) + angle1;
//    nx = cx + cos(angle3) * radius;
//    ny = cy + sin(angle3) * radius;
//    // send it to the planner
//    line(nx,ny);
//  }
//  
//  line(x,y);
//}

void arc(float i, float j, float x1, float y1, float dir){
  
  if(dir == -1){

        float xc=px+i,yc=py+j,radius=sqrt(i*i+j*j),theeta=asin((px-xc)/radius),theetaf=asin((x1-xc)/radius),dtheeta=0.10,tmpx1,tmpx2,tmpy1,tmpy2;

      Serial.print("X= ");
      Serial.print(px);
      Serial.println();
    
      Serial.print("y= ");
      Serial.print(py);
      Serial.println();
    
      Serial.print("x1= ");
      Serial.print(x1);
      Serial.println();
    
      Serial.print("y1= ");
      Serial.print(y1);
      Serial.println();
    
    if(px==x1){
      theeta=acos((py-yc)/radius);
      theetaf=acos((y1-yc)/radius);
    }

    if(px==xc){
      Serial.println("Gaand hili");
      theeta=(py>yc?0:pi);
    }
  
    if(x1==xc){
      Serial.println("Gaand hili 2");
      theetaf=(y1>yc?0:pi);
    }

    if(py==yc){
      Serial.println("Gaand chudi");
      theeta=(px>xc?pi/2:(-pi/2));
    }

  
    if(y1==yc){
      Serial.println("Gaand chudi 2");
      theetaf=(x1>xc?pi/2:(3*pi/2));
    }

    
      Serial.print("theeta ");
      Serial.print(theeta);
      Serial.println();
    
      Serial.print("theetaf");
      Serial.print(theetaf);
      Serial.println();

    while(theeta<theetaf){
      //Serial.println(theeta<theetaf);
      tmpx2=xc+(radius*sin(theeta+dtheeta));
      tmpy2=yc+(radius*cos(theeta+dtheeta));
      line(tmpx2,tmpy2);
      Serial.print(px);
      Serial.print("\t");
      Serial.print(py);
      Serial.print(";");
      Serial.println();
      theeta+=dtheeta;
      //Serial.println(theeta);
    }
  }
  else{
    float xc=px+i,yc=py+j,radius=sqrt(i*i+j*j),theeta=asin((py-yc)/radius),theetaf=asin((y1-yc)/radius),dtheeta=0.02,tmpx1,tmpx2,tmpy1,tmpy2;
    if(py==y1){
      theeta=acos((px-xc)/radius);
      theetaf=acos((x1-xc)/radius);
    }

    if(px==xc){
      Serial.println("pota paneer");
      theeta=(py>yc?pi/2:-pi/2);
    }

    
    if(py==yc){
      Serial.println("pota paneer 2");
      theeta=(px>xc?0:-pi);
    }

    
    if(x1==xc){
      Serial.println("pota paneer 3");
      theetaf=(y1>yc?pi/2:(3*pi/2));
    }

    
    if(y1==yc){
      Serial.println("pota paneer 4");
      theetaf=(x1>xc?0:pi);
    }

    
    //Serial.println("Print ho be madarchod");
    Serial.println(theeta);
    Serial.println(theetaf);
    while(theeta<theetaf){
    tmpx2=xc+(radius*cos(theeta+dtheeta));
    tmpy2=yc+(radius*sin(theeta+dtheeta));
    line(tmpx2,tmpy2);
    Serial.print(px);
    Serial.print("\ts");
    Serial.print(py);
    Serial.print(";");
    Serial.println();
    theeta+=dtheeta;
  }
}

    px=x1;  py=y1;
}

float parseNumber(char code,float val) {
  char *ptr=buffer;  // start at the beginning of buffer
  while((long)ptr > 1 && (*ptr) && (long)ptr < (long)buffer+sofar) {  // walk to the end
    if(*ptr==code) {  // if you find code on your walk,
      return atof(ptr+1);  // convert the digits that follow into a float and return it
    }
    ptr=strchr(ptr,' ')+1;  // take a step from here to the letter after the next space
  }
  return val;  // end reached, nothing found, return default val.
}


/**
 * write a string followed by a float to the serial line.  Convenient for debugging.
 * @input code the string.
 * @input val the float.
 */
void output(const char *code,float val) {
  Serial.print(code);
  Serial.println(val);
}


/**
 * print the current position, feedrate, and absolute mode.
 */
void where() {
  output("X",px);
  output("Y",py);
  output("F",fr);
  Serial.println(mode_abs?"ABS":"REL");
} 


/**
 * display helpful information
 */
void help() {
  Serial.print(F("GcodeCNCDemo2AxisV1 "));
  Serial.println(VERSION);
  Serial.println(F("Commands:"));
  Serial.println(F("G00 [X(steps)] [Y(steps)] [F(feedrate)]; - line"));
  Serial.println(F("G01 [X(steps)] [Y(steps)] [F(feedrate)]; - line"));
  Serial.println(F("G02 [X(steps)] [Y(steps)] [I(steps)] [J(steps)] [F(feedrate)]; - clockwise arc"));
  Serial.println(F("G03 [X(steps)] [Y(steps)] [I(steps)] [J(steps)] [F(feedrate)]; - counter-clockwise arc"));
  Serial.println(F("G04 P[seconds]; - delay"));
  Serial.println(F("G90; - absolute mode"));
  Serial.println(F("G91; - relative mode"));
  Serial.println(F("G92 [X(steps)] [Y(steps)]; - change logical position"));
  Serial.println(F("M18; - disable motors"));
  Serial.println(F("M100; - this help message"));
  Serial.println(F("M114; - report position and feedrate"));
  Serial.println(F("All commands must end with a newline."));
}


/**
 * Read the input buffer and find any recognized commands.  One G or M command per line.
 */
void processCommand() {
  Serial.println("process command");
  int cmd = parseNumber('G',-1);
  switch(cmd) {
  case  0:
  case  1:  // line
    line( parseNumber('X',(mode_abs?px:0)) + (mode_abs?0:px),
          parseNumber('Y',(mode_abs?py:0)) + (mode_abs?0:py) );
          feedrate(parseNumber('F',fr));
    break;
  case 2:
  case 3:   // arc
      feedrate(parseNumber('F',fr));
      arc(parseNumber('I',(mode_abs?px:0)) + (mode_abs?0:px),
          parseNumber('J',(mode_abs?py:0)) + (mode_abs?0:py),
          parseNumber('X',(mode_abs?px:0)) + (mode_abs?0:px),
          parseNumber('Y',(mode_abs?py:0)) + (mode_abs?0:py),
          (cmd==2) ? -1 : 1);
      break;
    
  case  4:  pause(parseNumber('P',0)*1000);  break;  // dwell
  case 90:  mode_abs=1;  break;  // absolute mode
  case 91:  mode_abs=0;  break;  // relative mode
  case 92:  // set logical position
    position( parseNumber('X',0),
              parseNumber('Y',0) );
    break;
  default:  break;
  }

  cmd = parseNumber('M',-1);
  switch(cmd) {
  case 18:  // disable motors
    disable();
    break;
  case 100:  help();  break;
  case 114:  where();  break;
  default:  break;
  }
}


/**
 * prepares the input buffer to receive a new message and tells the serial connected device it is ready for more.
 */
void ready() {
  sofar=0;  // clear input buffer
  Serial.print(F(">"));  // signal ready to receive input
}


/**
 * First thing this machine does on startup.  Runs only once.
 */
void setup() {
  Serial.begin(BAUD);  // open coms

  setup_controller();  
  position(0,0);  // set staring position
  feedrate((MAX_FEEDRATE + MIN_FEEDRATE)/2);  // set default speed
  //arc(5,0,10,0,-1);
  help();  // say hello
  ready();
}


/**
 * After setup() this machine will repeat loop() forever.
 */
void loop() {
  // listen for serial commands
  while(Serial.available() > 0) {  // if something is available
    char c=Serial.read();  // get it
    Serial.print(c);  // repeat it back so I know you got the message
    if(sofar<MAX_BUF-1) buffer[sofar++]=c;  // store it
    if((c=='\n') || (c == '\r')) {
      // entire message received
      buffer[sofar]=0;  // end the buffer so string functions work right
      Serial.print(F("\r\n"));  // echo a return character for humans
      processCommand();  // do something with the command
      ready();
    }
  }
}
