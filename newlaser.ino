#include<Stepper.h>

Stepper xmot(100,8,9),ymot(100,10,11);

float X=0.0,Y=0.0,xlc=0.01,ylc=0.01,pi=3.14;

void line(float x1, float y1, float x2, float y2){
  xmot.setSpeed(100);
  ymot.setSpeed(100);
  int xsteps, ysteps,count=0;
  bool xdir = x2 > x1, ydir = y2 > y1;
  xsteps=abs(x2-x1)/xlc;
  ysteps=abs(y2-y1)/ylc;
  if(xsteps>ysteps){
    while(count<ysteps){
      ymot.step(1);
      xmot.step((int)((float)xsteps/(float)ysteps));
      Y+=ylc*(ydir?1:-1);
      X+=(xlc)*(int)((float)ysteps/(float)(xsteps))*(xdir?1:-1);
      Serial.print(X);
      Serial.print(" ");
      Serial.print(Y);
      Serial.print(";");
      Serial.println();
      count++;
    }
  }
  else{
    while(count<xsteps){
      xmot.step(1);
      ymot.step((int)((float)ysteps/(float)xsteps));
      X+=xlc*(xdir?1:-1);
      Y+=(ylc)*(int)((float)xsteps/(float)(ysteps))*(ydir?1:-1);
      Serial.print(X);
      Serial.print(" ");
      Serial.print(Y);
      //Serial.print(";");
      Serial.println();
      count++;
    }    
  }
}

void circle(float radius){
  xmot.setSpeed(100);
  ymot.setSpeed(100);
  float theeta=0,dtheeta=0.01;
  while(theeta<2*pi){
    float x1,y1,x2,y2;
    x1=radius*cosf(theeta);
    y1=radius*sinf(theeta);
    x2=radius*cosf(theeta+dtheeta);
    y2=radius*sinf(theeta+dtheeta);
    line(x1,y1,x2,y2);
    X+=x1;
    Y+=y1;
    //Serial.println(X);
    //Serial.print(" ");
    //Serial.println(Y);
    theeta+=dtheeta;
  }
  //Serial.println(theeta);
}

void g02(float x1, float y1, float i, float j){
  float xc=x1+i,yc=y1+j,radius=sqrt(i*i+j*j),theeta=asin((X-xc)/radius),theetaf=asin((x1-xc)/radius),dtheeta=0.01,tmpx1,tmpx2,tmpy1,tmpy2;
  while(theeta<theetaf){
    tmpx1=xc+(radius*sin(theeta));
    tmpy1=yc+(radius*cos(theeta));
    tmpx2=xc+(radius*sin(theeta+dtheeta));
    tmpy2=yc+(radius*cos(theeta+dtheeta));
    line(tmpx1,tmpy1,tmpx2,tmpy2);
    Serial.println(X);
    Serial.print(" ");
    Serial.println(Y);
    theeta+=dtheeta;
    Serial.println(theeta);
  }
}


void g03(float x1, float y1, float i, float j){
  float xc=x1+i,yc=y1+j,radius=sqrt(i*i+j*j),theeta=asin((Y-yc)/radius),theetaf=asin((y1-yc)/radius),dtheeta=0.01,tmpx1,tmpx2,tmpy1,tmpy2;
  while(theeta<theetaf){
    tmpx1=xc+(radius*cos(theeta));
    tmpy1=yc+(radius*sin(theeta));
    tmpx2=xc+(radius*cos(theeta+dtheeta));
    tmpy2=yc+(radius*sin(theeta+dtheeta));
    line(tmpx1,tmpy1,tmpx2,tmpy2);
    theeta+=dtheeta;
  }
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  //Serial.print("[");
  //circle(5.0);
  g02(1,1,0.5,0.5);
 ///line(100,100,200,700);
}

void loop() {
  //Serial.print(" ");
  delay(1000);
  // put your main code here, to run repeatedly:

}
