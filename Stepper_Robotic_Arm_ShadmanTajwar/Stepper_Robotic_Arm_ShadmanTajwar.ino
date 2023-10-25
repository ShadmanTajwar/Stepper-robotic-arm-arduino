#include <AccelStepper.h>
#include <MultiStepper.h>
#include <Servo.h>

int dirPin[4]= {22, 26, 30, 34};
int stepPin[4]= {24, 28, 32, 36};
int switchPin[4]= {2, 3, 4, 5};
int dir[4]= {1, 1, 0, 1}; //homing direction
int m= 1;

AccelStepper j1 = AccelStepper(1, stepPin[0], dirPin[0]); //pull, dir
AccelStepper j2 = AccelStepper(1, stepPin[1], dirPin[1]);
AccelStepper j3 = AccelStepper(1, stepPin[2], dirPin[2]);
AccelStepper j4 = AccelStepper(1, stepPin[3], dirPin[3]);

MultiStepper steppers;

const float STEPS_PER_REV= 1600;
float da= 360/STEPS_PER_REV; 
float d[]= {0, 90, 360, 90, 360*2, -90};
float t= 0; 

float d1=169, d4=215, d6=0, l0=69, l2=305;
float q1=0, q2= 0, q3= 0, q4 =0;
float u(float t06[12][4][4]); 
float v(float t06[12][4][4]);
float w(float t06[12][4][4]);
float a(float t06[12][4][4]);
float b(float t06[12][4][4]);
float g(float t06[12][4][4]);
float xc, yc, zc, r, pwz, s, p;
float l, lmax, dx, dy, dz, dl, xt, yt, zt;
float t06[][4][4]={{{1,0,0,0},{0,0,-1,284},{0,1,0,474},{0,0,0,1}}, // 0
                        {{1,0,0,0},{0,0,-1,284},{0,1,0,200},{0,0,0,1}}, // 1
                        {{1,0,0,350},{0,0,-1,284},{0,1,0,200},{0,0,0,1}}, // 2
                        {{1,0,0,-350},{0,0,-1,284},{0,1,0,200},{0,0,0,1}}, // 3
                        {{1,0,0,0},{0,0,-1,284},{0,1,0,200},{0,0,0,1}}, // 4
                        {{1,0,0,0},{0,0,-1,500},{0,1,0,200},{0,0,0,1}}, //5
                        {{1,0,0,0},{0,0,-1,400},{0,1,0,200},{0,0,0,1}}, //6
                        {{1,0,0,0},{0,0,-1,400},{0,1,0,500},{0,0,0,1}}, //7
                        {{1,0,0,0},{0,0,-1,400},{0,1,0,200},{0,0,0,1}}, //8
                        {{1,0,0,0},{0,0,-1,284},{0,1,0,200},{0,0,0,1}}, //9
                        {{1,0,0,0},{0,0,-1,284},{0,1,0,474},{0,0,0,1}}}; //10
                         
int n[4]={30, 50, 50, 3}; //gear reduction
int dti[4]={90, 90, 90, 0}; //reset posn         
int z=0;

Servo  gripper;

void setup() {
  Serial.begin(9600);
  pinMode(13, OUTPUT);
  gripper.attach(10);

  delay(3000);

  while(m< 1)
  {
    pinMode(dirPin[m], OUTPUT);
    pinMode(stepPin[m], OUTPUT);
    pinMode(switchPin[m], INPUT);
    m++; 
  }
  m= 0; 

  while(m< 3)
  {
    while(digitalRead(switchPin[m])!= LOW)
    {
      digitalWrite(dirPin[m], dir[m]);
      digitalWrite(stepPin[m], HIGH);
      digitalWrite(stepPin[m], LOW);
      delayMicroseconds(400);
    }
    delay(2000);
    m++;
  }

  j1.setCurrentPosition(0);
  j2.setCurrentPosition(0);
  j3.setCurrentPosition(0);
  j4.setCurrentPosition(0);

  j1.setMaxSpeed(3800);
  j2.setMaxSpeed(3800);
  j3.setMaxSpeed(3800);
  j4.setMaxSpeed(2400);

  steppers.addStepper(j1);
  steppers.addStepper(j2);
  steppers.addStepper(j3);
  steppers.addStepper(j4);

  long positions[3];

  positions[0]= 50*n[0]; //1600 stepperrev, 0.225 degperstep, 10 deg~ 45 steps
  positions[1]= 40*n[1];
  positions[2]= -44*n[2];
  positions[3]= 0;
  
  steppers.moveTo(positions);
  steppers.runSpeedToPosition();

  j1.setCurrentPosition(0);
  j2.setCurrentPosition(0);
  j3.setCurrentPosition(0);
  j4.setCurrentPosition(0);

  delay(2000);

  Serial.print("Start- ");
  Serial.print("\n");
}

//----------------------------------------------------------------LOOP-------------------------------------------------------------------
void loop() {
  g0(t06, 0);
  delay(1000);
  g0(t06, 1);
  delay(1000);
  g1(1, 2, 200, 0.01, 0.5);
  delay(1000);
  g1(2, 3, 200, 0.01, 0.5);
  delay(1000);
  g1(3, 4, 200, 0.01, 0.5);
  delay(1000);
  g1(4, 5, 200, 0.01, 0.5);
  delay(1000);
  g1(5, 6, 200, 0.01, 0.5);
  delay(1000);
  g1(6, 7, 200, 0.01, 0.5);
  delay(1000);
  g1(7, 8, 200, 0.01, 0.5);
  delay(1000);
  g1(8, 9, 200, 0.01, 0.5);
  delay(1000);
  g0(t06, 10);
  delay(1000);
  m0();
}

void g0(float t06[][4][4], int k) //go between two points at max speed
{
  long positions[4];

  positions[0]= round((u(t06, k)-dti[0])*n[0]*(1)/da);
  positions[1]= round((v(t06, k)-dti[1])*n[1]*(1)/da);
  positions[2]= round((w(t06, k)-dti[2])*n[2]*(-1)/da);
  positions[3]= round((g(t06, k)-dti[3])*n[3]*(-1)/da);

  steppers.moveTo(positions);
  steppers.runSpeedToPosition(); 
}


void g1(int i, int f, int r, float acc0, float vmax) //go in straight line between two points at max speed
{
  l= 0;
  dx= t06[f][0][3]-t06[i][0][3];
  dy= t06[f][1][3]-t06[i][1][3];
  dz= t06[f][2][3]-t06[i][2][3];
  lmax= sqrt(pow(dx,2)+pow(dy,2)+pow(dz,2));
  dl= lmax/r;

  while(l<= lmax)
  {
    xt= t06[i][0][3]+(dx*l/lmax);
    yt= t06[i][1][3]+(dy*l/lmax);
    zt= t06[i][2][3]+(dz*l/lmax);
    float t07[1][4][4]= {{{1,0,0,xt},{0,0,-1,yt},{0,1,0,zt},{0,0,0,1}}};
    
    long positions[4];
    positions[0]= round((u(t07, 0)-dti[0])*n[0]*(1)/da);
    positions[1]= round((v(t07, 0)-dti[1])*n[1]*(1)/da);
    positions[2]= round((w(t07, 0)-dti[2])*n[2]*(-1)/da);
    positions[3]= round((g(t07, 0)-dti[3])*n[3]*(-1)/da);

    steppers.moveTo(positions);
    steppers.runSpeedToPosition(); 
  
    l+= dl;
  }
}

void g2(int i, int f, int r, float acc0, float vmax) //go in straight line between two points at vmax velocity with acceleration and deceleration- S curve profile
{
  float dx=t06[f][0][3]-t06[i][0][3];
  float dy=t06[f][1][3]-t06[i][1][3];
  float dz=t06[f][2][3]-t06[i][2][3];
  float lmax=sqrt(pow(dx,2)+pow(dy,2)+pow(dz,2));
  float tap= (vmax/acc0)*(1e+6);
  acc0= acc0*(1e-12);
  vmax= vmax*(1e-6);  
  float lap= 0.5*acc0*pow(tap,2);
  float lcsp= lmax-(lap+lap); //(0.5*acc0*pow(tap,2))-((vmax*tap)-(0.5*acc0*pow(tap,2))); float lcsp=lmax-(2*lap);
  float tcsp= ((lcsp)/(vmax))+tap;
  float tfin = tap+tcsp;
  int N= ceil(tfin/100000);
  long int tdjk= ceil(tfin/N);
  long int tk= tdjk; 
  unsigned long t= 0;
  unsigned long t0= micros();

  while(t<= tfin)
  {
    if(t+tdjk>= tk)
    {
      if(tk<= tap)
      {
        l= 0.5*acc0*pow(tk,2);
      }
      if(tk> tap && tk<= tcsp)
      {
        l= lap+(vmax*(tk-tap));
      }
      if(tk> tcsp)
      {
        l= lap+lcsp+(vmax*(tk-tcsp))-(0.5*acc0*pow((tk-tcsp),2));
      }
      if(tk>= tfin)
      {
        l= lmax;  
      }
      tk+= tdjk; 
    }

    float xt=t06[i][0][3]+(dx*l/lmax);
    float yt=t06[i][1][3]+(dy*l/lmax);
    float zt=t06[i][2][3]+(dz*l/lmax);
    float t07[1][4][4]= {{{1,0,0,xt},{0,0,-1,yt},{0,1,0,zt},{0,0,0,1}}};
    
    long positions[4];
    positions[0]= round((u(t07, 0)-dti[0])*n[0]*(1)/da);
    positions[1]= round((v(t07, 0)-dti[1])*n[1]*(1)/da);
    positions[2]= round((w(t07, 0)-dti[2])*n[2]*(-1)/da);
    positions[3]= round((g(t07, 0)-dti[3])*n[3]*(-1)/da);

    steppers.moveTo(positions);
    steppers.runSpeedToPosition();

    t= micros()-t0;
  }

  t0= micros; 
}

void m0() //stop
{
  while(1);
}

void g00(float m) //move m angle
{
  int z= 0;
  long positions[2];
  while(z<2)
  {
    positions[z]= round((m*n[z])/da);
    z++;
  }
  steppers.moveTo(positions);
  steppers.runSpeedToPosition(); 
}


float u(float t06[][4][4], int n) //j1 angle
{
  xc= t06[n][0][3]-(d6*t06[n][0][2]);
  yc= t06[n][1][3]-(d6*t06[n][1][2]);
  zc= t06[n][2][3]-(d6*t06[n][2][2]);

  q1= atan(yc/xc);
  
  if(xc> 1 && yc> 1)
  {
      q1= (q1*180/M_PI);
  }
  if(xc< 1 && yc> 1)
  {
      q1= (q1*180/M_PI)+180;
  }
  if(xc== 0 && yc> 1)
  {
      q1= 90;
  }
  
  return(q1); 
}

float v(float t06[][4][4], int n) //j2 angle
{
  xc= t06[n][0][3]-(d6*t06[n][0][2]);
  yc= t06[n][1][3]-(d6*t06[n][1][2]);
  zc= t06[n][2][3]-(d6*t06[n][2][2]);

  r= sqrt((xc*xc)+(yc*yc))-l0; //r=sqrt((pow(xc,2))+pow(yc,2))-l0;
  pwz= zc-d1;
  s= sqrt((r*r)+(pwz*pwz)); //s=sqrt((pow(r,2))+pow(pwz,2));
  float a= atan(pwz/r);
  float b= fabs(acos(((s*s)+(l2*l2)-(d4*d4))/(2*l2*s))); //b=fabs(acos((pow(s,2)+pow(l2,2)-pow(d4,2))/(2*l2*s)));
  q2= 1.5708-a-b; //q2=(M_PI/2)-a-b;
  q2= (q2*57.2958)+90; //q2=(q2*180/M_PI)+90;
  
  return(q2); 
}

float w(float t06[14][4][4], int n) //j3 angle
{
  xc= t06[n][0][3]-(d6*t06[n][0][2]);
  yc= t06[n][1][3]-(d6*t06[n][1][2]);
  zc= t06[n][2][3]-(d6*t06[n][2][2]);

  r= sqrt((xc*xc)+(yc*yc))-l0; //r=sqrt((pow(xc,2))+pow(yc,2))-l0;
  pwz=zc-d1;
  s= sqrt((r*r)+(pwz*pwz)); //s=sqrt((pow(r,2))+pow(pwz,2));
  float g= fabs(acos(((l2*l2)+(d4*d4)-(s*s))/(2*l2*d4))); //2*l2*d4=36000, g=fabs(acos((pow(l2,2)+pow(d4,2)-pow(s,2))/(36000))) !
  q3= (1.5708)-g; //q3=(M_PI/2)-g;
  q3= ((-1)*q3*57.2958)+90;
  
  return(q3); 
}

float g(float t06[14][4][4], int n) //j5 angle
{
  xc= t06[n][0][3]-(d6*t06[n][0][2]);
  yc= t06[n][1][3]-(d6*t06[n][1][2]);
  zc= t06[n][2][3]-(d6*t06[n][2][2]);

  r= sqrt((xc*xc)+(yc*yc))-l0; //r=sqrt((pow(xc,2))+pow(yc,2))-l0;
  pwz= zc-d1;
  s= sqrt((r*r)+(pwz*pwz)); //s=sqrt((pow(r,2))+pow(pwz,2));
  float a= atan(pwz/r);
  float b= fabs(acos(((s*s)+(l2*l2)-(d4*d4))/(2*l2*s))); //b=fabs(acos((pow(s,2)+pow(l2,2)-pow(d4,2))/(2*l2*s)));
  float g= fabs(acos(((l2*l2*1.0)+(d4*d4)-(s*s))/(2*l2*d4))); //g=fabs(acos((pow(l2,2)+pow(d4,2)-pow(s,2))/(36300)));//2*l2*d4=36000 
  p= 180-(57.2957*(b+g)); //p=180-(180*(b+g)/M_PI);
  q4= p-(a*57.2957)+90; //q4=p-(a*180/M_PI);
  
  return(q4); 
}

float a(float t06[4][4])
{
  float xc=t06[0][3]-(d6*t06[0][2]);
  float yc=t06[1][3]-(d6*t06[1][2]);
  float zc=t06[2][3]-(d6*t06[2][2]);

  float q1=atan(yc/xc);
  float r=sqrt((pow(xc,2))+pow(yc,2))-l0;
  float pwz=zc-d1;
  float s=sqrt((pow(r,2))+pow(pwz,2));
  float a=atan(pwz/r);
  float b=fabs(acos((pow(s,2)+pow(l2,2)-pow(d4,2))/(2*l2*s)));
  float g=fabs(acos((pow(l2,2)+pow(d4,2)-pow(s,2))/(2*l2*d4)));
  float q2=(M_PI/2)-a-b;
  float q3=(M_PI/2)-g;

  float r01q1[3][3]={{cos(q1),0,-sin(q1)},{sin(q1),0,cos(q1)},{0,-1,0}};
  float r01q3[3][3]={{cos(q3),0,-sin(q3)},{sin(q3),0,cos(q3)},{0,-1,0}};
  float r12q2[3][3]={{cos(q2-(M_PI/2)),-sin(q2-(M_PI/2)),0},{sin(q2-(M_PI/2)),cos(q2-(M_PI/2)),0},{0,0,1}};

  int i,j,k;
  float rs;
  float r02[3][3],r03[3][3],tp[3][3],r36[3][3];

  for(i=0;i<3;i++)
      {
      for(j=0;j<3;j++)
      {
          rs=0;
          for(k=0;k<3;k++)
              {
                  rs=rs+(r01q1[i][k]*r12q2[k][j]);
              }
          r02[i][j]=rs;
      }
      }

  for(i=0;i<3;i++)
      {
      for(j=0;j<3;j++)
      {
          rs=0;
          for(k=0;k<3;k++)
              {
                  rs=rs+(r02[i][k]*r01q3[k][j]);
              }
          r03[i][j]=rs;
      }
      }

  for(i=0;i<3;i++)
      {
      for(j=0;j<3;j++)
      {
          tp[j][i]=r03[i][j];
      }
      }

  for(i=0;i<3;i++)
      {
      for(j=0;j<3;j++)
      {
          rs=0;
          for(k=0;k<3;k++)
              {
                  rs=rs+(tp[i][k]*t06[k][j]);
              }
          r36[i][j]=rs;
      }
      }
  float q4=(fabs(atan(r36[1][2]/r36[0][2])+M_PI)*180/M_PI)-90;
  
  if(q4>180)
  {
    q4=180; 
  }
  
  return(q4); 
}

float b(float t06[4][4])
{
  float xc=t06[0][3]-(d6*t06[0][2]);
  float yc=t06[1][3]-(d6*t06[1][2]);
  float zc=t06[2][3]-(d6*t06[2][2]);

  float q1=atan(yc/xc);
  float r=sqrt((pow(xc,2))+pow(yc,2))-l0;
  float pwz=zc-d1;
  float s=sqrt((pow(r,2))+pow(pwz,2));
  float a=atan(pwz/r);
  float b=fabs(acos((pow(s,2)+pow(l2,2)-pow(d4,2))/(2*l2*s)));
  float g=fabs(acos((pow(l2,2)+pow(d4,2)-pow(s,2))/(2*l2*d4)));
  float q2=(M_PI/2)-a-b;
  float q3=(M_PI/2)-g;

  float r01q1[3][3]={{cos(q1),0,-sin(q1)},{sin(q1),0,cos(q1)},{0,-1,0}};
  float r01q3[3][3]={{cos(q3),0,-sin(q3)},{sin(q3),0,cos(q3)},{0,-1,0}};
  float r12q2[3][3]={{cos(q2-(M_PI/2)),-sin(q2-(M_PI/2)),0},{sin(q2-(M_PI/2)),cos(q2-(M_PI/2)),0},{0,0,1}};

  int i,j,k;
  float rs;
  float r02[3][3],r03[3][3],tp[3][3],r36[3][3];

  for(i=0;i<3;i++)
      {
      for(j=0;j<3;j++)
      {
          rs=0;
          for(k=0;k<3;k++)
              {
                  rs=rs+(r01q1[i][k]*r12q2[k][j]);
              }
          r02[i][j]=rs;
      }
      }

  for(i=0;i<3;i++)
      {
      for(j=0;j<3;j++)
      {
          rs=0;
          for(k=0;k<3;k++)
              {
                  rs=rs+(r02[i][k]*r01q3[k][j]);
              }
          r03[i][j]=rs;
      }
      }

  for(i=0;i<3;i++)
      {
      for(j=0;j<3;j++)
      {
          tp[j][i]=r03[i][j];
      }
      }

  for(i=0;i<3;i++)
      {
      for(j=0;j<3;j++)
      {
          rs=0;
          for(k=0;k<3;k++)
              {
                  rs=rs+(tp[i][k]*t06[k][j]);
              }
          r36[i][j]=rs;
      }
      }

  float q5=acos(r36[2][2])*180/M_PI;    
  
  if(q5>180||q5<45)
  {
    q5=90; 
  }
  
  
  return(q5); 
}

double sgn(double x)
{
  if (x > 0.0) return 1.0;
  if (x < 0.0) return -1.0;
  if (x == 0.0) return 0;
  return x;
}