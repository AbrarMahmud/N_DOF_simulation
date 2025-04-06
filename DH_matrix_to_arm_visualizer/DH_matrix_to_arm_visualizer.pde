import peasy.*;
import processing.serial.*;
PeasyCam cam;
Serial myPort;
//constants
float cm = 40; //defining 40 pixels = 1cm

float t_param = 1.0,increment=1.0;

Joint j1,j2,j3,j4,j5,j6; //simulated
Joint j1_ard_code,j2_ard_code,j3_ard_code,j4_ard_code,j5_ard_code,j6_ard_code; //direct from arduino
String myText="";  //received arduino signal from COM_port
float[] list=new float[9]; //6-joint angles + 3-positional variable
boolean phy_arm_avail = false;

void setup()
{
  size(1000,1000,P3D);
  if(phy_arm_avail){
  myPort=new Serial(this, "COM7", 2000000);
  myPort.bufferUntil('\n');
  }
  
  cam =new PeasyCam(this,1500);
  cam.setMinimumDistance(5);
  cam.setMaximumDistance(8000);
  

  

  
// just dh-table
// represent any VARIABLE type data with 0
// Input parameters :  || Theta  ||  Alpha  ||  r  ||  d  ||


//6DOF - surgical ROBOT
//Explanation video___ https://www.youtube.com/watch?v=wDus2EKLg3s    
  j1 = new Joint(0,PI/2,0*cm,2*cm,'R'); 
  j2 = new Joint(0,0,15*cm,0*cm,'R');       
  j3 = new Joint(0,PI/2,0*cm,0*cm,'R');       
  j4 = new Joint(0,-PI/2,0*cm,20*cm,'R');  
  j5 = new Joint(0,PI/2,0*cm,0*cm,'R'); 
  j6 = new Joint(0,0,0*cm,4*cm,'E');

  if(phy_arm_avail){
    j1_ard_code = new Joint(0,PI/2,0*cm,2*cm,'R'); 
    j2_ard_code = new Joint(0,0,15*cm,0*cm,'R');       
    j3_ard_code = new Joint(0,PI/2,0*cm,0*cm,'R');       
    j4_ard_code = new Joint(0,-PI/2,0*cm,20*cm,'R');  
    j5_ard_code = new Joint(0,PI/2,0*cm,0*cm,'R'); 
    j6_ard_code = new Joint(0,0,0*cm,4*cm,'E');
  }
  
//3DOF X,Y,Z-arm 
  //j1 = new Joint(0,PI/2,0*cm,2*cm,'R');  
  //j2 = new Joint(0,0,8*cm,0*cm,'R'); 
  //j3 = new Joint(0,0,8*cm,0*cm,'E');  

//SCARA-type robot
  //j1 = new Joint(0,0,8*cm,0*cm,'R');  //j1 itself a rot-joint and rotate around base-frame
  //j2 = new Joint(0,PI,8*cm,0*cm,'P'); //j2 itself a primatic-joint but rotate around j1-frame
  //j3 = new Joint(0,0,0*cm,0*cm,'E');  //j3 is the end-effector frame but moves prismaticly w.r.t j2-farme

//2DOF planner robot
  //j1 = new Joint(0,0,8*cm,0*cm,'R');  
  //j2 = new Joint(0,0,8*cm,0*cm,'R'); 
  //j3 = new Joint(0,0,2*cm,0*cm,'E');  
  
  //R = rotational joint
  //P = Prismatic joint
  //E = End-effector

} 
void serialEvent(Serial myPort)
{
  if (true)
  {
    myText=myPort.readStringUntil('\n');
    myText=trim(myText);
    list=float(split(myText,','));
    //if(list[1]<10){list[1]=10;}
    
  }
}

void draw()
{  
  directionalLight(250,250,250,500,500,-1000);
  background(100);
  float[] pos = new float[3];
  float[] result_var = new float[6];
  grid();

  pathGeneration(pos);
  inverse_kine(pos[0],pos[1],pos[2],0,result_var);


  
  if(phy_arm_avail){
    //Inputs positional var from arduino COM_port
    inverse_kine(list[6],list[7],list[8],0,result_var);

  }
  
  pushMatrix();
//joint udates

//6DOF - surgical ROBOT (simulated calculation)
  j1.updateJoint(result_var[0],'A');   // variable_type,A = Angle 
  j2.updateJoint(result_var[1],'A');   // variable_type,A = Angle
  j3.updateJoint(result_var[2],'A');   // variable_type,A = Angle
  j4.updateJoint(result_var[3],'A');   // variable_type,A = Angle 
  j5.updateJoint(result_var[4],'A');   // variable_type,A = Angle
  j6.updateJoint(result_var[5],'A');   // variable_type,A = Angle
  
  if(phy_arm_avail){
    // received signal from arduino
    j1_ard_code.updateJoint(list[0],'A');   // variable_type,A = Angle 
    j2_ard_code.updateJoint(list[1],'A');   // variable_type,A = Angle
    j3_ard_code.updateJoint(list[2],'A');   // variable_type,A = Angle
    j4_ard_code.updateJoint(list[3],'A');   // variable_type,A = Angle 
    j5_ard_code.updateJoint(list[4],'A');   // variable_type,A = Angle
    j6_ard_code.updateJoint(list[5],'A');   // variable_type,A = Angle
  }
  
//3DOF X,Y,Z-arm 
  //j1.updateJoint(result_var[0],'A');   // variable_type,A = Angle 
  //j2.updateJoint(result_var[1],'A');   // variable_type,A = Angle
  //j3.updateJoint(result_var[2],'A');   // variable_type,A = Angle
  
//2DOF planner robot  
  //j1.updateJoint(result_var[0],'A');   // variable_type,A = Angle 
  //j2.updateJoint(result_var[1],'A');   // variable_type,A = Angle
  //j3.updateJoint(result_var[2],'A');   // variable_type,A = Angle

//SCARA-type robot
  //j1.updateJoint(result_var[0],'A');   // variable_type,A = Angle 
  //j2.updateJoint(result_var[1],'A');   // variable_type,A = Angle
  //j3.updateJoint(result_var[2],'D');   // variable_type,D = distance
  popMatrix();
 

  

  
 
 //pushMatrix();
 //box(20,10,50);
 //
 //popMatrix();
 
 
 //pushMatrix();// joint 1
 
 //rotateX(0);
 //rotateZ(t_param*PI/6);
 //line(0,0,0,2*cm,0,0);//r1

 //translate(2*cm,0,4*cm);
 //line(0,0,0,0*cm,0,-4*cm);//d1
 //box(20,10,50);
 ////line(0,0,0,2*cm,0,0);
 ////popMatrix();
 
 
 ////pushMatrix(); //joint 2
 //rotateX(0);
 //rotateZ(t_param*PI/6);
 //line(0,0,0,4*cm,0,0);//r2 d2
 //translate(4*cm,0,0);
 //box(20,10,50);
 
 
 ////pushMatrix(); //joint 2
 //rotateX(0);
 //rotateZ(t_param*PI/6);
 //line(0,0,0,8*cm,0,0);//r2 d2
 //translate(8*cm,0,0);
 //box(20,10,50);
 
 ////translate(200,0,100);

 ////rotateX(0);
 ////rotateZ(t_param*PI/6);
 ////box(20,10,50);
 //popMatrix();
 
  //if(t_param>=5.2)
  //  increment = -1;
  //else if(t_param<4)
  //  increment = 1;
  
  t_param= t_param+increment*0.005;
}



void grid()
{
  strokeWeight(1);
  int i,j;
  for(i=-width/2;i<=width/2;i+=width/cm)
    line(i,-height/2,i,height/2);
  for(j=-height/2;j<=height/2 ; j+=height/cm)
    line(-width/2,j,width/2,j); 
  strokeWeight(2); 
  line(0,-height/2,0,height/2);
  line(-height/2,0,height/2,0);


  // Base Axis....generation

  strokeWeight(5);
  stroke(219, 0, 0);      //Red = X-axis
  line(0,0,0,width/2,0,0);
  stroke(1, 212, 86);     //Green = Y-axis
  line(0,0,0,0,-height/2,0);
  stroke(30, 35, 168);    //Blue = Z-axis
  line(0,0,0,0,0,width/2);
  
  stroke(200);
  strokeWeight(1);
}

void pathGeneration(float coordinate[])
{
  if(t_param >= 4*PI)
    t_param = 0.0;
  float t;
  noFill();
  beginShape();
  for(t=0.0;t<t_param;t += 0.01)
    {
      //coordinate[0] = 10;
      //coordinate[1] = 10;
      //coordinate[2] = 10;
      
      coordinate[0] = 5*cos(1*t)+10;
      coordinate[1] = 7*sin(1*t)+10;
      //coordinate[2] = 23;
      coordinate[2] = 3*cos(5*t)+20;
      //coordinate[0] = 6*1*cos(t);
      //coordinate[1] = 6*1*cos(2*t)+1;
      //coordinate[2] = 6*sin(t);
    
//2DOF planner robot (considering end-efector rotation)      
      //coordinate[0] = 2*cos(t)+4;
      //coordinate[1] = 6*sin(t)+4;
      //coordinate[2] = 0*6*sin(t);  

      curveVertex(coordinate[0]*cm,-1*coordinate[1]*cm,coordinate[2]*cm);
      
    }
  endShape();
  fill(200);
  
}

void inverse_kine(float x,float y,float z,float time_stamp,float t[])
{
//6DOF - surgical ROBOT
//Explanation video___ https://www.youtube.com/watch?v=wDus2EKLg3s

 //temporary variables to hold theta(just for ease of writing the code)
 float theta1,theta2,theta3,theta4,theta5,theta6;
 
 // End- effector orientation:
 float p11 = cos(time_stamp) ,p12 = 0,p13 = sin(time_stamp); // rotattion around Y-axis
 float p21 = 0               ,p22 = 1,p23 = 0;
 float p31 = -sin(time_stamp),p32 = 0,p33 = cos(time_stamp);
 
 //float p11 = 1,p12 = 0,p13 = 0;
 //float p21 = 0,p22 = 1,p23 = 0;
 //float p31 = 0,p32 = 0,p33 = 1;
 
 //Arm length parameter
 float r2 = 15;
 float d1 = 2, d4 = 20 ,d6 = 4;// d6 is the distance from the wrist to end effector
                              // d6 will get used to find the spatial postion of the wrist

 // Position of the wrist w.r.t base
 float x_ = x+d6*p13;
 float y_ = y+d6*p23;
 float z_ = z-d6*p33;
 
 //float x_ = x-d6*p13;
 //float y_ = y-d6*p23;
 //float z_ = z-d6*p33; 
 
 //float x_ = x+d6*p13;
 //float y_ = y+d6*p23;
 //float z_ = z+d6*p33;
 theta1 = atan2(y_,x_);//..............................................Theta_1

 float r3 = d4;
 float k1 = x_*cos(theta1)+y_*sin(theta1);
 float ct3 = ((z_-d1)*(z_-d1)+k1*k1-r2*r2-r3*r3)/(2*r2*r3);
 float st3_p = sqrt(1-ct3*ct3);
 
 theta3 = atan2(st3_p,ct3)-PI/2; //....................................Theta_3(PI/2 is super V.V.I)
                                 //                                    PI/2 should be added to the dh_table
                                 //                                    But my jupyter-notebook code cannot handle any
                                 //                                    Theta3+PI/2 type varible ....so i dealt with it in this code
 
 
 theta2 = atan2((r3*ct3+r2),(r3*st3_p))-atan2(k1,(d1-z_)); //..........Theta_2
 
 // Calculate Theta4,Theta5,Theta6
 float ct5 = p13*sin(theta2+theta3)*cos(theta1) + p23*sin(theta1)*sin(theta2+theta3) - p33*cos(theta2+theta3);
 float st5_p = sqrt(1-ct5*ct5);
 
 theta5 = atan2(st5_p,ct5);//..........................................Theta5
 
 
 float s5s6 = p12*sin(theta2 + theta3)*cos(theta1) + p22*sin(theta1)*sin(theta2+theta3) -p32*cos(theta2+theta3);
 float minus_s5c6 = p11*sin(theta2+theta3)*cos(theta1) + p21*sin(theta1)*sin(theta2+theta3)-p31*cos(theta2+theta3);
 
 theta6 = atan2(s5s6,-1*minus_s5c6);//.................................Theta5

 
 float s5s4 = p13*sin(theta1)-p23*cos(theta1);
 float s5c4 = p13*cos(theta1)*cos(theta2+theta3)+p23*sin(theta1)*cos(theta2+theta3)+p33*sin(theta2+theta3);
 
 theta4 = atan2(s5s4,s5c4);//..........................................Thata4
 
 t[0] = theta1 ; 
  t[1] = theta2 ; 
   t[2] = theta3 ; 
    t[3] = theta4 ; 
     t[4] = theta5 ; 
      t[5] = theta6 ; 
 
 
 
//3DOF X,Y,Z-arm  
  //float r2 = 8,r3 = 8,d1 =2;
  //t[0] = atan2(y,x); // Theta_1
  
  //float k1 = x*cos(t[0])+y*sin(t[0]);
  //float ct3 = ((z-d1)*(z-d1)+k1*k1-r2*r2-r3*r3)/(2*r2*r3);
  //float st3_p = sqrt(1-ct3*ct3);
  //t[2] = atan2(st3_p,ct3); //Theta_3
  
  //t[1] = atan2((r3*ct3+r2),(r3*st3_p))-atan2(k1,(d1-z)); //Theta_2

    
//2DOF planner robot (considering end-efector rotation)  
  //assumue end-effector rotating around Z0-axis
  //  |p11  p12  0  x|
  //  |p21  p22  0  y|
  //  |0    0    1  0|
  //  |0    0    0  1|
  //float val1 = (sin(time_stamp))/3;
  //float val2 = (cos(time_stamp));
  //float p11 = cos(atan2(val1,val2)); //Vector for end-effector rotation
  //float p21 = sin(atan2(val1,val2));

  //float ct2 = ((x-2*p11)*(x-2*p11)+(y-2*p21)*(y-2*p21)-2*64)/(2*64);
  //float st2_p = sqrt(1-ct2*ct2);
  //float st2_m = -1.0*st2_p;
  
  //t[1] = atan2(st2_p,ct2); // Theta_2
  
  //t[0] = atan2(-1*st2_p,(1+ct2))+atan2((y-2*p21),(x-2*p11)); // Theta_1
  //t[2] = -(t[0]+t[1])+atan2(p21,p11); // Theta_3


//SCARA-type robot
  //float ct2 = (x*x+y*y-2*64)/(2*64);
  //float st2_p = sqrt(1-ct2*ct2);
  //float st2_m = -1.0*st2_p;
  
  //t[1] = atan2(st2_p,ct2); // Theta_2
  
  //t[0] = atan2(-1*st2_p,(1+ct2))+atan2(y,x); // Theta_1
  //t[2] = -(z*cm); // distance_3
  
  
//Print Output  
      print("t[0]:");
      print(t[0]*180/PI);
      
      print("    t[1]:");
      print(t[1]*180/PI);
      
      print("    t[2]:");
      print(t[2]*180/PI);
      
      print("    t[3]:");
      print(t[3]*180/PI);
      
      print("    t[4]:");
      print(t[4]*180/PI);
      
      print("    t[5]:");
      print(t[5]*180/PI);
      
      print("        t_param______");
      print(time_stamp);
println("   ");
}
