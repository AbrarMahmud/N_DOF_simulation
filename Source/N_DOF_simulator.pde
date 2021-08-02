import peasy.*;
PeasyCam cam;

//constants
float cm = 40; //defining 40 pixels = 1cm

int frm_arm_len = 40;
float t_param = 0.0;
Link l0,l1,l2,l3,l4;



void setup()
{
  size(1000,1000,P3D);
  
  cam =new PeasyCam(this,700);
  cam.setMinimumDistance(50);
  cam.setMaximumDistance(1000);
  
  //.......                   (theta or a_(i)) <<....Becareful it is not a(i-1) as like the standard
  //-----float alpha_,float d_(i),float cont,char type,char pos_
  
  l0 = new Link(0,0,0,'R','S');        // 'S' = Stating link
  l1 = new Link(0,2*cm,4*cm,'R','N');  // 'N' = NULL
  l2 = new Link(0,cm,4*cm,'R','N');
  l3 = new Link(PI/2,0,-PI/2,'P','E'); // 'E' = Ending link
  
}

void draw()
{  
  directionalLight(250,250,250,500,500,-1000);
  background(100);
  float[] pos = new float[3];
  float[] res_var = new float[3];
  
  grid();
  pathGeneration(pos);
  inverse_kine_3DOF(pos[0],pos[1],pos[2],l1.a,l2.a,res_var);
  
  l0.updateLink(0,0,0,0);
  l1.updateLink(l0.a,0,0,res_var[0]);
  l2.updateLink(l1.a,0,0,res_var[1]);
  l3.updateLink(l2.a,0,0,res_var[2]);
  
  
  t_param= t_param+0.03;
}

void pathGeneration(float coordinate[])
{
  if(t_param >= 4*PI)
    t_param = 0.0;
  float t;
  noFill();
  beginShape();
  for(t=0.0;t<t_param;t += 0.1)
    {
      coordinate[0] = 4*cm*cos(t);
      coordinate[1] = 4*cm*cos(2*t)+1*cm;
      coordinate[2] = 4*cm*sin(t);
      
      curveVertex(coordinate[0],coordinate[1],coordinate[2]);
    
    }
  endShape();
  fill(200);
  
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
  strokeWeight(1);
}


// Inverse Kinematics... :)

void inverse_kine_3DOF(float x,float y,float z,float a1,float a2,float t[])
{
  float theta1_PLUS ,theta1_MINUS ,theta2_PLUS ,theta2_MINUS;
  float k1 ,k2_PLUS ,k2_MINUS;
  float cTheta2 ,sTheta2_PLUS,sTheta2_MINUS;

  cTheta2 = (x*x+y*y-a1*a1-a2*a2)/(2*a1*a2);
  println(x,y,z,a1,a2);
  if(abs(cTheta2)>1)
  {
    print("cos_Theta2 = ");
    print(cTheta2);
    println(" ------------Somethings Wrong!!");
    return;
  }
  
  sTheta2_PLUS = sqrt(1-cTheta2*cTheta2);
  sTheta2_MINUS = -1*sqrt(1-cTheta2*cTheta2);

  ///calculation of theta2/////////////////////////////////////////
  theta2_PLUS = atan2(sTheta2_PLUS,cTheta2);
  //
  theta2_MINUS = atan2(sTheta2_MINUS,cTheta2);
  ///Done//////////////////////////////////////////////////////////

  k1 = a2*cTheta2 +a1;
  k2_PLUS = a2*sTheta2_PLUS;
  k2_MINUS = a2*sTheta2_MINUS;

  ///calculation of theta1/////////////////////////////////////////
  theta1_PLUS = atan2(k1,k2_PLUS)-atan2(x,y);
  theta1_MINUS = atan2(k1,k2_MINUS)-atan2(x,y);



  ///Done//////////////////////////////////////////////////////////

  if(abs(theta1_MINUS)<=PI/2 && abs(theta2_MINUS)<= PI/2)
    {
      t[0] = theta1_MINUS;
      t[1] = theta2_MINUS;
      print("  --MINUS---   |");
      print(t[0]);
      print("|       |");
      println(t[1]);
    }
    else
    {
      t[0] = theta1_PLUS;
      t[1] = theta2_PLUS;
      print("  ---PLUS--   |");
      print(t[0]);
      print("|       |");
      println(t[1]);
    }
  
   t[2] = -z+3*cm;

}
