class Link
{
  float alpha,a,theta,d;
  float x_end,y_end,z_end;
  char state,pos;
  Frame f = new Frame();
  
  Link(float alpha_,float d_,float cont,char type,char pos_)
  {
    alpha = alpha_; // In Radian
    d = d_;
    pos = pos_;
    if(type == 'R')
      a = cont;
    else
      theta = cont;
    
    state = type;
  }
  
  
  void updateLink(float x,float y,float z,float var)
  {
    if(pos == 'S')
      pushMatrix();
    
    if(state == 'R')  
    {
      theta = var;
      rotateX(alpha);
      translate(x,y,z+d);
      f.updateFrame(x);
      rotateZ(theta);
      
      x_end = x*cos(theta)+x;
      y_end = x*sin(theta)+y;
      z_end = x*sin(alpha)+z+d;
      noStroke();
      sphere(10);
      stroke(200);  
      
    }
    else if(state == 'P')
    {
      a =var;
      
      rotateX(alpha);
      translate(x,y,z+d);
      rotateZ(theta);
      f.updateFrame(a);
        
      x_end = x+a;
      y_end = y;
      z_end = z;
      noStroke();
      box(20);
      stroke(200);
    }
    
    pushMatrix();
    translate(0,0,-d/2);
    box(4,4,d);
    popMatrix();
    
    pushMatrix();
    translate(a/2,0,0);
    box(a,10,10);
    popMatrix();
    
    
    if(pos == 'E')
      popMatrix();
   
  } 
}
