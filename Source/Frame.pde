class Frame
{
  float y_axis = 50;
  float z_axis = 50;
  Frame()
  {
  }
  
  void updateFrame(float a_)
  {
    fill(200,50);
    circle(0,0,50);
    strokeWeight(5);
    stroke(219, 0, 0);   //Red
    line(0,0,0,a_+50,0,0);
    fill(200);
    stroke(1, 212, 86);  //Green
    line(0,0,0,0,y_axis,0);
    stroke(30, 35, 168);  //Blue
    line(0,0,0,0,0,z_axis);
    stroke(200);
    strokeWeight(1);
           
  }
  
  
}
