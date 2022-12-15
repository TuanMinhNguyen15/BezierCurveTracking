float x_origin;
float y_origin;
float scaling = 200;
Point p_origin = new Point();

String[] config_data;
String[] map_data;
String[] planner_data;
String[] robot_data;
String[] obstacles_data;

float[] linear_bezier = {0,0,0,0};
float[] quadratic_bezier = {0,0,0,0,0,0};
float[] cubic_bezier = {0,0,0,0,0,0,0,0};

int num_curves;
int tsteps;
int horizon;
int num_obstacles;

int tstep = 0;
boolean recording = true;

public class Point{
  float x;
  float y;
 
};

Point transform(Point p){
  Point p_transformed = new Point();
  p_transformed.x = p_origin.x + scaling*p.x;
  p_transformed.y = p_origin.y - scaling*p.y;
  return p_transformed;
}

void draw_line(float x1,float y1,float x2,float y2){
  Point p1 = new Point();
  Point p2 = new Point();
  p1.x = x1;
  p1.y = y1;
  p2.x = x2;
  p2.y = y2;
  
  p1 = transform(p1);
  p2 = transform(p2);
  
  line(p1.x,p1.y,p2.x,p2.y);
}

void draw_point(float x,float y){
  Point p = new Point();
  p.x = x;
  p.y = y;
  
  p = transform(p);
 
  point(p.x,p.y);
  resetMatrix();
}

void draw_rect(float x,float y,float theta,float w,float h){
  Point p = new Point();
  p.x = x;
  p.y = y;
  
  p = transform(p);
  w = w*scaling;
  h = h*scaling;
  
  
  translate(p.x, p.y);
  rotate(-theta);
  
  rect(-w/2,-h/2,w,h);
  
  resetMatrix();
  
  
}

void draw_circle(float x, float y, float r){
  Point p = new Point();
  p.x = x;
  p.y = y;
  
  p = transform(p);
  r = r*scaling;
  
  circle(p.x,p.y,2*r);
}

void draw_map(){
  strokeWeight(7);
  
  int index = 0;
  String bezier_type;
  String[] data_line;
  for (int i = 0;i < num_curves;i++){
    bezier_type = map_data[index];
    switch(bezier_type){
       case "linear":
         data_line = map_data[index+1].split(",");
         linear_bezier[0] = Float.parseFloat(data_line[0]);
         linear_bezier[1] = Float.parseFloat(data_line[1]);
         
         data_line = map_data[index+2].split(",");
         linear_bezier[2] = Float.parseFloat(data_line[0]);
         linear_bezier[3] = Float.parseFloat(data_line[1]);
         
         draw_line(linear_bezier[0],linear_bezier[1],linear_bezier[2],linear_bezier[3]);
         
         index = index + 3;
         break;
       case "quadratic":
         break;
       case "cubic":
         break;
    }
  }
  
  strokeWeight(4);
}

void draw_planner(int step){
  String[] data_line;

  stroke(255,0,0);
  for(int i = 0 + (horizon+1)*step ; i <= horizon + (horizon+1)*step ; i++){
    data_line = planner_data[i].split(","); 
    draw_point(Float.parseFloat(data_line[0]),Float.parseFloat(data_line[1]));
  }
  stroke(0);
}

void draw_robot(int step){
  fill(255);
  String[] data_line;
  data_line = robot_data[step].split(",");
  draw_rect(Float.parseFloat(data_line[0]),Float.parseFloat(data_line[1]),Float.parseFloat(data_line[2]),0.2,0.1);
  noFill();
}

void draw_obstacles(){
  fill(195,255,50);
  int index = 0;
  String obstacle_shape;
  for (int i = 0;i < num_obstacles;i++){
    obstacle_shape = obstacles_data[index];
    switch(obstacle_shape){
       case "circle":
         draw_circle(Float.parseFloat(obstacles_data[index+1]),Float.parseFloat(obstacles_data[index+2]),Float.parseFloat(obstacles_data[index+3])); 
         index = index + 4;
         break;
         
       default:
         break;
    }
  }
  noFill();
}

void setup(){
  size(1500, 1000);
  x_origin = 150;
  y_origin = height/2;
  p_origin.x = x_origin;
  p_origin.y = y_origin;
  
  config_data    = loadStrings("config.txt");
  map_data       = loadStrings("map.txt");
  planner_data   = loadStrings("planner.txt");
  robot_data     = loadStrings("robot.txt");
  obstacles_data = loadStrings("obstacles.txt");
  
  num_curves    = Integer.parseInt(config_data[0]);
  tsteps        = Integer.parseInt(config_data[1]);
  horizon       = Integer.parseInt(config_data[2]);
  num_obstacles = Integer.parseInt(config_data[3]);
  
}

void draw(){
  if (tstep < tsteps){
    background(150);
    draw_map();
    draw_obstacles();
    draw_planner(tstep);
    draw_robot(tstep);
    tstep++;
    
    if (recording){
      saveFrame("frames/sim_####.png");
    } 
  }
}
