float x_origin;
float y_origin;
float scaling = 300;
Point p_origin = new Point();

String[] general_data;
String[][] config_data;
String[][] map_data;
String[][] planner_data;
String[][] robot_data;
String[] obstacles_data;

float[] linear_bezier = {0,0,0,0};
float[] quadratic_bezier = {0,0,0,0,0,0};
float[] cubic_bezier = {0,0,0,0,0,0,0,0};

int tsteps;
int num_agents;
int num_obstacles;

int[] num_curves;
int[] horizon;


int tstep = 0;
boolean recording = false;

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

void draw_quadratic_bezier(float x1,float y1,float x2,float y2,float x3,float y3){
  noFill();
  
  Point p1 = new Point();
  Point p2 = new Point();
  Point p3 = new Point();
  
  p1.x = x1;
  p1.y = y1;
  p2.x = x2;
  p2.y = y2;
  p3.x = x3;
  p3.y = y3;
  
  p1 = transform(p1);
  p2 = transform(p2);
  p3 = transform(p3);
  
  beginShape();
  vertex(p1.x,p1.y);
  quadraticVertex(p2.x,p2.y, p3.x,p3.y);
  endShape();
}

void draw_cubic_bezier(float x1,float y1,float x2,float y2,float x3,float y3,float x4,float y4){
  noFill();
  
  Point p1 = new Point();
  Point p2 = new Point();
  Point p3 = new Point();
  Point p4 = new Point();
  
  p1.x = x1;
  p1.y = y1;
  p2.x = x2;
  p2.y = y2;
  p3.x = x3;
  p3.y = y3;
  p4.x = x4;
  p4.y = y4;
  
  p1 = transform(p1);
  p2 = transform(p2);
  p3 = transform(p3);
  p4 = transform(p4);
  
  beginShape();
  vertex(p1.x,p1.y);
  bezierVertex(p2.x,p2.y, p3.x,p3.y,p4.x,p4.y);
  endShape();
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
  
  int index;
  String bezier_type;
  String[] data_line;

  for (int agent_id = 0; agent_id < num_agents; agent_id++){
    index = 0;
    for (int i = 0;i < num_curves[agent_id];i++){
      bezier_type = map_data[agent_id][index];
      switch(bezier_type){
         case "linear":
           data_line = map_data[agent_id][index+1].split(",");
           linear_bezier[0] = Float.parseFloat(data_line[0]);
           linear_bezier[1] = Float.parseFloat(data_line[1]);
           
           data_line = map_data[agent_id][index+2].split(",");
           linear_bezier[2] = Float.parseFloat(data_line[0]);
           linear_bezier[3] = Float.parseFloat(data_line[1]);
           
           draw_line(linear_bezier[0],linear_bezier[1],linear_bezier[2],linear_bezier[3]);
           
           index = index + 3;
           break;
         case "quadratic":
           data_line = map_data[agent_id][index+1].split(",");
           quadratic_bezier[0] = Float.parseFloat(data_line[0]);
           quadratic_bezier[1] = Float.parseFloat(data_line[1]);
           
           data_line = map_data[agent_id][index+2].split(",");
           quadratic_bezier[2] = Float.parseFloat(data_line[0]);
           quadratic_bezier[3] = Float.parseFloat(data_line[1]);
           
           data_line = map_data[agent_id][index+3].split(",");
           quadratic_bezier[4] = Float.parseFloat(data_line[0]);
           quadratic_bezier[5] = Float.parseFloat(data_line[1]);
           
           draw_quadratic_bezier(quadratic_bezier[0],quadratic_bezier[1],quadratic_bezier[2],quadratic_bezier[3],quadratic_bezier[4],quadratic_bezier[5]);
           
           index = index + 4;
           
           break;
         case "cubic":
           data_line = map_data[agent_id][index+1].split(",");
           cubic_bezier[0] = Float.parseFloat(data_line[0]);
           cubic_bezier[1] = Float.parseFloat(data_line[1]);
           
           data_line = map_data[agent_id][index+2].split(",");
           cubic_bezier[2] = Float.parseFloat(data_line[0]);
           cubic_bezier[3] = Float.parseFloat(data_line[1]);
           
           data_line = map_data[agent_id][index+3].split(",");
           cubic_bezier[4] = Float.parseFloat(data_line[0]);
           cubic_bezier[5] = Float.parseFloat(data_line[1]);
           
           data_line = map_data[agent_id][index+4].split(",");
           cubic_bezier[6] = Float.parseFloat(data_line[0]);
           cubic_bezier[7] = Float.parseFloat(data_line[1]);
           
           draw_cubic_bezier(cubic_bezier[0],cubic_bezier[1],cubic_bezier[2],cubic_bezier[3],cubic_bezier[4],cubic_bezier[5],cubic_bezier[6],cubic_bezier[7]);
           
           index = index + 5;
           
           break;
      }
    }
  }
  
  strokeWeight(4);
}

void draw_planner(int step){
  stroke(255,0,0);
  
  String[] data_line;
  for (int agent_id = 0; agent_id < num_agents; agent_id++){
    for(int i = 0 + (horizon[agent_id]+1)*step ; i <= horizon[agent_id] + (horizon[agent_id]+1)*step ; i++){
      data_line = planner_data[agent_id][i].split(","); 
      draw_point(Float.parseFloat(data_line[0]),Float.parseFloat(data_line[1]));
    }
  }
  
  stroke(0);
}

void draw_robot(int step){
  fill(255);
  
  String[] data_line;
  for(int agent_id = 0; agent_id < num_agents; agent_id++){
    data_line = robot_data[agent_id][step].split(",");
    draw_rect(Float.parseFloat(data_line[0]),Float.parseFloat(data_line[1]),Float.parseFloat(data_line[2]),0.2,0.1);
  }
  
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
  x_origin = 300;
  y_origin = height/2;
  p_origin.x = x_origin;
  p_origin.y = y_origin;
  
  general_data     = loadStrings("general.txt");
  tsteps           = Integer.parseInt(general_data[0]);
  num_agents       = Integer.parseInt(general_data[1]);
  num_obstacles    = Integer.parseInt(general_data[2]);
  
  config_data = new String[num_agents][];
  map_data = new String[num_agents][];
  planner_data = new String[num_agents][];
  robot_data = new String[num_agents][];
  
  num_curves = new int[num_agents];
  horizon   =  new int[num_agents];
  
  for (int i = 0; i < num_agents; i++){
    config_data[i]    = loadStrings("config"+(i+1)+".txt");
    num_curves[i] = Integer.parseInt(config_data[i][0]);
    horizon[i]    = Integer.parseInt(config_data[i][1]);
    
    map_data[i]       = loadStrings("map"+(i+1)+".txt");
    planner_data[i]   = loadStrings("planner"+(i+1)+".txt");
    robot_data[i]     = loadStrings("robot"+(i+1)+".txt");
  }
  
  obstacles_data = loadStrings("obstacles.txt");
  
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
