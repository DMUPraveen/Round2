
#include <webots/robot.h>
#include <webots/range_finder.h>
#include <webots/camera.h>
#include <webots/display.h>
#include <stdio.h>
#include <stdlib.h>


#include <gripper.h>
#include <arm.h>
#include <base.h>

#define TIME_STEP 32

struct blob{
int corner_x;
int corner_y;
int width;
int height;
int initialized;
};
typedef struct blob Blob;



void step(){
  if(wb_robot_step(TIME_STEP) == -1) {
  wb_robot_cleanup();
  exit(0);
  }
}


void set_pixel(WbDeviceTag Display,int color[3],int x,int y){
  int colour = ((color[0])+0+color[3]);
  if(colour < 10 && color[1]>10){colour = 0xffffff;}
  else{colour = 0x000000;}
  
  wb_display_set_color(Display,colour);
  wb_display_draw_pixel(Display,x,y);
}


void draw_display(WbDeviceTag Display,int image[][3],int width,int height){
for(int y=0; y< height; y++){
  for(int x = 0; x<width;x++){
  set_pixel(Display,image[height*y +x],x,y);
  }
}
}


void get_image(WbDeviceTag Camera,int width,int height,int image_array[][3]){
const unsigned char *image = wb_camera_get_image(Camera);
 
for(int y=0; y<height; y++){
  for(int x = 0; x<width;x++){
  
    int R = wb_camera_image_get_red(image,width,x,y);
    int G = wb_camera_image_get_green(image,width,x,y);
    int B = wb_camera_image_get_blue(image,width,x,y);
    //printf("(%d ,%d ,%d) ",R,G,B);
    image_array[height*y+x][0] = R;
    image_array[height*y+x][1] = G;
    image_array[height*y+x][2] = B;
    }
}
}


void filter_image(int image_array[][3],int filter_array[],int width,int height){
for(int y=0; y< height; y++){
  for(int x = 0; x<width;x++){
    int R = image_array[height*y+x][0];
    int G = image_array[height*y+x][1];
    int B = image_array[height*y+x][2];
    if(R+B<10 && G>10){
    filter_array[height*y+x] = 1;
    }
    else{
    filter_array[height*y+x] = 0;
    }
  }
}
}



void wait(float time){
  float initial = wb_robot_get_time();

  while(wb_robot_get_time() - initial < time){
    step();
  }
}

int isnear(Blob* blob,int boundry,int x,int y){
  int cx = blob -> corner_x;
  int cy = blob -> corner_y;
  int w = blob -> width;
  int h = blob -> height;
  int d = boundry;
  if((x > cx -d && x < cx +w+d) &&(y>cy-d && y<cy + h+d)){/*printf("near\n")*/return 1;}
  else{return 0;}



}

void blob_update(Blob* blob,int x, int y){
  int cx = blob -> corner_x;
  int cy = blob -> corner_y;
  int w = blob -> width;
  int h = blob -> height;
  //printf("first: %d ,%d \n",x,cx);
  if(x < cx){blob -> corner_x = x;}
  if(y < cy){blob -> corner_y = y;}
  if(x >cx+w){blob -> width = (x - cx);}
  //printf("second: %d ,%d \n",x,cx);
  if(x >cx+h){blob -> height = (y -cy);}
  
}

void findblobs(Blob blob_array[],int filter_array[],int width,int height,int boundry){
for(int y=0; y< height; y++){
  for(int x = 0; x<width;x++){
  int pixel = filter_array[height*y+x];
  if(pixel){
  //printf("found \n");
  //int i = 0;
  int last = 0;
  int new = 0;
  for(int i=0;i<50;i++){
    //printf("%d \n",i);
    
    if(blob_array[i].initialized ==0){last = i;break;}
    int status = isnear(&blob_array[i],boundry,x,y); 
    if(status){
    //printf("%d \n",isnear(&blob_array[i],boundry,x,y));
      //printf("%d \n",i);
      //printf("y:%d \n",y);
      blob_update(&blob_array[i],x,y);
      new += status;
    
    }
 }
    if(new == 0){
    //printf("setting blob %d \n",i);
    blob_array[last].initialized = 1;
    blob_array[last].corner_x = x;
    blob_array[last].corner_y = y;
    blob_array[last].width = 0;
    blob_array[last].height = 0; 
  
  }
  
  
  
  
  }

  }
}
  

}

int main(int argc, char **argv) {
  wb_robot_init();
  
  //getting device tags for range finder and camera
  WbDeviceTag range_finder;
  WbDeviceTag camera;
  WbDeviceTag display;
  range_finder = wb_robot_get_device("range-finder");
  camera = wb_robot_get_device("camera");
  display = wb_robot_get_device("display"); 
  wb_range_finder_enable(range_finder,TIME_STEP);
  wb_camera_enable(camera,TIME_STEP);
  int display_width = wb_display_get_width(display);
  int display_height = wb_display_get_height(display);
  int camera_width = wb_camera_get_width(camera);
  int camera_height = wb_camera_get_height(camera);
  int image_array[camera_width*camera_height][3];
  int filter_array[camera_width*camera_height];
  printf("width: %d,height: %d \n",camera_width,camera_height);
  wait(1.0);
  
  ///////////
  
  arm_init();
  
    while (1) {
  step();
  get_image(camera,camera_width,camera_height,image_array);
  draw_display(display,image_array,camera_width,camera_height);
  filter_image(image_array,filter_array,camera_width,camera_height);
  Blob blob_array[50] = {{0,0,0,0,1},{0,0,0,0,0},{0,0,0,0,0},{0,0,0,0,0},{0,0,0,0,0},{0,0,0,0,0},{0,0,0,0,0},{0,0,0,0,0},{0,0,0,0,0},{0,0,0,0,0},{0,0,0,0,0},{0,0,0,0,0},{0,0,0,0,0},{0,0,0,0,0},{0,0,0,0,0},{0,0,0,0,0},{0,0,0,0,0},{0,0,0,0,0},{0,0,0,0,0},{0,0,0,0,0},{0,0,0,0,0},{0,0,0,0,0},{0,0,0,0,0},{0,0,0,0,0},{0,0,0,0,0},{0,0,0,0,0},{0,0,0,0,0},{0,0,0,0,0},{0,0,0,0,0},{0,0,0,0,0},{0,0,0,0,0},{0,0,0,0,0},{0,0,0,0,0},{0,0,0,0,0},{0,0,0,0,0},{0,0,0,0,0},{0,0,0,0,0},{0,0,0,0,0},{0,0,0,0,0},{0,0,0,0,0},{0,0,0,0,0},{0,0,0,0,0},{0,0,0,0,0},{0,0,0,0,0},{0,0,0,0,0},{0,0,0,0,0},{0,0,0,0,0},{0,0,0,0,0},{0,0,0,0,0},{0,0,0,0,0}};  

  //printf("finding blobs \n");
  findblobs(blob_array,filter_array,camera_width,camera_height,2);
  
    for(int i=0;i<50;i++){
      if(blob_array[i].initialized ==0){/*printf("blob count: %d \n",i-1)*/;break;}
      wb_display_set_color(display,0xff0000);
      if(blob_array[i].width >0 && blob_array[i].height >0){
      wb_display_draw_rectangle(display,blob_array[i].corner_x,blob_array[i].corner_y,blob_array[i].width,blob_array[i].height);
      //printf("corner_x: %d, corner_y: %d, width: %d, height: %d,intialized: %d,blobs: %d \n",blob_array[i].corner_x,blob_array[i].corner_y,blob_array[i].width,blob_array[i].height,blob_array[i].initialized,i);
      char num[5];
      sprintf(num,"%d",blob_array[i].height);
      wb_display_draw_text(display,num,blob_array[i].corner_x+(blob_array[i].width/2),blob_array[i].corner_y+(blob_array[i].height/2));
    //wait(1);
    }
    }
    
    
    
  //printf("corner_x: %d, corner_y: %d, width: %d, height: %d,intialized: %d \n",blob_array[1].corner_x,blob_array[1].corner_y,blob_array[1].width,blob_array[1].height,blob_array[1].initialized);
    //printf("%d\n",i);
    }
    
  wb_robot_cleanup();

  return 0;
}
