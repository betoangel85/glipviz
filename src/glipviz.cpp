#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
// #include <uv_msgs/ImageContour.h>
// #include <uv_msgs/ImageBoundingBox.h>
#include <uv_msgs/ImageBoundingBoxListStamped.h>
#include <uv_msgs/ImageContourListStamped.h>
#include <uv_msgs/ImagePointListStamped.h>
#include <cv_bridge/cv_bridge.h>
#include <iostream>
#include <stdio.h>
#include <GL/freeglut.h>
#include <glip.h>

#define default_ID "camera"
#define defaultImageTopic "/camera/rgb/image_rect_color"
// #define defaultBBoxesTopic "/glipviz/boundingBoxes"
// #define defaultContoursTopic "/glipviz/Contours"

#define maxBoxes 8
#define maxContourPts 16000
#define maxPts 16000
#define defaultWidth 640
#define defaultHeight 480


namespace enc = sensor_msgs::image_encodings;

ros::Subscriber sub,sub2,sub3,sub4;

std::string imageTopic,bBoxesTopic,contoursTopic,pointsTopic;

int bBoxData[128],contourData[64000],ptsData[32000];

glipImageSt *imageGlip;
glipDataSt * BBox, * Contour, * Points;
int window;
unsigned char *imageData;
double msgBBoxTime=0.0,currentTime,deltaBBoxTime; 
double msgContourTime=0.0,deltaContourTime; 
double msgPointsTime=0.0,deltaPointsTime; 
double inactivityTime=0.3;

void pointsCallback(const uv_msgs::ImagePointListStamped& ptsmsg)
{
  int nPts=ptsmsg.NoPoints;
  int i,j,contourPts=0,i4,j4_2,cSize;
  
  msgPointsTime = ros::Time::now().toSec();

  for (i=0;i<nPts;i++){
    ptsData[i*2]=ptsmsg.points[i].u; 
    ptsData[i*2+1]=ptsmsg.points[i].v;
  }

  Points->NoPts=nPts;  
  glipRedisplayImage(window);

}

void contoursCallback(const uv_msgs::ImageContourListStamped& contours)
{
  int nCont=contours.NoContours;
  int i,j,contourPts=0,i4,j4_2,cSize;

  msgContourTime = ros::Time::now().toSec();

  for (i=0;i<nCont;i++){
    contourPts+=contours.contours[i].points.size()/2;
    if (contourPts>maxContourPts) { 
      nCont=i-1;
      break;
    }
  }

  for (i=0;i<nCont;i++){
    i4=i*4;
    //first point in contour [i]
    contourData[i4]=contours.contours[i].points[0].u; 
    contourData[i4+1]=contours.contours[i].points[0].v;
    cSize=contours.contours[i].points.size()/2.0; //Contour[i] size

    for (j=1;j<cSize;j++){
      j4_2=j*4-2;
      contourData[i4+j4_2]=contours.contours[i].points[j].u;
      contourData[i4+j4_2+1]=contours.contours[i].points[j].v;
      contourData[i4+j4_2+2]=contours.contours[i].points[j].u;
      contourData[i4+j4_2+3]=contours.contours[i].points[j].v;
    }
    contourData[i4+j4_2+4]=contours.contours[i].points[j].u;
    contourData[i4+j4_2+5]=contours.contours[i].points[j].v;
  }
  Contour->NoPts=contourPts;  
  glipRedisplayImage(window);
 

}


void boxesCallback(const uv_msgs::ImageBoundingBoxListStamped& bBoxes)
{
  int nBoxes=bBoxes.NoBoxes;
  if (nBoxes>maxBoxes) nBoxes=maxBoxes;

  msgBBoxTime = ros::Time::now().toSec();

  for (int i=0;i<nBoxes;i++){
    bBoxData[i*16+0]= bBoxes.boxes[i].cornerPoints[0].u;     
    bBoxData[i*16+1]= bBoxes.boxes[i].cornerPoints[0].v;  
    bBoxData[i*16+2]= bBoxes.boxes[i].cornerPoints[1].u;     
    bBoxData[i*16+3]= bBoxes.boxes[i].cornerPoints[1].v; 
    bBoxData[i*16+4]= bBoxes.boxes[i].cornerPoints[1].u;     
    bBoxData[i*16+5]= bBoxes.boxes[i].cornerPoints[1].v; 
    bBoxData[i*16+6]= bBoxes.boxes[i].cornerPoints[2].u;     
    bBoxData[i*16+7]= bBoxes.boxes[i].cornerPoints[2].v; 
    bBoxData[i*16+8]= bBoxes.boxes[i].cornerPoints[2].u;     
    bBoxData[i*16+9]= bBoxes.boxes[i].cornerPoints[2].v; 
    bBoxData[i*16+10]= bBoxes.boxes[i].cornerPoints[3].u;     
    bBoxData[i*16+11]= bBoxes.boxes[i].cornerPoints[3].v; 
    bBoxData[i*16+12]= bBoxes.boxes[i].cornerPoints[3].u;     
    bBoxData[i*16+13]= bBoxes.boxes[i].cornerPoints[3].v; 
    bBoxData[i*16+14]= bBoxes.boxes[i].cornerPoints[0].u;     
    bBoxData[i*16+15]= bBoxes.boxes[i].cornerPoints[0].v; 
  }
  BBox->NoPts=nBoxes*8;  
  glipRedisplayImage(window);

}

int convertCVPTRDataType2GLIP(int type)
{
  int imgDataType;

  switch (type) {
  case 0: imgDataType=5121;
   break;
  case 1: imgDataType=5120;
   break;
  case 2: imgDataType=5123;
   break;
  case 3: imgDataType=5122;
   break;
  case 4: imgDataType=5125;
   break;
  case 5: imgDataType=5126;
   break;
  case 6: imgDataType=5127;
   break;
  default:
    break;
 }
  return imgDataType;
}


void glipDisplay(const sensor_msgs::ImageConstPtr& msg)
{
  cv_bridge::CvImageConstPtr cv_ptr;
  int imgFormat,imgDataType;

  try { 
    cv_ptr = cv_bridge::toCvShare(msg,enc::RGB8);
  } catch (cv_bridge::Exception& e) {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }

  imageGlip->data=cv_ptr->image.data;
  // imageFormat=

  if (imageGlip->width!=cv_ptr->image.cols) imageGlip->width=cv_ptr->image.cols;
  if (imageGlip->height!=cv_ptr->image.rows) imageGlip->height=cv_ptr->image.rows;
  // if (imageGlip->
  // imgDataType=convertCVPTRDataType2GLIP(cv_ptr->image.depth());
  // printf("dims = %d %d %d\n",cv_ptr->image.depth(),imageGlip->type,imgDataType);
  // if (imageGlip->type!=imgDataType) imageGlip->type=imgDataType;

  glipRedisplayImage(window);
  glutMainLoopEvent();

}

int getParams(ros::NodeHandle nh)
{
  int params=0;

  nh.getParam("glipviz/image",imageTopic);
  if (imageTopic.size()==0)  imageTopic=defaultImageTopic;
  else params++;
  sub = nh.subscribe(imageTopic, 1, glipDisplay);

  nh.getParam("glipviz/boxes",bBoxesTopic);
  if (bBoxesTopic.size()!=0) {
    sub2 = nh.subscribe(bBoxesTopic, 1, boxesCallback);
  //bBoxesTopic=defaultBBoxesTopic;
    params++;
  }

  nh.getParam("glipviz/contours",contoursTopic);
  if (contoursTopic.size()!=0) {
    sub3 = nh.subscribe(contoursTopic, 1, contoursCallback);
    //contoursTopic=defaultContoursTopic;
    params++;
  }

  nh.getParam("glipviz/points",pointsTopic);
  if (contoursTopic.size()!=0) {
    sub4 = nh.subscribe(pointsTopic, 1, pointsCallback);
    params++;
  }

  return params;
}



int main(int argc, char **argv)
{
  ros::init(argc, argv, "glipviz");  
  ros::NodeHandle _nh;
  //  std::string imageTopic,bBoxesTopic,contoursTopic;

  int width=defaultWidth;
  int height=defaultHeight;

  getParams(_nh);

  glutInitWindowSize(width,height);
  glutInit(&argc,argv);
  glutInitDisplayMode (GLUT_RGB);

  imageGlip=(glipImageSt*)glipCreateImage(width,height,1,GLIP_RGB,GL_UNSIGNED_BYTE);
  imageGlip->signY=-1;
  //  imageData=(unsigned char*)malloc(width*height*3*sizeof(char));
  //  imageGlip->data=imageData;
  window=glipDisplayImage(imageGlip,"glip Visualization",0);
  BBox=(glipDataSt*)glipCreateDataSt(0,2,2,GLIP_COLOR_GREEN,GLIP_LINES,
				     GLIP_INT,bBoxData); 
  glipDrawInImage(window,BBox);

  Contour=(glipDataSt*)glipCreateDataSt(0,2,2,GLIP_COLOR_RED,GLIP_LINES,
					GLIP_INT,contourData);
  glipDrawInImage(window,Contour);

  Points=(glipDataSt*)glipCreateDataSt(0,2,2,GLIP_COLOR_BLUE,GLIP_POINTS,
					GLIP_INT,ptsData);
  glipDrawInImage(window,Points);


  while (ros::ok()) { 
    ros::spinOnce();

    currentTime = ros::Time::now().toSec();

    deltaBBoxTime= currentTime-msgBBoxTime;
    if (deltaBBoxTime>inactivityTime) BBox->NoPts=0;        
    
    deltaContourTime= currentTime-msgContourTime;
    if (deltaContourTime>inactivityTime) Contour->NoPts=0;      

    deltaPointsTime= currentTime-msgPointsTime;
    if (deltaPointsTime>inactivityTime) Points->NoPts=0;        

  }
  return 0;
}
