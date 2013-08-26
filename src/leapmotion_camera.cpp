/*
 ** Authors: Elina Lijouvni, Andrew J. Chen
 ** License: GPL v3
 */

#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>

#include <cv.h>
#include <highgui.h>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

namespace enc = sensor_msgs::image_encodings;

static const char WINDOW[] = "Image window";

#define DISPLAY 1

#define VFRAME_WIDTH  640
#define VFRAME_HEIGHT 120
#define VFRAME_SIZE   (VFRAME_WIDTH * VFRAME_HEIGHT)

typedef struct frame_s frame_t;
struct frame_s
{
  IplImage* frame;
  uint32_t id;
  uint32_t data_len;
  uint32_t state;
};

#define UVC_STREAM_EOF                                  (1 << 1)

class LeapPublisher
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Publisher left_image_pub_;
  image_transport::Publisher right_image_pub_;
  ros::Rate r_;
  int seq;
  
public:
  LeapPublisher()
    : it_(nh_), r_(100)
  {
    left_image_pub_ = it_.advertise("left", 1);
    right_image_pub_ = it_.advertise("right", 1);
#if DISPLAY    
    cv::namedWindow(WINDOW);
#endif    
  }

  ~LeapPublisher()
  {
#if DISPLAY    
    cv::destroyWindow(WINDOW);
#endif    
  }

  void process_video_frame(frame_t *frame)
  {
    int key;
    cv::Mat image(frame->frame);

    cv::Mat channels[3];
    cv::split(image, channels);

    ros::Time now = ros::Time::now();

    cv_bridge::CvImage left_out_msg;
    left_out_msg.header.seq = seq;
    left_out_msg.header.stamp = now;
    left_out_msg.header.frame_id = "left";
    left_out_msg.encoding = sensor_msgs::image_encodings::MONO8;
    left_out_msg.image = channels[1];
    left_image_pub_.publish(left_out_msg.toImageMsg());

    cv_bridge::CvImage right_out_msg;
    // right_out_msg.header = TODO
    right_out_msg.header.seq = seq;
    right_out_msg.header.stamp = now;
    right_out_msg.header.frame_id = "right";
    right_out_msg.encoding = sensor_msgs::image_encodings::MONO8;
    right_out_msg.image = channels[2];
    right_image_pub_.publish(right_out_msg.toImageMsg());

    seq++;
#if DISPLAY
    cv::imshow(WINDOW, image);
#endif    
    r_.sleep();
    key = cvWaitKey(1);
  }

  void process_usb_frame(frame_t *frame, unsigned char *data, int size)
  {
    int i;

    int bHeaderLen = data[0];
    int bmHeaderInfo = data[1];

    uint32_t dwPresentationTime = *( (uint32_t *) &data[2] );
    // printf("frame time: %u\n", dwPresentationTime);

    if (frame->id == 0)
      frame->id = dwPresentationTime;

    for (i = bHeaderLen; i < size ; i += 2) {
      if (frame->data_len >= VFRAME_SIZE)
        break ;

      CvScalar s;
      s.val[2] = data[i];
      s.val[1] = data[i+1];
      s.val[0] = 0;
      int x = frame->data_len % VFRAME_WIDTH;
      int y = frame->data_len / VFRAME_WIDTH;
      cvSet2D(frame->frame, 2 * y,     x, s);
      cvSet2D(frame->frame, 2 * y + 1, x, s);
      frame->data_len++;
    }

    if (bmHeaderInfo & UVC_STREAM_EOF) {
      // printf("End-of-Frame.  Got %i\n", frame->data_len);

      if (frame->data_len != VFRAME_SIZE) {
        // printf("wrong frame size got %i expected %i\n", frame->data_len, VFRAME_SIZE);
        frame->data_len = 0;
        frame->id = 0;
        return ;
      }

      process_video_frame(frame);
      frame->data_len = 0;
      frame->id = 0;
    }
    else {
      if (dwPresentationTime != frame->id && frame->id > 0) {
        // printf("mixed frame TS: dropping frame\n");
        frame->id = dwPresentationTime;
      /* frame->data_len = 0; */
      /* frame->id = 0; */
      /* return ; */
      }
    }
  }
};

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "leap_bridge");

  LeapPublisher lp;

  frame_t frame;
  memset(&frame, 0, sizeof (frame));
  frame.frame = cvCreateImage( cvSize(VFRAME_WIDTH, 2 * VFRAME_HEIGHT), IPL_DEPTH_8U, 3);

  while (ros::ok()) {
    unsigned char data[16384];
    int usb_frame_size;

    if ( feof(stdin))
      break ;

    fread(&usb_frame_size, sizeof (usb_frame_size), 1, stdin);
    fread(data, usb_frame_size, 1, stdin);

    lp.process_usb_frame(&frame, data, usb_frame_size);
    ros::spinOnce();
  }

  return (0);
}
