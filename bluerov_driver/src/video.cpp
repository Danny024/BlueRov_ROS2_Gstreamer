/**
 * Name : Daniel Eneh
 * Date : 29-10-2024
 * Email : danieleneh024@gmail.com
 * 
 * 
 */
#include <iostream>
#include <opencv2/opencv.hpp>
#include <gst/gst.h>
#include <gst/app/gstappsink.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <cv_bridge/cv_bridge.h>

using namespace std::chrono_literals;

class Video {
public:
    Video(int port) : port(port), frame_available(false) {
        std::cout << "[INFO] Initializing GStreamer..." << std::endl;
        gst_init(nullptr, nullptr);
        run();
    }

    cv::Mat getFrame() const { return frame; }
    bool isFrameAvailable() const { return frame_available; }
    int getPort() const { return port; }  // Add a getter function for port

private:
    int port;
    cv::Mat frame;
    bool frame_available;
    GstElement *video_pipe = nullptr;
    GstElement *video_sink = nullptr;

    void run() {
        std::cout << "[INFO] Setting up GStreamer pipeline..." << std::endl;
        startGst({
            "udpsrc port=" + std::to_string(port),
            "! application/x-rtp, payload=96 ! rtph264depay ! h264parse ! avdec_h264",
            "! decodebin ! videoconvert ! video/x-raw,format=(string)BGR ! videoconvert",
            "! appsink emit-signals=true sync=false max-buffers=2 drop=true name=appsink0"
        });
        
        if (video_sink) {
            g_signal_connect(video_sink, "new-sample", G_CALLBACK(callback), this);
            std::cout << "[INFO] Connected to new-sample signal." << std::endl;
        } else {
            std::cerr << "[ERROR] Failed to get video sink element." << std::endl;
        }
    }

    void startGst(const std::vector<std::string> &config) {
        std::string command = join(config, " ");
        video_pipe = gst_parse_launch(command.c_str(), nullptr);
        if (!video_pipe) {
            std::cerr << "[ERROR] Failed to create GStreamer pipeline." << std::endl;
            return;
        }

        gst_element_set_state(video_pipe, GST_STATE_PLAYING);
        std::cout << "[INFO] GStreamer pipeline set to PLAYING state." << std::endl;

        video_sink = gst_bin_get_by_name(GST_BIN(video_pipe), "appsink0");
    }

    static cv::Mat gstToOpenCV(GstSample *sample) {
        GstBuffer *buffer = gst_sample_get_buffer(sample);
        GstCaps *caps = gst_sample_get_caps(sample);
        GstStructure *capsStruct = gst_caps_get_structure(caps, 0);

        int width, height;
        gst_structure_get_int(capsStruct, "width", &width);
        gst_structure_get_int(capsStruct, "height", &height);
        std::cout << "[INFO] Frame width: " << width << ", height: " << height << std::endl;

        GstMapInfo map;
        gst_buffer_map(buffer, &map, GST_MAP_READ);

        cv::Mat img(height, width, CV_8UC3, map.data);
        gst_buffer_unmap(buffer, &map);

        std::cout << "[DEBUG] Frame captured and converted to OpenCV format." << std::endl;
        return img.clone();  
    }

    static GstFlowReturn callback(GstElement *sink, gpointer user_data) {
        Video *self = static_cast<Video*>(user_data);
        GstSample *sample = gst_app_sink_pull_sample(GST_APP_SINK(sink));
        if (!sample) {
            std::cerr << "[ERROR] Failed to pull sample from GStreamer sink." << std::endl;
            return GST_FLOW_ERROR;
        }

        self->frame = gstToOpenCV(sample);
        gst_sample_unref(sample);
        self->frame_available = true;

        std::cout << "[DEBUG] Frame available for processing." << std::endl;
        return GST_FLOW_OK;
    }

    static std::string join(const std::vector<std::string> &elements, const std::string &delimiter) {
        std::ostringstream os;
        for (auto it = elements.begin(); it != elements.end(); ++it) {
            if (it != elements.begin()) os << delimiter;
            os << *it;
        }
        return os.str();
    }
};

class BluerovVideo : public rclcpp::Node, public Video {
public:
    BluerovVideo() 
    : Node("bluerov2_video"), 
      Video(declare_parameter("port", 5600)),  // Using parameter for port
      topic(declare_parameter("topic", "/bluerov2/camera/compressed"))  // Using parameter for topic
    {
        pub_img = this->create_publisher<sensor_msgs::msg::CompressedImage>(topic, 1);
        timer = this->create_wall_timer(0.5ms, std::bind(&BluerovVideo::looper, this));
        std::cout << "[INFO] BluerovVideo node initialized with topic: " << topic << " and port: " << getPort() << std::endl;
    }

private:
    std::string topic;
    rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr pub_img;
    rclcpp::TimerBase::SharedPtr timer;

    void looper() {
        if (!isFrameAvailable()) {
            std::cout << "[DEBUG] No frame available yet." << std::endl;
            return;
        }

        auto frame = getFrame();
        std::cout << "[DEBUG] Frame received for publishing." << std::endl;

        // Resize and prepare the frame for publishing
        cv::resize(frame, frame, cv::Size(), 1.0, 1.0);

        // Create and populate a CompressedImage message
        sensor_msgs::msg::CompressedImage msg;
        msg = *cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", frame).toCompressedImageMsg();
        std::cout << "[DEBUG] Frame converted to CompressedImage message." << std::endl;

        // Publish the message
        pub_img->publish(msg);
        std::cout << "[INFO] Published frame to topic: " << topic << std::endl;

        // Display frame with OpenCV
        cv::imshow("frame", frame);
        if (cv::waitKey(1) == 'q') {
            std::cout << "[INFO] Quitting due to 'q' key press." << std::endl;
            rclcpp::shutdown();
        }
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<BluerovVideo>();
    std::cout << "[INFO] Bluerov2 video node started" << std::endl;
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
