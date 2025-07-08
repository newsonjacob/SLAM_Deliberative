#include <System.h>
#include <opencv2/core/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <unistd.h>  // for usleep and close
#include <netinet/in.h>
#include <arpa/inet.h>
#include <chrono>
#include <Tracking.h>  // Access tracker internals

using namespace std;

// Sends a single pose matrix over the existing socket connection
bool SendPose(int sockfd, const cv::Mat& Tcw) {
    // Flatten 3x4 pose matrix into 12 floats
    float data[12];
    for (int i = 0; i < 3; ++i) 
        for (int j = 0; j < 4; ++j) 
            data[i * 4 + j] = Tcw.at<float>(i, j);

    // DEBUG: print sent pose
    cout << "Sending pose data: ";
    for (int i = 0; i < 12; ++i)
        cout << data[i] << " ";
    cout << endl;

    ssize_t sent = send(sockfd, data, sizeof(data), 0);
    return sent == sizeof(data);
}

int main(int argc, char **argv) {
    string base_path = "/home/jacob/slam_ws/ORB_SLAM2_clean/app/";

    if (argc < 4 || argc > 5) {
        cerr << "Usage: ./custom_slam path_to_vocabulary path_to_settings associate.txt [receiver_ip]" << endl;
        return 1;
    }

    string vocab = argv[1];
    string settings = argv[2];
    string assoc_file = argv[3];
    string receiver_ip = (argc == 5) ? argv[4] : "172.23.16.1"; // default IP or pass as argument

    // DEBUG: start timer before vocabulary load
    cout << "[DEBUG] Starting vocabulary load..." << endl;
    auto start = chrono::steady_clock::now();

    ORB_SLAM2::System SLAM(vocab, settings, ORB_SLAM2::System::RGBD, true);

    auto end = chrono::steady_clock::now();
    cout << "[DEBUG] Vocabulary loaded in "
         << chrono::duration_cast<chrono::seconds>(end - start).count()
         << " seconds." << endl;

    // Load association file
    ifstream file(assoc_file);
    if (!file.is_open()) {
        cerr << "Failed to open associate file: " << assoc_file << endl;
        return 1;
    }

    vector<string> rgb_files, depth_files;
    vector<double> timestamps;

    string line;
    while (getline(file, line)) {
        if (line.empty()) continue;
        stringstream ss(line);
        double t_rgb, t_d;
        string rgb, depth;
        ss >> t_rgb >> rgb >> t_d >> depth;
        timestamps.push_back(t_rgb);
        rgb_files.push_back(rgb);
        depth_files.push_back(depth);
    }

    cout << "Processing " << rgb_files.size() << " frames..." << endl;

    // --- Open socket once before sending all frames ---
    int sockfd = socket(AF_INET, SOCK_STREAM, 0);
    if (sockfd < 0) {
        cerr << "Socket creation failed" << endl;
        return 1;
    }

    sockaddr_in serv_addr{};
    serv_addr.sin_family = AF_INET;
    serv_addr.sin_port = htons(5005);
    if (inet_pton(AF_INET, receiver_ip.c_str(), &serv_addr.sin_addr) <= 0) {
        cerr << "Invalid address/ Address not supported: " << receiver_ip << endl;
        close(sockfd);
        return 1;
    }

    if (connect(sockfd, (struct sockaddr*)&serv_addr, sizeof(serv_addr)) < 0) {
        cerr << "Connection to Python receiver failed at IP: " << receiver_ip << endl;
        close(sockfd);
        return 1;
    }
    cout << "[DEBUG] Connected to Python receiver at " << receiver_ip << ", starting pose transmission..." << endl;

    // Process each frame
    for (size_t i = 0; i < rgb_files.size(); ++i) {
        string rgb_path = base_path + rgb_files[i];
        string depth_path = base_path + depth_files[i];

        cout << "[DEBUG] Frame " << i << " timestamps - RGB: " << timestamps[i] << endl;
        cout << "[DEBUG] Loading RGB image from: " << rgb_path << endl;
        cout << "[DEBUG] Loading Depth image from: " << depth_path << endl;

        cv::Mat imRGB = cv::imread(rgb_path, cv::IMREAD_UNCHANGED);
        cv::Mat imD = cv::imread(depth_path, cv::IMREAD_UNCHANGED);

        if (imRGB.empty() || imD.empty()) {
            cerr << "[Warning] Skipping frame " << i << " due to failed image load." << endl;
            continue;
        }

        SLAM.TrackRGBD(imRGB, imD, timestamps[i]);
        int state = SLAM.GetTrackingState();
        cout << "[DEBUG] Tracking state: " << state << endl;

        cv::Mat Tcw = SLAM.GetTracker()->mCurrentFrame.mTcw; // Access current frame's pose
        // ORB-SLAM2 tracking states: 0=NOT_INITIALIZED, 1=TRACKING, 2=LOST (verify in your version)
        const int TRACKING_STATE = 1;  // likely 'tracking' state in your code
        if (state == TRACKING_STATE && !Tcw.empty()) {
            float tx = Tcw.at<float>(0, 3);
            float ty = Tcw.at<float>(1, 3);
            float tz = Tcw.at<float>(2, 3);
            cout << "Sending translation: x=" << tx << ", y=" << ty << ", z=" << tz << endl;
            cout << "Sending pose for frame " << i << "..." << endl;

            if (!SendPose(sockfd, Tcw)) { // Send pose data
                cerr << "Failed to send pose data at frame " << i << endl;
                break;  // Optionally break on send failure
            }
        } else {
            cout << "[DEBUG] Tracking lost or empty pose at frame " << i << endl;
        }

        usleep(100000);  // ~10 FPS
    }

    if (close(sockfd) < 0) {
        cerr << "Error closing socket" << endl;
    }

    SLAM.Shutdown();

    return 0;
}
