#include <System.h>
#include <opencv2/core/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <iostream>
#include <vector>
#include <arpa/inet.h>
#include <unistd.h>
#include <fstream>  // for std::ofstream
#include <ctime>
#include <iomanip>  // for std::setw, std::setfill

using namespace std;

// Helper function to receive exactly n bytes
bool recv_all(int sock, char* buffer, int len) {
    int total = 0;
    int retries = 0;

    while (total < len) {
        int received = recv(sock, buffer + total, len - total, 0);

        if (received == 0 && total == 0 && retries < 1) {
            std::cerr << "[WARN] recv() returned 0 on first attempt — retrying once..." << std::endl;
            retries++;
            sleep(1); // brief pause to wait for client
            continue;
        }

        if (received <= 0) {
            std::cerr << "[ERROR] recv() returned " << received << " at byte " << total << " of " << len << std::endl;
            return false;
        }

        total += received;
    }
    return true;
}

void log_event(const std::string& msg) {
    std::ofstream log("/mnt/h/Documents/AirSimExperiments/Reactive_OpticalFlow2/slam_server_debug.log", std::ios::app);
    if (log.is_open()) {
        std::time_t t = std::time(nullptr);
        log << "[" << std::put_time(std::localtime(&t), "%F %T") << "] " << msg << std::endl;
        log.flush(); // Ensure log is written immediately
    }
}


int main(int argc, char **argv) {
    log_event("=== SLAM TCP Server launched ===");
    std::cout << "[BOOT] SLAM TCP Server launched" << std::endl;

    if (argc < 3) {
        cerr << "Usage: ./tcp_slam_server path_to_vocabulary path_to_settings" << endl;
        return 1;
    }

    string vocab = argv[1];
    string settings = argv[2];

    // Initialize ORB-SLAM2
    ORB_SLAM2::System SLAM(vocab, settings, ORB_SLAM2::System::RGBD, true);
    cout << "[INFO] SLAM system initialized." << endl;

    // Setup TCP server
    int server_fd = socket(AF_INET, SOCK_STREAM, 0);
    if (server_fd == -1) {
        cerr << "Socket creation failed" << endl;
        return 1;
    }

    int opt = 1;
    setsockopt(server_fd, SOL_SOCKET, SO_REUSEADDR | SO_REUSEPORT, &opt, sizeof(opt));

    sockaddr_in address;
    address.sin_family = AF_INET;
    inet_pton(AF_INET, "172.23.31.187", &address.sin_addr);  // use your WSL IP here
    address.sin_port = htons(6000);

    if (bind(server_fd, (struct sockaddr *)&address, sizeof(address)) < 0) { // Bind the socket to the address
        cerr << "Bind failed" << endl;
        close(server_fd);
        return 1;
    }

    if (listen(server_fd, 1) < 0) { // Listen for incoming connections
        cerr << "Listen failed" << endl;
        close(server_fd);
        return 1;
    }

    cout << "[INFO] Waiting for Python streamer on port 6000..." << endl;

    int addrlen = sizeof(address);

    log_event("Calling accept() and waiting for streamer...");
    std::cout << "[INFO] Calling accept() and waiting for streamer..." << std::endl;
    std::cout.flush();

    int sock = accept(server_fd, (struct sockaddr *)&address, (socklen_t*)&addrlen);
    if (sock < 0) {
        log_event("[ERROR] Failed to accept() client connection.");
        cerr << "Accept failed" << endl;
        close(server_fd);
        return 1;
    }
    log_event("✅ Client connection accepted");
    std::cout << "[BOOT] Client connection accepted" << std::endl;
    std::cout.flush();


    if (sock < 0) {
        cerr << "Accept failed" << endl;
        close(server_fd);
        return 1;
    }
    cout << "[INFO] Connected to Python streamer!" << endl;

    bool slam_ready_flag_written = false;
    bool clean_exit = true;
    int frame_counter = 0; // Add frame counter

    while (true) {
        log_event("----- Begin image receive loop -----");

        // Log frame counter and timestamp at start of each loop
        double loop_timestamp = (double)cv::getTickCount() / cv::getTickFrequency();
        {
            std::ostringstream oss;
            oss << "Frame #" << frame_counter << " | Loop timestamp: " << std::fixed << std::setprecision(6) << loop_timestamp;
            log_event(oss.str());
        }

        // --- Receive RGB image ---
        char rgb_header[12];
        uint32_t net_height, net_width, net_bytes;
        uint32_t rgb_height, rgb_width, rgb_bytes;

        std::cout << "Reading RGB header (12 bytes)..." << std::endl;
        log_event("Waiting to receive 12-byte RGB header...");
        bool got_header = recv_all(sock, rgb_header, 12);
        if (!got_header) {
            std::cerr << "[ERROR] Failed to receive full 12-byte RGB header." << std::endl;
            log_event("Failed to receive full 12-byte RGB header. Closing connection (likely client disconnect or crash).");
            std::ofstream fail_flag("/mnt/h/Documents/AirSimExperiments/Reactive_OpticalFlow2/flags/slam_failed.flag");
            if (fail_flag.is_open()) fail_flag.close();
            clean_exit = false;
            break;
        }
        // Log the raw header bytes in hex
        std::ostringstream oss;
        oss << "Raw RGB header bytes: ";
        for (int i = 0; i < 12; ++i) {
            oss << std::hex << std::uppercase << std::setw(2) << std::setfill('0')
                << (static_cast<unsigned int>(static_cast<unsigned char>(rgb_header[i]))) << " ";
        }
        log_event(oss.str());
        log_event("Received 12-byte RGB header.");

        // Convert network byte order to host byte order
        memcpy(&net_height, rgb_header, 4);
        memcpy(&net_width,  rgb_header + 4, 4);
        memcpy(&net_bytes, rgb_header + 8, 4);
        // Convert from network byte order to host byte order
        rgb_height = ntohl(net_height);
        rgb_width  = ntohl(net_width);
        rgb_bytes  = ntohl(net_bytes);

        std::cout << "[DEBUG] RGB Header received: "
                << "height=" << rgb_height << ", width=" << rgb_width
                << ", bytes=" << rgb_bytes << std::endl;

        if (rgb_bytes != rgb_height * rgb_width * 3) {
            std::cerr << "[ERROR] RGB byte count mismatch. Expected: "
                      << (rgb_height * rgb_width * 3) << ", Got: " << rgb_bytes << std::endl;
            log_event("Disconnect: RGB byte count mismatch.");
            clean_exit = false;
            break;
        }

        cout << "[DEBUG] Expecting RGB " << rgb_height << "x" << rgb_width
         << " (" << rgb_height * rgb_width * 3 << " bytes), got: " << rgb_bytes << endl;


        // Ensure RGB image size matches expected dimensions
        vector<uchar> rgb_buffer(rgb_bytes);
        if (!recv_all(sock, (char*)rgb_buffer.data(), rgb_bytes)) {
            log_event("Disconnect: Failed to receive full RGB image data.");
            clean_exit = false;
            break;
        }

        // ✅ Touch slam_ready.flag to signal SLAM TCP server is ready (only once)
        log_event("About to create slam_ready.flag (pre-check passed)");
        if (!slam_ready_flag_written) {
            std::ofstream flag_file("/mnt/h/Documents/AirSimExperiments/Reactive_OpticalFlow2/flags/slam_ready.flag");
            if (flag_file.is_open()) {
            log_event("slam_ready.flag opened successfully for writing.");
                flag_file.close();
                std::cout << "[INFO] Created slam_ready.flag — SLAM server ready." << std::endl;
                log_event("Created slam_ready.flag — SLAM server ready.");
            } else {
                std::cerr << "[WARN] Could not create slam_ready.flag." << std::endl;
                log_event("Could not create slam_ready.flag.");
            }
            slam_ready_flag_written = true;
        }

        // Check if the received size matches the expected size
        cv::Mat imRGB(rgb_height, rgb_width, CV_8UC3, rgb_buffer.data());
        // Optionally clone to avoid data overwrite in next recv
        imRGB = imRGB.clone();

        // --- Receive Depth image ---
        uint32_t d_height, d_width, d_bytes;
        if (!recv_all(sock, (char*)&d_height, 4)) {
            log_event("Disconnect: Failed to receive depth height.");
            clean_exit = false;
            break;
        }
        if (!recv_all(sock, (char*)&d_width, 4)) {
            log_event("Disconnect: Failed to receive depth width.");
            clean_exit = false;
            break;
        }
        if (!recv_all(sock, (char*)&d_bytes, 4)) {
            log_event("Disconnect: Failed to receive depth bytes.");
            clean_exit = false;
            break;
        }

        // Convert network byte order to host byte order
        d_height = ntohl(d_height);
        d_width  = ntohl(d_width);
        d_bytes  = ntohl(d_bytes);
        cout << "[DEBUG] Expecting Depth " << d_height << "x" << d_width
         << " (" << d_height * d_width * 4 << " bytes), got: " << d_bytes << endl;

        // Log depth header parsing
        {
            std::ostringstream oss;
            oss << "Depth header: height=" << d_height << ", width=" << d_width << ", bytes=" << d_bytes;
            log_event(oss.str());
        }

        // Ensure depth image size matches expected dimensions
        if (d_bytes != d_height * d_width * 4) {
            std::ostringstream oss;
            oss << "Disconnect: Depth byte count mismatch. Expected: "
                << (d_height * d_width * 4) << ", Got: " << d_bytes;
            log_event(oss.str());
            clean_exit = false;
            break;
        }

        std::vector<char> raw_depth(d_bytes);
        if (!recv_all(sock, raw_depth.data(), d_bytes)) {
            log_event("Disconnect: Failed to receive full depth image data.");
            clean_exit = false;
            break;
        }

        // Convert to float*
        float* float_ptr = reinterpret_cast<float*>(raw_depth.data());
        cv::Mat imD(d_height, d_width, CV_32F, float_ptr);
        imD = imD.clone();  // clone to detach from raw_depth buffer

        // Normalize depth image to 16-bit unsigned integer
        double timestamp = (double)cv::getTickCount() / cv::getTickFrequency();

        // --- Process with SLAM ---
        cout << "[INFO] Processing new frame..." << endl;
        {
            std::ostringstream oss;
            oss << "Calling SLAM.TrackRGBD at timestamp=" << std::fixed << std::setprecision(6) << timestamp
                << " | Frame #" << frame_counter;
            log_event(oss.str());
        }
        try {
            SLAM.TrackRGBD(imRGB, imD, timestamp);
        } catch (const std::exception& ex) {
            std::ostringstream oss;
            oss << "Exception in SLAM.TrackRGBD: " << ex.what();
            log_event(oss.str());
            std::cerr << "[ERROR] Exception in SLAM.TrackRGBD: " << ex.what() << std::endl;
        } catch (...) {
            log_event("Unknown exception in SLAM.TrackRGBD.");
            std::cerr << "[ERROR] Unknown exception in SLAM.TrackRGBD." << std::endl;
        }

        frame_counter++; // Increment at end of each loop
    }

    if (clean_exit) {
        log_event("Image receive loop exited cleanly (no error).");
        std::cout << "[INFO] Image receive loop exited cleanly (no error)." << std::endl;
    } else {
        log_event("Image receive loop exited due to error or disconnect.");
        std::cout << "[INFO] Image receive loop exited due to error or disconnect." << std::endl;
    }
    log_event("Connection closed or error occurred. Shutting down main loop.");
    std::cout << "[INFO] Connection closed or error occurred. Shutting down." << std::endl;
    close(sock);
    close(server_fd);
    SLAM.Shutdown();
    return 0;
}
