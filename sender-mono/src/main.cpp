///////////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2024, STEREOLABS.
//
// All rights reserved.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
// OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
// LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
// DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
// THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
///////////////////////////////////////////////////////////////////////////

// OpenGL Viewer(!=0) or no viewer (0)
#define DISPLAY_OGL 1

// ZED include
#include "GLViewer.hpp"
#include "PracticalSocket.h"
#include "json.hpp"
#include "Util.h"
#include "opencv/cv.hpp"
#include <sl/Camera.hpp>

using namespace sl;

nlohmann::json toJSON(int frame_id, int serial_number, sl::Timestamp timestamp, sl::Pose& cam_pose, sl::COORDINATE_SYSTEM coord_sys, sl::UNIT coord_unit);
nlohmann::json toJSON(int frame_id, sl::Timestamp timestamp, sl::Bodies& bodies, int id, sl::BODY_FORMAT body_format, sl::COORDINATE_SYSTEM coord_sys, sl::UNIT coord_unit);

void print(string msg_prefix, ERROR_CODE err_code = ERROR_CODE::SUCCESS, string msg_suffix = "");

bool visual_debug = false;
bool apply_mask = true;
int x_offset = 0;

// Type of data send 
enum class ZEDLiveLinkRole
{
    Transform = 0,
    Camera,
    Animation
};

// Defines the Coordinate system and unit used in this sample
static const sl::COORDINATE_SYSTEM coord_sys = sl::COORDINATE_SYSTEM::RIGHT_HANDED_Y_UP;
static const sl::UNIT coord_unit = sl::UNIT::MILLIMETER;

/// ----------------------------------------------------------------------------
/// ----------------------------------------------------------------------------
/// -------------------------------- MAIN LOOP ---------------------------------
/// ----------------------------------------------------------------------------
/// ----------------------------------------------------------------------------

int main(int argc, char **argv) {

    ZEDConfig zed_config;
    std::string zed_config_file("ZEDLiveLinkConfig.json"); // Default name and location.
    if (argc == 2)
    {
        zed_config_file = argv[1];
        std::cout << "Loading " << zed_config_file << " config file.";
    }
    else if (argc == 4) {
        apply_mask = std::stoi(argv[1]);
        visual_debug = std::stoi(argv[2]);
        x_offset= std::stoi(argv[3]);
    }
    else if ((argc >  4))
    {
        std::cout << "Unexecpected arguments, exiting..." << std::endl;
        return EXIT_FAILURE;
    }
    else {
        std::cout << "Trying to load default config file 'ZEDLiveLinkConfig.json' " << std::endl;
    }
    readZEDConfig(zed_config_file, zed_config);
    std::cout << "Starting LiveLink sender" << endl;

    Camera zed;
    InitParameters init_parameters;
    init_parameters.camera_resolution = zed_config.resolution;
    init_parameters.camera_fps = zed_config.fps;
    init_parameters.depth_mode = zed_config.depth_mode;
    init_parameters.coordinate_system = coord_sys;
    init_parameters.coordinate_units = coord_unit;
    init_parameters.grab_compute_capping_fps = zed_config.grab_compute_capping_fps;

    init_parameters.input = zed_config.input;
    init_parameters.svo_real_time_mode = true;

    // Open the camera
    auto returned_state = zed.open(init_parameters);
    if (returned_state != ERROR_CODE::SUCCESS) {
        print("Open Camera", returned_state, "\nExit program.");
        zed.close();
        return EXIT_FAILURE;
    }

    /**/
    auto camera_config = zed.getCameraInformation().camera_configuration;

    cv::Mat ROI = cv::Mat(camera_config.resolution.height, camera_config.resolution.width, CV_8UC1, cv::Scalar(0));
    ROI.setTo(0);
    
    if (apply_mask) {
        ROI = cv::imread("mask_center.png", cv::IMREAD_GRAYSCALE);
        cv::resize(ROI, ROI, cv::Size(camera_config.resolution.width, camera_config.resolution.height), 0, 0, cv::INTER_LINEAR);
    }
    // Attempt at ROI with zed
    /*cv::Rect selection_rect;
    selection_rect.x = 0;
    selection_rect.y = 0;
    selection_rect.width = 600;
    selection_rect.height = 720;
    cv::rectangle(ROI, selection_rect, cv::Scalar(255), -1);
    sl::Mat mask(camera_config.resolution, sl::MAT_TYPE::U8_C4);
    auto rect = cv::Rect(400, 50, camera_config.resolution.width / 3.5, camera_config.resolution.height - 50);
    cv::Mat cvImage(camera_config.resolution.height, camera_config.resolution.width, CV_8UC4, mask.getPtr<sl::uchar1>(sl::MEM::CPU));
    cv::rectangle(cvImage, rect, cv::Scalar(255, 255, 255, 255), -1);
    sl::ERROR_CODE roi_status = zed.setRegionOfInterest(mask);
    if (roi_status != sl::ERROR_CODE::SUCCESS) {
        std::cerr << "ROI setup failed: " << sl::toString(roi_status) << std::endl;
        return EXIT_FAILURE;
    }
    */
    // Enable Positional tracking (mandatory for body tracking) -------------------------------------------------------
    PositionalTrackingParameters positional_tracking_parameters;
    positional_tracking_parameters.set_floor_as_origin = zed_config.set_floor_as_origin;
    // If the camera is static, uncomment the following line to have better performance.
    positional_tracking_parameters.set_as_static = zed_config.set_as_static;
    positional_tracking_parameters.enable_pose_smoothing = zed_config.enable_pose_smoothing;
    positional_tracking_parameters.enable_area_memory = zed_config.enable_area_memory;
   

    returned_state = zed.enablePositionalTracking(positional_tracking_parameters);
    if (returned_state != ERROR_CODE::SUCCESS) {
        print("enable Positional Tracking", returned_state, "\nExit program.");
        zed.close();
        return EXIT_FAILURE;
    }


    BodyTrackingParameters body_tracking_params;
    if (zed_config.send_bodies)
    {
        // Enable the Body tracking module -------------------------------------------------------------------------------------

        body_tracking_params.enable_tracking = true; // track people across grabs
        body_tracking_params.enable_body_fitting = true; // smooth skeletons moves
        body_tracking_params.body_format = zed_config.body_format;
        body_tracking_params.detection_model = zed_config.detection_model;
        body_tracking_params.max_range = zed_config.max_range;
        returned_state = zed.enableBodyTracking(body_tracking_params);
        if (returned_state != ERROR_CODE::SUCCESS) {
            print("enable Body Tracking", returned_state, "\nExit program.");
            zed.close();
            return EXIT_FAILURE;
        }

    }

#if DISPLAY_OGL
    GLViewer viewer;
    viewer.init(argc, argv);
#endif

    Pose cam_pose;
    cam_pose.pose_data.setIdentity();

    // Configure body tracking runtime parameters
    BodyTrackingRuntimeParameters body_tracking_parameters_rt;
    body_tracking_parameters_rt.detection_confidence_threshold = zed_config.detection_confidence;
    body_tracking_parameters_rt.minimum_keypoints_threshold = zed_config.minimum_keypoints_threshold;
    body_tracking_parameters_rt.skeleton_smoothing = zed_config.skeleton_smoothing;

    // Create ZED Bodies filled in the main loop
    Bodies bodies;

    bool run = true;

    // ----------------------------------
    // UDP ------------------------------
    // ----------------------------------
    std::string servAddress;
    unsigned short servPort;
    UDPSocket sock;

    if (zed_config.connection_type == CONNECTION_TYPE::MULTICAST) sock.setMulticastTTL(1);

    servAddress = zed_config.udp_ip;
    servPort = zed_config.udp_port;

    std::cout << "Sending data at " << servAddress << ":" << servPort << std::endl;

    // ----------------------------------
    // UDP ------------------------------
    // ----------------------------------

    RuntimeParameters rt_params = new RuntimeParameters();
    rt_params.measure3D_reference_frame = REFERENCE_FRAME::WORLD;
    int frame_id = 0;

    //sl::Mat zed_image(resolution, MAT_TYPE::U8_C4);
    //cv::Mat cvImage(resolution.height, resolution.width, CV_8UC4, zed_image.getPtr<sl::uchar1>(MEM::CPU));

        // BODY_38 skeleton bone connections
        const std::vector<std::pair<int, int>> BODY_38_BONES = {

            // Spine
            {0, 1}, {1, 2}, {2, 3}, {3, 4}, // Pelvis to Neck
            {4, 5}, // Neck to Nose
            
            // Face
            {5, 6}, {5, 7}, // Nose to Eyes
            {6, 8}, {7, 9}, // Eyes to Ears
            
            // Shoulders to elbows
            {10, 12}, {12, 14}, {14, 16}, // Left arm
            {11, 13}, {13, 15}, {15, 17}, // Right arm

            // Clavicles
            {4, 10}, {4, 11}, // Neck to left/right clavicle

            // Hips to knees to ankles
            {0, 18}, {0, 19}, // Pelvis to left/right hip
            {18, 20}, {20, 22}, // Left leg
            {19, 21}, {21, 23}, // Right leg

            // Feet (toes and heels)
            {22, 24}, {24, 26}, {22, 28}, // Left foot
            {23, 25}, {25, 27}, {23, 29}, // Right foot

            // Left hand fingers
            {16, 30}, {16, 32}, {16, 34}, {16, 36},

            // Right hand fingers
            {17, 31}, {17, 33}, {17, 35}, {17, 37}
        };


    SetCtrlHandler();
    while (!exit_app)
    {
        auto start = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
        auto err = zed.grab(rt_params);

        if (err == ERROR_CODE::SUCCESS)
        {
            sl::Timestamp ts = zed.getTimestamp(sl::TIME_REFERENCE::IMAGE);
            if (zed_config.send_bodies)
            {
                // Retrieve Detected Human Bodies
                zed.retrieveBodies(bodies, body_tracking_parameters_rt);

                sl::Mat zed_image(camera_config.resolution, MAT_TYPE::U8_C4);
                zed.retrieveImage(zed_image, VIEW::LEFT);
                cv::Mat cvImage(camera_config.resolution.height, camera_config.resolution.width, CV_8UC4, zed_image.getPtr<sl::uchar1>(MEM::CPU));

                cv::Mat gray_rgba;
                cv::cvtColor(ROI, gray_rgba, cv::COLOR_GRAY2BGRA);
                cv::Mat blended;
                cv::addWeighted(cvImage, 1.0, gray_rgba, 0.5, 0.0, blended);
                // Find and delete bodies outside the ROI
                if (apply_mask)
                {
                    int inside = 0;
                    int outside = 0;
                    std::vector<int> to_delete;
                    for (const auto person : bodies.body_list) {

                        // Center of the skeleton using lowest points of pelvis and neck
                        int xx = (person.keypoint_2d[0].x + person.keypoint_2d[4].x) / 2;
                        int yy = (person.keypoint_2d[0].y + person.keypoint_2d[4].y) / 2;

                        if (visual_debug)
                        {
                            // Drawing the bbox 
                            /*cv::Point p1(person.bounding_box_2d[0].x, person.bounding_box_2d[0].y);
                            cv::Point p2(person.bounding_box_2d[2].x, person.bounding_box_2d[2].y);
                            rectangle(blended, p1, p2, cv::Scalar(255, 0, 0), 4);*/

                            // Drawing a circle in center
                            cv::circle(blended, cv::Point(xx, yy), 20, cv::Scalar(0, 0, 255), 2);


                            // Draw bones
                            const auto& joints = person.keypoint_2d;
                            for (const auto& joint_pair : BODY_38_BONES) {
                                if (joint_pair.first < joints.size() && joint_pair.second < joints.size()) {
                                    const auto& p1 = joints[joint_pair.first];
                                    const auto& p2 = joints[joint_pair.second];
                                    if (std::isfinite(p1.x) && std::isfinite(p1.y) &&
                                        std::isfinite(p2.x) && std::isfinite(p2.y)) {
                                        cv::line(blended, cv::Point(p1.x, p1.y), cv::Point(p2.x, p2.y), cv::Scalar(255, 0, 0), 2);
                                    }
                                }
                            }
                        }

                        bool in_frame_check = xx >= 0 && xx < camera_config.resolution.width && yy >= 0 && yy < camera_config.resolution.height;
                        if (in_frame_check && (int)ROI.at<uchar>(yy, xx) <= 127) {
                            to_delete.push_back(person.id);
                            outside++;
                            continue;
                        }
                        inside++;
                        if (visual_debug)
                            std::cout << "x: " << xx << " y: " << yy << " pixel: " << (int)ROI.at<uchar>(xx, yy) << std::endl;
                    }

                    if (visual_debug)
                        std::cout << "inside: " << inside << " outside: " << outside << std::endl;

                    for (auto it = bodies.body_list.begin(); it != bodies.body_list.end();) {
                        bool flag = false;
                        for (auto id : to_delete) {
                            if ((*it).id == id) {
                                bodies.body_list.erase(it);
                                flag = true;
                                break;
                            }
                        }
                        if (!flag)
                            ++it;
                    }
                }


                if (visual_debug) {
                    cv::imshow("blended", blended);
                    cv::waitKey(2);
                }
                
#if DISPLAY_OGL
                //Update GL View
                viewer.updateData(bodies, cam_pose.pose_data);
#endif

                if (bodies.is_new) 
                {
                    try
                    {
                        // send body data one at a time instead of as one single packet.
                        for (int i = 0; i < bodies.body_list.size(); i++)
                        {
                            std::string data_to_send = toJSON(frame_id, ts, bodies, i, body_tracking_params.body_format, coord_sys, coord_unit).dump();
                            sock.sendTo(data_to_send.data(), data_to_send.size(), servAddress, servPort);
                        }
                    }
                    catch (SocketException& e)
                    {
                        cerr << e.what() << endl;
                        //exit(1);
                    }
                }
            }

            if (zed_config.send_camera_pose)
            {
                zed.getPosition(cam_pose);
                std::string data_to_send = toJSON(frame_id, zed.getCameraInformation().serial_number, ts, cam_pose, coord_sys, coord_unit).dump();
                sock.sendTo(data_to_send.data(), data_to_send.size(), servAddress, servPort);
            }

            frame_id++;
        }
        else if (err == sl::ERROR_CODE::END_OF_SVOFILE_REACHED)
        {
            frame_id = 0;
            zed.setSVOPosition(0);
        }
        else
        {
            print("error grab", returned_state, "\nExit program.");
        }

#if DISPLAY_OGL
        run = viewer.isAvailable();
#endif

       
        auto stop = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
        //std::cout << stop - start << " ms" << std::endl;
    }

#if DISPLAY_OGL
    viewer.exit();
#endif

    // Release Bodies
    bodies.body_list.clear();

    // Disable modules
    zed.disableBodyTracking();
    zed.disablePositionalTracking();
    zed.close();

    return EXIT_SUCCESS;
}



/// ----------------------------------------------------------------------------
/// ----------------------------------------------------------------------------
/// ----------------------------- DATA FORMATTING ------------------------------
/// ----------------------------------------------------------------------------
/// ----------------------------------------------------------------------------


void print(string msg_prefix, ERROR_CODE err_code, string msg_suffix) {
    cout << "[Sample]";
    if (err_code != ERROR_CODE::SUCCESS)
        cout << "[Error]";
    cout << " " << msg_prefix << " ";
    if (err_code != ERROR_CODE::SUCCESS) {
        cout << " | " << toString(err_code) << " : ";
        cout << toVerbose(err_code);
    }
    if (!msg_suffix.empty())
        cout << " " << msg_suffix;
    cout << endl;
}

// Create the json sent to the clients
nlohmann::json toJSON(int frame_id, int serial_number, sl::Timestamp timestamp, sl::Pose& cam_pose, sl::COORDINATE_SYSTEM coord_sys, sl::UNIT coord_unit)
{
    nlohmann::json j;

    j["serial_number"] = serial_number;
    j["frame_id"] = frame_id;
    j["timestamp"] = timestamp.data_ns;
    j["role"] = ZEDLiveLinkRole::Camera;
    j["coordinate_system"] = coord_sys;
    j["coordinate_unit"] = coord_unit;

    j["camera_position"] = nlohmann::json::object();
    j["camera_position"]["x"] = isnan(cam_pose.getTranslation().x) ? 0 : cam_pose.getTranslation().x;
    j["camera_position"]["y"] = isnan(cam_pose.getTranslation().y) ? 0 : cam_pose.getTranslation().y;
    j["camera_position"]["z"] = isnan(cam_pose.getTranslation().z) ? 0 : cam_pose.getTranslation().z;

    j["camera_orientation"] = nlohmann::json::object();
    j["camera_orientation"]["x"] = isnan(cam_pose.getOrientation().x) ? 0 : cam_pose.getOrientation().x;
    j["camera_orientation"]["y"] = isnan(cam_pose.getOrientation().x) ? 0 : cam_pose.getOrientation().y;
    j["camera_orientation"]["z"] = isnan(cam_pose.getOrientation().x) ? 0 : cam_pose.getOrientation().z;
    j["camera_orientation"]["w"] = isnan(cam_pose.getOrientation().x) ? 0 : cam_pose.getOrientation().w;

    return j;
}


// send one skeleton at a time
nlohmann::json toJSON(int frame_id, sl::Timestamp timestamp, sl::Bodies& bodies, int id, sl::BODY_FORMAT body_format, sl::COORDINATE_SYSTEM coord_sys, sl::UNIT coord_unit)
{
    nlohmann::json j;

    j["frame_id"] = frame_id;
    j["timestamp"] = timestamp.data_ns;
    j["role"] = ZEDLiveLinkRole::Animation;
    j["body_format"] = body_format;
    j["is_new"] = (bool)bodies.is_new;
    j["coordinate_system"] = coord_sys;
    j["coordinate_unit"] = coord_unit;
    j["nb_bodies"] = bodies.body_list.size();

    if (id < bodies.body_list.size())
    {
        auto body = bodies.body_list[id];

        j["tracking_state"] = (int)body.tracking_state;
        j["action_state"] = (int)body.action_state;
        j["id"] = body.id;
        j["position"] = nlohmann::json::object();
        j["position"]["x"] = isnan(body.position.x) ? 0 : body.position.x;
        j["position"]["y"] = isnan(body.position.y) ? 0 : body.position.y;
        j["position"]["z"] = isnan(body.position.z) ? 0 : body.position.z;

        j["confidence"] = isnan(body.confidence) ? 0 : body.confidence;

        j["keypoint_confidence"] = nlohmann::json::array();
        for (auto& i : body.keypoint_confidence)
        {
            j["keypoint_confidence"].push_back(isnan(i) ? 0 : i);
        }

        j["keypoint"] = nlohmann::json::array();
        for (auto& i : body.keypoint)
        {
            nlohmann::json e;
            e["x"] = isnan(i.x) ? 0 : i.x;
            e["y"] = isnan(i.y) ? 0 : i.y;
            e["z"] = isnan(i.z) ? 0 : i.z;
            j["keypoint"].push_back(e);
        }
        j["local_position_per_joint"] = nlohmann::json::array();
        for (auto& i : body.local_position_per_joint)
        {
            nlohmann::json e;
            e["x"] = isnan(i.x) ? 0 : i.x;
            e["y"] = isnan(i.y) ? 0 : i.y;
            e["z"] = isnan(i.z) ? 0 : i.z;
            j["local_position_per_joint"].push_back(e);
        }
        j["local_orientation_per_joint"] = nlohmann::json::array();
        for (auto& i : body.local_orientation_per_joint)
        {
            nlohmann::json e;
            e["x"] = isnan(i.x) ? 0 : i.x;
            e["y"] = isnan(i.y) ? 0 : i.y;
            e["z"] = isnan(i.z) ? 0 : i.z;
            e["w"] = isnan(i.w) ? 0 : i.w;
            j["local_orientation_per_joint"].push_back(e);
        }

        j["global_root_posititon"] = nlohmann::json::object();
        j["global_root_posititon"]["x"] = isnan(body.keypoint[0].x) ? 0 : body.keypoint[0].x + x_offset;
        j["global_root_posititon"]["y"] = isnan(body.keypoint[0].y) ? 0 : body.keypoint[0].y;
        j["global_root_posititon"]["z"] = isnan(body.keypoint[0].z) ? 0 : body.keypoint[0].z;

        j["global_root_orientation"] = nlohmann::json::object();
        j["global_root_orientation"]["x"] = isnan(body.global_root_orientation.x) ? 0 : body.global_root_orientation.x;
        j["global_root_orientation"]["y"] = isnan(body.global_root_orientation.y) ? 0 : body.global_root_orientation.y;
        j["global_root_orientation"]["z"] = isnan(body.global_root_orientation.z) ? 0 : body.global_root_orientation.z;
        j["global_root_orientation"]["w"] = isnan(body.global_root_orientation.w) ? 0 : body.global_root_orientation.w;
    }

    return j;
}

struct ROIdata
{
    const int radius = 50;
    cv::Point2i last_pt;
    cv::Mat mask, seeds, image;
    bool selectInProgress_frgrnd = false;
    bool selectInProgress_backgrnd = false;
    bool isInit = false;
    cv::Mat im_bgr, frgrnd, bckgrnd;

    void init(sl::Resolution resolution) {
        mask = cv::Mat(resolution.height, resolution.width, CV_8UC1);
        mask.setTo(0);
        seeds = cv::Mat(resolution.height, resolution.width, CV_8UC1);
        seeds.setTo(cv::GrabCutClasses::GC_PR_BGD);
        image = cv::Mat(resolution.height, resolution.width, CV_8UC4);
        image.setTo(127);
        isInit = false;
        frgrnd.release();
        bckgrnd.release();
    }

    void set(bool background, cv::Point current_pt) {
        cv::line(seeds, current_pt, last_pt, cv::Scalar(background ? cv::GrabCutClasses::GC_BGD : cv::GrabCutClasses::GC_PR_FGD), radius);
        cv::line(image, current_pt, last_pt, cv::Scalar(background ? cv::Scalar::all(0) : cv::Scalar::all(255)), radius);
        last_pt = current_pt;
    }

    void updateImage(cv::Mat& im) {
        cv::addWeighted(image, 0.5, im, 0.5, 0, im);
    }

    void compute(cv::Mat& cvImage) {
        cv::cvtColor(cvImage, im_bgr, cv::COLOR_BGRA2BGR);
        cv::Mat seeds_cpy;
        seeds.copyTo(seeds_cpy);
        cv::grabCut(im_bgr, seeds_cpy, cv::Rect(0, 0, im_bgr.cols, im_bgr.rows), frgrnd, bckgrnd, 1, isInit ? cv::GrabCutModes::GC_EVAL : cv::GrabCutModes::GC_INIT_WITH_MASK);

        mask.setTo(255);
        mask.setTo(0, seeds_cpy & 1);
        cv::erode(mask, mask, cv::Mat(5, 5, CV_8UC1));

        isInit = true;
    }
};
