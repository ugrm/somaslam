//  launch-prefix="gdb -ex run --args"
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>
#include <visualization_msgs/Marker.h>

#include <chrono>

#include "data_provider.h"
#include "drone.h"
#include "ls_extractor/visualizer.h"
#include "ls_extractor/ros_utils.h"
#include "multicloud2.h"

// #define PUB_TEXT 1

using namespace std::chrono;
constexpr float NaN = std::numeric_limits<float>().quiet_NaN();

inline void write_result_line(std::ofstream& file, const g2o::SE2& estimate, double time) {
    double x = estimate[0];
    double y = estimate[1];
    double theta = estimate[2];
    file << "FLASER 0 " << x << ' ' << y << ' ' << theta << ' ' << x << ' ' << y << ' ' << theta << ' ' << time << " myhost " << time << '\n';
}

void write_result_odom(std::ofstream& file, const g2o::SE2& base_est, const DeltaVector& odom) {
    auto it = odom.begin();
    write_result_line(file, base_est, it->dt);
    it++;
    for (; it != odom.end(); it++) {
        auto estimate = base_est * it->dpose;
        write_result_line(file, estimate, it->dt);
    }
}

void write_result_odom2(std::ofstream& file, const g2o::SE2& base_est2, const DeltaVector& odom) {
    auto it = odom.begin();
    write_result_line(file, base_est2, it->dt);
    it++;
    for (; it != odom.end(); it++) {
        auto estimate = base_est2 * it->dpose;
        write_result_line(file, estimate, it->dt);
    }
}

visualization_msgs::Marker text_marker(const std::string& frame_id, const std::string& text) {
    visualization_msgs::Marker odom_text_marker;
    odom_text_marker.header.frame_id = frame_id;
    odom_text_marker.text = text;
    odom_text_marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    odom_text_marker.scale.z = 1.0;
    odom_text_marker.color.r = 0.0;
    odom_text_marker.color.g = 0.0;
    odom_text_marker.color.b = 0.0;
    odom_text_marker.color.a = 1.0;
    return odom_text_marker;
}

int main(int argc, char** argv) {
    // std::this_thread::sleep_for(std::chrono::seconds(5));
    ros::init(argc, argv, "log_runner");
    ros::NodeHandle nh;
    XmlRpc::XmlRpcValue slam_config;
    ros::param::get("~slam", slam_config);

    DeltaVector deltas;
    g2o::SE2 cur_pose, zero_pose, last_pose(NaN, NaN, NaN);
    ros::Time last_stamp;
    nav_msgs::Odometry odometry;
    sensor_msgs::LaserScan full_scan;
    odometry.header.frame_id = "odom";
    full_scan.header.frame_id = odometry.child_frame_id = "base_link";
#ifdef PUB_TEXT
    auto odom_text = text_marker(odometry.header.frame_id, "Raw Odometry");
    auto cor_odom_text = text_marker(odometry.header.frame_id, "Corrected Odometry");
#endif
    const int sample_size = slam_config["scan_size"];
    const float range_max = (double)slam_config["range_max"];

    full_scan.range_min = 0.0;
    full_scan.range_max = 10.0;
    full_scan.angle_min = (double)slam_config["angle_min"];
    full_scan.angle_max = (double)slam_config["angle_max"];
    auto& full_range = full_scan.ranges;

    MulticloudConverter pc_converter(nh, slam_config);
    ls_extractor::LineSegmentExtractor extractor;
    {
        XmlRpc::XmlRpcValue  ls_config;
        ros::param::get("~line_extractor", ls_config);
        ls_extractor::parseParams(extractor.params, ls_config);
    }
    ls_extractor::SegmentVisualizer ls_vis("base_link", nh);

    auto odom_pub = nh.advertise<nav_msgs::Odometry>("odom", 1);
    auto full_scan_pub = nh.advertise<sensor_msgs::LaserScan>("full_scan", 1);
    auto odom_text_pub = nh.advertise<visualization_msgs::Marker>("odom_text", 1);
    auto cor_odom_text_pub = nh.advertise<visualization_msgs::Marker>("cor_odom_text", 1);

    Drone drone(nh, slam_config);
    Visualizer slam_vis(drone, nh, slam_config);
    const int m_interval = slam_config["match_interval"];
    std::string dataset_dir, dataset_name;
    ros::param::get("~dataset_dir", dataset_dir);
    ros::param::get("~dataset_name", dataset_name);
    auto provider = create_data_provider(slam_config["data_provider"], dataset_dir + dataset_name + ".log");
    double time = -1e10;

    std::ofstream frontend_time(dataset_dir + dataset_name + ".ftime");
    std::ofstream backend_time(dataset_dir + dataset_name + ".btime");
    std::ofstream dataset_time(dataset_dir + dataset_name + ".dtime");
    std::ofstream outfile(dataset_dir + dataset_name + ".result");
    std::ofstream outfile2(dataset_dir + dataset_name + ".result2");
    
    frontend_time << std::fixed;
    backend_time << std::fixed;
    dataset_time << std::fixed;
    outfile << std::fixed;
    outfile2 << std::fixed;
    
    auto callback = [&](int k) {
        odometry.header.stamp = ros::Time(time);
        if (!std::isnan(last_pose[0])) {
            auto delta = last_pose.inverse() * cur_pose;
            zero_pose = zero_pose * delta;
            deltas.push_back({odometry.header.stamp.toSec() - last_stamp.toSec(), delta});
            last_pose = cur_pose;
        }
        last_pose = cur_pose;
        last_stamp = odometry.header.stamp;

        tf::Transform current_scan_tf;
        current_scan_tf.getOrigin().setValue(zero_pose[0], zero_pose[1], 0.0);
        current_scan_tf.getBasis().setRPY(0.0, 0.0, zero_pose[2]);
        odometry.pose.pose.position.x = current_scan_tf.getOrigin().getX();
        odometry.pose.pose.position.y = current_scan_tf.getOrigin().getY();
        odometry.pose.pose.position.z = 0.0;
        tf::quaternionTFToMsg(current_scan_tf.getRotation(), odometry.pose.pose.orientation);
        
        
        int full_size = full_range.size();
        full_scan.angle_increment = (full_scan.angle_max - full_scan.angle_min) / (full_size - 1);
        if (sample_size == full_size) {
            memcpy(pc_converter.scan.ranges.data(), full_range.data(), sample_size * sizeof(float));
        } else {
            int increment = full_size / (sample_size - 1);
            
            for (int j = 0, i = 0; j < sample_size - 1; j++, i += increment) {
                pc_converter.scan.ranges[j] = std::min(full_range[i], range_max);
                float angle = pc_converter.scan.angle_min + full_scan.angle_increment * i;
                pc_converter.table[j].cos_v = cos(angle);
                pc_converter.table[j].sin_v = sin(angle);
            }
            

            pc_converter.table.back().cos_v = cos(pc_converter.scan.angle_max);
            pc_converter.table.back().sin_v = sin(pc_converter.scan.angle_max);
            pc_converter.scan.ranges.back() = full_range.back();
        }
       

        /*
        // ------------------- Custom lidar selection ------------------ //
        int full_size = full_range.size();
        full_scan.angle_increment = (full_scan.angle_max - full_scan.angle_min) / (full_size - 1);

        sensor_msgs::LaserScan selected_scan = full_scan; // for rviz
        selected_scan.ranges.assign(full_size, std::numeric_limits<float>::quiet_NaN());

        std::vector<int> indices_to_keep;

        if (full_size == 180) {
            indices_to_keep = {
                169, 172, 175, 178, 1, 4, 7, 10,    // 12 o'clock
                34, 37, 40, 43, 46, 49, 52, 55,     // 9 o'clock
                79, 82, 85, 88, 91, 94, 97, 100,    // 6 o'clock
                124, 127, 130, 133, 136, 139, 142, 145 // 3 o'clock
            };
        } else if (full_size == 361) {
            indices_to_keep = {
                338, 344, 350, 356, 2, 8, 14, 20,       // 12 o'clock
                68, 74, 80, 86, 92, 98, 104, 110,       // 9 o'clock
                158, 164, 170, 176, 182, 188, 194, 200, // 6 o'clock
                248, 254, 260, 266, 272, 278, 284, 290  // 3 o'clock
            };
        } else {
            ROS_ERROR("Unsupported lidar point count: %d", full_size);
            return;
        }

        // Print selected indices to verify in terminal
        std::cout << "Selected lidar indices:";
        for (int idx : indices_to_keep) {
            std::cout << " " << idx;
        }
        std::cout << std::endl;

        int num_desired_points = indices_to_keep.size();
        pc_converter.scan.ranges.resize(num_desired_points);
        pc_converter.table.resize(num_desired_points);

        for (int j = 0; j < num_desired_points; j++) {
            int idx = indices_to_keep[j];
            float angle = full_scan.angle_min + idx * full_scan.angle_increment;

            float range = std::min(full_range[idx], range_max);
            pc_converter.scan.ranges[j] = range;
            pc_converter.table[j].cos_v = cos(angle);
            pc_converter.table[j].sin_v = sin(angle);

            // Set data for rviz LaserScan
            selected_scan.ranges[idx] = range;
        }

        // publish to rviz clearly here:
        static ros::Publisher selected_scan_pub = nh.advertise<sensor_msgs::LaserScan>("selected_scan", 1);
        selected_scan_pub.publish(selected_scan);
        // ------------------- End of custom lidar selection ------------------ //
        */


        if (pc_converter.update(odometry.header.stamp, deltas, current_scan_tf)) {
            // frontend
            auto start_t = std::chrono::steady_clock::now();
            extractor.extract_lines(pc_converter.pcv);
            drone.msgCallback(extractor.segments, odometry, pc_converter.scan);
            frontend_time << (std::chrono::steady_clock::now() - start_t).count() * 1e-9 << '\n';

            ls_vis.visualize(extractor.segments, odometry.header.stamp);
            if (k % m_interval == 0) {
                
                start_t = std::chrono::steady_clock::now();
                drone.loop_closer.precompute();
                drone.loop_closer.match();
                backend_time << (std::chrono::steady_clock::now() - start_t).count() * 1e-9 << '\n';
            }
        }

        odom_pub.publish(odometry);
        full_scan_pub.publish(full_scan);
#ifdef PUB_TEXT
        odom_text.pose = odometry.pose.pose;
        odom_text.pose.position.x -= 1.0;
        odom_text.pose.position.y -= 1.0;
        odom_text_pub.publish(odom_text);

        cor_odom_text.pose = drone.cor_pose_msg.pose;
        cor_odom_text.pose.position.x -= 1.0;
        cor_odom_text.pose.position.y -= 1.0;
        cor_odom_text_pub.publish(cor_odom_text);
#endif
    };

    auto final_cleanup = [&drone, &slam_vis]() {
        slam_vis.stop();
        drone.loop_closer.loop_closure_min_score = 0.5;
        drone.loop_closer.precompute();
        drone.loop_closer.match();

        for (auto& edge : drone.pose_graph.all_closures) {
            edge.computeError();
            if (edge.chi2() > 11.345) {
                std::cout << "removing edge: " << edge.chi2() << std::endl;
                drone.pose_graph.opt.removeEdge(&edge);
                drone.pose_graph.closures.erase(&edge);
                drone.pose_graph.false_closures.insert(&edge);
            }
        }

        // auto it1 = drone.lm_graph.poses.begin();
        // auto it2 = drone.pose_graph.poses.begin();
        // auto est = it1->pose.estimate();
        // for (; it2 != drone.pose_graph.poses.end(); it1++, it2++) {
        //     it2->pose.setEstimate(est);
        //     it2->edge.setMeasurement(it1->edge.measurement());
        //     est *= it1->edge.measurement();
        // }
        // slam_vis.visualize_landmarks(ros::Time::now());
        // std::this_thread::sleep_for(std::chrono::seconds(5));

        drone.pose_graph.opt.initializeOptimization();
        drone.pose_graph.opt.optimize(20);
        slam_vis.visualize_landmarks(ros::Time::now());
    };

    bool realtime;
    int _rate;
    ros::param::get("~rate", _rate);
    ros::param::get("~realtime", realtime);
    slam_vis.start(slam_config["visualize_rate"]);

    if (realtime) {
        std::cout << "using simulated realtime node" << std::endl;
        std::atomic<bool> flag{provider->get_data(time, cur_pose, full_range) && ros::ok()};
        std::thread lc_thread([&]() {
            while (flag) {
                drone.loop_closer.precompute();
                drone.loop_closer.match();
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
            }
            final_cleanup();
        });
        double prev_time = time;
        callback(-1);
        while (flag) {
            flag = provider->get_data(time, cur_pose, full_range) && ros::ok();
            auto prev_clock = std::chrono::steady_clock::now();
            callback(-1);
            double time_diff = time - prev_time;
            double sleep_time = time_diff / _rate - (std::chrono::steady_clock::now() - prev_clock).count() * 1e-9;
            prev_time = time;
            // std::cout.setf(std::cout.fixed);
            // std::cout << "time diff: " << time_diff << " | time : "  << time << " | sleep time: " << sleep_time << std::endl;
            if (sleep_time > 0.0)
                std::this_thread::sleep_for(std::chrono::duration<double>(sleep_time));
        }
        lc_thread.join();
    } else {
        std::this_thread::sleep_for(std::chrono::milliseconds(2000));
        std::cout << "using fix rate node" << std::endl;
        ros::Rate rate(_rate);
        int k = 0;
        provider->get_data(time, cur_pose, full_range) && ros::ok();
        dataset_time << time << '\n';
        callback(k++);
        for (; provider->get_data(time, cur_pose, full_range) && ros::ok(); k++) {
            dataset_time << time << '\n';
            callback(k);
            rate.sleep();
        }
        final_cleanup();
    }
    frontend_time.flush();
    backend_time.flush();
    dataset_time.flush();

    auto lit = drone.lm_graph.poses.begin();
    auto pit = drone.pose_graph.poses.begin();
    for (auto end = drone.lm_graph.poses.begin() + drone.loop_closer.last_opt_pose_index; lit != end; lit++, pit++) {
        write_result_odom(outfile, pit->pose.estimate(), lit->odom);
        write_result_odom2(outfile2, lit->pose.estimate(), lit->odom);
    }
    g2o::SE2 base_est = (pit - 1)->pose.estimate();
    g2o::SE2 base_est2 = (lit - 1)->pose.estimate();
    for (auto end = drone.lm_graph.poses.end(); lit != end; lit++) {
        base_est *= ((lit - 1)->pose.estimate().inverse() * lit->pose.estimate());
        base_est2 *= ((lit - 1)->pose.estimate().inverse() * lit->pose.estimate());
        write_result_odom(outfile, base_est, lit->odom);
        write_result_odom2(outfile2, base_est2, lit->odom);
    }
    outfile.flush();
    outfile2.flush();

    std::cout << "Log runner finished. Press Ctrl-C to shutdown" << std::endl;
    ros::waitForShutdown();
}