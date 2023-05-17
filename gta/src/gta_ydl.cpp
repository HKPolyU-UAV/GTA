#include "include/gta_ydl.h"

void gta_ns::TrackingNodelet::imgsyn_cb(const sensor_msgs::CompressedImage::ConstPtr &rgbimage, 
                                        const sensor_msgs::ImageConstPtr &dpimage)
{
    cv_bridge::CvImageConstPtr depth_ptr;
    try
    {
        frame = cv::imdecode(cv::Mat(rgbimage->data), 1);
        res = cv::imdecode(cv::Mat(rgbimage->data), 1);
        gt = cv::imdecode(cv::Mat(rgbimage->data), 1);
        depth_ptr  = cv_bridge::toCvCopy(dpimage, dpimage->encoding);
    }
    catch (cv_bridge::Exception &e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
    }
    cv::Mat image_dep = depth_ptr->image;
    // cout << frame.size << endl;
    

    double time_start, time_end;

    if (!frame.empty())
    {
        time_start = ros::Time::now().toSec();

        double ticks = 0;
        int i = 0;

        cv::Point center_true;
        double fps;
        double fps_average = 0;
        vector<double> fpss;

        double starttime = ros::Time::now().toSec();
        detect_yolo(frame);
        double endtime = ros::Time::now().toSec();
        double deltatime = endtime - starttime;
        fps = 1 / deltatime;
        // cout << "fps: " << fps << endl;
        if (fpss.size() > 2000)
            fpss.clear();
        fpss.push_back(fps);
        fps_average = accumulate(fpss.begin(), fpss.end(), 0.0) / fpss.size();
        // cout << "fps_avg: " << fps_average << endl;

        vector<Object> temp = objects;
        // cout << "temp size: " << temp.size() << endl;

        cv::Rect predRect;
        cv::Rect temprect;
        cv::Rect interested;
        vector<Object> potential;
        vector<float> potential_c;
        vector<float> potential_c_area;
        cv::Mat tempp;

        double prob;

        if (temp.size() != 0)
        {
            for (auto stuff : temp)
            {
                // cout << stuff.classnameofdetection << endl;
                if (stuff.classnameofdetection == "uav")
                {
                    potential.push_back(stuff);
                    potential_c.push_back(stuff.prob); // confidence
                    potential_c_area.push_back(stuff.rect.area());
                }
            }
            // cout << "potential size: " << potential.size() << endl;

            if (potential.size() != 0)
            {
                int maxElementIndex = max_element(potential_c_area.begin(), potential_c_area.end()) - potential_c_area.begin();
                interested = potential[maxElementIndex].rect;
                temprect = potential[maxElementIndex].rect;


                tempp = potential[maxElementIndex].frame;
                prob = potential[maxElementIndex].prob;
                cout<<"prob: "<<prob<<endl;

                if(prob > 0.93)
                {
                    int depthbox_w = potential[maxElementIndex].rect.width * 0.35;
                    int depthbox_h = potential[maxElementIndex].rect.height * 0.35;
                    depthbox_vertice1 = cv::Point(potential[maxElementIndex].center_bdbox.x - depthbox_w / 2, potential[maxElementIndex].center_bdbox.y - depthbox_w / 2);
                    depthbox_vertice2 = cv::Point(potential[maxElementIndex].center_bdbox.x + depthbox_w / 2, potential[maxElementIndex].center_bdbox.y + depthbox_h / 2);
                    center_true = cv::Point(interested.x + interested.width / 2, interested.y + interested.height / 2);
                    cv::rectangle(res, interested, CV_RGB(255, 255, 0), 1);

                    /*********************get depthdata******************/
                    cv::Rect letsgetdepth(depthbox_vertice1, depthbox_vertice2);
                    cv::Mat ROI(image_dep, letsgetdepth);
                    cv::Mat ROIframe;
                    ROI.copyTo(ROIframe);
                    vector<cv::Point> nonzeros;
                    cv::findNonZero(ROIframe, nonzeros);
                    vector<double> nonzerosvalue;
                    for (auto temp : nonzeros)
                    {
                        double depth_temo = ROIframe.at<ushort>(temp);
                        nonzerosvalue.push_back(depth_temo);
                    }
                    double depth_average = 0.0;
                    int k = 0;
                    if (nonzerosvalue.size() != 0)
                    {
                        for(int i=0; i<nonzerosvalue.size(); i++)
                        {
                            if (nonzerosvalue[i] <=4.0)
                            {
                                depth_average = depth_average + nonzerosvalue[i];
                                k++;
                            }
                        }
                    }
                    depth_average =  depth_average / k;
                    double depth_cam = 0.001 * depth_average; // depth from camera

                    Vec4 x, y;

                    x[2] = depth_cam;
                    x[0] = depth_cam * (center_true.x - K_cam(0, 2)) / K_cam(0, 0);
                    x[1] = depth_cam * (center_true.y - K_cam(1, 2)) / K_cam(1, 1);
                    x[3] = 1;
                    y = mat_cam_lidar.inverse() * x;

                    if (y[0] != 0 && y[0] <= 4)
                    {
                        img_meas_pos[2] = y[2];
                        img_meas_pos[0] = y[0];
                        img_meas_pos[1] = y[1];
                    }
                    else
                    {
                        img_meas_pos[2] = 0;
                        img_meas_pos[0] = 0;
                        img_meas_pos[1] = 0;
                    }
                    yolo_in = true;

                    // cout<<"yolo img_meas_pos: "<<img_meas_pos<<endl;
                    /*********************get depthdata******************/
                }
            }
        }
        time_end = ros::Time::now().toSec();
        // cout<<"ms: "<<time_end-time_start<<endl<<endl;
        // cv::imshow("Yolo", frame);
        // cv::imshow("Tracking...", res);
        // cv::waitKey(20);
    }
}

inline void gta_ns::TrackingNodelet::YoloNodeInit(const char *parampath_input, const char *binfile_input)
{
    this->cnn_local->opt.num_threads = 8;            // You need to compile with libgomp for multi thread support
    this->cnn_local->opt.use_vulkan_compute = false; // You need to compile with libvulkan for gpu support

    this->cnn_local->opt.use_winograd_convolution = true;
    this->cnn_local->opt.use_sgemm_convolution = true;
    this->cnn_local->opt.use_fp16_packed = true;
    this->cnn_local->opt.use_fp16_storage = true;
    this->cnn_local->opt.use_fp16_arithmetic = true;
    this->cnn_local->opt.use_packing_layout = true;
    this->cnn_local->opt.use_shader_pack8 = false;
    this->cnn_local->opt.use_image_storage = false;

    int succeed = -1;

    try
    {
        succeed = this->cnn_local->load_param(parampath_input);
        succeed = this->cnn_local->load_model(binfile_input);
        if (succeed != 0)
            throw "bad doggie!";
    }
    catch (...)
    {
        cerr << "check! Initiaion fail!\n";
        exit(0);
    }

    this->cnn_local->load_param(parampath_input);
    this->cnn_local->load_model(binfile_input);
    intiated = true;
    printf("initialization succeed\n");
}

void gta_ns::TrackingNodelet::detect_yolo(cv::Mat &bgr)
{
    int img_w = bgr.cols;
    int img_h = bgr.rows;
    this->total_start = std::chrono::steady_clock::now();

    ncnn::Mat in = ncnn::Mat::from_pixels_resize(bgr.data, ncnn::Mat::PIXEL_BGR2RGB, bgr.cols, bgr.rows, target_size, target_size);

    const float mean_vals[3] = {0, 0, 0};
    const float norm_vals[3] = {1 / 255.f, 1 / 255.f, 1 / 255.f};
    in.substract_mean_normalize(mean_vals, norm_vals);

    ncnn::Extractor ex = cnn_local->create_extractor();
    ex.input("data", in);
    ncnn::Mat out;
    ex.extract("yolo1", out);

    objects.clear();
    for (int i = 0; i < out.h; i++)
    {
        const float *values = out.row(i);

        Object object;
        object.label = values[0];
        object.prob = values[1];
        object.rect.x = values[2] * img_w;
        object.rect.y = values[3] * img_h;
        object.rect.width = values[4] * img_w - object.rect.x;
        object.rect.height = values[5] * img_h - object.rect.y;
        object.classnameofdetection = class_names[object.label];
        object.frame = bgr;
        object.center_bdbox = cv::Point(object.rect.x + object.rect.width / 2, object.rect.y + object.rect.height / 2);
        objects.push_back(object);
    }
    this->total_end = std::chrono::steady_clock::now();
    total_fps = 1000.0 / std::chrono::duration_cast<std::chrono::milliseconds>(total_end - total_start).count();
    this->appro_fps = total_fps;
    for (size_t i = 0; i < objects.size(); i++)
    {
        const Object &obj = objects[i];
        // fprintf(stderr, "%d = %.5f at %.2f %.2f %.2f x %.2f\n", obj.label, obj.prob,
        //         obj.rect.x, obj.rect.y, obj.rect.width, obj.rect.height);
    }
    draw_objects(bgr, objects);
}

void gta_ns::TrackingNodelet::draw_objects(cv::Mat &bgr, const std::vector<Object> &objects)
{
    for (size_t i = 0; i < objects.size(); i++)
    {
        const Object &obj = objects[i];

        // fprintf(stderr, "%d = %.5f at %.2f %.2f %.2f x %.2f\n", obj.label, obj.prob,
        //         obj.rect.x, obj.rect.y, obj.rect.width, obj.rect.height);

        cv::rectangle(bgr, obj.rect, cv::Scalar(255, 0, 0));

        char text[256];
        sprintf(text, "%s %.1f%%", class_names[obj.label], obj.prob * 100);

        int baseLine = 0;
        cv::Size label_size = cv::getTextSize(text, cv::FONT_HERSHEY_SIMPLEX, 0.5, 1, &baseLine);

        int x = obj.rect.x;
        int y = obj.rect.y - label_size.height - baseLine;
        if (y < 0)
            y = 0;
        if (x + label_size.width > bgr.cols)
            x = bgr.cols - label_size.width;

        cv::rectangle(bgr, cv::Rect(cv::Point(x, y), cv::Size(label_size.width, label_size.height + baseLine)),
                      cv::Scalar(255, 255, 255), -1);

        cv::putText(bgr, text, cv::Point(x, y + label_size.height),
                    cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 0));
    }
}

void gta_ns::TrackingNodelet::pcl_cb(const livox_ros_driver::CustomMsg::ConstPtr &pclmsg)
{
    pcl::PointXYZ temppc;
    int plsize = pclmsg->point_num;
    pc->clear();
    pc->header.stamp = pclmsg->header.stamp.toSec();
    timestamp = pclmsg->header.stamp;
    for (int i = 0; i < plsize; i++)
    {
        temppc.x = pclmsg->points[i].x;
        temppc.y = pclmsg->points[i].y;
        temppc.z = pclmsg->points[i].z;
        pc->points.push_back(temppc);
    }

    // remove NAN points
    std::vector<int> NAN_indices;
    pcl::removeNaNFromPointCloud(*pc, *pc, NAN_indices);
    removeZeros(pc);

    cluster_true = false;

    //run kalman filter
    if (kf_init)
    {
        
        //**********kalman filter prediction start***********
        double precTick = ticks;
        ticks = (double)cv::getTickCount();
        dT = (ticks - precTick) / cv::getTickFrequency(); // seconds
        // dT = 0.03;
        cout<<"dT: "<<dT<<" ms"<<endl;
        // >>>> Matrix A
        kf.transitionMatrix.at<float>(3) = dT;
        kf.transitionMatrix.at<float>(10) = dT;
        kf.transitionMatrix.at<float>(17) = dT;
        // <<<< Matrix A

        state = kf.predict();

        state_pre[0] = state.at<float>(0);
        state_pre[1] = state.at<float>(1);
        state_pre[2] = state.at<float>(2);
        //**********kalman filter prediction end***********

        // yolo_in = false;
        if (yolo_in)
        {
            cout<<"initialized yolo in "<<endl;
            init_yolo++;
            cv::Point ver_1 = depthbox_vertice1;
            cv::Point ver_2 = depthbox_vertice2;
            Vector3f img_meas_pos_in = img_meas_pos;
            //******************get measure start*****************//
            pcl::PointCloud<pcl::PointXYZ>::Ptr depth_pcl(new pcl::PointCloud<pcl::PointXYZ>);
            for (int i = 0; i < obj_pc->points.size(); i++)
            {
                Vec4 X;
                Vec3 Y;
                X[0] = obj_pc->points[i].x;
                X[1] = obj_pc->points[i].y;
                X[2] = obj_pc->points[i].z;
                X[3] = 1;
                Y = P_cam * mat_cam_lidar * X;
                cv::Point pt;
                pt.x = Y[0] / Y[2];
                pt.y = Y[1] / Y[2];
                // cout<<"lidar point on RGB image: "<<pt.x<<" "<<pt.y<<endl;

                if (pt.x >= ver_2.x || pt.y >= ver_2.y || pt.x <= ver_1.x || pt.y <= ver_1.y)
                {
                    continue;
                }
                else
                {
                    depth_pcl->points.push_back(obj_pc->points[i]);
                }

                // cv::circle(overlay, pt, 1, cv::Scalar(0, 255, 0), cv::FILLED);
            }
            // cout << "init depth_pcl->points.size(): " << depth_pcl->points.size() << endl;
            if (depth_pcl->points.size() >= 3)
            {
                cluster(depth_pcl, pc_meas_pos);
            }
            else
            {
                measured = false;
                notFoundCount++;
                if (notFoundCount > 100)
                {
                    kf_init = false;
                }
            }
            //******************get measure end*******************

            //********************verify measure start*************
            float a, b, c, d, e;
            a = last_meas_pos[0] - img_meas_pos_in[0];
            b = last_meas_pos[1] - img_meas_pos_in[1];
            c = last_meas_pos[2] - img_meas_pos_in[2];
            e = a * a + b * b + c * c;
            if (cluster_true)
            {
                a = last_meas_pos[0] - pc_meas_pos[0];
                b = last_meas_pos[1] - pc_meas_pos[1];
                c = last_meas_pos[2] - pc_meas_pos[2];
                d = a * a + b * b + c * c;
                if (pc_meas_pos[0] != 0 && d < 0.8)
                {
                    meas_pos[0] = pc_meas_pos[0];
                    meas_pos[1] = pc_meas_pos[1];
                    meas_pos[2] = pc_meas_pos[2];
                    measured = true;
                    notFoundCount = 0;
                }
                else
                {
                    measured = false;
                    notFoundCount++;
                    if (notFoundCount > 100)
                    {
                        kf_init = false;
                    }
                }
                cluster_true = false;
            }
            else if (img_meas_pos_in[0] > 0 && img_meas_pos_in[0] <= 4 && e < 0.8)
            {
                meas_pos[0] = img_meas_pos_in[0];
                meas_pos[1] = img_meas_pos_in[1];
                meas_pos[2] = img_meas_pos_in[2];
                measured = true;
                notFoundCount = 0;
            }
            else
            {
                measured = false;
                notFoundCount++;
                if (notFoundCount > 100)
                {
                    kf_init = false;
                }
            }
            //********************verify measure end***************

            //************kalman filter correction start***********
            if (measured)
            {
                meas.at<float>(0) = meas_pos[0];
                meas.at<float>(1) = meas_pos[1];
                meas.at<float>(2) = meas_pos[2];
                kf.correct(meas); // Kalman Correction
                state_true[0] = state.at<float>(0);
                state_true[1] = state.at<float>(1);
                state_true[2] = state.at<float>(2);
                last_meas_pos[0] = state_true[0];
                last_meas_pos[1] = state_true[1];
                last_meas_pos[2] = state_true[2];
                gta::obj obj_pos_body;
                obj_pos_body.X_c = state_true[0];
                obj_pos_body.Y_c = state_true[1];
                obj_pos_body.Z_c = state_true[2];
                obj_pos_body.header.stamp = pclmsg->header.stamp;
                pub_meas.publish(obj_pos_body);
                // send.header.stamp = ros::Time::now();
                show_meas_count++;
                cout << "meas count: " << show_meas_count << endl;
                cout << "state_true: " << state_true << endl
                     << endl;
            }
            else
            {
                last_meas_pos[0] = state_pre[0];
                last_meas_pos[1] = state_pre[1];
                last_meas_pos[2] = state_pre[2];
                gta::obj obj_pos_body;
                obj_pos_body.X_c = state_pre[0];
                obj_pos_body.Y_c = state_pre[1];
                obj_pos_body.Z_c = state_pre[2];
                obj_pos_body.header.stamp = pclmsg->header.stamp;
                pub_meas.publish(obj_pos_body);
                // send.header.stamp = ros::Time::now();
                show_pre_count++;
                cout << "pre measure count: " << show_pre_count << endl;
                cout << "state_pre: " << state_pre << endl
                     << endl;
            }
            //************kalman filter correction end*************
            yolo_in = false;
        }

        else
        {
            //************cluster based on prediction****************
            double filter_size = 0.3;
            // cout << "before pcl_track->points.size(): " << obj_pc->points.size() << endl;
            PTfilter(obj_pc, filter_size);
            cout<<"obj_pc size: "<<obj_pc->points.size()<<endl;
            // cout << "obj_pc->points.size(): " << obj_pc->points.size() << endl;
            if (obj_pc->points.size() < 3)
            {
                measured = false;
                notFoundCount++;
                if (notFoundCount > 100)
                {
                    kf_init = false;
                }
            }
            else 
            {
                // vector<Point> dbscan_points;
                // for (int i = 0; i < obj_pc->points.size(); i++)
                // {
                //     Point temp_point;
                //     temp_point.x = obj_pc->points[i].x;
                //     temp_point.y = obj_pc->points[i].y;
                //     temp_point.z = obj_pc->points[i].z;
                //     temp_point.clusterID = UNCLASSIFIED;
                //     dbscan_points.push_back(temp_point);
                // }
                // Cluster
                // DBSCAN ds(MINIMUM_POINTS, EPSILON, dbscan_points);
                // ds.run();
                // dbscan_points.clear();
                // dbscan_points = ds.getMaxClusteredPoints();
                // CalculateCentroid(dbscan_points, pc_meas_pos);
                cluster(obj_pc, pc_meas_pos);
            }
            //************cluster based on prediction****************

            //********************verify measure start*************
            if (cluster_true)
            {
                float a, b, c, d;
                a = last_meas_pos[0] - pc_meas_pos[0];
                b = last_meas_pos[1] - pc_meas_pos[1];
                c = last_meas_pos[2] - pc_meas_pos[2];
                d = a * a + b * b + c * c;
                if (pc_meas_pos[0] != 0 && d < 0.8)
                {
                    meas_pos[0] = pc_meas_pos[0];
                    meas_pos[1] = pc_meas_pos[1];
                    meas_pos[2] = pc_meas_pos[2];
                    measured = true;
                    notFoundCount = 0;
                }
                else
                {
                    measured = false;
                    notFoundCount++;
                    if (notFoundCount > 100)
                    {
                        kf_init = false;
                    }
                }
                cluster_true = false;
            }
            else
            {
                measured = false;
                notFoundCount++;
                if (notFoundCount > 100)
                {
                    kf_init = false;
                }
            }
            //********************verify measure end***************

            //************kalman filter correction start***********
            if (measured)
            {
                meas.at<float>(0) = meas_pos[0];
                meas.at<float>(1) = meas_pos[1];
                meas.at<float>(2) = meas_pos[2];
                kf.correct(meas); // Kalman Correction
                state_true[0] = state.at<float>(0);
                state_true[1] = state.at<float>(1);
                state_true[2] = state.at<float>(2);
                last_meas_pos[0] = state_true[0];
                last_meas_pos[1] = state_true[1];
                last_meas_pos[2] = state_true[2];
                gta::obj obj_pos_body;
                obj_pos_body.X_c = state_true[0];
                obj_pos_body.Y_c = state_true[1];
                obj_pos_body.Z_c = state_true[2];
                obj_pos_body.header.stamp = pclmsg->header.stamp;
                pub_meas.publish(obj_pos_body);
                // send.header.stamp = ros::Time::now();
                show_meas_count++;
                cout << "measure count: " << show_meas_count << endl;
                cout << "state_true: " << state_true << endl
                     << endl;
            }
            else
            {
                last_meas_pos[0] = state_pre[0];
                last_meas_pos[1] = state_pre[1];
                last_meas_pos[2] = state_pre[2];
                gta::obj obj_pos_body;
                obj_pos_body.X_c = state_pre[0];
                obj_pos_body.Y_c = state_pre[1];
                obj_pos_body.Z_c = state_pre[2];
                obj_pos_body.header.stamp = pclmsg->header.stamp;
                pub_meas.publish(obj_pos_body);
                // send.header.stamp = ros::Time::now();
                show_pre_count++;
                cout << "pre measure count: " << show_pre_count << endl;
                cout << "state_pre: " << state_pre << endl
                     << endl;
            }
            //************kalman filter correction end*************
        }
        cout<<"init_yolo count: "<<init_yolo<<endl;
    }

    else
    {
        // yolo_in = false;
        if (yolo_in)
        {
            cout<<"Not initialized yolo in "<<endl;
            Notinit_yolo++;
            cv::Point ver_1 = depthbox_vertice1;
            cv::Point ver_2 = depthbox_vertice2;
            Vector3f img_meas_pos_in = img_meas_pos;
            //******************get measure start*****************//
            pcl::PointCloud<pcl::PointXYZ>::Ptr depth_pcl(new pcl::PointCloud<pcl::PointXYZ>);
            for (int i = 0; i <= obj_pc->points.size(); i++)
            {
                Vec4 X;
                Vec3 Y;
                X[0] = obj_pc->points[i].x;
                X[1] = obj_pc->points[i].y;
                X[2] = obj_pc->points[i].z;
                X[3] = 1;
                Y = P_cam * mat_cam_lidar * X;
                cv::Point pt;
                pt.x = Y[0] / Y[2];
                pt.y = Y[1] / Y[2];
                // cout<<"lidar point on RGB image: "<<pt.x<<" "<<pt.y<<endl;

                if (pt.x >= ver_2.x || pt.y >= ver_2.y || pt.x <= ver_1.x || pt.y <= ver_1.y)
                {
                    continue;
                }
                else
                {
                    depth_pcl->points.push_back(obj_pc->points[i]);
                }

                // cv::circle(overlay, pt, 1, cv::Scalar(0, 255, 0), cv::FILLED);
            }
            // cout << "init depth_pcl->points.size(): " << depth_pcl->points.size() << endl;
            if (depth_pcl->points.size() >= 3)
            {
                pc_yolo = true;
                cluster(depth_pcl, pc_meas_pos);
            }
            else
            {
                measured = false;
                pc_yolo = false;
            }
            //******************get measure end*******************

            //********************verify measure start*************
            if (pc_yolo)
            {
                if (pc_meas_pos[0] > 0 && pc_meas_pos[0] < 5)
                {
                    meas_pos[0] = pc_meas_pos[0];
                    meas_pos[1] = pc_meas_pos[1];
                    meas_pos[2] = pc_meas_pos[2];
                    measured = true;
                    notFoundCount = 0;
                }
                else
                {
                    measured = false;
                }
                cluster_true = false;
            }
            else if (img_meas_pos_in[0] > 0 && img_meas_pos_in[0] <= 4)
            {
                meas_pos[0] = img_meas_pos_in[0];
                meas_pos[1] = img_meas_pos_in[1];
                meas_pos[2] = img_meas_pos_in[2];
                measured = true;
                notFoundCount = 0;
            }
            else
            {
                measured = false;
            }
            //********************verify measure end***************

            //************kalman filter initialization start***********
            if (measured)
            {
                meas.at<float>(0) = meas_pos[0];
                meas.at<float>(1) = meas_pos[1];
                meas.at<float>(2) = meas_pos[2];
                last_meas_pos[0] = meas.at<float>(0);
                last_meas_pos[1] = meas.at<float>(1);
                last_meas_pos[2] = meas.at<float>(2);
                // cout<<"!kf_init pre_meas_pos: "<<pre_meas_pos<<endl;

                // >>>> Initialization
                kf.errorCovPre.at<float>(0) = 1; // px
                kf.errorCovPre.at<float>(7) = 1; // px
                kf.errorCovPre.at<float>(14) = 1;
                kf.errorCovPre.at<float>(21) = 1;
                kf.errorCovPre.at<float>(28) = 1; // px
                kf.errorCovPre.at<float>(35) = 1; // px

                state.at<float>(0) = meas.at<float>(0);
                state.at<float>(1) = meas.at<float>(1);
                state.at<float>(2) = meas.at<float>(2);
                state.at<float>(3) = 0;
                state.at<float>(4) = 0;
                state.at<float>(5) = 0;
                // <<<< Initialization

                kf.statePost = state;
                kf_init = true;
            }
            //************kalman filter initialization end*************
            yolo_in = false;
        }
        cout<<"Notinit_yolo count: "<<Notinit_yolo<<endl;
    }
}

void gta_ns::TrackingNodelet::PTfilter(pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_track, double cluster_size)
{
    float x_low, y_low, z_low, x_up, y_up, z_up;
    x_low = state_true[0]-0.25 * cluster_size;
    y_low = state_true[1]-0.3 * cluster_size;
    z_low = state_true[2]-0.3 * cluster_size;
    x_up = state_true[0]+0.25 * cluster_size;
    y_up = state_true[1]+0.6 * cluster_size;
    z_up = state_true[2]+0.3 * cluster_size;
    pcl::PassThrough<pcl::PointXYZ> pt;
    // for(int i = 0; i<11; i++)
    // {
    //     cout<<"pointcloud sample: "<< pcl_track->points[i+50]<<endl;
    // }
    // Filter in z direction
    pt.setInputCloud(pcl_track);
    pt.setFilterFieldName("z");
    pt.setFilterLimits(z_low, z_up);
    pt.setFilterLimitsNegative(false);
    pt.filter(*pcl_track);
    // Filter in x direction
    pt.setInputCloud(pcl_track);
    pt.setFilterFieldName("x");
    pt.setFilterLimits(x_low, x_up);
    pt.setFilterLimitsNegative(false);
    pt.filter(*pcl_track);
    // Filter in y direction
    pt.setInputCloud(pcl_track);
    pt.setFilterFieldName("y");
    pt.setFilterLimits(y_low, y_up);
    pt.setFilterLimitsNegative(false);
    pt.filter(*pcl_track);
    cout<<"pcl_track size: "<<pcl_track->points.size()<<endl;

    sensor_msgs::PointCloud2 PT_pc;
    pcl::toROSMsg(*pcl_track, PT_pc);
    PT_pc.header.frame_id = "world";
    PT_pc.header.stamp = timestamp;
    pub_PTpc.publish(PT_pc);
}

void gta_ns::TrackingNodelet::cluster(pcl::PointCloud<pcl::PointXYZ>::Ptr in_pcl, Vec3 &obj_pc_pos)
{
    

    // Creating the KdTree object for the search method of the extraction
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud(in_pcl);

    // Cluster
    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setClusterTolerance(0.06); // 10cm
    ec.setMinClusterSize(2);
    ec.setMaxClusterSize(50);
    ec.setSearchMethod(tree);
    ec.setInputCloud(in_pcl);
    ec.extract(cluster_indices);
    cout<<"cluster_indices: "<<cluster_indices.size()<<endl;

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster(new pcl::PointCloud<pcl::PointXYZ>);

    if(!kf_init)
    {
        int j = 0;
        for (const auto &cluster : cluster_indices)
        {
            if (j <= cluster.indices.size())
            {
                cloud_cluster->clear();
                j = cluster.indices.size();
                for (const auto &idx : cluster.indices)
                {
                    cloud_cluster->push_back((*in_pcl)[idx]);
                    // cout<<"(*in_pcl)[idx]: "<<(*in_pcl)[idx]<<endl;
                } //*
                cloud_cluster->width = cloud_cluster->size();
                cloud_cluster->height = 1;
                cloud_cluster->is_dense = true;
            }
        }
        Vector4f centroid_int;
        pcl::compute3DCentroid(*cloud_cluster, centroid_int);
        if(centroid_int[0] > 0 && centroid_int[0]<=4.0)
        {
            obj_pc_pos[0] = centroid_int[0];
            obj_pc_pos[1] = centroid_int[1];
            obj_pc_pos[2] = centroid_int[2];
            cluster_true = true;
        }
        
    }

    else
    {
        double a, b, c, d, e=100000;
        Vector3f centroid;

        for (const auto &cluster : cluster_indices)
        {
            cloud_cluster->clear();
            for (const auto &idx : cluster.indices)
            {
                cloud_cluster->push_back((*in_pcl)[idx]);
                // cout<<"(*in_pcl)[idx]: "<<(*in_pcl)[idx]<<endl;
            }
            cloud_cluster->width = cloud_cluster->size();
            cloud_cluster->height = 1;
            cloud_cluster->is_dense = true;
            Vector4f centroid_depth;
            pcl::compute3DCentroid(*cloud_cluster, centroid_depth);
            a = last_meas_pos[0] - centroid_depth[0];
            b = last_meas_pos[1] - centroid_depth[1];
            c = last_meas_pos[2] - centroid_depth[2];
            d = a * a + b * b + c * c;
            if (d < e && centroid_depth[0]<=4.0)
            {
                centroid[0] = centroid_depth[0];
                centroid[1] = centroid_depth[1];
                centroid[2] = centroid_depth[2];
                e = d;
            }
        }
        if(centroid[0] > 0)
        {
            obj_pc_pos[0] = centroid[0];
            obj_pc_pos[1] = centroid[1];
            obj_pc_pos[2] = centroid[2];
            cluster_true = true;
        }
        
    }

   

    // cout<<"cluster pc_meas_pos: "<<pc_meas_pos<<endl;
}

void gta_ns::TrackingNodelet::removeZeros(pcl::PointCloud<pcl::PointXYZ>::Ptr pc)
{
    obj_pc->clear();
    for(int i = 0; i<pc->points.size(); i++)
    {
        if(pc->points[i].x == 0)
        {
            continue;
        }
        else{
            obj_pc->points.push_back(pc->points[i]);
        }
    }
    obj_pc->width = obj_pc->size();
    obj_pc->height = 1;
    obj_pc->is_dense = true;
}

// void gta_ns::TrackingNodelet::CalculateCentroid(vector<Point> dbscan_points, Vec3 &pc_meas_pos)
// {
//     double x = 0, y = 0, z = 0;
//     for(int i=0; i<dbscan_points.size(); i++)
//     {
//         x = x+dbscan_points[i].x;
//         y = y+dbscan_points[i].x;
//         z = z+dbscan_points[i].x;
//     }
//     pc_meas_pos[0] = x/dbscan_points.size();
//     pc_meas_pos[1] = y/dbscan_points.size();
//     pc_meas_pos[2] = z/dbscan_points.size();
//     cluster_true = true;
// }


