#include "include/gta_pc.h"

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

        double tpf = 0;
        int w = 200, h = 200;
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
        bool got = false;

        if (temp.size() != 0)
        {
            for (auto stuff : temp)
            {
                // cout << stuff.classnameofdetection << endl;
                if (stuff.classnameofdetection == "ball")
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

                got = true;

                tpf = appro_fps;
                w = interested.width;
                h = interested.height;
                tempp = potential[maxElementIndex].frame;
                prob = potential[maxElementIndex].prob;
                // cout<<"prob: "<<prob<<endl;

                int depthbox_w = potential[maxElementIndex].rect.width * 0.75;
                int depthbox_h = potential[maxElementIndex].rect.height * 0.75;
                depthbox_vertice1 = cv::Point(potential[maxElementIndex].center_bdbox.x - depthbox_w / 2, potential[maxElementIndex].center_bdbox.y - depthbox_w / 2);
                depthbox_vertice2 = cv::Point(potential[maxElementIndex].center_bdbox.x + depthbox_w / 2, potential[maxElementIndex].center_bdbox.y + depthbox_h / 2);
                yolo_in = true;
                if (prob > 0.3)
                {
                    center_true = cv::Point(interested.x + interested.width / 2, interested.y + interested.height / 2);
                    cv::rectangle(res, interested, CV_RGB(255, 255, 0), 1);
                    gta::vertice box_vertices;
                    box_vertices.X_a = depthbox_vertice1.x;
                    box_vertices.Y_a = depthbox_vertice1.y;
                    box_vertices.X_b = depthbox_vertice2.x;
                    box_vertices.Y_b = depthbox_vertice2.y;
                    box_vertices.center_x = center_true.x;
                    box_vertices.center_y = center_true.y;
                    box_vertices.header.stamp = rgbimage->header.stamp;
                    pub_ver.publish(box_vertices);
                    // cout<<"yolo timestamp: "<<box_vertices.header.stamp<<endl;
                    // cout<<"lala"<<endl;
                }

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
                    double depth_average;
                    if (nonzerosvalue.size() != 0)
                        depth_average = accumulate(nonzerosvalue.begin(), nonzerosvalue.end(), 0.0) / nonzerosvalue.size();
                    double depth_cam = 0.001 * depth_average; // depth from camera
                    img_meas_pos[2] = depth_cam;
                    img_meas_pos[0] = depth_cam * (center_true.x - K_cam(0,2))/K_cam(0,0);
                    img_meas_pos[1] = depth_cam * (center_true.y - K_cam(1,2))/K_cam(1,1);
                /*********************get depthdata******************/
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
    for (int i = 0; i < plsize; i++)
    {
        temppc.x = pclmsg->points[i].x;
        temppc.y = pclmsg->points[i].y;
        temppc.z = pclmsg->points[i].z;
        pc->points.push_back(temppc);
    }
    cout<<"pc->points.size(): "<<pc->points.size()<<endl;

    //run kalman filter
    if (!kf_init)
    {
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

    double precTick = ticks;
    ticks = (double)cv::getTickCount();
    dT = (ticks - precTick) / cv::getTickFrequency(); // seconds
    // >>>> Matrix A
    kf.transitionMatrix.at<float>(3) = dT;
    kf.transitionMatrix.at<float>(10) = dT;
    kf.transitionMatrix.at<float>(17) = dT;

    state = kf.predict();

    state_pre[0] = state.at<float>(0);
    state_pre[1] = state.at<float>(1);
    state_pre[2] = state.at<float>(2);

    if(yolo_in)
    {
        Vec4 X;
        Vec3 Y;
        pcl::PointCloud<pcl::PointXYZ>::Ptr depth_pcl(new pcl::PointCloud<pcl::PointXYZ>);
        for (int i = 0; i <= pc->points.size(); i++)
        {

            X[0] = pc->points[i].x;
            X[1] = pc->points[i].y;
            X[2] = pc->points[i].z;
            X[3] = 1;
            Y = P_cam * mat_lidar_cam * X;
            cv::Point pt;
            pt.x = Y[0] / Y[2];
            pt.y = Y[1] / Y[2];
            // cout<<"lidar point on RGB image: "<<pt.x<<" "<<pt.y<<endl;

            if (pt.x >= depthbox_vertice2.x || pt.y >= depthbox_vertice2.y || pt.x <= depthbox_vertice1.x || pt.y <= depthbox_vertice1.y)
            {
                continue;
            }
            else
            {
                depth_pcl->points.push_back(pc->points[i]);
            }
        }
        cout << "depth_pcl->points.size(): " << depth_pcl->points.size() << endl;
        if(depth_pcl->points.size() < 2)
        {
            use_dpeth = true;
            meas_pos[0] = img_meas_pos[0];
            meas_pos[1] = img_meas_pos[1];
            meas_pos[2] = img_meas_pos[2];
            yolo_in = false;
        }
        else
        {
            cluster(depth_pcl);
            meas_pos[0] = pc_meas_pos[0];
            meas_pos[1] = pc_meas_pos[1];
            meas_pos[2] = pc_meas_pos[2];
            use_dpeth = false;
            yolo_in = false;
        }
    }

    else
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_track(new pcl::PointCloud<pcl::PointXYZ>);
        double filter_size = 0.7;
        PTfilter(pcl_track, filter_size);
        if (pcl_track->points.size() < 2)
        {
            use_dpeth = true;
            meas_pos[0] = img_meas_pos[0];
            meas_pos[1] = img_meas_pos[1];
            meas_pos[2] = img_meas_pos[2];
        }
        else
        {
            cluster(pcl_track);
            meas_pos[0] = pc_meas_pos[0];
            meas_pos[1] = pc_meas_pos[1];
            meas_pos[2] = pc_meas_pos[2];
            use_dpeth = false;
        }
    }
    
    meas.at<float>(0) = meas_pos[0];
    meas.at<float>(1) = meas_pos[1];
    meas.at<float>(2) = meas_pos[2];

    kf.correct(meas); // Kalman Correction

    state_true[0] = state.at<float>(0);
    state_true[1] = state.at<float>(1);
    state_true[2] = state.at<float>(2);

    if(state_true[2] != 0)
    {
        gta::obj obj_pos_body;
        obj_pos_body.X_c = state_true[0];
        obj_pos_body.Y_c = state_true[1];
        obj_pos_body.Z_c = state_true[2];
        obj_pos_body.header.stamp = ros::Time::now();
        pub_meas.publish(obj_pos_body);
    }

}

void gta_ns::TrackingNodelet::PTfilter(pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_track, double cluster_size)
{
    float x_low, y_low, z_low, x_up, y_up, z_up;
    x_low = state_pre[0]-0.5 * cluster_size;
    y_low = state_pre[1]-0.5 * cluster_size;
    z_low = state_pre[2]-0.5 * cluster_size;
    x_up = state_pre[0]+0.5 * cluster_size;
    y_up = state_pre[1]+0.5 * cluster_size;
    z_up = state_pre[2]+0.5 * cluster_size;
    pcl::PassThrough<pcl::PointXYZ> pt;
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
}

void gta_ns::TrackingNodelet::cluster(pcl::PointCloud<pcl::PointXYZ>::Ptr depth_pcl)
{
    // remove NAN points
    std::vector<int> NAN_indices;
    pcl::removeNaNFromPointCloud(*depth_pcl, *depth_pcl, NAN_indices);

    // Creating the KdTree object for the search method of the extraction
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud(depth_pcl);

    // Cluster
    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setClusterTolerance(0.05); // 5cm
    ec.setMinClusterSize(1);
    ec.setMaxClusterSize(25000);
    ec.setSearchMethod(tree);
    ec.setInputCloud(depth_pcl);
    ec.extract(cluster_indices);

    int j = 0;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster(new pcl::PointCloud<pcl::PointXYZ>);
    for (const auto &cluster : cluster_indices)
    {
        for (const auto &idx : cluster.indices)
        {
            cloud_cluster->push_back((*depth_pcl)[idx]);
        } //*
        cloud_cluster->width = cloud_cluster->size();
        cloud_cluster->height = 1;
        cloud_cluster->is_dense = true;
        j++;
    }

    Vector4f centroid_depth;
    // pcl::compute3DCentroid(*depth_pcl, centroid_depth);
    pcl::compute3DCentroid(*cloud_cluster, centroid_depth);
    pc_meas_pos[0] = centroid_depth[0];
    pc_meas_pos[1] = centroid_depth[1];
    pc_meas_pos[2] = centroid_depth[2];
}