#include "include/yolo.h"

void YoloNode::img_cb(const sensor_msgs::CompressedImage::ConstPtr& rgbimage)
{
    if (intiated)
    {
        try
        {
            frame = cv::imdecode(cv::Mat(rgbimage->data), 1);
            res = cv::imdecode(cv::Mat(rgbimage->data), 1);
            gt = cv::imdecode(cv::Mat(rgbimage->data), 1);
        }
        catch (cv_bridge::Exception &e)
        {
            ROS_ERROR("cv_bridge exception: %s", e.what());
        }
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
                    cv::Point depthbox_vertice1 = cv::Point(potential[maxElementIndex].center_bdbox.x - depthbox_w / 2, potential[maxElementIndex].center_bdbox.y - depthbox_w / 2);
                    cv::Point depthbox_vertice2 = cv::Point(potential[maxElementIndex].center_bdbox.x + depthbox_w / 2, potential[maxElementIndex].center_bdbox.y + depthbox_h / 2);

                    if (prob > 0.7)
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
                }
            }
            time_end=ros::Time::now().toSec();
            // cout<<"ms: "<<time_end-time_start<<endl<<endl;
            
            cv::imshow("Yolo", frame);
            cv::imshow("Tracking...", res);
            cv::waitKey(20);
        }
    }
}

inline void YoloNode::YoloNodeletClassInit(const char *parampath_input, const char *binfile_input)
{
    this->cnn_local->opt.num_threads = 8; //You need to compile with libgomp for multi thread support
        this->cnn_local->opt.use_vulkan_compute = false; //You need to compile with libvulkan for gpu support

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
            if(succeed!=0)
                throw "bad doggie!";
        }
        catch(...)
        {
            cerr << "check! Initiaion fail!\n";
            exit(0);
        }

        this->cnn_local->load_param(parampath_input);
        this->cnn_local->load_model(binfile_input);
        intiated = true;
        printf("initialization succeed\n");
}


void YoloNode::detect_yolo(cv::Mat& bgr)
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
        const float* values = out.row(i);

        Object object;
        object.label = values[0];
        object.prob = values[1];
        object.rect.x = values[2] * img_w;
        object.rect.y = values[3] * img_h;
        object.rect.width = values[4] * img_w - object.rect.x;
        object.rect.height = values[5] * img_h - object.rect.y;
        object.classnameofdetection = class_names[object.label];
        object.frame = bgr;
        object.center_bdbox = cv::Point(object.rect.x+object.rect.width/2, object.rect.y+object.rect.height/2);
        objects.push_back(object);
    }
    this->total_end = std::chrono::steady_clock::now();
    total_fps = 1000.0 / std::chrono::duration_cast<std::chrono::milliseconds>(total_end - total_start).count();
    this->appro_fps = total_fps;
    for (size_t i = 0; i < objects.size(); i++)
    {
        const Object& obj = objects[i];
        // fprintf(stderr, "%d = %.5f at %.2f %.2f %.2f x %.2f\n", obj.label, obj.prob,
        //         obj.rect.x, obj.rect.y, obj.rect.width, obj.rect.height);
    }
    draw_objects(bgr, objects);
}


void YoloNode::draw_objects(cv::Mat& bgr, const std::vector<Object>& objects)
{
    for (size_t i = 0; i < objects.size(); i++)
    {
        const Object& obj = objects[i];

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

