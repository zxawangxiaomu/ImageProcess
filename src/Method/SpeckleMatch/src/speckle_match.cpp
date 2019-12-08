#include "speckle_match.h"
#include "Eigen/Dense"
#include <functional>

bool SpeckleMatch::TemplateMatch(const cv::Mat &ir_img, const cv::Mat &template_img, const cv::Point2f guide_pt, const SearchWindow &swin, cv::Point2f &matched_pt)
{
    // 对输入的IR图进行高斯滤波
    cv::Mat ir_img_gaussian = ir_img.clone();
//    cv::GaussianBlur(ir_img, ir_img_gaussian, cv::Size(5, 5), 8); // 窗口大小为5x5，sigma > 2
    int template_half_cols = (template_img.cols - 1) / 2;
    int template_half_rows = (template_img.rows - 1) / 2;
    // 根据窗口大小定义匹配(匹配区域需要比搜索区域大，不然边界点难以匹配)
    cv::Rect match_region(guide_pt.x - swin.left - template_half_cols,
        guide_pt.y - swin.top - template_half_rows,
        swin.left + swin.right + template_half_cols * 2 + 1,
        swin.top + swin.bottom + template_half_rows * 2 + 1);

    // 这个搜索区域的最小坐标是(0, 0)，最大坐标是(ir_img_gaussian.cols - 1, ir_img_gaussian.rows - 1)
    JudgeOutOfBound(match_region, 0, 0, ir_img_gaussian.cols - 1, ir_img_gaussian.rows - 1);
    // 使用模板与搜索区域部分进行匹配
    cv::Mat match_result;
    cv::matchTemplate(ir_img_gaussian(match_region), template_img, match_result, cv::TM_CCOEFF_NORMED);
    cv::Point best_point;
    double max_value = 0;
    cv::minMaxLoc(match_result, nullptr, &max_value, nullptr, &best_point);
    if(max_value < corse_cc_th_)
    {
        return false;
    }
    // 将坐标回归到原IR图上面
    matched_pt = best_point + match_region.tl() + cv::Point(template_half_cols, template_half_rows);
    return true;
}


/**
 *参考《摄影测量学》第二版p164 二、单点最小二乘影像匹配
 */
bool SpeckleMatch::SubPixel(const cv::Mat &ir_img, const cv::Mat &template_img, const cv::Point2f guide_pt, cv::Point2f &sub_pixel_pt)
{
    // 残差模型:
    // f(x, y) + e(x, y) = h0 + h1 * g(x_, y_)
    // x_ = a0 + a1 * x + a2 * y
    // y_ = b0 + b1 * x + b2 * y
    // p = [r0, r1, a0, a1, a2, b0, b1, b2], dp = [dr0, dr1, da0, da1, da2, db0, db1, db2]
    Eigen::VectorXf p(8), last_p(8), dp(8);;
    p(0) = 0;            // r0
    p(1) = 1;            // r1
	p(2) = guide_pt.x - (template_img.cols - 1) * 0.5f;   // a0
    p(3) = 1;            // a1
    p(4) = 0;            // a2
	p(5) = guide_pt.y - (template_img.rows - 1) * 0.5f;   // b0
    p(6) = 0;            // b1
    p(7) = 1;            // b2

    int r = template_img.rows;
    int c = template_img.cols;
    int data_num = r * c;
    Eigen::MatrixXf A(data_num, 8);
    Eigen::VectorXf l(data_num);
    float last_coeff = 0;
    float coeff = 0;

    while (true)
    {
        int idx = 0;

        float h[2] = {p(0), p(1)};
        float a[3] = {p(2), p(3), p(4)};
        float b[3] = {p(5), p(6), p(7)};

        float sum_g1_g2 = 0;
        float sum_g1_sqare = 0;
        float sum_g2_sqare = 0;

        for(int y = 0; y < template_img.rows; ++y)
        {
            for(int x = 0; x < template_img.cols; ++x)
            {
                // 求解几何变形与辐射变形变化量:
                // f(x, y) - g_o(x, y) = dr0 + g_o(x, y) * dr1 +
                //                       g_x * da0 + g_x * x * da1 + g_x * y * da2 +
                //                       g_y * db0 + g_y * x * db1 + g_y * y * db2
                // 几何校正
                float x2 = a[0] + a[1]*x + a[2]*y;
                float y2 = b[0] + b[1]*x + b[2]*y;

                // 重采样
                float g2 = BiLinear(ir_img, x2, y2);

                // 辐射变形校正
                g2 = h[0] + h[1] * g2;

                // 组织数据---迭代条件
                float g1 = template_img.ptr<uint8_t>(y)[x];
                sum_g1_g2 += g1 * g2;
                sum_g1_sqare += g1 * g1;
                sum_g2_sqare += g2 * g2;

                // 组织数据---变形参数
                A(idx, 0) = 1;                                      // dr0
                A(idx, 1) = g2;                                     // dr1 这里的g2已经是辐射校正后的g2了
                A(idx, 2) = h[1]*Gx(ir_img, cv::Point2f(x2, y2));   // da0 原文是在辐射校正后的图像上计算梯度和光强的，
                A(idx, 3) = h[1]*x2*Gx(ir_img, cv::Point2f(x2, y2));// da1 这里我没有对整幅ref图做辐射校正，Gx是基于原
                A(idx, 4) = h[1]*y2*Gx(ir_img, cv::Point2f(x2, y2));// da2 图计算的，所以这里梯度多了一个h1，即:
                A(idx, 5) = h[1]*Gy(ir_img, cv::Point2f(x2, y2));   // db0 ((h0 + h1 * gl) - (h0 + h1 * gr))/2 = h1*(gl - gr)/2
                A(idx, 6) = h[1]*x2*Gy(ir_img, cv::Point2f(x2, y2));// db1 ((h0 + h1 * gu) - (h0 + h1 * gd))/2 = h1*(gu - gd)/2
                A(idx, 7) = h[1]*y2*Gy(ir_img, cv::Point2f(x2, y2));// db2

                l(idx) = g1 - g2;
                idx++;
            }
        }

        // 是否满足停止迭代条件---相关系数
        coeff = sum_g1_g2 * sum_g1_g2 / (sum_g1_sqare * sum_g2_sqare);
        if(coeff < last_coeff)
        {
            p = last_p;
            break;
        }
        last_coeff = coeff;
        last_p = p;

        // 计算变形参数
        dp = A.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(l);

		dp = 1*dp;

        float dh[2] = {dp(0), dp(1)};
        float da[3] = {dp(2), dp(3), dp(4)};
        float db[3] = {dp(5), dp(6), dp(7)};

        // 更新变形参数---几何变形
        a[0] = a[0] + da[0] + a[0]*da[1] + b[0]*da[2];
        a[1] = a[1] + a[1]*da[1] + b[1]*da[2];
        a[2] = a[2] + a[2]*da[1] + b[2]*da[2];
        b[0] = b[0] + db[0] + a[0]*db[1] + b[0]*db[2];
        b[1] = b[1] + a[1]*db[1] + b[1]*db[2];
        b[2] = b[2] + a[2]*db[1] + b[2]*db[2];

        // 更新变形参数---辐射变形
        h[0] = h[0] + dh[0] + h[0]*dh[1];
        h[1] = h[1] + h[1]*dh[1];

        // 更新到p中
        p << h[0], h[1], a[0], a[1], a[2], b[0], b[1], b[2];
    }

    if(coeff < refine_cc_th_)
    {
        return false;
    }
	 
	// 应用得到的几何畸变
    float a[3] = {p(2), p(3), p(4)};
    float b[3] = {p(5), p(6), p(7)};

    // template的中心点坐标
	float x = (template_img.cols - 1) * 0.5f;
	float y = (template_img.rows - 1) * 0.5f;
    sub_pixel_pt.x = a[0] + a[1]*x + a[2]*y;
    sub_pixel_pt.y = b[0] + b[1]*x + b[2]*y;
}

float SpeckleMatch::Gx(const cv::Mat &img, const cv::Point2f pt)
{
    // 左右点的坐标
    cv::Point2f pt_l;
    pt_l.x = pt.x - 1;
    pt_l.y = pt.y;
    cv::Point2f pt_r;
    pt_r.x = pt.x + 1;
    pt_r.y = pt.y;

    // 约束点的坐标, 如果点处于边界，这里的处理相当于边缘复制
    std::function<void(cv::Point2f&)> ConstraintPoint = [&img](cv::Point2f& pt){
        if(pt.x < 0)            pt.x = 0;
        if(pt.x > img.cols - 1) pt.x = img.cols - 1;
        if(pt.y < 0)            pt.y = 0;
        if(pt.y > img.rows - 1) pt.y = img.rows - 1;
    };
    ConstraintPoint(pt_l);
    ConstraintPoint(pt_r);

    // 左右点光强
    int g_l = BiLinear(img, pt_l.x, pt_l.y);
    int g_r = BiLinear(img, pt_r.x, pt_r.y);
    return (g_r - g_l) / 2.0f;
}

float SpeckleMatch::Gy(const cv::Mat &img, const cv::Point2f pt)
{
    // 上下点的坐标
    cv::Point2f pt_u;
    pt_u.x = pt.x;
    pt_u.y = pt.y - 1;
    cv::Point2f pt_d;
    pt_d.x = pt.x;
    pt_d.y = pt.y + 1;

    // 约束点的坐标, 如果点处于边界，这里的处理相当于边缘复制
    std::function<void(cv::Point2f&)> ConstraintPoint = [&img](cv::Point2f& pt){
        if(pt.x < 0)            pt.x = 0;
        if(pt.x > img.cols - 1) pt.x = img.cols - 1;
        if(pt.y < 0)            pt.y = 0;
        if(pt.y > img.rows - 1) pt.y = img.rows - 1;
    };
    ConstraintPoint(pt_u);
    ConstraintPoint(pt_d);

    // 上下点光强
    int g_u = BiLinear(img, pt_u.x, pt_u.y);
    int g_d = BiLinear(img, pt_d.x, pt_d.y);
    return (g_d - g_u) / 2.0f;
}

int SpeckleMatch::BiLinear(cv::Mat src, float x, float y, int defaultVal)
{

    int ix = (int)x;
    int iy = (int)y;
    float a[4];
    if (ix<0 || iy<0 || ix + 1>src.cols - 1 || iy + 1>src.rows - 1)
    {
        return defaultVal;
    }
    float dx = x - ix;
    float dy = y - iy;

    a[0] = src.at<unsigned char>(iy, ix);
    a[1] = src.at<unsigned char>(iy, ix + 1) - a[0];
    a[2] = src.at<unsigned char>(iy + 1, ix) - a[0];
    a[3] = src.at<unsigned char>(iy + 1, ix + 1) - a[0] - a[1] - a[2];

    return std::round(a[0] + a[1] * dx + a[2] * dy + a[3] * dx*dy);
}
