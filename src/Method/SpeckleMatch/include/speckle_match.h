#ifndef _SPECKLE_MATCH_H
#define _SPECKLE_MATCH_H

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <stdint.h>

typedef struct SearchWindow_s
{
        uint8_t top;
        uint8_t bottom;
        uint8_t left;
        uint8_t right;
}SearchWindow;

class SpeckleMatch
{
public:
    SpeckleMatch() = default;
    virtual ~SpeckleMatch() = default;

    /**
     * @brief SetMatchThreahold 设置匹配阈值
     * @param corse_cc_th       粗匹配阈值
     * @param refine_cc_th      精匹配阈值
     */
    bool SetMatchThreashold(double corse_cc_th, double refine_cc_th);

    /**
     * @brief TemplateMatch 模板匹配
     * @param ref_img       参考图
     * @param template_img  模板图
     * @param guide_pt      指导点
     * @param swin          搜索窗口
     * @param matched_pt    模板中心点对应的匹配点
     * @return              匹配成功时返回true
     */
    bool TemplateMatch(const cv::Mat& ir_img, const cv::Mat& template_img,
                       const cv::Point2f guide_pt, const SearchWindow& swin, cv::Point2f& matched_pt);

    /**
     * @brief SubPixel      亚像素
     * @param ref_img       参考图
     * @param template_img  模板图
     * @param guide_pt      指导点
     * @param sub_pixel_pt  指导点的亚像素位置
     */
    bool SubPixel(const cv::Mat& ir_img, const cv::Mat& template_img,
                  const cv::Point2f guide_pt, cv::Point2f& sub_pixel_pt);
private:
    /**
     * @brief JudgeOutOfBound   判断ROI是否会越界，并修正ROI的大小
     * @param rect              ROI区域
     * @param min_x             x方向最小边界条件
     * @param min_y             y方向最小边界条件
     * @param max_x             x方向最大边界条件
     * @param max_y             y方向最大边界条件
     */
    void JudgeOutOfBound(cv::Rect &rect, int min_x, int min_y, int max_x, int max_y)
    {
            if (rect.x < min_x)
                    rect.x = min_x;
            if (rect.y < min_y)
                    rect.y = min_y;
            if (rect.x + rect.width > max_x)
                    rect.width = max_x - rect.x;
            if (rect.y + rect.height > max_y)
                    rect.height = max_y - rect.y;
    }

    /**
     * @brief Gx    x方向的梯度
     * @param img   灰度图像
     * @param pt    参考点
     * @return      返回参考点处x方向的梯度
     */
    float Gx(const cv::Mat& img, const cv::Point2f pt);

    /**
     * @brief Gy    y方向的梯度
     * @param img   灰度图像
     * @param pt    参考点
     * @return      返回参考点处y方向的梯度
     */
    float Gy(const cv::Mat& img, const cv::Point2f pt);

    /**
     * @brief BiLinear      双线性插值
     * @param src           原图
     * @param x
     * @param y
     * @param defaultVal    缺省值(如果所搜索的坐标越界将返回此值)
     * @return              原图在(x, y)处的灰度
     */
    int BiLinear(cv::Mat src, float x, float y, int defaultVal = 0);

private:
    double corse_cc_th_ = 0.6;    // 粗匹配相关性阈值
    double refine_cc_th_ = 0.8;   // 精匹配相关系阈值
};

#endif
